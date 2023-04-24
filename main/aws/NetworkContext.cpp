#include "NetworkContext.hpp"

#include <limits>
#include <algorithm>

#include "esp_tls.h"
#include "esp_log.h"
#include "sys/socket.h"

namespace
{
void esp_tls_t_deleter( esp_tls_t* context )
{
    if( context != nullptr ){
        esp_tls_conn_destroy( context );
    }
}
}

TLSNetworkContextInitParam::TLSNetworkContextInitParam()
{}

TLSNetworkContextInitParam::~TLSNetworkContextInitParam()
{}

bool TLSNetworkContextInitParam::Initialize(
        const std::string& interface_name,
        const std::string& server_root_ca_pem,
        const std::string& client_cert_pem,
        const std::string& client_key_pem,
        const std::string& hostname,
        int port_number,
        BaseType_t  disable_sni,
        const std::vector<std::string>& alpn_protos,
        uint32_t network_timeout_ms
    )
{
    // check string size to prepend overflow
    if(( server_root_ca_pem.length() >= std::numeric_limits<uint32_t>::max() )
     ||( client_cert_pem.length() >= std::numeric_limits<uint32_t>::max() )
     ||( client_key_pem.length() >= std::numeric_limits<uint32_t>::max() )
     ||( hostname.length() >= std::numeric_limits<uint32_t>::max() ))
    {
        return false;
    }
     
    m_InterfaceName = interface_name;
    m_ServerRootCAPem = server_root_ca_pem;
    m_ClientCertPem = client_cert_pem;
    m_ClientKeyPem = client_key_pem;
    m_HostName = hostname;
    m_PortNumber = port_number;
    m_DisableSNI = disable_sni;
    m_AlpnProtos = alpn_protos;
    m_NetworkTimeoutMs = network_timeout_ms;

    for( auto& str : m_AlpnProtos ){
        m_AlpnProtosBuffer.push_back( &str[0] );
    }
    // add nullptr as end mark
    m_AlpnProtosBuffer.push_back( nullptr );

    return true;
}

const char* TLSNetworkContextInitParam::GetInterfaceName() const
{
    return m_InterfaceName.c_str();
}

uint32_t TLSNetworkContextInitParam::GetInterfaceNameLength() const
{
    return m_InterfaceName.length();
}

const char* TLSNetworkContextInitParam::GetServerRootCAPem() const
{
    return m_ServerRootCAPem.c_str();
}

uint32_t TLSNetworkContextInitParam::GetServerRootCAPemLength() const
{  
    return m_ServerRootCAPem.length();
}

const char* TLSNetworkContextInitParam::GetClientCertPem() const
{
    return m_ClientCertPem.c_str();
}

uint32_t TLSNetworkContextInitParam::GetClientCertPemLength() const
{
    return m_ClientCertPem.length();
}

const char* TLSNetworkContextInitParam::GetClientKeyPem() const
{
    return m_ClientKeyPem.c_str();
}

uint32_t TLSNetworkContextInitParam::GetClientKeyLength() const
{
    return m_ClientKeyPem.length();
}

const char* TLSNetworkContextInitParam::GetHostName() const
{
    return m_HostName.c_str();
}

uint32_t TLSNetworkContextInitParam::GetHostNameLength() const
{
    return m_HostName.length();
}
    
int TLSNetworkContextInitParam::GetPortNumber() const
{
    return m_PortNumber;
}
    
BaseType_t TLSNetworkContextInitParam::GetDisableSni() const
{
    return m_DisableSNI;
}

char** TLSNetworkContextInitParam::GetAlpnProtos()
{
    return m_AlpnProtosBuffer.data();
}

uint32_t TLSNetworkContextInitParam::GetNetworkTimeoutMs() const
{
    return m_NetworkTimeoutMs;
}



ESPTlsContext::ESPTlsContext()
    :
    m_Mutex(),
    m_InitParam(),
    m_TlsHandle( nullptr ),
    m_IfReq( nullptr )
{}

ESPTlsContext::~ESPTlsContext()
{}

bool ESPTlsContext::Initialize( const std::shared_ptr<TLSNetworkContextInitParam>& param )
{
    m_InitParam = param;
    return m_Mutex.IsValid();
}

bool ESPTlsContext::Connect()
{
    if( !m_Mutex.IsValid() || m_TlsHandle != nullptr ){
        return false;
    }

    ESP_LOGI( "ESPTlsContext", "ESPTlsContext Connect invoked. initparam[%p]", m_InitParam.get() );
    esp_tls_cfg_t esp_tls_config = {};

    esp_tls_config.alpn_protos = const_cast<const char**>(m_InitParam->GetAlpnProtos());
    esp_tls_config.cacert_buf = reinterpret_cast<const unsigned char*>(m_InitParam->GetServerRootCAPem());
    esp_tls_config.cacert_bytes = m_InitParam->GetServerRootCAPemLength() + 1;      // Add null character length
    esp_tls_config.clientcert_buf = reinterpret_cast<const unsigned char*>(m_InitParam->GetClientCertPem());
    esp_tls_config.clientcert_bytes = m_InitParam->GetClientCertPemLength() + 1;    // Add null character length
    esp_tls_config.clientkey_buf = reinterpret_cast<const unsigned char*>(m_InitParam->GetClientKeyPem());
    esp_tls_config.clientkey_bytes = m_InitParam->GetClientKeyLength() + 1;         // Add null character length
    esp_tls_config.use_secure_element = false;
    esp_tls_config.timeout_ms = m_InitParam->GetNetworkTimeoutMs();
    esp_tls_config.skip_common_name = m_InitParam->GetDisableSni();
    // This application designed that EventLoop runs other task,
    // so recv()/send() function to access transport layer must be non blocking.
    esp_tls_config.non_block = true;                                                
    esp_tls_config.ds_data = nullptr;

    m_IfReq = std::make_shared<ifreq>();
    esp_tls_config.if_name = m_IfReq.get();
    
    // determine network interface
    int ifname_len = std::min<uint32_t>( IFNAMSIZ - 1, m_InitParam->GetInterfaceNameLength() );
    ESP_LOGI( "ESPTlsContext", "ifname original[%s], ifname length[%d]", m_InitParam->GetInterfaceName(), ifname_len );
    std::copy_n( m_InitParam->GetInterfaceName(), ifname_len, esp_tls_config.if_name->ifr_name );
    esp_tls_config.if_name->ifr_name[ifname_len] = '\0';                            // terminate by null character
    ESP_LOGI( "ESPTlsContext", "ifname created. bind interface[%s]", esp_tls_config.if_name->ifr_name );

    LockGuard mutex( m_Mutex );
    m_TlsHandle = std::shared_ptr<esp_tls_t>( esp_tls_init(), esp_tls_t_deleter );

    int result = 0;
    while(1){
        result = esp_tls_conn_new_sync ( 
                    m_InitParam->GetHostName(),
                    m_InitParam->GetHostNameLength(),
                    m_InitParam->GetPortNumber(),
                    &esp_tls_config, 
                    m_TlsHandle.get() );

        // If return == 0, connection state is in progress. Recall connection api.
        if( result != 0 ){
            break;
        }
    }

    if( result < 0 ){
        // connection establish error, delete tls handle
        m_TlsHandle = nullptr;
        return false;
    }

    // success
    return true;
}

bool ESPTlsContext::Disconnect()
{
    if( m_TlsHandle == nullptr ){
        return false;
    }

    LockGuard mutex( m_Mutex );
    m_TlsHandle = nullptr;

    return true;
}

int32_t ESPTlsContext::Send( const uint8_t* data, size_t datalen )
{
    if( m_TlsHandle == nullptr || data == nullptr ){
        return -1;
    }
    if( datalen == 0 ){
        return 0;
    }

    int32_t bytes_sent = 0;
    LockGuard mutex( m_Mutex );

    bytes_sent = esp_tls_conn_write( m_TlsHandle.get(), data, datalen );

    if( bytes_sent == ESP_TLS_ERR_SSL_WANT_WRITE || bytes_sent == ESP_TLS_ERR_SSL_WANT_READ ){
        return 0;               // More read/write data needded. Needs to be called again this function
    }
    if( bytes_sent >= 0 ){
        return bytes_sent;      // success
    }

    return bytes_sent;          // error some reason
}

int32_t ESPTlsContext::Recv( uint8_t* data, size_t datalen )
{
    if( m_TlsHandle == nullptr || data == nullptr ){
        ESP_LOGW( "ESPTlsContext", "invalid argument." );
        return -1;
    }
    if( datalen == 0 ){
        ESP_LOGW( "ESPTlsContext", "datalength = 0" );
        return 0;
    }

    int32_t bytes_read = 0;
    LockGuard mutex( m_Mutex );

    bytes_read = esp_tls_conn_read( m_TlsHandle.get(), data, datalen );

    if( bytes_read > 0 ){
        return bytes_read;      // success
    }
    if( bytes_read == 0 ){
        ESP_LOGW( "ESPTlsContext", "error, because connection closed" );
        return -1;              // error, because connection closed
    }
    if( bytes_read == ESP_TLS_ERR_SSL_WANT_WRITE || bytes_read == ESP_TLS_ERR_SSL_WANT_READ ){
                                // ESP_TLS_ERR_SSL_WANT_WRITE/ESP_TLS_ERR_SSL_WANT_READ
        return 0;               // More read/write data needded. Needs to be called again this function
    }

    ESP_LOGW( "ESPTlsContext", "error, reason [%d, %08X]", -bytes_read, (uint32_t)-bytes_read );
    return bytes_read;          // error some reason
}
