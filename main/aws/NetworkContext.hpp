#ifndef    I_NETWORK_CONTEXT_HPP_INCLUDED
#define    I_NETWORK_CONTEXT_HPP_INCLUDED

#include <cstdint>
#include <string>
#include <vector>
#include <memory>

#include "esp_tls.h"

#include "util/Mutex.hpp"

class I_NetworkContext
{
public:

    I_NetworkContext() {}
    virtual ~I_NetworkContext() {}

    virtual bool Connect() = 0;
    virtual bool Disconnect() = 0;
    virtual int32_t Send( const uint8_t* data, size_t datalen ) = 0;
    virtual int32_t Recv( uint8_t* data, size_t datalen ) = 0;

private:
};

class TLSNetworkContextInitParam
{
public:

    TLSNetworkContextInitParam();
    ~TLSNetworkContextInitParam();

    // DO NOT COPY
    TLSNetworkContextInitParam( const TLSNetworkContextInitParam& ) = delete;
    TLSNetworkContextInitParam& operator=( const TLSNetworkContextInitParam& ) = delete;

    bool Initialize(
        const std::string& interface_name,
        const std::string& server_root_ca_pem,
        const std::string& client_cert_pem,
        const std::string& client_key_pem,
        const std::string& hostname,
        int port_number,
        BaseType_t  disable_sni,
        const std::vector<std::string>& alpn_protos,
        const uint32_t network_timeout_ms
    );

    const char* GetInterfaceName() const;
    uint32_t GetInterfaceNameLength() const;
    const char* GetServerRootCAPem() const;
    uint32_t GetServerRootCAPemLength() const;
    const char* GetClientCertPem() const;
    uint32_t GetClientCertPemLength() const;
    const char* GetClientKeyPem() const;
    uint32_t GetClientKeyLength() const;
    const char* GetHostName() const;
    uint32_t GetHostNameLength() const;
    
    int GetPortNumber() const;
    
    BaseType_t GetDisableSni() const;
    char** GetAlpnProtos();
    uint32_t GetNetworkTimeoutMs() const;

private:

    std::string m_InterfaceName;
    std::string m_ServerRootCAPem;
    std::string m_ClientCertPem;
    std::string m_ClientKeyPem;
    std::string m_HostName;
    int m_PortNumber;
    BaseType_t m_DisableSNI;
    std::vector<std::string> m_AlpnProtos;
    std::vector<char*> m_AlpnProtosBuffer;
    uint32_t m_NetworkTimeoutMs;
};

class ESPTlsContext : public I_NetworkContext
{
public:

    ESPTlsContext();
    virtual ~ESPTlsContext();

    ESPTlsContext( ESPTlsContext& ) = delete;
    ESPTlsContext& operator=( ESPTlsContext& ) = delete;

    bool Initialize( const std::shared_ptr<TLSNetworkContextInitParam>& param );
    virtual bool Connect();
    virtual bool Disconnect();
    virtual int32_t Send( const uint8_t* data, size_t datalen );
    virtual int32_t Recv( uint8_t* data, size_t datalen );

private:

    Mutex m_Mutex;

    std::shared_ptr<TLSNetworkContextInitParam> m_InitParam;
    std::shared_ptr<esp_tls_t> m_TlsHandle;
    std::shared_ptr<ifreq>     m_IfReq;
};


#endif    // I_NETWORK_CONTEXT_HPP_INCLUDED