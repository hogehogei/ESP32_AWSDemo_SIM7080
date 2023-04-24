
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "cxx_include/esp_modem_dte.hpp"
#include "esp_modem_config.h"
#include "cxx_include/esp_modem_api.hpp"
#include "esp_event.h"
#include "sdkconfig.h"

#include "aws/NetworkContext.hpp"
#include "aws/AWSMqtt.hpp"
#include "aws/ExponentialBackOff.hpp"
#include "util/Logger.hpp"

using namespace esp_modem;

#if defined(CONFIG_EXAMPLE_FLOW_CONTROL_NONE)
#define EXAMPLE_FLOW_CONTROL ESP_MODEM_FLOW_CONTROL_NONE
#elif defined(CONFIG_EXAMPLE_FLOW_CONTROL_SW)
#define EXAMPLE_FLOW_CONTROL ESP_MODEM_FLOW_CONTROL_SW
#elif defined(CONFIG_EXAMPLE_FLOW_CONTROL_HW)
#define EXAMPLE_FLOW_CONTROL ESP_MODEM_FLOW_CONTROL_HW
#endif

template <typename T>
struct ThreadParam {
    ThreadParam()
        : join(false), value() {}

    MutexValue<bool> join;
    T value;
};


#ifdef __cplusplus
extern "C"{
#endif

static Logger s_Logger = { "AWS_LTE_DEMO" , Logger::L_Debug };
static EventGroupHandle_t event_group = NULL;
static const int CONNECT_BIT = BIT0;
static const int GOT_DATA_BIT = BIT2;

extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");

//
//  static functions
//
static bool SIM_WaitUntilWakeup( std::shared_ptr<DCE> dce );
static void AWS_MQTT_SendDemo( MQTTConnection* conn );
static void AWS_MQTT_ConnTask( void* parameter );
static std::shared_ptr<MQTTConnection> CreateMQTTConnection();

static void on_ppp_changed( void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data )
{
    s_Logger.Info( "PPP state changed event %d", event_id );
    //ESP_LOGI(TAG, "PPP state changed event %d", event_id);
    if( event_id == NETIF_PPP_ERRORUSER ){
        /* User interrupted event from esp-netif */
        esp_netif_t *netif = reinterpret_cast<esp_netif_t*>( event_data );
        s_Logger.Info( "User interrupted event from netif:%p", reinterpret_cast<uint32_t>(netif) );
        //ESP_LOGI(TAG, "User interrupted event from netif:%p", netif);
    }
}

static void on_ip_event( void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data )
{
    s_Logger.Debug( "IP event! %d", event_id );
    if( event_id == IP_EVENT_PPP_GOT_IP ){
        esp_netif_dns_info_t dns_info;

        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        esp_netif_t *netif = event->esp_netif;

        s_Logger.Info( "Modem Connect to PPP Server" );
        
        s_Logger.Info( "~~~~~~~~~~~~~~" );
        s_Logger.Info( "IP          : %d.%d.%d.%d", IP2STR(&event->ip_info.ip) );
        s_Logger.Info( "Netmask     : %d.%d.%d.%d", IP2STR(&event->ip_info.netmask) );
        s_Logger.Info( "Gateway     : %d.%d.%d.%d", IP2STR(&event->ip_info.gw) );
        esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info);
        s_Logger.Info( "Name Server1: %d.%d.%d.%d", IP2STR(&dns_info.ip.u_addr.ip4) );
        esp_netif_get_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns_info);
        s_Logger.Info( "Name Server2: %d.%d.%d.%d", IP2STR(&dns_info.ip.u_addr.ip4) );
        s_Logger.Info( "~~~~~~~~~~~~~~" );
        xEventGroupSetBits(event_group, CONNECT_BIT);

        s_Logger.Info( "GOT ip event!!!" );
    } 
    else if ( event_id == IP_EVENT_PPP_LOST_IP ){
        s_Logger.Info( "Modem Disconnect from PPP Server" );
    } 
    else if ( event_id == IP_EVENT_GOT_IP6 ){
        s_Logger.Info( "GOT IPv6 event!" );

        ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
        s_Logger.Info( "Got IPv6 address %04X.%04X.%04X.%04X.%04X.%04X.%04X.%04X", IPV62STR(event->ip6_info.ip) );
    }
}

void app_main(void)
{
    /* Init and register system/core components */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &on_ip_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, &on_ppp_changed, NULL));

    /* Configure the PPP netif */
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG( CONFIG_EXAMPLE_MODEM_PPP_APN );
    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
    esp_netif_inherent_config* p = const_cast<esp_netif_inherent_config*>(netif_ppp_config.base);
    p->if_key = "ppp0";
    s_Logger.Info( "configure PPP netif, name[%s]", netif_ppp_config.base->if_key );
    esp_netif_t *esp_netif = esp_netif_new( &netif_ppp_config );
    assert(esp_netif);

    event_group = xEventGroupCreate();
    
    /* Configure the DTE */
    esp_modem::dte_config dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    /* setup UART specific configuration based on kconfig options */
    dte_config.uart_config.port_num = UART_NUM_1;
    dte_config.uart_config.data_bits = UART_DATA_8_BITS;
    dte_config.uart_config.stop_bits = UART_STOP_BITS_1;
    dte_config.uart_config.parity = UART_PARITY_DISABLE;
    dte_config.uart_config.baud_rate  = 115200;
    dte_config.uart_config.tx_io_num = CONFIG_EXAMPLE_MODEM_UART_TX_PIN;
    dte_config.uart_config.rx_io_num = CONFIG_EXAMPLE_MODEM_UART_RX_PIN;
    dte_config.uart_config.rts_io_num = CONFIG_EXAMPLE_MODEM_UART_RTS_PIN;
    dte_config.uart_config.cts_io_num = CONFIG_EXAMPLE_MODEM_UART_CTS_PIN;
    dte_config.uart_config.flow_control = EXAMPLE_FLOW_CONTROL;
    dte_config.uart_config.rx_buffer_size = CONFIG_EXAMPLE_MODEM_UART_RX_BUFFER_SIZE;
    dte_config.uart_config.tx_buffer_size = CONFIG_EXAMPLE_MODEM_UART_TX_BUFFER_SIZE;
    dte_config.uart_config.event_queue_size = CONFIG_EXAMPLE_MODEM_UART_EVENT_QUEUE_SIZE;
    dte_config.task_stack_size = CONFIG_EXAMPLE_MODEM_UART_EVENT_TASK_STACK_SIZE;
    dte_config.task_priority = CONFIG_EXAMPLE_MODEM_UART_EVENT_TASK_PRIORITY;
    dte_config.dte_buffer_size = CONFIG_EXAMPLE_MODEM_UART_RX_BUFFER_SIZE / 2;
    auto uart_dte = create_uart_dte( &dte_config );
    std::shared_ptr<DCE> dce = create_generic_dce( &dce_config, uart_dte, esp_netif );

    assert( dce );

    if( dte_config.uart_config.flow_control == ESP_MODEM_FLOW_CONTROL_HW ){
        if( dce->set_flow_control( 2, 2 ) != command_result::OK ){  //2/2 means HW Flow Control.
            s_Logger.Error( "Failed to set the set_flow_control mode" );
            return;
        }
        s_Logger.Info( "HW set_flow_control OK" );
    }

    // wait for wakeup LTE Module
    s_Logger.Info( "APN Name: [%s]", CONFIG_EXAMPLE_MODEM_PPP_APN );
    s_Logger.Info( "Wait for wakeup LTE Module..." );

    if( !SIM_WaitUntilWakeup( dce ) ){
        return;
    }

    xEventGroupClearBits( event_group, CONNECT_BIT | GOT_DATA_BIT );

    if( !dce->set_mode(modem_mode::DATA_MODE) ){
        s_Logger.Error( "esp_modem_set_mode(ESP_MODEM_MODE_DATA) failed." );
        return;
    }

    /* Wait for IP address */
    s_Logger.Info( "Waiting for getting IP address..." );
    xEventGroupWaitBits( event_group, CONNECT_BIT, pdFALSE, pdFALSE, portMAX_DELAY );
 
    while(1) {
        // Create MQTT Connection
        std::shared_ptr<MQTTConnection> mqtt_connection = CreateMQTTConnection();
        if( !mqtt_connection.get() ){
            s_Logger.Error( "Create MQTTConnection failed." );
        }

        ThreadParam<MQTTConnection*> threadparam;
        threadparam.value = mqtt_connection.get();

        // Create AWS MQTT connection loop
        s_Logger.Info( "Create AWS_MQTT_ConnTask..." );
        TaskHandle_t aws_mqtt_task_handle;
        xTaskCreatePinnedToCore( AWS_MQTT_ConnTask,            // task function 
                                "AWS_MQTT_ConnTask",           // task name
                                1024 * 8,                      // task stack size
                                &threadparam,                  // task argv
                                configMAX_PRIORITIES - 3,      // task priority, ESP32 library tasks runs (max-2) priority
                                &aws_mqtt_task_handle,         // task handle
                                1 );                           // CoreID 0:main core, 1:sub core

        // Demo application 
        AWS_MQTT_SendDemo( mqtt_connection.get() );

        // Join thread
        while( threadparam.join.Get() == false ) {
            vTaskDelay( pdMS_TO_TICKS(10) );
        }
    }

    // UART DTE clean-up
    esp_netif_destroy( esp_netif );
}

static bool SIM_WaitUntilWakeup( std::shared_ptr<DCE> dce )
{
    constexpr int k_RetryCountMax = 5;
    int retry_count = 0;
    int rssi = 0;
    int ber = 0;
    bool is_get_signal_quality_ok = false;

    while( !is_get_signal_quality_ok ){
        if( dce->get_signal_quality( rssi, ber ) == command_result::OK ){
            s_Logger.Info( "Signal quality: rssi=%d, ber=%d", rssi, ber );
            is_get_signal_quality_ok = true;
        }
        else {
            ++retry_count;
            if( retry_count >= k_RetryCountMax ){
                s_Logger.Error( "esp_modem_get_signal_quality Retry exhausted. abort initialize SIM" );
                break;
            }
            else {
                s_Logger.Warn( "esp_modem_get_signal_quality failed. Retry[%d]", retry_count );
            }
            
            vTaskDelay( pdMS_TO_TICKS(10000) );
        }
    }

    return is_get_signal_quality_ok;
}

static void AWS_MQTT_SendDemo( MQTTConnection* conn )
{
    // main loop
    int64_t pubdata_prev_time_ms = GetCurrentTimeMs();

    s_Logger.Info( "AWS_MQTT_SendDemo() invoked. conneciton handle[%p]", conn );

    while(1){
        MQTTConnection::State state = conn->GetState();
        int64_t current_time_ms = GetCurrentTimeMs();
        //s_Logger.Debug( "CurrentState [%d]", state );

        if( state == MQTTConnection::STATE_DISCONNECTED ){
            s_Logger.Info( "TLS retry connect invoked." );
            conn->Connect();
        }
        else if( state == MQTTConnection::STATE_MQTT_CONNECTION_RETRY ){

        }
        else if( state == MQTTConnection::STATE_CONNECTED ){
            //s_Logger.Debug( "MQTTConnected." );
            int64_t pubdata_elapsed_time_ms = current_time_ms - pubdata_prev_time_ms;
            if( pubdata_elapsed_time_ms >= 10000 ){
                MQTTPubData pubdata = { "tempareture", "25.0" };
                conn->Publish( pubdata );
                s_Logger.Info( "Publish data topic=[%s], data=[%s]", pubdata.Topic().c_str(), pubdata.Data().c_str() );
                pubdata_prev_time_ms = GetCurrentTimeMs();
            }
        }
        else if( state == MQTTConnection::STATE_ERROR ){
            s_Logger.Warn( "MQTT ErrorState! Retry connection." );
            break;
        }
        else {
        }

        vTaskDelay( pdMS_TO_TICKS(500) );
    }
}

static void AWS_MQTT_ConnTask( void* parameter )
{
    ThreadParam<MQTTConnection*>* param = reinterpret_cast<ThreadParam<MQTTConnection*>*>(parameter);
    MQTTConnection* conn = param->value;

    while( conn->GetState() != MQTTConnection::STATE_ERROR ){
        conn->EventLoop();
    }

    param->join = true;

    vTaskDelete( nullptr );
}

static std::shared_ptr<MQTTConnection> CreateMQTTConnection()
{
    // Create TLS context
    /* AWS IoT requires devices to send the Server Name Indication (SNI)
     * extension to the Transport Layer Security (TLS) protocol and provide
     * the complete endpoint address in the host_name field. Details about
     * SNI for AWS IoT can be found in the link below.
     * https://docs.aws.amazon.com/iot/latest/developerguide/transport-security.html */
    std::vector<std::string> alpn_protocols;
    if( CONFIG_AWS_MQTT_PORT == 443 )
    {
        /* Pass the ALPN protocol name depending on the port being used.
         * Please see more details about the ALPN protocol for the AWS IoT MQTT
         * endpoint in the link below.
         * https://aws.amazon.com/blogs/iot/mqtt-with-tls-client-authentication-on-port-443-why-it-is-useful-and-how-it-works/
         *
         * For username and password based authentication in AWS IoT,
         * #AWS_IOT_PASSWORD_ALPN is used. More details can be found in the
         * link below.
         * https://docs.aws.amazon.com/iot/latest/developerguide/custom-authentication.html
         */

        #ifdef CLIENT_USERNAME
            alpn_protocols.push_back = AWS_IOT_PASSWORD_ALPN;
        #else
            alpn_protocols.push_back( CONFIG_AWS_IOT_MQTT_ALPN );
        #endif
    }

    auto init_param = std::make_shared<TLSNetworkContextInitParam>();
    if( !init_param->Initialize(
            //"ppp0",                                   // interface name that specified when creating net_if(ESP_NETIF_INHERENT_DEFAULT_PPP)
            "",
            std::string(reinterpret_cast<const char*>(aws_root_ca_pem_start)),         // server root CA
            std::string(reinterpret_cast<const char*>(certificate_pem_crt_start)),     // client cert
            std::string(reinterpret_cast<const char*>(private_pem_key_start)),         // client key
            std::string(CONFIG_AWS_IOT_ENDPOINT),       // hostname
            CONFIG_AWS_MQTT_PORT,                       // portnum
            0,                                          // disable sni
            alpn_protocols,                             // alpn protocol strings
            CONFIG_TLS_NETWORK_TIMEOUT_MS               // network timeout ms
        ) )
    {
        s_Logger.Error( "TLSNetwork parameter setting creation failed!" );
        return nullptr;
    }

    std::unique_ptr<ESPTlsContext> tls_context( new ESPTlsContext() );
    if( !tls_context->Initialize( init_param ) ){
        s_Logger.Error( "TLSNetwork initialize failed!" );
        return nullptr;
    }

    // Create MQTT connection context
    std::shared_ptr<MQTTConnection> mqtt_connection = std::make_shared<MQTTConnection>();
    if( !mqtt_connection->Initialize(std::move(tls_context)) ){
        s_Logger.Error( "MQTT Connection initialize failed!" );
        return nullptr;
    }

    return mqtt_connection;
}

#ifdef __cplusplus
}
#endif