#ifndef    AWS_MQTT_HPP_INCLUDED
#define    AWS_MQTT_HPP_INCLUDED

// MQTT api headers
#include "core_mqtt.h"
#include "core_mqtt_state.h"

#include <queue>
#include <string>
#include <memory>

#include "aws/NetworkContext.hpp"
#include "aws/ExponentialBackOff.hpp"
#include "interface/InterfaceUserImpl.hpp"
#include "util/Mutex.hpp"


/**
 * @brief   MQTT Publish message
 */
class MQTTPubData
{
public:
    MQTTPubData( const std::string& topic, const std::string& data );
    ~MQTTPubData();

    uint16_t PacketID() const;
    const std::string& Topic() const;
    const std::string& Data() const;

private:

    uint16_t    m_PacketID;
    std::string m_Topic;
    std::string m_Data;
};


/**
 * @brief   AWS coreMQTT library wrapper
 * 
 */
class MQTTConnection
{
public:

    enum State
    {
        STATE_DISCONNECTED,
        STATE_MQTT_CONNECTION_RETRY,
        STATE_CONNECTED,
        STATE_DISCONNECT_CLEANUP,
        STATE_ERROR,
    };

    static constexpr int sk_OutgoingPublishDataBufSize = 10;

public:
    
    MQTTConnection();
    ~MQTTConnection();
    
    // DO NOT COPY
    MQTTConnection( MQTTConnection& ) = delete;
    MQTTConnection& operator=( const MQTTConnection& ) = delete;

    bool Initialize( std::unique_ptr<I_NetworkContext> tls_context );
    bool Connect();
    bool Publish( const MQTTPubData& data );
    void EventLoop();
    State GetState() const;
    
private:

    static void AWSCoreMQTTEventHandler( MQTTContext_t* mqtt_context,
                                         MQTTPacketInfo_t* packet_info,
                                         MQTTDeserializedInfo_t* deserialized_info );
    
    void awsMQTTCoreProcessLoopWithTimeOut();
    void retryConnectionWithBackOff();
    bool tryConnection();
    bool establishMqttSession();
    void calculateConnectNextBackOff();
    void sendPublishQueue();
    void disconnectCleanUp();

    std::unique_ptr<I_NetworkContext> m_NetworkInterface;

    MutexValue<State>         m_State;
    MQTTContext_t             m_MQTTContext;

    // for Network buffer
    NetworkContext_t          m_NetworkContext;
    std::vector<uint8_t>      m_NetworkBuffer;

    // for send publish QoS1/QoS2 
    std::vector<MQTTPubAckInfo_t> m_OutgoingPublishDataBuf;

    // for Connect backoff algorithm
    ExponentialBackOff        m_ConnectBackOff;
    uint32_t                  m_TryConnectPrevTimeMs;
    // for Publish backoff algorithm
    ExponentialBackOff        m_PubSendBackOff;
    uint32_t                  m_PubPrevSendTimeMs;

    // for publish data queue
    Mutex                     m_PubQueueMutex;
    MutexValue<bool>          m_SendPubQueue;
    std::queue<MQTTPubData>   m_PubDataQueue;
};


#endif    // AWS_MQTT_HPP_INCLUDED