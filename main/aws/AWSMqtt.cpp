#include "aws/AWSMqtt.hpp"
#include "util/Mutex.hpp"
#include "util/Logger.hpp"
#include "interface/InterfaceUserImpl.hpp"


// 
// constant variables
//
static const ExponentialBackOff sk_ExponentialBackoffDefault = {
    CONFIG_AWS_BACKOFF_BASE_WAIT_DEFAULT_MS, 
    CONFIG_AWS_BACKOFF_MAX_WAIT_DEFAULT_MS, 
    CONFIG_AWS_BACKOFF_MAX_ATTEMPT
};

//
// static variables
//
static Logger s_Logger = { "AWS_MQTT" , Logger::L_Debug };
static uint16_t s_MQTTPublishGlobalPacketID = 0; 


MQTTPubData::MQTTPubData( const std::string& topic, const std::string& data )
    :
    m_PacketID( ++s_MQTTPublishGlobalPacketID ),
    m_Topic( topic ),
    m_Data( data )
{}

MQTTPubData::~MQTTPubData()
{}

uint16_t MQTTPubData::PacketID() const
{
    return m_PacketID;
}

const std::string& MQTTPubData::Topic() const
{
    return m_Topic;
}

const std::string& MQTTPubData::Data() const
{
    return m_Data;
}


MQTTConnection::MQTTConnection()
    :
    m_NetworkInterface(),
    m_State( STATE_DISCONNECTED ),
    m_MQTTContext(),
    m_NetworkContext(),
    m_NetworkBuffer(),
    m_OutgoingPublishDataBuf( sk_OutgoingPublishDataBufSize ),
    m_ConnectBackOff( sk_ExponentialBackoffDefault ),
    m_TryConnectPrevTimeMs( 0 ),
    m_PubSendBackOff( sk_ExponentialBackoffDefault ),
    m_PubPrevSendTimeMs( 0 ),
    m_PubQueueMutex(),
    m_SendPubQueue( false ),
    m_PubDataQueue()
{}

MQTTConnection::~MQTTConnection()
{}

bool MQTTConnection::Initialize( std::unique_ptr<I_NetworkContext> tls_context )
{
    m_NetworkInterface = std::move(tls_context);
    m_NetworkContext.Context = m_NetworkInterface.get();
    s_Logger.Info( "MQTTConnection::Initialize() invoked. network context[%p]", m_NetworkInterface.get() );

    // Create TransportInterface
    TransportInterface_t transport;
    transport.pNetworkContext = &m_NetworkContext;
    transport.send = TransportSend;
    transport.recv = TransportReceive;
    transport.writev = nullptr;

    // Set network buffer
    MQTTFixedBuffer_t network_buffer;
    m_NetworkBuffer.resize( CONFIG_AWS_MQTT_NETWORK_BUFFER_SIZE );
    network_buffer.pBuffer = m_NetworkBuffer.data();
    network_buffer.size = m_NetworkBuffer.size();

    // Initialize MQTT
    MQTTStatus_t mqtt_status = MQTT_Init( &m_MQTTContext,
                                          &transport,
                                          GetCurrentTimeMs,
                                          AWSCoreMQTTEventHandler,
                                          &network_buffer );

    if( mqtt_status != MQTTSuccess ){
        s_Logger.Error( "MQTT_Init() in MQTTConnection failed. result[%s]", MQTT_Status_strerror(mqtt_status) );
        return false;
    }
    
    mqtt_status = MQTT_InitStatefulQoS( &m_MQTTContext,
                                        m_OutgoingPublishDataBuf.data(),
                                        m_OutgoingPublishDataBuf.size(),
                                        nullptr,
                                        0 );
    
    if( mqtt_status != MQTTSuccess ){
        s_Logger.Error( "MQTT_InitStatfulQoS() in MQTTConnection failed. result[%s]", MQTT_Status_strerror(mqtt_status) );
        return false;
    }

    s_Logger.Info( "MQTTConnection::Initialize() result[%d]", mqtt_status );

    return true;
}

bool MQTTConnection::Connect()
{
    LockGuard guard(m_State.mutex);
    
    if( m_State.value == STATE_DISCONNECTED ){
        m_State.value = STATE_MQTT_CONNECTION_RETRY;
        return true;
    }

    return false;
}

bool MQTTConnection::Publish( const MQTTPubData& data )
{
    if( m_State.Get() == STATE_CONNECTED ){
        LockGuard mutex_lock( m_PubQueueMutex );
        m_PubDataQueue.push( data );

        return true;
    }

    return false;
}

void MQTTConnection::AWSCoreMQTTEventHandler( MQTTContext_t * mqtt_context,
                                              MQTTPacketInfo_t * packet_info,
                                              MQTTDeserializedInfo_t * deserialized_info )
{
    uint16_t packet_identifier;

    assert( mqtt_context != NULL );
    assert( packet_info != NULL );
    assert( deserialized_info != NULL );

    /* Suppress unused parameter warning when asserts are disabled in build. */
    ( void ) mqtt_context;

    packet_identifier = deserialized_info->packetIdentifier;

    /* Handle incoming publish. The lower 4 bits of the publish packet
     * type is used for the dup, QoS, and retain flags. Hence masking
     * out the lower bits to check if the packet is publish. */
    if( ( packet_info->type & 0xF0U ) == MQTT_PACKET_TYPE_PUBLISH )
    {
        // subscribe not implemented
    }
    else
    {
        /* Handle other packets. */
        switch( packet_info->type )
        {
            case MQTT_PACKET_TYPE_SUBACK:
                // subscribe not implemented
                break;

            case MQTT_PACKET_TYPE_UNSUBACK:
                // subscribe not implemented
                break;

            case MQTT_PACKET_TYPE_PINGRESP:
                /* Nothing to be done from application as library handles
                 * PINGRESP. */
                s_Logger.Warn( "PINGRESP should not be handled by the application callback when using MQTT_ProcessLoop." );
                break;

            case MQTT_PACKET_TYPE_PUBREC:
                s_Logger.Info( "PUBREC received for packet id %u", packet_identifier );
                /* Cleanup publish packet when a PUBREC is received. */
                //leanupOutgoingPublishWithPacketID( packet_identifier );
                break;

            case MQTT_PACKET_TYPE_PUBREL:
                /* Nothing to be done from application as library handles
                 * PUBREL. */
                s_Logger.Info( "PUBREL received for packet id %u", packet_identifier );
                break;

            case MQTT_PACKET_TYPE_PUBCOMP:
                /* Nothing to be done from application as library handles
                 * PUBCOMP. */
                s_Logger.Info( "PUBCOMP received for packet id %u", packet_identifier );
                break;

            /* Any other packet type is invalid. */
            default:
                s_Logger.Error( "Unknown packet type received:(%02X)", packet_info->type );
                break;
        }
    }
}

void MQTTConnection::EventLoop()
{
    switch( m_State.Get() ){
    case STATE_DISCONNECTED:
        // do nothing
        break;
    case STATE_MQTT_CONNECTION_RETRY:
        retryConnectionWithBackOff();
        break;
    case STATE_CONNECTED:
        sendPublishQueue();
        awsMQTTCoreProcessLoopWithTimeOut();
        break;
    case STATE_DISCONNECT_CLEANUP:
        disconnectCleanUp();
        break;
    case STATE_ERROR:
        // do nothing
        break;
    default:
        break;
    }
}

MQTTConnection::State MQTTConnection::GetState() const
{
    return m_State.Get();
}

void MQTTConnection::awsMQTTCoreProcessLoopWithTimeOut()
{
    uint32_t mqtt_process_loop_timeout_ms = 0;
    uint32_t current_time_ms = 0;

    MQTTStatus_t mqtt_status = MQTTSuccess;

    current_time_ms = m_MQTTContext.getTime();
    mqtt_process_loop_timeout_ms = current_time_ms + CONFIG_AWS_MQTT_CORE_PROCESS_LOOP_TIMEOUT_MS;

    s_Logger.Info( "Process Loop start." );
    // Call MQTT_ProcessLoop multiple times a timeout happens, or
    // MQTT_ProcessLoop fails.
    while( ( current_time_ms < mqtt_process_loop_timeout_ms ) &&
           ( mqtt_status == MQTTSuccess || mqtt_status == MQTTNeedMoreBytes ) )
    {
        mqtt_status = MQTT_ProcessLoop( &m_MQTTContext );
        current_time_ms = m_MQTTContext.getTime();
    }

    if( ( mqtt_status != MQTTSuccess ) && ( mqtt_status != MQTTNeedMoreBytes ) )
    {
        s_Logger.Error( "MQTT_ProcessLoop returned with status = %s.", MQTT_Status_strerror( mqtt_status ) );
        m_State = STATE_DISCONNECT_CLEANUP;
    }
}

void MQTTConnection::retryConnectionWithBackOff()
{
    uint32_t interval = GetCurrentTimeMs() - m_TryConnectPrevTimeMs;
    if( interval >= m_ConnectBackOff.GetNextRetryTimeMs() ){
        tryConnection();
    }
}

bool MQTTConnection::tryConnection()
{
    s_Logger.Info( "MQTTConnection::tryConnection() invoked." );

    bool connect_result = m_NetworkInterface->Connect();
    bool return_status = false;

    if( connect_result ){
        s_Logger.Info( "MQTTConneciton::Connect() succeeded. Try establish MQTT connection." );
        /* Sends an MQTT Connect packet using the established TLS session,
         * then waits for connection acknowledgment (CONNACK) packet. */
        bool is_established_session = establishMqttSession();

        if( is_established_session ){
            s_Logger.Info( "MQTTConnection::establishMqttSession() succeeded. established MQTT connection completedly." );
            // Connection established
            m_SendPubQueue = true;
            return_status  = true;

            m_State = STATE_CONNECTED;
        }
    }

    if( return_status ){
        m_ConnectBackOff = sk_ExponentialBackoffDefault;
    }
    else {
        m_NetworkInterface->Disconnect();
        m_TryConnectPrevTimeMs = GetCurrentTimeMs();
        m_ConnectBackOff.CalculateNextBackOff();

        if( m_ConnectBackOff.IsExhausted() ){
            s_Logger.Error( "MQTTConnection retry exhausted. Can't establish connection." );
            m_State = STATE_ERROR;
        }
        else {
            s_Logger.Warn( "MQTTConnection::establishMqttSession() failed. Disconnect TLS and retry. CurrentMs[%d] retry interval[%d]", m_TryConnectPrevTimeMs, m_ConnectBackOff.GetNextRetryTimeMs() );
        }
    }

    return return_status;    
}

bool MQTTConnection::establishMqttSession()
{
    int return_status = EXIT_SUCCESS;
    MQTTStatus_t mqtt_status;
    MQTTConnectInfo_t connect_info;
    bool session_present = false;

    /* Establish MQTT session by sending a CONNECT packet. */

    /* If #createCleanSession is true, start with a clean session
     * i.e. direct the MQTT broker to discard any previous session data.
     * If #createCleanSession is false, directs the broker to attempt to
     * reestablish a session which was already present. */
    // Create clean session every time.
    // If CreateCleanSession is false, reestablish a session with keep previous session data.
    connect_info.cleanSession = false;

    /* The client identifier is used to uniquely identify this MQTT client to
     * the MQTT broker. In a production device the identifier can be something
     * unique, such as a device serial number. */
    connect_info.pClientIdentifier = CONFIG_AWS_MQTT_CLIENT_IDENTIFIER;
    connect_info.clientIdentifierLength = static_cast<uint16_t>(sizeof(CONFIG_AWS_MQTT_CLIENT_IDENTIFIER) - 1);

    /* The maximum time interval in seconds which is allowed to elapse
     * between two Control Packets.
     * It is the responsibility of the Client to ensure that the interval between
     * Control Packets being sent does not exceed the this Keep Alive value. In the
     * absence of sending any other Control Packets, the Client MUST send a
     * PINGREQ Packet. */
    connect_info.keepAliveSeconds = CONFIG_AWS_MQTT_KEEP_ALIVE_INTERVAL_SECONDS;

    /* Username and password for authentication. Not used in this demo. */
    connect_info.pUserName = NULL;
    connect_info.userNameLength = 0U;
    connect_info.pPassword = NULL;
    connect_info.passwordLength = 0U;

    /* Send MQTT CONNECT packet to broker. */
    mqtt_status = MQTT_Connect( &m_MQTTContext, &connect_info, nullptr, CONFIG_AWS_MQTT_CONNACK_RECV_TIMEOUT_MS, &session_present );

    if( mqtt_status != MQTTSuccess ){
        return_status = EXIT_FAILURE;
        s_Logger.Error( "Connection with MQTT broker failed with status %s.",
                         MQTT_Status_strerror( mqtt_status ) );
    } 
    else {
        s_Logger.Info( "MQTT connection successfully established with broker." );
    }

    return return_status == MQTTSuccess;
}

void MQTTConnection::sendPublishQueue()
{
    MQTTStatus_t mqtt_status = MQTTSuccess;

    s_Logger.Debug( "sendPublishQueue" );
    if( m_SendPubQueue.Get() == true ){
        LockGuard mutex_lock( m_PubQueueMutex );
        if( !m_PubDataQueue.empty() ){
            MQTTPubData data = m_PubDataQueue.front();

            MQTTPublishInfo_t pubinfo;
            pubinfo.qos = MQTTQoS0;
            pubinfo.retain          = false;
            pubinfo.dup             = false;
            pubinfo.pTopicName      = data.Topic().c_str();
            pubinfo.topicNameLength = data.Topic().length();
            pubinfo.pPayload        = data.Data().c_str();
            pubinfo.payloadLength   = data.Data().length();

            s_Logger.Debug( "Publish data from queue: topic[%s], data[%s]", data.Topic().c_str(), data.Data().c_str() );
            mqtt_status = MQTT_Publish( &m_MQTTContext, &pubinfo, data.PacketID() );
            if( mqtt_status == MQTTSuccess ){
                s_Logger.Info( "Publish data from queue: topic[%s], data[%s] success!", data.Topic().c_str(), data.Data().c_str() );
                m_PubDataQueue.pop();
                m_PubSendBackOff = sk_ExponentialBackoffDefault;
            }
            else {
                s_Logger.Error( "Sending duplicate PUBLISH failed with status %s.", MQTT_Status_strerror( mqtt_status ) );

                // WAIT send publish until next timing
                m_SendPubQueue = false;

                m_PubPrevSendTimeMs = GetCurrentTimeMs();
                m_PubSendBackOff.CalculateNextBackOff();
                
                if( m_PubSendBackOff.IsExhausted() ){
                    m_State = STATE_DISCONNECT_CLEANUP;
                }
            }
        }
    }
    else {
        uint32_t interval = GetCurrentTimeMs() - m_PubPrevSendTimeMs;
        s_Logger.Debug( "Send publish stop interval : %d", interval );
        if( interval >= m_PubSendBackOff.GetNextRetryTimeMs() ){
            // reach resending time.
            m_SendPubQueue = true;
        }
    }
}

void MQTTConnection::disconnectCleanUp()
{
    // Clear publish send queue
    m_SendPubQueue = false;
    m_PubDataQueue = std::queue<MQTTPubData>();

    // Reset connection backoff algorithm
    m_ConnectBackOff = sk_ExponentialBackoffDefault;
    m_TryConnectPrevTimeMs = 0;
    // Reset publish backoff algorithm
    m_PubSendBackOff = sk_ExponentialBackoffDefault;
    m_PubPrevSendTimeMs = 0;

    // disconnect MQTT, Layer4(TCP/TLS)
    MQTT_Disconnect( &m_MQTTContext );
    m_NetworkInterface->Disconnect();

    m_State = STATE_DISCONNECTED;
    s_Logger.Info( "MQTTConnection::disconnectCleanUp() completed." );
}
