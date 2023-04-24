#ifndef CORE_MQTT_CONFIG_H
#define CORE_MQTT_CONFIG_H

/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */

/**
 * @ingroup mqtt_constants
 * @brief Maximum number of vectors in subscribe and unsubscribe packet.
 */
#ifndef MQTT_SUB_UNSUB_MAX_VECTORS
    #define MQTT_SUB_UNSUB_MAX_VECTORS    ( 4U )
#endif

/**
 * @brief The number of retries for receiving CONNACK.
 *
 * The MQTT_MAX_CONNACK_RECEIVE_RETRY_COUNT will be used only when the
 * timeoutMs parameter of #MQTT_Connect is passed as 0 . The transport
 * receive for CONNACK will be retried MQTT_MAX_CONNACK_RECEIVE_RETRY_COUNT
 * times before timing out. A value of 0 for this config will cause the
 * transport receive for CONNACK  to be invoked only once.
 *
 * <b>Possible values:</b> Any positive 16 bit integer. <br>
 * <b>Default value:</b> `5`
 */
#ifndef MQTT_MAX_CONNACK_RECEIVE_RETRY_COUNT
/* Default value for the CONNACK receive retries. */
    #define MQTT_MAX_CONNACK_RECEIVE_RETRY_COUNT    ( 5U )
#endif

/**
 * @brief Maximum number of milliseconds to wait for a ping response to a ping
 * request as part of the keep-alive mechanism.
 *
 * If a ping response is not received before this timeout, then
 * #MQTT_ProcessLoop will return #MQTTKeepAliveTimeout.
 *
 * @note If this value is more than half of the keep alive interval, and the
 * server does not receive the previous ping request, then it is likely that the
 * server will disconnect the client before #MQTTKeepAliveTimeout can be returned.
 *
 * @note If a dummy implementation of the #MQTTGetCurrentTimeFunc_t timer function,
 * is supplied to the library, then the keep-alive mechanism is not supported by the
 * #MQTT_ProcessLoop API function. In that case, the value of #MQTT_PINGRESP_TIMEOUT_MS
 * is irrelevant to the behavior of the library.
 *
 * <b>Possible values:</b> Any positive integer up to SIZE_MAX. <br>
 * <b>Default value:</b> `5000`
 */
#ifndef MQTT_PINGRESP_TIMEOUT_MS
/* Wait 5 seconds by default for a ping response. */
    #define MQTT_PINGRESP_TIMEOUT_MS    ( 5000U )
#endif

/**
 * @brief Maximum number of milliseconds of TX inactivity to wait
 * before initiating a PINGREQ
 *
 * @note If this value is less than the keep alive interval than
 * it will be used instead.
 *
 * <b>Possible values:</b> Any positive integer up to SIZE_MAX. <br>
 * <b>Default value:</b> '30000'
 */
#ifndef PACKET_TX_TIMEOUT_MS
    #define PACKET_TX_TIMEOUT_MS    ( 30000U )
#endif

/**
 * @brief Maximum number of milliseconds of RX inactivity to wait
 * before initiating a PINGREQ
 *
 * <b>Possible values:</b> Any positive integer up to SIZE_MAX. <br>
 * <b>Default value:</b> '30000'
 *
 */
#ifndef PACKET_RX_TIMEOUT_MS
    #define PACKET_RX_TIMEOUT_MS    ( 30000U )
#endif

/**
 * @brief The maximum duration between non-empty network reads while
 * receiving an MQTT packet via the #MQTT_ProcessLoop or #MQTT_ReceiveLoop
 * API functions.
 *
 * When an incoming MQTT packet is detected, the transport receive function
 * may be called multiple times until all of the expected number of bytes of the
 * packet are received. This timeout represents the maximum polling duration that
 * is allowed without any data reception from the network for the incoming packet.
 *
 * If the timeout expires, the #MQTT_ProcessLoop and #MQTT_ReceiveLoop functions
 * return #MQTTRecvFailed.
 *
 * @note If a dummy implementation of the #MQTTGetCurrentTimeFunc_t timer function,
 * is supplied to the library, then #MQTT_RECV_POLLING_TIMEOUT_MS MUST be set to 0.
 *
 * <b>Possible values:</b> Any positive 32 bit integer. Recommended to use a
 * small timeout value. <br>
 * <b>Default value:</b> `10`
 *
 */
#ifndef MQTT_RECV_POLLING_TIMEOUT_MS
    #define MQTT_RECV_POLLING_TIMEOUT_MS    ( 10U )
#endif

/**
 * @brief The maximum duration allowed to send an MQTT packet over the transport
 * interface.
 *
 * When sending an MQTT packet, the transport send or writev functions may be
 * called multiple times until all of the required number of bytes are sent.
 * This timeout represents the maximum duration that is allowed to send the MQTT
 * packet while calling the transport send or writev functions.
 *
 * If the timeout expires, #MQTTSendFailed will be returned by the public API
 * functions.
 *
 * @note If a dummy implementation of the #MQTTGetCurrentTimeFunc_t timer function,
 * is supplied to the library, then #MQTT_SEND_TIMEOUT_MS MUST be set to 0.
 *
 * <b>Possible values:</b> Any positive 32 bit integer. <br>
 * <b>Default value:</b> `20000`
 *
 */
#ifndef MQTT_SEND_TIMEOUT_MS
    #define MQTT_SEND_TIMEOUT_MS    ( 20000U )
#endif

#ifdef MQTT_SEND_RETRY_TIMEOUT_MS
    #error MQTT_SEND_RETRY_TIMEOUT_MS is deprecated. Instead use MQTT_SEND_TIMEOUT_MS.
#endif

/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */

#endif /* ifndef CORE_MQTT_CONFIG_H */
