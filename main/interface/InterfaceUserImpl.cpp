
#include "interface/InterfaceUserImpl.hpp"
#include "aws/NetworkContext.hpp"

#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

int32_t TransportReceive( NetworkContext_t* network_context,
                          void* buffer,
                          size_t recvbyte )
{
    I_NetworkContext* net = network_context->Context;
    return net->Recv( reinterpret_cast<uint8_t*>(buffer), recvbyte );
}

int32_t TransportSend( NetworkContext_t* network_context,
                       const void* buffer,
                       size_t sendbyte )
{
    I_NetworkContext* net = network_context->Context;
    return net->Send( reinterpret_cast<const uint8_t*>(buffer), sendbyte );
}

uint32_t GetCurrentTimeMs()
{
    // 現実的に int64 がオーバーフローしてマイナスになることはないので良しとする
    uint64_t time_us = static_cast<uint64_t>(esp_timer_get_time());
    return static_cast<uint32_t>(time_us / 1000);
}

#ifdef __cplusplus
}
#endif
