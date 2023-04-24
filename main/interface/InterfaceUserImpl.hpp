#ifndef INTERFACE_USER_IMPL_HPP
#define INTERFACE_USER_IMPL_HPP

#include "transport_interface.h"

class I_NetworkContext;
struct NetworkContext
{
    I_NetworkContext* Context;
};

#ifdef __cplusplus
extern "C" {
#endif

int32_t TransportReceive( NetworkContext_t* network_context,
                          void* buffer,
                          size_t recvbyte );

int32_t TransportSend( NetworkContext_t* network_context,
                       const void* buffer,
                       size_t sendbyte );

uint32_t GetCurrentTimeMs();

#ifdef __cplusplus
}
#endif

#endif          // INTERFACE_USER_IMPL_HPP