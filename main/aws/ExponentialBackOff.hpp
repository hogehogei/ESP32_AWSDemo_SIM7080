#ifndef EXPONENTIAL_BACKOFF_HPP
#define EXPONENTIAL_BACKOFF_HPP

#include <cstdint>
#include "backoff_algorithm.h"

/// @brief  AWS Exponential backoff algorithm wrapper
class ExponentialBackOff
{
public:
    ExponentialBackOff( uint16_t backoff_base,
                        uint16_t max_backoff, 
                        uint32_t max_attempts );

    bool IsExhausted() const;
    void CalculateNextBackOff();
    uint16_t GetNextRetryTimeMs() const;

private:

    uint32_t generateRandomNumber() const;

    BackoffAlgorithmStatus_t   m_Status;
    BackoffAlgorithmContext_t  m_Param;
    uint16_t m_NextRetryTimeMs;
};

#endif    // EXPONENTIAL_BACKOFF_HPP