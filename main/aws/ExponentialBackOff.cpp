#include "ExponentialBackOff.hpp"

#include "esp_random.h"

ExponentialBackOff::ExponentialBackOff( uint16_t backoff_base, uint16_t max_backoff, uint32_t max_attempts )
    : 
    m_Status( BackoffAlgorithmSuccess ),
    m_Param(),
    m_NextRetryTimeMs(0)
{
    BackoffAlgorithm_InitializeParams(
        &m_Param,
        backoff_base,
        max_backoff,
        max_attempts
    );
}

bool ExponentialBackOff::IsExhausted() const
{  
    return m_Status == BackoffAlgorithmRetriesExhausted;
}

void ExponentialBackOff::CalculateNextBackOff()
{
    m_Status = BackoffAlgorithm_GetNextBackoff( &m_Param, generateRandomNumber(), &m_NextRetryTimeMs );
}

uint16_t ExponentialBackOff::GetNextRetryTimeMs() const
{
    return m_Status == BackoffAlgorithmSuccess ? m_NextRetryTimeMs : 0;
}

uint32_t ExponentialBackOff::generateRandomNumber() const
{
    return esp_random();
}