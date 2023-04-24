#include "Mutex.hpp"

Mutex::Mutex()
    : m_Mutex( xSemaphoreCreateMutex() ), 
      m_IsLocked( pdFALSE )
{}

Mutex::~Mutex()
{
    if( m_Mutex != nullptr ){
        vSemaphoreDelete( m_Mutex );
    }
}

bool Mutex::TryLock( uint32_t block_time )
{
    if( m_Mutex == nullptr ){
        return false;
    }

    m_IsLocked = xSemaphoreTake( m_Mutex, block_time );
    
    return m_IsLocked == pdTRUE;
}

void Mutex::Unlock()
{
    // すでにロックが取れているはずなので、m_IsLocked へのアクセスは競合しない
    // もしロックが取れていない場合、m_IsLocked への書き換えは発生しないので、データ競合は発生しない。
    if(( m_Mutex != nullptr ) 
     &&( m_IsLocked == pdTRUE ))
    {
        m_IsLocked = pdFALSE;
        xSemaphoreGive( m_Mutex );
    }
}

bool Mutex::IsValid() const
{
    return m_Mutex != nullptr;
}


LockGuard::LockGuard( Mutex& mutex )
 : m_Mutex( mutex )
{
    mutex.TryLock();
}

LockGuard::~LockGuard()
{
    m_Mutex.Unlock();
}
