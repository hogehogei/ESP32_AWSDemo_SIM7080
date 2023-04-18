#include "Mutex.hpp"

Mutex::Mutex()
    : m_Mutex( xSemaphoreCreateMutex() ), 
      m_IsLocked( pdFALSE )
{}

Mutex::~Mutex()
{}

bool Mutex::TryLock( portTickType block_time )
{
    if( m_Mutex == NULL )
    {
        return false;
    }

    m_IsLocked = xSemaphoreTake( m_Mutex, block_time );
    
    return m_IsLocked == pdTRUE;
}

void Mutex::Unlock()
{
    if(( m_Mutex != NULL ) 
     &&( m_IsLocked == pdTRUE ))
    {
        m_IsLocked = pdFALSE;
        xSemaphoreGive( m_Mutex );
    }
}

bool Mutex::IsValid() const
{
    return m_Mutex != NULL;
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
