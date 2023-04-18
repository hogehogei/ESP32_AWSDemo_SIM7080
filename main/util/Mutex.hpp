#ifndef MUTEX_HPP
#define MUTEX_HPP

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/// @brief  FreeRTOS Mutex wrapper
class Mutex
{
public:

    Mutex();
    ~Mutex();

    // DO NOT COPY
    Mutex( Mutex& ) = delete;
    Mutex& operator=( Mutex& ) = delete;

    /// @brief  Try lock mutex
    /// @param  block_time  LockMutex blocking time.
    /// @return true if succeeded, otherwise false.
    bool TryLock( portTickType block_time = portMAX_DELAY );
    
    /// @brief  Unlock mutex
    void Unlock();
    
    /// @brief  Get mutex is valid?
    /// @return true if valid mutex, otherwise false.
    bool IsValid() const;

private:

    xSemaphoreHandle  m_Mutex;
    BaseType_t        m_IsLocked;
};

/// @brief  Mutex utility for "scoped locking pattern"
class LockGuard
{
public:

    /// @brief  automatically lock mutex.
    /// @param mutex 
    LockGuard( Mutex& mutex );
    /// @brief  automatically unlock mutex when destructing.
    ~LockGuard();

    // DO NOT COPY
    LockGuard( const LockGuard& ) = delete;
    LockGuard& operator=( const LockGuard& ) = delete;

private:

    Mutex& m_Mutex;
};

/// @brief  Value class protected from data race by mutex
//          simple implementation, so cannot protect that value is pointer, reference...
template <typename T>
class MutexValue
{
public:

    MutexValue()
        : m_Mutex()
    {}
    MutexValue( const T& initial_value )
        : m_Mutex(),
          m_Value( initial_value )
    {}
    ~MutexValue()
    {}

    // DO NOT COPY
    MutexValue( Mutex& ) = delete;
    MutexValue& operator=( Mutex& ) = delete;

    MutexValue& operator=( const T& value )
    {
        m_Mutex.TryLock();
        m_Value = value;
        m_Mutex.Unlock();

        return *this;
    }

    bool IsValid() const
    {
        return m_Mutex.IsValid();
    }

    T Get()
    {
        m_Mutex.TryLock();
        T value = m_Value;
        m_Mutex.Unlock();

        return value;
    }

private:

    Mutex m_Mutex;
    T     m_Value;
};  


#endif  // MUTEX_HPP