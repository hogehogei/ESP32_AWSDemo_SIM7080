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
    bool TryLock( uint32_t block_time = portMAX_DELAY );
    
    /// @brief  Unlock mutex
    void Unlock();
    
    /// @brief  Get mutex is valid?
    /// @return true if valid mutex, otherwise false.
    bool IsValid() const;

private:

    SemaphoreHandle_t  m_Mutex;
    BaseType_t         m_IsLocked;
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
        : mutex()
    {}
    MutexValue( const T& initial_value )
        : mutex(),
          value( initial_value )
    {}
    ~MutexValue()
    {}

    // DO NOT COPY
    MutexValue( Mutex& ) = delete;
    MutexValue& operator=( Mutex& ) = delete;

    MutexValue& operator=( const T& t )
    {
        mutex.TryLock();
        value = t;
        mutex.Unlock();

        return *this;
    }

    bool IsValid() const
    {
        return mutex.IsValid();
    }

    T Get() const
    {
        mutex.TryLock();
        T ret = value;
        mutex.Unlock();

        return ret;
    }

    mutable Mutex mutex;
    T value;
};  


#endif  // MUTEX_HPP