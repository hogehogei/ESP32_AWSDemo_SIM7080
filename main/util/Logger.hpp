#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <string>
#include "esp_log.h"

class Logger
{
public:

    enum LogLevel
    {
        L_Silent = 0,
        L_Error,
        L_Warning,
        L_Info,
        L_Debug,
        L_Verbose,
    };

    Logger( const char* tag, LogLevel level = LogLevel::L_Debug )
        : 
        m_Tag( tag ),
        m_CurrentLogLevel( level )
    {}

    template <typename ... Args>
    Logger& Error( const std::string& fmt, Args&& ... args ){
        if( m_CurrentLogLevel >= LogLevel::L_Error ){
            //esp_log_writev( ESP_LOG_ERROR, m_Tag.data(), fmt.c_str(), std::forward<Args>(args) ... );
            std::string s = Build( fmt, std::forward<Args>(args) ... );
            ESP_LOGE( m_Tag, "%s", s.c_str() );
        }

        return *this;
    }

    template <typename ... Args>
    Logger& Warn( const std::string& fmt, Args&& ... args ){
        if( m_CurrentLogLevel >= LogLevel::L_Warning ){
            //esp_log_writev( ESP_LOG_WARN, m_Tag.data(), fmt.c_str(), std::forward<Args>(args) ... );
            std::string s = Build( fmt, std::forward<Args>(args) ... );
            ESP_LOGW( m_Tag, "%s", s.c_str() );
        }

        return *this;
    }

    template <typename ... Args>
    Logger& Info( const std::string& fmt, Args&& ... args ){
        if( m_CurrentLogLevel >= LogLevel::L_Info ){
            //esp_log_writev( ESP_LOG_INFO, m_Tag.data(), fmt.c_str(), std::forward<Args>(args) ... );
            std::string s = Build( fmt, std::forward<Args>(args) ... );
            ESP_LOGI( m_Tag, "%s", s.c_str() );
        }

        return *this;
    }

    template <typename ... Args>
    Logger& Debug( const std::string& fmt, Args&& ... args ){
        if( m_CurrentLogLevel >= LogLevel::L_Debug ){
            //esp_log_writev( ESP_LOG_DEBUG, m_Tag.data(), fmt.c_str(), std::forward<Args>(args) ... );
            std::string s = Build( fmt, std::forward<Args>(args) ... );
            ESP_LOGD( m_Tag, "%s", s.c_str() );
        }

        return *this;
    }

    template <typename ... Args>
    Logger& Verbose( const std::string& fmt, Args&& ... args ){
        if( m_CurrentLogLevel >= LogLevel::L_Verbose ){
            std::string s = Build( fmt, std::forward<Args>(args) ... );
            ESP_LOGV( m_Tag, "%s", s.c_str() );
        }

        return *this;
    }

    Logger& SetLogLevel( LogLevel level )
    {
        m_CurrentLogLevel = level;
        return *this;
    }

private:

    template <typename ... Args>
    std::string Build( const std::string& fmt, Args&& ... args )
    {
        size_t len = std::snprintf( nullptr, 0, fmt.c_str(), std::forward<Args>(args) ... );
        std::vector<char> buf( len + 1 );
        std::snprintf( &buf[0], len + 1, fmt.c_str(), std::forward<Args>(args) ... );
        return std::string(&buf[0], &buf[0] + len);
    }

    const char* m_Tag;
    LogLevel    m_CurrentLogLevel;
};

#endif      // LOGGER_HPP