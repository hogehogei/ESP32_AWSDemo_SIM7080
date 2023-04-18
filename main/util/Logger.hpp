#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <format>

class Logger
{
public:

    enum LogLevel
    {
        Error = 0,
        Warning,
        Info,
        Debug,
        Verbose,
        Stop,
    };

    Logger()
        : m_CurrentLogLevel(Debug)
    {}

    template <typename Args...>
    Logger& Error( std::format_string<Args...> fmt, const Args&... args ){
        if( m_CurrentLogLevel <= LogLevel::Error ){
            std::string s = Build( fmt, args );
            ESP_LOGE( m_Tag, s.c_str() );
        }
    }

    template <typename Args...>
    Logger& Warn( std::format_string<Args...> fmt, const Args&... args ){
        if( m_CurrentLogLevel <= LogLevel::Warn ){
            std::string s = Build( fmt, args );
            ESP_LOGW( m_Tag, s.c_str() );
        }
    }

    template <typename Args...>
    Logger& Info( std::format_string<Args...> fmt, const Args&... args ){
        if( m_CurrentLogLevel <= LogLevel::Info ){
            std::string s = Build( fmt, args );
            ESP_LOGI( m_Tag, s.c_str() );
        }
    }

    template <typename Args...>
    Logger& Debug( std::format_string<Args...> fmt, const Args&... args ){
        if( m_CurrentLogLevel <= LogLevel::Debug ){
            std::string s = Build( fmt, args );
            ESP_LOGI( m_Tag, s.c_str() );
        }
    }

    template <typename Args...>
    Logger& Verbose( std::format_string<Args...> fmt, const Args&... args ){
        if( m_CurrentLogLevel <= LogLevel::Verbose ){
            std::string s = Build( fmt, args );
            ESP_LOGV( m_Tag, s.c_str() );
        }
    }

    Logger& SetLogLevel( LogLevel level )
    {
        m_CurrentLogLevel = level;
        return *this;
    }

private:

    template <typename Args...>
    std::string Build( std::format_string<Args...> fmt, const Args&... args ){
        std::string buf;
        std::format_to( std::back_inserter(buf), fmt, args );
        return buf;
    }

    LogLevel m_CurrentLogLevel;
};

#endif      // LOGGER_HPP