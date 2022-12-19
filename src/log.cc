#include <atlas/log.h>

namespace HumanoidRobot
{
    void Log::init()
    {
        spdlog::set_pattern("%^[%T] [%n] %v%$");
        sCoreLogger = spdlog::stdout_color_mt("ROBOT");
        sClientLogger = spdlog::stdout_color_mt("APP");
    }
    void Log::info(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);
        sCoreLogger->info(fmt, args);
        va_end(args);
    }
    void Log::warn(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);
        sCoreLogger->warn(fmt, args);
        va_end(args);
    }
    void Log::error(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);
        sCoreLogger->error(fmt, args);
        va_end(args);
    }
    void Log::debug(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);
        sCoreLogger->debug(fmt, args);
        va_end(args);
    }
    void Log::trace(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);
        sCoreLogger->trace(fmt, args);
        va_end(args);
    }

    void Log::setLevel(const char *level)
    {
        if (strcmp(level, "trace") == 0)
        {
            sCoreLogger->set_level(spdlog::level::trace);
            sClientLogger->set_level(spdlog::level::trace);
        }
        else if (strcmp(level, "debug") == 0)
        {
            sCoreLogger->set_level(spdlog::level::debug);
            sClientLogger->set_level(spdlog::level::debug);
        }
        else if (strcmp(level, "info") == 0)
        {
            sCoreLogger->set_level(spdlog::level::info);
            sClientLogger->set_level(spdlog::level::info);
        }
        else if (strcmp(level, "warn") == 0)
        {
            sCoreLogger->set_level(spdlog::level::warn);
            sClientLogger->set_level(spdlog::level::warn);
        }
        else if (strcmp(level, "error") == 0)
        {
            sCoreLogger->set_level(spdlog::level::err);
            sClientLogger->set_level(spdlog::level::err);
        }
        else if (strcmp(level, "critical") == 0)
        {
            sCoreLogger->set_level(spdlog::level::critical);
            sClientLogger->set_level(spdlog::level::critical);
        }
        else if (strcmp(level, "off") == 0)
        {
            sCoreLogger->set_level(spdlog::level::off);
            sClientLogger->set_level(spdlog::level::off);
        }
        else
        {
            sCoreLogger->set_level(spdlog::level::trace);
            sClientLogger->set_level(spdlog::level::trace);
        }
    }
} // namespace Atlas
