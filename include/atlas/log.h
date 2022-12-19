/**
 * @file log.h
 * @author franklinselva (franklinselva10@gmail.com)
 * @brief Logging for the project. Depends SPDLOG
 * @version 0.1
 * @date 2022-12-12
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef ATLAS_LOG_H
#define ATLAS_LOG_H

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

namespace HumanoidRobot
{
  class Log
  {
  public:
    static void init();
    static void setLevel(const char *level);

    void info(const char *fmt, ...);
    void warn(const char *fmt, ...);
    void error(const char *fmt, ...);
    void debug(const char *fmt, ...);
    void trace(const char *fmt, ...);

    inline static std::shared_ptr<spdlog::logger> &getCoreLogger() { return sCoreLogger; }
    inline static std::shared_ptr<spdlog::logger> &getClientLogger() { return sClientLogger; }

  private:
    inline static std::shared_ptr<spdlog::logger> sCoreLogger;
    inline static std::shared_ptr<spdlog::logger> sClientLogger;
  };
} // namespace HumanoidRobot

#pragma once
// TODO: Compile time log level
#define LOG_INFO(...) ::HumanoidRobot::Log::getCoreLogger()->info(__VA_ARGS__)
#define LOG_WARN(...) ::HumanoidRobot::Log::getCoreLogger()->warn(__VA_ARGS__)
#define LOG_ERROR(...) ::HumanoidRobot::Log::getCoreLogger()->error(__VA_ARGS__)
#define LOG_DEBUG(...) ::HumanoidRobot::Log::getCoreLogger()->debug(__VA_ARGS__)
#define LOG_TRACE(...) ::HumanoidRobot::Log::getCoreLogger()->trace(__VA_ARGS__)

#endif // ATLAS_LOG_H