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

namespace Atlas
{
  class Log
  {
  public:
    static void init();
    inline static std::shared_ptr<spdlog::logger> &getCoreLogger() { return sCoreLogger; }
    inline static std::shared_ptr<spdlog::logger> &getClientLogger() { return sClientLogger; }

  private:
    static std::shared_ptr<spdlog::logger> sCoreLogger;
    static std::shared_ptr<spdlog::logger> sClientLogger;
  };
} // namespace Atlas

#endif // ATLAS_LOG_H