#pragma once

#include "foundation/core/types.hpp"
#include "foundation/logging/log_level.hpp"
#include "foundation/logging/sink.hpp"
#include "foundation/threading/mutex.hpp"

#include <format>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace foundation {

class Logger {
public:
    Logger();
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;

    void add_sink(std::unique_ptr<LogSink> sink);
    void set_level(LogLevel level);
    [[nodiscard]] LogLevel level() const noexcept;

    void log(LogLevel level, std::string_view message);
    void flush();

    void trace(std::string_view message);
    void debug(std::string_view message);
    void info(std::string_view message);
    void warning(std::string_view message);
    void error(std::string_view message);
    void fatal(std::string_view message);

    template <typename... Args>
    void trace(std::format_string<Args...> fmt, Args&&... args) {
        if (level_ <= LogLevel::Trace) {
            log(LogLevel::Trace, std::format(fmt, std::forward<Args>(args)...));
        }
    }

    template <typename... Args>
    void debug(std::format_string<Args...> fmt, Args&&... args) {
        if (level_ <= LogLevel::Debug) {
            log(LogLevel::Debug, std::format(fmt, std::forward<Args>(args)...));
        }
    }

    template <typename... Args>
    void info(std::format_string<Args...> fmt, Args&&... args) {
        if (level_ <= LogLevel::Info) {
            log(LogLevel::Info, std::format(fmt, std::forward<Args>(args)...));
        }
    }

    template <typename... Args>
    void warning(std::format_string<Args...> fmt, Args&&... args) {
        if (level_ <= LogLevel::Warning) {
            log(LogLevel::Warning, std::format(fmt, std::forward<Args>(args)...));
        }
    }

    template <typename... Args>
    void error(std::format_string<Args...> fmt, Args&&... args) {
        if (level_ <= LogLevel::Error) {
            log(LogLevel::Error, std::format(fmt, std::forward<Args>(args)...));
        }
    }

    template <typename... Args>
    void fatal(std::format_string<Args...> fmt, Args&&... args) {
        if (level_ <= LogLevel::Fatal) {
            log(LogLevel::Fatal, std::format(fmt, std::forward<Args>(args)...));
        }
    }

    [[nodiscard]] static Logger& global();

private:
    LogLevel level_;
    std::vector<std::unique_ptr<LogSink>> sinks_;
    Mutex mutex_;
};

} // namespace foundation

// ── Convenience macros ────────────────────────────────────────────────────────
#define FOUNDATION_LOG_TRACE(...)    ::foundation::Logger::global().trace(__VA_ARGS__)
#define FOUNDATION_LOG_DEBUG(...)    ::foundation::Logger::global().debug(__VA_ARGS__)
#define FOUNDATION_LOG_INFO(...)     ::foundation::Logger::global().info(__VA_ARGS__)
#define FOUNDATION_LOG_WARNING(...)  ::foundation::Logger::global().warning(__VA_ARGS__)
#define FOUNDATION_LOG_ERROR(...)    ::foundation::Logger::global().error(__VA_ARGS__)
#define FOUNDATION_LOG_FATAL(...)    ::foundation::Logger::global().fatal(__VA_ARGS__)
