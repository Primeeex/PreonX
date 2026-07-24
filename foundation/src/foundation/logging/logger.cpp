#include "foundation/logging/logger.hpp"
#include "foundation/logging/console_sink.hpp"

#include <utility>

namespace foundation {

Logger::Logger() : level_(LogLevel::Info) {
    sinks_.push_back(std::make_unique<ConsoleSink>());
}

Logger::~Logger() = default;

void Logger::add_sink(std::unique_ptr<LogSink> sink) {
    LockGuard lock(mutex_);
    sinks_.push_back(std::move(sink));
}

void Logger::set_level(LogLevel level) {
    level_ = level;
}

LogLevel Logger::level() const noexcept {
    return level_;
}

void Logger::log(LogLevel level, std::string_view message) {
    if (level < level_) return;

    LockGuard lock(mutex_);
    for (auto& sink : sinks_) {
        sink->write(level, message);
    }
}

void Logger::flush() {
    LockGuard lock(mutex_);
    for (auto& sink : sinks_) {
        sink->flush();
    }
}

void Logger::trace(std::string_view message) {
    log(LogLevel::Trace, message);
}

void Logger::debug(std::string_view message) {
    log(LogLevel::Debug, message);
}

void Logger::info(std::string_view message) {
    log(LogLevel::Info, message);
}

void Logger::warning(std::string_view message) {
    log(LogLevel::Warning, message);
}

void Logger::error(std::string_view message) {
    log(LogLevel::Error, message);
}

void Logger::fatal(std::string_view message) {
    log(LogLevel::Fatal, message);
}

Logger& Logger::global() {
    static Logger instance;
    return instance;
}

} // namespace foundation
