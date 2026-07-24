#include "foundation/logging/log_level.hpp"

namespace foundation {

const char* log_level_name(LogLevel level) noexcept {
    switch (level) {
    case LogLevel::Trace:
        return "TRACE";
    case LogLevel::Debug:
        return "DEBUG";
    case LogLevel::Info:
        return "INFO";
    case LogLevel::Warning:
        return "WARN";
    case LogLevel::Error:
        return "ERROR";
    case LogLevel::Fatal:
        return "FATAL";
    case LogLevel::Off:
        return "OFF";
    }
    return "UNKNOWN";
}

const char* log_level_color(LogLevel level) noexcept {
    switch (level) {
    case LogLevel::Trace:
        return "\033[90m";
    case LogLevel::Debug:
        return "\033[36m";
    case LogLevel::Info:
        return "\033[32m";
    case LogLevel::Warning:
        return "\033[33m";
    case LogLevel::Error:
        return "\033[31m";
    case LogLevel::Fatal:
        return "\033[35m";
    case LogLevel::Off:
        return "\033[0m";
    }
    return "\033[0m";
}

} // namespace foundation
