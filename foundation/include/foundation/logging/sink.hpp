#pragma once

#include "foundation/core/types.hpp"
#include "foundation/logging/log_level.hpp"

#include <format>
#include <string>

namespace foundation {

class LogSink {
public:
    virtual ~LogSink() = default;

    virtual void write(LogLevel level, std::string_view message) = 0;

    virtual void flush() = 0;
};

} // namespace foundation
