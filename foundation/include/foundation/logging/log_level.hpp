#pragma once

#include "foundation/core/types.hpp"

#include <string_view>

namespace foundation {

enum class LogLevel : u8 {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warning = 3,
    Error = 4,
    Fatal = 5,
    Off = 6,
};

[[nodiscard]] const char* log_level_name(LogLevel level) noexcept;
[[nodiscard]] const char* log_level_color(LogLevel level) noexcept;

} // namespace foundation
