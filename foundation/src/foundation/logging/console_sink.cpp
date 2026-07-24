#include "foundation/logging/console_sink.hpp"
#include "foundation/logging/log_level.hpp"

#include <cstdio>

namespace foundation {

ConsoleSink::ConsoleSink() : output_(stderr), color_enabled_(true) {}

ConsoleSink::ConsoleSink(std::FILE* output) : output_(output), color_enabled_(true) {}

ConsoleSink::ConsoleSink(ConsoleSink&&) noexcept = default;
ConsoleSink& ConsoleSink::operator=(ConsoleSink&&) noexcept = default;

void ConsoleSink::write(LogLevel level, std::string_view message) {
    if (color_enabled_) {
        const char* color = log_level_color(level);
        const char* reset = "\033[0m";
        std::fprintf(output_, "%s[%s]%s %.*s\n", color, log_level_name(level), reset,
                     static_cast<int>(message.size()), message.data());
    } else {
        std::fprintf(output_, "[%s] %.*s\n", log_level_name(level), static_cast<int>(message.size()),
                     message.data());
    }
}

void ConsoleSink::flush() {
    std::fflush(output_);
}

void ConsoleSink::set_color_enabled(bool enabled) {
    color_enabled_ = enabled;
}

bool ConsoleSink::color_enabled() const noexcept {
    return color_enabled_;
}

} // namespace foundation
