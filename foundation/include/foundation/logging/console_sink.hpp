#pragma once

#include "foundation/logging/sink.hpp"

#include <cstdio>

namespace foundation {

class ConsoleSink : public LogSink {
public:
    ConsoleSink();
    explicit ConsoleSink(std::FILE* output);
    ~ConsoleSink() override = default;

    ConsoleSink(const ConsoleSink&) = delete;
    ConsoleSink& operator=(const ConsoleSink&) = delete;
    ConsoleSink(ConsoleSink&&) noexcept;
    ConsoleSink& operator=(ConsoleSink&&) noexcept;

    void write(LogLevel level, std::string_view message) override;
    void flush() override;

    void set_color_enabled(bool enabled);
    [[nodiscard]] bool color_enabled() const noexcept;

private:
    std::FILE* output_;
    bool color_enabled_;
};

} // namespace foundation
