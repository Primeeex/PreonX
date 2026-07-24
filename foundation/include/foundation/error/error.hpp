#pragma once

#include "foundation/error/error_code.hpp"

#include <string>
#include <utility>

namespace foundation {

class Error {
public:
    Error() noexcept : code_(ErrorCode::None) {}

    explicit Error(ErrorCode code) noexcept : code_(code) {}

    Error(ErrorCode code, std::string message) noexcept
        : code_(code), message_(std::move(message)) {}

    [[nodiscard]] bool has_error() const noexcept {
        return is_error(code_);
    }

    [[nodiscard]] bool has_value() const noexcept {
        return is_success(code_);
    }

    [[nodiscard]] explicit operator bool() const noexcept {
        return has_value();
    }

    [[nodiscard]] ErrorCode code() const noexcept {
        return code_;
    }

    [[nodiscard]] const std::string& message() const noexcept {
        return message_;
    }

    [[nodiscard]] std::string_view code_name() const noexcept {
        return error_code_name(code_);
    }

    [[nodiscard]] std::string_view code_message() const noexcept {
        return error_code_message(code_);
    }

    void clear() noexcept {
        code_ = ErrorCode::None;
        message_.clear();
    }

    [[nodiscard]] bool operator==(const Error& other) const noexcept {
        return code_ == other.code_;
    }

    [[nodiscard]] bool operator!=(const Error& other) const noexcept {
        return code_ != other.code_;
    }

private:
    ErrorCode code_;
    std::string message_;
};

[[nodiscard]] inline Error make_error(ErrorCode code) {
    return Error(code);
}

[[nodiscard]] inline Error make_error(ErrorCode code, std::string message) {
    return Error(code, std::move(message));
}

} // namespace foundation
