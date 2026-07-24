#pragma once

#include "foundation/error/error.hpp"
#include "foundation/error/error_code.hpp"

#include <optional>
#include <string>
#include <utility>

namespace foundation {

/// A lightweight Result type that holds either a value or an error.
/// Used instead of exceptions for normal runtime errors.
template <typename T>
class Result {
public:
    Result(T val) noexcept(std::is_nothrow_move_constructible_v<T>) // NOLINT(google-explicit-constructor)
        : value_(std::move(val)), has_value_(true) {}

    Result(foundation::Error e) noexcept // NOLINT(google-explicit-constructor)
        : error_(std::move(e)), has_value_(false) {}

    Result(ErrorCode code) noexcept // NOLINT(google-explicit-constructor)
        : error_(code), has_value_(false) {}

    [[nodiscard]] bool has_value() const noexcept {
        return has_value_;
    }

    [[nodiscard]] bool has_error() const noexcept {
        return !has_value_;
    }

    [[nodiscard]] explicit operator bool() const noexcept {
        return has_value_;
    }

    [[nodiscard]] T& value() & {
        return value_.value();
    }

    [[nodiscard]] const T& value() const & {
        return value_.value();
    }

    [[nodiscard]] T&& value() && {
        return std::move(value_.value());
    }

    [[nodiscard]] const foundation::Error& error() const noexcept {
        return error_;
    }

    [[nodiscard]] ErrorCode error_code() const noexcept {
        return error_.code();
    }

    T value_or(T default_value) const& {
        return has_value_ ? value_.value() : std::move(default_value);
    }

private:
    std::optional<T> value_;
    foundation::Error error_;
    bool has_value_;
};

/// Specialization for void.
template <>
class Result<void> {
public:
    Result() noexcept : has_value_(true) {}

    Result(foundation::Error e) noexcept // NOLINT(google-explicit-constructor)
        : error_(std::move(e)), has_value_(false) {}

    Result(ErrorCode code) noexcept // NOLINT(google-explicit-constructor)
        : error_(code), has_value_(false) {}

    [[nodiscard]] bool has_value() const noexcept {
        return has_value_;
    }

    [[nodiscard]] bool has_error() const noexcept {
        return !has_value_;
    }

    [[nodiscard]] explicit operator bool() const noexcept {
        return has_value_;
    }

    [[nodiscard]] const foundation::Error& error() const noexcept {
        return error_;
    }

    [[nodiscard]] ErrorCode error_code() const noexcept {
        return error_.code();
    }

private:
    foundation::Error error_;
    bool has_value_;
};

} // namespace foundation
