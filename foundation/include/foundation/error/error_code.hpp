#pragma once

#include "foundation/core/types.hpp"

#include <string_view>

namespace foundation {

enum class ErrorCode : u32 {
    None = 0,

    // Generic
    Unknown = 1,
    InvalidParameter = 2,
    OutOfMemory = 3,
    NotSupported = 4,
    NotInitialized = 5,
    AlreadyInitialized = 6,
    Internal = 7,

    // Filesystem
    FileNotFound = 100,
    FileAccessDenied = 101,
    FileAlreadyExists = 102,
    FileTooLarge = 103,
    ReadFailed = 104,
    WriteFailed = 105,
    PathInvalid = 106,

    // Memory
    AllocationFailed = 200,
    DeallocationFailed = 201,
    BufferOverflow = 202,
    BufferUnderflow = 203,

    // Threading
    ThreadCreateFailed = 300,
    LockFailed = 301,
    TimedOut = 302,

    // Custom error categories start here
    CustomBase = 10000,
};

[[nodiscard]] inline constexpr bool is_success(ErrorCode code) noexcept {
    return code == ErrorCode::None;
}

[[nodiscard]] inline constexpr bool is_error(ErrorCode code) noexcept {
    return code != ErrorCode::None;
}

[[nodiscard]] const char* error_code_name(ErrorCode code) noexcept;
[[nodiscard]] const char* error_code_message(ErrorCode code) noexcept;

} // namespace foundation
