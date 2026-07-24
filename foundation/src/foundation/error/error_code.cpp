#include "foundation/error/error_code.hpp"

namespace foundation {

const char* error_code_name(ErrorCode code) noexcept {
    switch (code) {
    case ErrorCode::None:
        return "None";
    case ErrorCode::Unknown:
        return "Unknown";
    case ErrorCode::InvalidParameter:
        return "InvalidParameter";
    case ErrorCode::OutOfMemory:
        return "OutOfMemory";
    case ErrorCode::NotSupported:
        return "NotSupported";
    case ErrorCode::NotInitialized:
        return "NotInitialized";
    case ErrorCode::AlreadyInitialized:
        return "AlreadyInitialized";
    case ErrorCode::Internal:
        return "Internal";
    case ErrorCode::FileNotFound:
        return "FileNotFound";
    case ErrorCode::FileAccessDenied:
        return "FileAccessDenied";
    case ErrorCode::FileAlreadyExists:
        return "FileAlreadyExists";
    case ErrorCode::FileTooLarge:
        return "FileTooLarge";
    case ErrorCode::ReadFailed:
        return "ReadFailed";
    case ErrorCode::WriteFailed:
        return "WriteFailed";
    case ErrorCode::PathInvalid:
        return "PathInvalid";
    case ErrorCode::AllocationFailed:
        return "AllocationFailed";
    case ErrorCode::DeallocationFailed:
        return "DeallocationFailed";
    case ErrorCode::BufferOverflow:
        return "BufferOverflow";
    case ErrorCode::BufferUnderflow:
        return "BufferUnderflow";
    case ErrorCode::ThreadCreateFailed:
        return "ThreadCreateFailed";
    case ErrorCode::LockFailed:
        return "LockFailed";
    case ErrorCode::TimedOut:
        return "TimedOut";
    default:
        return "CustomError";
    }
}

const char* error_code_message(ErrorCode code) noexcept {
    switch (code) {
    case ErrorCode::None:
        return "No error";
    case ErrorCode::Unknown:
        return "An unknown error occurred";
    case ErrorCode::InvalidParameter:
        return "An invalid parameter was provided";
    case ErrorCode::OutOfMemory:
        return "Out of memory";
    case ErrorCode::NotSupported:
        return "Operation not supported";
    case ErrorCode::NotInitialized:
        return "Not initialized";
    case ErrorCode::AlreadyInitialized:
        return "Already initialized";
    case ErrorCode::Internal:
        return "Internal error";
    case ErrorCode::FileNotFound:
        return "File not found";
    case ErrorCode::FileAccessDenied:
        return "File access denied";
    case ErrorCode::FileAlreadyExists:
        return "File already exists";
    case ErrorCode::FileTooLarge:
        return "File too large";
    case ErrorCode::ReadFailed:
        return "Read operation failed";
    case ErrorCode::WriteFailed:
        return "Write operation failed";
    case ErrorCode::PathInvalid:
        return "Invalid path";
    case ErrorCode::AllocationFailed:
        return "Memory allocation failed";
    case ErrorCode::DeallocationFailed:
        return "Memory deallocation failed";
    case ErrorCode::BufferOverflow:
        return "Buffer overflow";
    case ErrorCode::BufferUnderflow:
        return "Buffer underflow";
    case ErrorCode::ThreadCreateFailed:
        return "Thread creation failed";
    case ErrorCode::LockFailed:
        return "Lock acquisition failed";
    case ErrorCode::TimedOut:
        return "Operation timed out";
    default:
        return "Custom error";
    }
}

} // namespace foundation
