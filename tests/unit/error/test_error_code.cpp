#include <gtest/gtest.h>

#include "foundation/error/error_code.hpp"

namespace {

TEST(ErrorCodeTest, NoneIsSuccess) {
    EXPECT_TRUE(foundation::is_success(foundation::ErrorCode::None));
    EXPECT_FALSE(foundation::is_error(foundation::ErrorCode::None));
}

TEST(ErrorCodeTest, ErrorsAreDetected) {
    EXPECT_TRUE(foundation::is_error(foundation::ErrorCode::Unknown));
    EXPECT_TRUE(foundation::is_error(foundation::ErrorCode::FileNotFound));
    EXPECT_TRUE(foundation::is_error(foundation::ErrorCode::OutOfMemory));
    EXPECT_FALSE(foundation::is_success(foundation::ErrorCode::FileNotFound));
}

TEST(ErrorCodeTest, CodeNamesAreSet) {
    EXPECT_STREQ(foundation::error_code_name(foundation::ErrorCode::None), "None");
    EXPECT_STREQ(foundation::error_code_name(foundation::ErrorCode::FileNotFound), "FileNotFound");
    EXPECT_STREQ(foundation::error_code_name(foundation::ErrorCode::AllocationFailed), "AllocationFailed");
}

TEST(ErrorCodeTest, CodeMessagesAreSet) {
    EXPECT_NE(foundation::error_code_message(foundation::ErrorCode::None), nullptr);
    EXPECT_NE(foundation::error_code_message(foundation::ErrorCode::FileNotFound), nullptr);
    EXPECT_NE(foundation::error_code_message(foundation::ErrorCode::Unknown), nullptr);
}

TEST(ErrorCodeTest, AllCodesHaveNamesAndMessages) {
    // Test a range of error codes to ensure none are missing
    foundation::ErrorCode codes[] = {
        foundation::ErrorCode::None,
        foundation::ErrorCode::Unknown,
        foundation::ErrorCode::InvalidParameter,
        foundation::ErrorCode::OutOfMemory,
        foundation::ErrorCode::NotSupported,
        foundation::ErrorCode::NotInitialized,
        foundation::ErrorCode::AlreadyInitialized,
        foundation::ErrorCode::Internal,
        foundation::ErrorCode::FileNotFound,
        foundation::ErrorCode::FileAccessDenied,
        foundation::ErrorCode::FileAlreadyExists,
        foundation::ErrorCode::FileTooLarge,
        foundation::ErrorCode::ReadFailed,
        foundation::ErrorCode::WriteFailed,
        foundation::ErrorCode::PathInvalid,
        foundation::ErrorCode::AllocationFailed,
        foundation::ErrorCode::DeallocationFailed,
        foundation::ErrorCode::BufferOverflow,
        foundation::ErrorCode::BufferUnderflow,
        foundation::ErrorCode::ThreadCreateFailed,
        foundation::ErrorCode::LockFailed,
        foundation::ErrorCode::TimedOut,
    };

    for (auto code : codes) {
        EXPECT_NE(foundation::error_code_name(code), nullptr);
        EXPECT_NE(foundation::error_code_message(code), nullptr);
    }
}

} // namespace
