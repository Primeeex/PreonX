#include <gtest/gtest.h>

#include "foundation/error/result.hpp"

#include <string>

namespace {

TEST(ResultTest, SuccessValue) {
    foundation::Result<int> result(42);
    EXPECT_TRUE(result.has_value());
    EXPECT_FALSE(result.has_error());
    EXPECT_EQ(result.value(), 42);
}

TEST(ResultTest, SuccessString) {
    foundation::Result<std::string> result(std::string("hello"));
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), "hello");
}

TEST(ResultTest, ErrorValue) {
    foundation::Result<int> result(foundation::ErrorCode::FileNotFound);
    EXPECT_FALSE(result.has_value());
    EXPECT_TRUE(result.has_error());
    EXPECT_EQ(result.error_code(), foundation::ErrorCode::FileNotFound);
}

TEST(ResultTest, ErrorObject) {
    foundation::Error err(foundation::ErrorCode::WriteFailed, "disk full");
    foundation::Result<int> result(err);
    EXPECT_TRUE(result.has_error());
    EXPECT_EQ(result.error_code(), foundation::ErrorCode::WriteFailed);
}

TEST(ResultTest, ValueOrDefault) {
    foundation::Result<int> success(42);
    foundation::Result<int> failure(foundation::ErrorCode::Unknown);

    EXPECT_EQ(success.value_or(0), 42);
    EXPECT_EQ(failure.value_or(0), 0);
}

TEST(ResultTest, BoolConversion) {
    foundation::Result<int> success(42);
    foundation::Result<int> failure(foundation::ErrorCode::Unknown);

    EXPECT_TRUE(static_cast<bool>(success));
    EXPECT_FALSE(static_cast<bool>(failure));
}

TEST(ResultVoidTest, Success) {
    foundation::Result<void> result;
    EXPECT_TRUE(result.has_value());
    EXPECT_FALSE(result.has_error());
}

TEST(ResultVoidTest, Error) {
    foundation::Result<void> result(foundation::ErrorCode::Unknown);
    EXPECT_FALSE(result.has_value());
    EXPECT_TRUE(result.has_error());
}

} // namespace
