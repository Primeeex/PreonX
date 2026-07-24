#include <gtest/gtest.h>

#include "foundation/error/error.hpp"

namespace {

TEST(ErrorTest, DefaultConstructedHasNoError) {
    foundation::Error err;
    EXPECT_FALSE(err.has_error());
    EXPECT_TRUE(err.has_value());
    EXPECT_TRUE(static_cast<bool>(err));
    EXPECT_EQ(err.code(), foundation::ErrorCode::None);
}

TEST(ErrorTest, ConstructedWithCode) {
    foundation::Error err(foundation::ErrorCode::FileNotFound);
    EXPECT_TRUE(err.has_error());
    EXPECT_FALSE(err.has_value());
    EXPECT_FALSE(static_cast<bool>(err));
    EXPECT_EQ(err.code(), foundation::ErrorCode::FileNotFound);
}

TEST(ErrorTest, ConstructedWithCodeAndMessage) {
    foundation::Error err(foundation::ErrorCode::FileNotFound, "file not found at /path/to/file");
    EXPECT_TRUE(err.has_error());
    EXPECT_EQ(err.code(), foundation::ErrorCode::FileNotFound);
    EXPECT_EQ(err.message(), "file not found at /path/to/file");
}

TEST(ErrorTest, Clear) {
    foundation::Error err(foundation::ErrorCode::FileNotFound);
    EXPECT_TRUE(err.has_error());
    err.clear();
    EXPECT_FALSE(err.has_error());
    EXPECT_TRUE(err.has_value());
}

TEST(ErrorTest, MakeError) {
    auto err = foundation::make_error(foundation::ErrorCode::WriteFailed, "disk full");
    EXPECT_TRUE(err.has_error());
    EXPECT_EQ(err.code(), foundation::ErrorCode::WriteFailed);
    EXPECT_EQ(err.message(), "disk full");
}

TEST(ErrorTest, EqualityComparison) {
    foundation::Error a(foundation::ErrorCode::FileNotFound);
    foundation::Error b(foundation::ErrorCode::FileNotFound);
    foundation::Error c(foundation::ErrorCode::WriteFailed);

    EXPECT_TRUE(a == b);
    EXPECT_TRUE(a != c);
}

} // namespace
