#include <gtest/gtest.h>

#include "foundation/core/assert.hpp"

namespace {

TEST(ASSERT_DEATH, AssertionFailureAborts) {
    EXPECT_DEATH(PREONX_ASSERT(false), "ASSERTION FAILED");
}

TEST(ASSERT_DEATH, AssertionWithMessageAborts) {
    EXPECT_DEATH(PREONX_ASSERT_MSG(false, "custom message"), "custom message");
}

TEST(ASSERT_Passes, AssertionPassesOnTrue) {
    PREONX_ASSERT(true);
    PREONX_ASSERT_MSG(true, "should not fail");
}

TEST(ASSERT_DEATH, UnreachableAborts) {
    EXPECT_DEATH(PREONX_UNREACHABLE(), "Unreachable");
}

} // namespace
