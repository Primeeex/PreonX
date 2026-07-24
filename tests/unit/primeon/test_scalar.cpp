#include <gtest/gtest.h>
#include <primeon/math/scalar/scalar.hpp>

using namespace primeon::math;

TEST(ScalarMath, Constants) {
    EXPECT_NEAR(kPi, 3.14159265f, 1e-5f);
    EXPECT_NEAR(kTwoPi, 6.28318530f, 1e-5f);
    EXPECT_NEAR(kHalfPi, 1.57079632f, 1e-5f);
    EXPECT_NEAR(kDegToRad, kPi / 180.0f, 1e-7f);
    EXPECT_NEAR(kRadToDeg, 180.0f / kPi, 1e-5f);
}

TEST(ScalarMath, Abs) {
    EXPECT_EQ(primeon::math::abs(-5.0f), 5.0f);
    EXPECT_EQ(primeon::math::abs(5.0f), 5.0f);
    EXPECT_EQ(primeon::math::abs(0.0f), 0.0f);
}

TEST(ScalarMath, Square) {
    EXPECT_EQ(square(3.0f), 9.0f);
    EXPECT_EQ(square(-3.0f), 9.0f);
    EXPECT_EQ(square(0.0f), 0.0f);
}

TEST(ScalarMath, Cube) {
    EXPECT_EQ(cube(3.0f), 27.0f);
    EXPECT_EQ(cube(-3.0f), -27.0f);
    EXPECT_EQ(cube(0.0f), 0.0f);
}

TEST(ScalarMath, Sign) {
    EXPECT_EQ(sign(-5.0f), -1.0f);
    EXPECT_EQ(sign(5.0f), 1.0f);
    EXPECT_EQ(sign(0.0f), 0.0f);
}

TEST(ScalarMath, Clamp) {
    EXPECT_EQ(clamp(5.0f, 0.0f, 10.0f), 5.0f);
    EXPECT_EQ(clamp(-5.0f, 0.0f, 10.0f), 0.0f);
    EXPECT_EQ(clamp(15.0f, 0.0f, 10.0f), 10.0f);
}

TEST(ScalarMath, Saturate) {
    EXPECT_EQ(saturate(0.5f), 0.5f);
    EXPECT_EQ(saturate(-1.0f), 0.0f);
    EXPECT_EQ(saturate(2.0f), 1.0f);
}

TEST(ScalarMath, Lerp) {
    EXPECT_NEAR(lerp(0.0f, 10.0f, 0.0f), 0.0f, kEpsilon);
    EXPECT_NEAR(lerp(0.0f, 10.0f, 1.0f), 10.0f, kEpsilon);
    EXPECT_NEAR(lerp(0.0f, 10.0f, 0.5f), 5.0f, kEpsilon);
}

TEST(ScalarMath, Smoothstep) {
    EXPECT_NEAR(smoothstep(0.0f, 1.0f, 0.0f), 0.0f, kEpsilon);
    EXPECT_NEAR(smoothstep(0.0f, 1.0f, 1.0f), 1.0f, kEpsilon);
    EXPECT_NEAR(smoothstep(0.0f, 1.0f, 0.5f), 0.5f, kEpsilon);
}

TEST(ScalarMath, NearEqual) {
    EXPECT_TRUE(nearEqual(1.0f, 1.0f));
    EXPECT_TRUE(nearEqual(1.0f, 1.0f + kEpsilon * 0.5f));
    EXPECT_FALSE(nearEqual(1.0f, 2.0f));
}

TEST(ScalarMath, NearZero) {
    EXPECT_TRUE(nearZero(0.0f));
    EXPECT_TRUE(nearZero(kEpsilon * 0.5f));
    EXPECT_FALSE(nearZero(1.0f));
}

TEST(ScalarMath, IsPowerOfTwo) {
    EXPECT_TRUE(isPowerOfTwo(1u));
    EXPECT_TRUE(isPowerOfTwo(2u));
    EXPECT_TRUE(isPowerOfTwo(4u));
    EXPECT_TRUE(isPowerOfTwo(1024u));
    EXPECT_FALSE(isPowerOfTwo(0u));
    EXPECT_FALSE(isPowerOfTwo(3u));
    EXPECT_FALSE(isPowerOfTwo(5u));
}

TEST(ScalarMath, NextPowerOfTwo) {
    EXPECT_EQ(nextPowerOfTwo(0u), 1u);
    EXPECT_EQ(nextPowerOfTwo(1u), 1u);
    EXPECT_EQ(nextPowerOfTwo(2u), 2u);
    EXPECT_EQ(nextPowerOfTwo(3u), 4u);
    EXPECT_EQ(nextPowerOfTwo(5u), 8u);
    EXPECT_EQ(nextPowerOfTwo(100u), 128u);
}

TEST(ScalarMath, Wrap) {
    EXPECT_NEAR(wrap(5.0f, 0.0f, 10.0f), 5.0f, kEpsilon);
    EXPECT_NEAR(wrap(15.0f, 0.0f, 10.0f), 5.0f, kEpsilon);
    EXPECT_NEAR(wrap(-5.0f, 0.0f, 10.0f), 5.0f, kEpsilon);
}
