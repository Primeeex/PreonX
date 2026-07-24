#include <gtest/gtest.h>
#include <primeon/math/vector/vector4.hpp>

using namespace primeon::math;

TEST(Vector4, DefaultConstruction) {
    Vector4 v;
    EXPECT_EQ(v.x, 0.0f);
    EXPECT_EQ(v.y, 0.0f);
    EXPECT_EQ(v.z, 0.0f);
    EXPECT_EQ(v.w, 0.0f);
}

TEST(Vector4, ValueConstruction) {
    Vector4 v(1.0f, 2.0f, 3.0f, 4.0f);
    EXPECT_EQ(v.x, 1.0f);
    EXPECT_EQ(v.y, 2.0f);
    EXPECT_EQ(v.z, 3.0f);
    EXPECT_EQ(v.w, 4.0f);
}

TEST(Vector4, Addition) {
    Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
    Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);
    Vector4 r = a + b;
    EXPECT_EQ(r.x, 6.0f);
    EXPECT_EQ(r.y, 8.0f);
    EXPECT_EQ(r.z, 10.0f);
    EXPECT_EQ(r.w, 12.0f);
}

TEST(Vector4, Subtraction) {
    Vector4 a(5.0f, 6.0f, 7.0f, 8.0f);
    Vector4 b(1.0f, 2.0f, 3.0f, 4.0f);
    Vector4 r = a - b;
    EXPECT_EQ(r.x, 4.0f);
    EXPECT_EQ(r.y, 4.0f);
    EXPECT_EQ(r.z, 4.0f);
    EXPECT_EQ(r.w, 4.0f);
}

TEST(Vector4, ScalarMultiply) {
    Vector4 v(1.0f, 2.0f, 3.0f, 4.0f);
    Vector4 r = v * 2.0f;
    EXPECT_EQ(r.x, 2.0f);
    EXPECT_EQ(r.y, 4.0f);
    EXPECT_EQ(r.z, 6.0f);
    EXPECT_EQ(r.w, 8.0f);
}

TEST(Vector4, DotProduct) {
    Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
    Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);
    EXPECT_EQ(a.dot(b), 70.0f);
}

TEST(Vector4, Length) {
    Vector4 v(1.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_NEAR(v.length(), 1.0f, kEpsilon);
}

TEST(Vector4, Normalize) {
    Vector4 v(0.0f, 0.0f, 3.0f, 4.0f);
    Vector4 n = v.normalized();
    EXPECT_NEAR(n.length(), 1.0f, kEpsilon);
    EXPECT_NEAR(n.z, 0.6f, kEpsilon);
    EXPECT_NEAR(n.w, 0.8f, kEpsilon);
}

TEST(Vector4, Distance) {
    Vector4 a(0.0f, 0.0f, 0.0f, 0.0f);
    Vector4 b(1.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_NEAR(a.distanceTo(b), 1.0f, kEpsilon);
}

TEST(Vector4, Equality) {
    Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
    Vector4 b(1.0f, 2.0f, 3.0f, 4.0f);
    Vector4 c(1.0f, 2.0f, 3.0f, 5.0f);
    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
}
