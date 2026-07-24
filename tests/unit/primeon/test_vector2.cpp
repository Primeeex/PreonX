#include <gtest/gtest.h>
#include <primeon/math/vector/vector2.hpp>

using namespace primeon::math;

TEST(Vector2, DefaultConstruction) {
    Vector2 v;
    EXPECT_EQ(v.x, 0.0f);
    EXPECT_EQ(v.y, 0.0f);
}

TEST(Vector2, ValueConstruction) {
    Vector2 v(1.0f, 2.0f);
    EXPECT_EQ(v.x, 1.0f);
    EXPECT_EQ(v.y, 2.0f);
}

TEST(Vector2, Addition) {
    Vector2 a(1.0f, 2.0f);
    Vector2 b(3.0f, 4.0f);
    Vector2 r = a + b;
    EXPECT_EQ(r.x, 4.0f);
    EXPECT_EQ(r.y, 6.0f);
}

TEST(Vector2, Subtraction) {
    Vector2 a(5.0f, 6.0f);
    Vector2 b(2.0f, 3.0f);
    Vector2 r = a - b;
    EXPECT_EQ(r.x, 3.0f);
    EXPECT_EQ(r.y, 3.0f);
}

TEST(Vector2, ScalarMultiply) {
    Vector2 v(1.0f, 2.0f);
    Vector2 r = v * 3.0f;
    EXPECT_EQ(r.x, 3.0f);
    EXPECT_EQ(r.y, 6.0f);
}

TEST(Vector2, DotProduct) {
    Vector2 a(1.0f, 2.0f);
    Vector2 b(3.0f, 4.0f);
    EXPECT_EQ(a.dot(b), 11.0f);
}

TEST(Vector2, CrossProduct) {
    Vector2 a(1.0f, 0.0f);
    Vector2 b(0.0f, 1.0f);
    EXPECT_EQ(a.cross(b), 1.0f);
    EXPECT_EQ(b.cross(a), -1.0f);
}

TEST(Vector2, Length) {
    Vector2 v(3.0f, 4.0f);
    EXPECT_NEAR(v.length(), 5.0f, kEpsilon);
}

TEST(Vector2, LengthSq) {
    Vector2 v(3.0f, 4.0f);
    EXPECT_NEAR(v.lengthSq(), 25.0f, kEpsilon);
}

TEST(Vector2, Normalize) {
    Vector2 v(3.0f, 4.0f);
    Vector2 n = v.normalized();
    EXPECT_NEAR(n.length(), 1.0f, kEpsilon);
    EXPECT_NEAR(n.x, 0.6f, kEpsilon);
    EXPECT_NEAR(n.y, 0.8f, kEpsilon);
}

TEST(Vector2, Distance) {
    Vector2 a(0.0f, 0.0f);
    Vector2 b(3.0f, 4.0f);
    EXPECT_NEAR(a.distanceTo(b), 5.0f, kEpsilon);
}

TEST(Vector2, Negate) {
    Vector2 v(1.0f, -2.0f);
    Vector2 r = -v;
    EXPECT_EQ(r.x, -1.0f);
    EXPECT_EQ(r.y, 2.0f);
}

TEST(Vector2, Equality) {
    Vector2 a(1.0f, 2.0f);
    Vector2 b(1.0f, 2.0f);
    Vector2 c(1.0f, 3.0f);
    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
}

TEST(Vector2, Perpendicular) {
    Vector2 v(1.0f, 0.0f);
    Vector2 p = v.perpendicular();
    EXPECT_NEAR(v.dot(p), 0.0f, kEpsilon);
}

TEST(Vector2, Constants) {
    EXPECT_EQ(kVector2Zero.x, 0.0f);
    EXPECT_EQ(kVector2Zero.y, 0.0f);
    EXPECT_EQ(kVector2One.x, 1.0f);
    EXPECT_EQ(kVector2One.y, 1.0f);
    EXPECT_EQ(kVector2UnitX.x, 1.0f);
    EXPECT_EQ(kVector2UnitY.y, 1.0f);
}

TEST(Vector2, Reflected) {
    Vector2 v(1.0f, -1.0f);
    Vector2 normal(0.0f, 1.0f);
    Vector2 r = v.reflected(normal);
    EXPECT_NEAR(r.x, 1.0f, kEpsilon);
    EXPECT_NEAR(r.y, 1.0f, kEpsilon);
}

TEST(Vector2, Angle) {
    Vector2 a(1.0f, 0.0f);
    Vector2 b(0.0f, 1.0f);
    EXPECT_NEAR(a.angleTo(b), kHalfPi, kEpsilon);
}

TEST(Vector2, Lerp) {
    Vector2 a(0.0f, 0.0f);
    Vector2 b(10.0f, 10.0f);
    Vector2 r = Vector2::lerp(a, b, 0.5f);
    EXPECT_NEAR(r.x, 5.0f, kEpsilon);
    EXPECT_NEAR(r.y, 5.0f, kEpsilon);
}
