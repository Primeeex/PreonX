#include <gtest/gtest.h>
#include <primeon/math/vector/vector3.hpp>

using namespace primeon::math;

TEST(Vector3, DefaultConstruction) {
    Vector3 v;
    EXPECT_EQ(v.x, 0.0f);
    EXPECT_EQ(v.y, 0.0f);
    EXPECT_EQ(v.z, 0.0f);
}

TEST(Vector3, ValueConstruction) {
    Vector3 v(1.0f, 2.0f, 3.0f);
    EXPECT_EQ(v.x, 1.0f);
    EXPECT_EQ(v.y, 2.0f);
    EXPECT_EQ(v.z, 3.0f);
}

TEST(Vector3, Addition) {
    Vector3 a(1.0f, 2.0f, 3.0f);
    Vector3 b(4.0f, 5.0f, 6.0f);
    Vector3 r = a + b;
    EXPECT_EQ(r.x, 5.0f);
    EXPECT_EQ(r.y, 7.0f);
    EXPECT_EQ(r.z, 9.0f);
}

TEST(Vector3, Subtraction) {
    Vector3 a(5.0f, 6.0f, 7.0f);
    Vector3 b(2.0f, 3.0f, 4.0f);
    Vector3 r = a - b;
    EXPECT_EQ(r.x, 3.0f);
    EXPECT_EQ(r.y, 3.0f);
    EXPECT_EQ(r.z, 3.0f);
}

TEST(Vector3, ScalarMultiply) {
    Vector3 v(1.0f, 2.0f, 3.0f);
    Vector3 r = v * 3.0f;
    EXPECT_EQ(r.x, 3.0f);
    EXPECT_EQ(r.y, 6.0f);
    EXPECT_EQ(r.z, 9.0f);
}

TEST(Vector3, ScalarDivide) {
    Vector3 v(6.0f, 9.0f, 12.0f);
    Vector3 r = v / 3.0f;
    EXPECT_EQ(r.x, 2.0f);
    EXPECT_EQ(r.y, 3.0f);
    EXPECT_EQ(r.z, 4.0f);
}

TEST(Vector3, DotProduct) {
    Vector3 a(1.0f, 2.0f, 3.0f);
    Vector3 b(4.0f, 5.0f, 6.0f);
    EXPECT_EQ(a.dot(b), 32.0f);
}

TEST(Vector3, CrossProduct) {
    Vector3 a(1.0f, 0.0f, 0.0f);
    Vector3 b(0.0f, 1.0f, 0.0f);
    Vector3 c = a.cross(b);
    EXPECT_EQ(c.x, 0.0f);
    EXPECT_EQ(c.y, 0.0f);
    EXPECT_EQ(c.z, 1.0f);
}

TEST(Vector3, CrossProductAntiCommutative) {
    Vector3 a(1.0f, 2.0f, 3.0f);
    Vector3 b(4.0f, 5.0f, 6.0f);
    Vector3 ab = a.cross(b);
    Vector3 ba = b.cross(a);
    EXPECT_EQ(ab, -ba);
}

TEST(Vector3, Length) {
    Vector3 v(1.0f, 2.0f, 2.0f);
    EXPECT_NEAR(v.length(), 3.0f, kEpsilon);
}

TEST(Vector3, Normalize) {
    Vector3 v(0.0f, 3.0f, 4.0f);
    Vector3 n = v.normalized();
    EXPECT_NEAR(n.length(), 1.0f, kEpsilon);
    EXPECT_NEAR(n.x, 0.0f, kEpsilon);
    EXPECT_NEAR(n.y, 0.6f, kEpsilon);
    EXPECT_NEAR(n.z, 0.8f, kEpsilon);
}

TEST(Vector3, Distance) {
    Vector3 a(0.0f, 0.0f, 0.0f);
    Vector3 b(1.0f, 2.0f, 2.0f);
    EXPECT_NEAR(a.distanceTo(b), 3.0f, kEpsilon);
}

TEST(Vector3, Negate) {
    Vector3 v(1.0f, -2.0f, 3.0f);
    Vector3 r = -v;
    EXPECT_EQ(r.x, -1.0f);
    EXPECT_EQ(r.y, 2.0f);
    EXPECT_EQ(r.z, -3.0f);
}

TEST(Vector3, Abs) {
    Vector3 v(-1.0f, -2.0f, -3.0f);
    Vector3 r = v.abs();
    EXPECT_EQ(r.x, 1.0f);
    EXPECT_EQ(r.y, 2.0f);
    EXPECT_EQ(r.z, 3.0f);
}

TEST(Vector3, MinMax) {
    Vector3 a(1.0f, 5.0f, 3.0f);
    Vector3 b(2.0f, 4.0f, 6.0f);
    Vector3 mn = Vector3::min(a, b);
    Vector3 mx = Vector3::max(a, b);
    EXPECT_EQ(mn.x, 1.0f);
    EXPECT_EQ(mn.y, 4.0f);
    EXPECT_EQ(mn.z, 3.0f);
    EXPECT_EQ(mx.x, 2.0f);
    EXPECT_EQ(mx.y, 5.0f);
    EXPECT_EQ(mx.z, 6.0f);
}

TEST(Vector3, Lerp) {
    Vector3 a(0.0f, 0.0f, 0.0f);
    Vector3 b(2.0f, 4.0f, 6.0f);
    Vector3 r = Vector3::lerp(a, b, 0.5f);
    EXPECT_NEAR(r.x, 1.0f, kEpsilon);
    EXPECT_NEAR(r.y, 2.0f, kEpsilon);
    EXPECT_NEAR(r.z, 3.0f, kEpsilon);
}

TEST(Vector3, Constants) {
    EXPECT_EQ(kVector3Zero.x, 0.0f);
    EXPECT_EQ(kVector3One.x, 1.0f);
    EXPECT_EQ(kVector3UnitX.x, 1.0f);
    EXPECT_EQ(kVector3UnitY.y, 1.0f);
    EXPECT_EQ(kVector3UnitZ.z, 1.0f);
}

TEST(Vector3, Reflected) {
    Vector3 v(1.0f, -1.0f, 0.0f);
    Vector3 normal(0.0f, 1.0f, 0.0f);
    Vector3 r = v.reflected(normal);
    EXPECT_NEAR(r.x, 1.0f, kEpsilon);
    EXPECT_NEAR(r.y, 1.0f, kEpsilon);
    EXPECT_NEAR(r.z, 0.0f, kEpsilon);
}
