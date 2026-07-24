#include <gtest/gtest.h>
#include <primeon/math/matrix/matrix2.hpp>

using namespace primeon::math;

TEST(Matrix2, Identity) {
    Matrix2 m = Matrix2::identity();
    EXPECT_EQ(m(0, 0), 1.0f);
    EXPECT_EQ(m(1, 1), 1.0f);
    EXPECT_EQ(m(0, 1), 0.0f);
    EXPECT_EQ(m(1, 0), 0.0f);
}

TEST(Matrix2, Determinant) {
    Matrix2 m = Matrix2::identity();
    EXPECT_NEAR(m.determinant(), 1.0f, kEpsilon);

    Matrix2 s = Matrix2::scale(2.0f, 3.0f);
    EXPECT_NEAR(s.determinant(), 6.0f, kEpsilon);
}

TEST(Matrix2, Inverse) {
    Matrix2 m = Matrix2::scale(2.0f, 3.0f);
    Matrix2 inv = m.inverse();
    Matrix2 result = m * inv;
    EXPECT_NEAR(result(0, 0), 1.0f, kEpsilon);
    EXPECT_NEAR(result(1, 1), 1.0f, kEpsilon);
    EXPECT_NEAR(result(0, 1), 0.0f, kEpsilon);
    EXPECT_NEAR(result(1, 0), 0.0f, kEpsilon);
}

TEST(Matrix2, Transpose) {
    Matrix2 m = Matrix2::rotation(0.5f);
    Matrix2 t = m.transposed();
    EXPECT_NEAR(m(0, 1), t(1, 0), kEpsilon);
    EXPECT_NEAR(m(1, 0), t(0, 1), kEpsilon);
}

TEST(Matrix2, MultiplyIdentity) {
    Matrix2 a = Matrix2::rotation(0.5f);
    Matrix2 id = Matrix2::identity();
    Matrix2 r = a * id;
    EXPECT_NEAR(r(0, 0), a(0, 0), kEpsilon);
    EXPECT_NEAR(r(0, 1), a(0, 1), kEpsilon);
    EXPECT_NEAR(r(1, 0), a(1, 0), kEpsilon);
    EXPECT_NEAR(r(1, 1), a(1, 1), kEpsilon);
}

TEST(Matrix2, Scale) {
    Matrix2 m = Matrix2::scale(2.0f, 3.0f);
    Vector2 v(1.0f, 1.0f);
    Vector2 r = m * v;
    EXPECT_NEAR(r.x, 2.0f, kEpsilon);
    EXPECT_NEAR(r.y, 3.0f, kEpsilon);
}

TEST(Matrix2, Rotation) {
    Matrix2 m = Matrix2::rotation(kHalfPi);
    Vector2 v(1.0f, 0.0f);
    Vector2 r = m * v;
    EXPECT_NEAR(r.x, 0.0f, kEpsilon);
    EXPECT_NEAR(r.y, 1.0f, kEpsilon);
}
