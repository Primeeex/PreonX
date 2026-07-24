#include <gtest/gtest.h>
#include <primeon/math/matrix/matrix3.hpp>

using namespace primeon::math;

TEST(Matrix3, Identity) {
    Matrix3 m = Matrix3::identity();
    EXPECT_EQ(m(0, 0), 1.0f);
    EXPECT_EQ(m(1, 1), 1.0f);
    EXPECT_EQ(m(2, 2), 1.0f);
    EXPECT_EQ(m(0, 1), 0.0f);
    EXPECT_EQ(m(0, 2), 0.0f);
    EXPECT_EQ(m(1, 0), 0.0f);
    EXPECT_EQ(m(1, 2), 0.0f);
    EXPECT_EQ(m(2, 0), 0.0f);
    EXPECT_EQ(m(2, 1), 0.0f);
}

TEST(Matrix3, Determinant) {
    Matrix3 m = Matrix3::identity();
    EXPECT_NEAR(m.determinant(), 1.0f, kEpsilon);

    Matrix3 s = Matrix3::scale(2.0f, 3.0f, 4.0f);
    EXPECT_NEAR(s.determinant(), 24.0f, kEpsilon);
}

TEST(Matrix3, Inverse) {
    Matrix3 m = Matrix3::scale(2.0f, 3.0f, 4.0f);
    Matrix3 inv = m.inverse();
    Matrix3 result = m * inv;
    EXPECT_NEAR(result(0, 0), 1.0f, kEpsilon);
    EXPECT_NEAR(result(1, 1), 1.0f, kEpsilon);
    EXPECT_NEAR(result(2, 2), 1.0f, kEpsilon);
    EXPECT_NEAR(result(0, 1), 0.0f, kEpsilon);
    EXPECT_NEAR(result(0, 2), 0.0f, kEpsilon);
    EXPECT_NEAR(result(1, 0), 0.0f, kEpsilon);
    EXPECT_NEAR(result(1, 2), 0.0f, kEpsilon);
    EXPECT_NEAR(result(2, 0), 0.0f, kEpsilon);
    EXPECT_NEAR(result(2, 1), 0.0f, kEpsilon);
}

TEST(Matrix3, Transpose) {
    Matrix3 m = Matrix3::rotationX(0.5f);
    Matrix3 t = m.transposed();
    EXPECT_NEAR(m(0, 1), t(1, 0), kEpsilon);
    EXPECT_NEAR(m(0, 2), t(2, 0), kEpsilon);
    EXPECT_NEAR(m(1, 2), t(2, 1), kEpsilon);
}

TEST(Matrix3, Scale) {
    Matrix3 m = Matrix3::scale(2.0f, 3.0f, 4.0f);
    Vector3 v(1.0f, 1.0f, 1.0f);
    Vector3 r = m * v;
    EXPECT_NEAR(r.x, 2.0f, kEpsilon);
    EXPECT_NEAR(r.y, 3.0f, kEpsilon);
    EXPECT_NEAR(r.z, 4.0f, kEpsilon);
}

TEST(Matrix3, MultiplyIdentity) {
    Matrix3 a = Matrix3::rotationX(0.5f);
    Matrix3 id = Matrix3::identity();
    Matrix3 r = a * id;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_NEAR(r(i, j), a(i, j), kEpsilon);
}
