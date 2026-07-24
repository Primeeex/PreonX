#include <gtest/gtest.h>
#include <primeon/math/matrix/matrix4.hpp>
#include <primeon/math/matrix/matrix3.hpp>

using namespace primeon::math;

TEST(Matrix4, Identity) {
    Matrix4 m = Matrix4::identity();
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            EXPECT_NEAR(m(i, j), (i == j) ? 1.0f : 0.0f, kEpsilon);
}

TEST(Matrix4, Determinant) {
    Matrix4 m = Matrix4::identity();
    EXPECT_NEAR(m.determinant(), 1.0f, kEpsilon);
}

TEST(Matrix4, Translation) {
    Matrix4 m = Matrix4::translation(1.0f, 2.0f, 3.0f);
    Vector4 p(0.0f, 0.0f, 0.0f, 1.0f);
    Vector4 r = m * p;
    EXPECT_NEAR(r.x, 1.0f, kEpsilon);
    EXPECT_NEAR(r.y, 2.0f, kEpsilon);
    EXPECT_NEAR(r.z, 3.0f, kEpsilon);
    EXPECT_NEAR(r.w, 1.0f, kEpsilon);
}

TEST(Matrix4, Scale) {
    Matrix4 m = Matrix4::scale(2.0f, 3.0f, 4.0f);
    Vector4 p(1.0f, 1.0f, 1.0f, 1.0f);
    Vector4 r = m * p;
    EXPECT_NEAR(r.x, 2.0f, kEpsilon);
    EXPECT_NEAR(r.y, 3.0f, kEpsilon);
    EXPECT_NEAR(r.z, 4.0f, kEpsilon);
}

TEST(Matrix4, TransformPoint) {
    Matrix4 m = Matrix4::translation(10.0f, 20.0f, 30.0f);
    Vector3 p(1.0f, 2.0f, 3.0f);
    Vector3 r = m.transformPoint(p);
    EXPECT_NEAR(r.x, 11.0f, kEpsilon);
    EXPECT_NEAR(r.y, 22.0f, kEpsilon);
    EXPECT_NEAR(r.z, 33.0f, kEpsilon);
}

TEST(Matrix4, TransformDirection) {
    Matrix4 m = Matrix4::translation(10.0f, 20.0f, 30.0f);
    Vector3 d(1.0f, 0.0f, 0.0f);
    Vector3 r = m.transformDirection(d);
    EXPECT_NEAR(r.x, 1.0f, kEpsilon);
    EXPECT_NEAR(r.y, 0.0f, kEpsilon);
    EXPECT_NEAR(r.z, 0.0f, kEpsilon);
}

TEST(Matrix4, Inverse) {
    Matrix4 m = Matrix4::translation(1.0f, 2.0f, 3.0f) * Matrix4::scale(2.0f, 3.0f, 4.0f);
    Matrix4 inv = m.inverse();
    Matrix4 result = m * inv;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            EXPECT_NEAR(result(i, j), (i == j) ? 1.0f : 0.0f, kEpsilon);
}

TEST(Matrix4, MultiplyIdentity) {
    Matrix4 a = Matrix4::translation(1.0f, 2.0f, 3.0f) * Matrix4::rotationXYZ(0.5f, 0.3f, 0.1f);
    Matrix4 id = Matrix4::identity();
    Matrix4 r = a * id;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            EXPECT_NEAR(r(i, j), a(i, j), kEpsilon);
}

TEST(Matrix4, Upper3x3) {
    Matrix4 m = Matrix4::scale(2.0f, 3.0f, 4.0f);
    Matrix3 upper = m.upper3x3();
    EXPECT_NEAR(upper(0, 0), 2.0f, kEpsilon);
    EXPECT_NEAR(upper(1, 1), 3.0f, kEpsilon);
    EXPECT_NEAR(upper(2, 2), 4.0f, kEpsilon);
}

TEST(Matrix4, TranslationVector) {
    Matrix4 m = Matrix4::translation(5.0f, 10.0f, 15.0f);
    Vector3 t = m.translation();
    EXPECT_NEAR(t.x, 5.0f, kEpsilon);
    EXPECT_NEAR(t.y, 10.0f, kEpsilon);
    EXPECT_NEAR(t.z, 15.0f, kEpsilon);
}

TEST(Matrix4, Transpose) {
    Matrix4 m = Matrix4::translation(1.0f, 2.0f, 3.0f) * Matrix4::rotationXYZ(0.5f, 0.3f, 0.1f);
    Matrix4 t = m.transposed();
    EXPECT_NEAR(m(0, 1), t(1, 0), kEpsilon);
    EXPECT_NEAR(m(0, 2), t(2, 0), kEpsilon);
    EXPECT_NEAR(m(0, 3), t(3, 0), kEpsilon);
    EXPECT_NEAR(m(1, 2), t(2, 1), kEpsilon);
    EXPECT_NEAR(m(1, 3), t(3, 1), kEpsilon);
    EXPECT_NEAR(m(2, 3), t(3, 2), kEpsilon);
}
