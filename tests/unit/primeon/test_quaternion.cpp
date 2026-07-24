#include <gtest/gtest.h>
#include <primeon/math/quaternion/quaternion.hpp>

using namespace primeon::math;

TEST(Quaternion, Identity) {
    Quaternion q = Quaternion::identity();
    EXPECT_NEAR(q.x, 0.0f, kEpsilon);
    EXPECT_NEAR(q.y, 0.0f, kEpsilon);
    EXPECT_NEAR(q.z, 0.0f, kEpsilon);
    EXPECT_NEAR(q.w, 1.0f, kEpsilon);
}

TEST(Quaternion, FromAxisAngle) {
    Quaternion q = Quaternion::fromAxisAngle(kVector3UnitY, kHalfPi);
    Vector3 v(1.0f, 0.0f, 0.0f);
    Vector3 r = q.rotate(v);
    EXPECT_NEAR(r.x, 0.0f, kEpsilon);
    EXPECT_NEAR(r.y, 0.0f, kEpsilon);
    EXPECT_NEAR(r.z, -1.0f, kEpsilon);
}

TEST(Quaternion, FromEuler) {
    Quaternion q = Quaternion::fromEuler(0.0f, kHalfPi, 0.0f);
    EXPECT_NEAR(q.length(), 1.0f, kEpsilon);
}

TEST(Quaternion, Normalize) {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion n = q.normalized();
    EXPECT_NEAR(n.length(), 1.0f, kEpsilon);
}

TEST(Quaternion, Conjugate) {
    Quaternion q = Quaternion::fromAxisAngle(kVector3UnitZ, 0.5f);
    Quaternion c = q.conjugate();
    EXPECT_NEAR(c.x, -q.x, kEpsilon);
    EXPECT_NEAR(c.y, -q.y, kEpsilon);
    EXPECT_NEAR(c.z, -q.z, kEpsilon);
    EXPECT_NEAR(c.w, q.w, kEpsilon);
}

TEST(Quaternion, Inverse) {
    Quaternion q = Quaternion::fromAxisAngle(kVector3UnitY, 0.7f);
    Quaternion inv = q.inverse();
    Quaternion product = q * inv;
    EXPECT_NEAR(product.x, 0.0f, kEpsilon);
    EXPECT_NEAR(product.y, 0.0f, kEpsilon);
    EXPECT_NEAR(product.z, 0.0f, kEpsilon);
    EXPECT_NEAR(product.w, 1.0f, kEpsilon);
}

TEST(Quaternion, RotateXAxis) {
    Quaternion q = Quaternion::fromAxisAngle(kVector3UnitX, kHalfPi);
    Vector3 v(0.0f, 1.0f, 0.0f);
    Vector3 r = q.rotate(v);
    EXPECT_NEAR(r.x, 0.0f, kEpsilon);
    EXPECT_NEAR(r.y, 0.0f, kEpsilon);
    EXPECT_NEAR(r.z, 1.0f, kEpsilon);
}

TEST(Quaternion, RotateZAxis) {
    Quaternion q = Quaternion::fromAxisAngle(kVector3UnitZ, kHalfPi);
    Vector3 v(1.0f, 0.0f, 0.0f);
    Vector3 r = q.rotate(v);
    EXPECT_NEAR(r.x, 0.0f, kEpsilon);
    EXPECT_NEAR(r.y, 1.0f, kEpsilon);
    EXPECT_NEAR(r.z, 0.0f, kEpsilon);
}

TEST(Quaternion, ToMatrixAndBack) {
    Quaternion q = Quaternion::fromAxisAngle(Vector3(1.0f, 1.0f, 0.0f).normalized(), 0.8f);
    Matrix3 m = q.toMatrix3();
    Vector3 v(1.0f, 2.0f, 3.0f);
    Vector3 viaMat = m * v;
    Vector3 viaQuat = q.rotate(v);
    EXPECT_NEAR(viaMat.x, viaQuat.x, kEpsilon);
    EXPECT_NEAR(viaMat.y, viaQuat.y, kEpsilon);
    EXPECT_NEAR(viaMat.z, viaQuat.z, kEpsilon);
}

TEST(Quaternion, ForwardUpRight) {
    Quaternion q = Quaternion::identity();
    EXPECT_NEAR(q.forward().z, 1.0f, kEpsilon);
    EXPECT_NEAR(q.up().y, 1.0f, kEpsilon);
    EXPECT_NEAR(q.right().x, 1.0f, kEpsilon);
}

TEST(Quaternion, DotProduct) {
    Quaternion a = Quaternion::identity();
    Quaternion b = Quaternion::identity();
    EXPECT_NEAR(a.dot(b), 1.0f, kEpsilon);
}

TEST(Quaternion, Slerp) {
    Quaternion a = Quaternion::identity();
    Quaternion b = Quaternion::fromAxisAngle(kVector3UnitY, kPi);
    Quaternion r = slerp(a, b, 0.5f);
    EXPECT_NEAR(r.length(), 1.0f, kEpsilon);
}

TEST(Quaternion, SlerpTo) {
    Quaternion a = Quaternion::identity();
    Quaternion b = Quaternion::fromAxisAngle(kVector3UnitY, kPi);
    Quaternion r = slerpTo(a, b, 0.5f);
    EXPECT_NEAR(r.length(), 1.0f, kEpsilon);
}

TEST(Quaternion, ToEuler) {
    Quaternion q = Quaternion::fromEuler(0.3f, 0.5f, 0.0f);
    Vector3 euler = q.toEuler();
    Quaternion q2 = Quaternion::fromEuler(euler.x, euler.y, euler.z);
    Vector3 v(1.0f, 2.0f, 3.0f);
    Vector3 r1 = q.rotate(v);
    Vector3 r2 = q2.rotate(v);
    EXPECT_NEAR(r1.x, r2.x, kEpsilon);
    EXPECT_NEAR(r1.y, r2.y, kEpsilon);
    EXPECT_NEAR(r1.z, r2.z, kEpsilon);
}

TEST(Quaternion, LookRotation) {
    Quaternion q = Quaternion::lookRotation(kVector3UnitZ, kVector3UnitY);
    Vector3 fwd = q.forward();
    EXPECT_NEAR(fwd.z, 1.0f, kEpsilon);
}
