#include <gtest/gtest.h>
#include <primeon/math/transform/transform.hpp>

using namespace primeon::math;

TEST(Transform, DefaultConstruction) {
    Transform t;
    EXPECT_EQ(t.position.x, 0.0f);
    EXPECT_EQ(t.position.y, 0.0f);
    EXPECT_EQ(t.position.z, 0.0f);
    EXPECT_EQ(t.rotation, Quaternion::identity());
    EXPECT_EQ(t.scale.x, 1.0f);
    EXPECT_EQ(t.scale.y, 1.0f);
    EXPECT_EQ(t.scale.z, 1.0f);
}

TEST(Transform, ToMatrixAndBack) {
    Transform t;
    t.position = Vector3(1.0f, 2.0f, 3.0f);
    t.rotation = Quaternion::fromAxisAngle(kVector3UnitY, 0.5f);
    t.scale = Vector3(2.0f, 2.0f, 2.0f);

    Matrix4 m = t.toMatrix();
    Vector3 p(1.0f, 0.0f, 0.0f);
    Vector3 result = m.transformPoint(p);
    Vector3 expected = t.rotation.rotate(Vector3(2.0f, 0.0f, 0.0f)) + t.position;
    EXPECT_NEAR(result.x, expected.x, kEpsilon);
    EXPECT_NEAR(result.y, expected.y, kEpsilon);
    EXPECT_NEAR(result.z, expected.z, kEpsilon);
}

TEST(Transform, TransformPoint) {
    Transform t;
    t.position = Vector3(10.0f, 0.0f, 0.0f);
    Vector3 p(1.0f, 0.0f, 0.0f);
    Vector3 result = t.transformPoint(p);
    EXPECT_NEAR(result.x, 11.0f, kEpsilon);
    EXPECT_NEAR(result.y, 0.0f, kEpsilon);
    EXPECT_NEAR(result.z, 0.0f, kEpsilon);
}

TEST(Transform, TransformDirection) {
    Transform t;
    t.position = Vector3(100.0f, 100.0f, 100.0f);
    t.rotation = Quaternion::fromAxisAngle(kVector3UnitZ, kHalfPi);
    Vector3 d(1.0f, 0.0f, 0.0f);
    Vector3 result = t.transformDirection(d);
    EXPECT_NEAR(result.x, 0.0f, kEpsilon);
    EXPECT_NEAR(result.y, 1.0f, kEpsilon);
    EXPECT_NEAR(result.z, 0.0f, kEpsilon);
}

TEST(Transform, InverseTransformPoint) {
    Transform t;
    t.position = Vector3(5.0f, 10.0f, 15.0f);
    Matrix4 m = t.toMatrix();
    Matrix4 inv = m.inverse();
    Vector3 p(6.0f, 11.0f, 16.0f);
    Vector3 result = inv.transformPoint(p);
    EXPECT_NEAR(result.x, 1.0f, kEpsilon);
    EXPECT_NEAR(result.y, 1.0f, kEpsilon);
    EXPECT_NEAR(result.z, 1.0f, kEpsilon);
}

TEST(Transform, OperatorMultiply) {
    Transform parent;
    parent.position = Vector3(10.0f, 0.0f, 0.0f);

    Transform child;
    child.position = Vector3(1.0f, 0.0f, 0.0f);

    Transform combined = parent * child;
    EXPECT_NEAR(combined.position.x, 11.0f, kEpsilon);
    EXPECT_NEAR(combined.position.y, 0.0f, kEpsilon);
    EXPECT_NEAR(combined.position.z, 0.0f, kEpsilon);
}

TEST(Transform, Inverse) {
    Transform t;
    t.position = Vector3(1.0f, 2.0f, 3.0f);
    t.rotation = Quaternion::fromAxisAngle(kVector3UnitY, 0.5f);
    t.scale = Vector3(2.0f, 2.0f, 2.0f);

    Matrix4 m = t.toMatrix();
    Transform inv = t.inverse();
    Matrix4 invMatrix = inv.toMatrix();
    Matrix4 product = m * invMatrix;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            EXPECT_NEAR(product(i, j), (i == j) ? 1.0f : 0.0f, kEpsilon);
}
