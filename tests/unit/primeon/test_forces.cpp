#include <gtest/gtest.h>
#include <primeon/dynamics/forces/forces.hpp>

using namespace primeon::math;

TEST(Force, DefaultConstruction) {
    Force f;
    EXPECT_TRUE(f.vector.isZero());
    EXPECT_TRUE(f.point.isZero());
}

TEST(Force, VectorConstruction) {
    Force f(Vector3(1.0f, 2.0f, 3.0f));
    EXPECT_NEAR(f.vector.x, 1.0f, kEpsilon);
    EXPECT_NEAR(f.vector.y, 2.0f, kEpsilon);
    EXPECT_NEAR(f.vector.z, 3.0f, kEpsilon);
}

TEST(Force, Accumulation) {
    ForceAccumulator acc;
    acc.addForce(Vector3(1.0f, 0.0f, 0.0f));
    acc.addForce(Vector3(0.0f, 2.0f, 0.0f));
    acc.addForce(Vector3(0.0f, 0.0f, 3.0f));
    EXPECT_NEAR(acc.force().x, 1.0f, kEpsilon);
    EXPECT_NEAR(acc.force().y, 2.0f, kEpsilon);
    EXPECT_NEAR(acc.force().z, 3.0f, kEpsilon);
}

TEST(Force, TorqueFromOffset) {
    ForceAccumulator acc;
    // Force of (0, -10, 0) at offset (1, 0, 0) => torque = (1,0,0) x (0,-10,0) = (0,0,-10)
    acc.addForce(Force(Vector3(0.0f, -10.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f)));
    EXPECT_NEAR(acc.force().y, -10.0f, kEpsilon);
    EXPECT_NEAR(acc.torque().z, -10.0f, kEpsilon);
}

TEST(Force, Clear) {
    ForceAccumulator acc;
    acc.addForce(Vector3(5.0f, 5.0f, 5.0f));
    EXPECT_FALSE(acc.isZero());
    acc.clear();
    EXPECT_TRUE(acc.isZero());
}

TEST(Force, AddOperator) {
    Force a(Vector3(1.0f, 0.0f, 0.0f));
    Force b(Vector3(0.0f, 1.0f, 0.0f));
    Force c = a + b;
    EXPECT_NEAR(c.vector.x, 1.0f, kEpsilon);
    EXPECT_NEAR(c.vector.y, 1.0f, kEpsilon);
}

TEST(Impulse, DefaultConstruction) {
    Impulse imp;
    EXPECT_TRUE(imp.linear.isZero());
    EXPECT_TRUE(imp.angular.isZero());
}

TEST(Impulse, Accumulation) {
    Impulse a(Vector3(1.0f, 0.0f, 0.0f), Vector3(0.0f, 1.0f, 0.0f));
    Impulse b(Vector3(0.0f, 0.0f, 1.0f), Vector3(1.0f, 0.0f, 0.0f));
    Impulse c = a + b;
    EXPECT_NEAR(c.linear.x, 1.0f, kEpsilon);
    EXPECT_NEAR(c.linear.z, 1.0f, kEpsilon);
    EXPECT_NEAR(c.angular.x, 1.0f, kEpsilon);
    EXPECT_NEAR(c.angular.y, 1.0f, kEpsilon);
}

TEST(Gravity, GravitationalForce) {
    f32 mass = 10.0f;
    Vector3 force = gravitationalForce(mass);
    EXPECT_NEAR(force.x, 0.0f, kEpsilon);
    EXPECT_NEAR(force.y, -9.80665f * 10.0f, kLargeEpsilon);
    EXPECT_NEAR(force.z, 0.0f, kEpsilon);
}

TEST(Gravity, GravitationalPotentialEnergy) {
    f32 pe = gravitationalPotentialEnergy(10.0f, 5.0f);
    // PE = m * g * h = 10 * 9.80665 * 5
    EXPECT_NEAR(pe, 10.0f * 9.80665f * 5.0f, kLargeEpsilon);
}

TEST(Gravity, ZeroHeight) {
    f32 pe = gravitationalPotentialEnergy(10.0f, 0.0f);
    EXPECT_NEAR(pe, 0.0f, kEpsilon);
}

TEST(Force, Equality) {
    Force a(Vector3(1.0f, 2.0f, 3.0f));
    Force b(Vector3(1.0f, 2.0f, 3.0f));
    Force c(Vector3(3.0f, 2.0f, 1.0f));
    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
}
