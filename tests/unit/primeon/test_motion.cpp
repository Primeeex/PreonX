#include <gtest/gtest.h>
#include <primeon/dynamics/motion/motion.hpp>
#include <primeon/dynamics/mass/mass.hpp>

using namespace primeon::math;

TEST(ParticleState, DefaultConstruction) {
    ParticleState ps;
    EXPECT_TRUE(ps.position.isZero());
    EXPECT_TRUE(ps.velocity.isZero());
}

TEST(ParticleState, ValueConstruction) {
    ParticleState ps(Vector3(1.0f, 2.0f, 3.0f), Vector3(4.0f, 5.0f, 6.0f));
    EXPECT_NEAR(ps.position.x, 1.0f, kEpsilon);
    EXPECT_NEAR(ps.velocity.z, 6.0f, kEpsilon);
}

TEST(ParticleState, ApplyImpulse) {
    ParticleState ps(Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    MassProperties mass = MassProperties::fromMass(2.0f);
    ps.applyImpulse(Vector3(10.0f, 0.0f, 0.0f), mass.inverseMass);
    EXPECT_NEAR(ps.velocity.x, 5.0f, kEpsilon);
}

TEST(ParticleState, Equality) {
    ParticleState a(Vector3(1, 2, 3), Vector3(4, 5, 6));
    ParticleState b(Vector3(1, 2, 3), Vector3(4, 5, 6));
    ParticleState c(Vector3(0, 0, 0), Vector3(0, 0, 0));
    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
}

TEST(BodyState, DefaultConstruction) {
    BodyState bs;
    EXPECT_TRUE(bs.position.isZero());
    EXPECT_EQ(bs.rotation, Quaternion::identity());
    EXPECT_TRUE(bs.linearVelocity.isZero());
    EXPECT_TRUE(bs.angularVelocity.isZero());
}

TEST(BodyState, ApplyLinearImpulse) {
    BodyState bs;
    MassProperties mass = MassProperties::fromMass(2.0f);
    bs.applyLinearImpulse(Vector3(10.0f, 0.0f, 0.0f), mass.inverseMass);
    EXPECT_NEAR(bs.linearVelocity.x, 5.0f, kEpsilon);
}

TEST(BodyState, ApplyAngularImpulse) {
    BodyState bs;
    InertiaTensor inertia = InertiaTensor::solidSphere(1.0f, 1.0f);
    bs.applyAngularImpulse(Vector3(0.0f, 0.0f, 1.0f), inertia.inverse().I);
    EXPECT_NEAR(bs.angularVelocity.z, 1.0f / 0.4f, kEpsilon);
}

TEST(BodyState, VelocityAtPoint) {
    BodyState bs;
    bs.linearVelocity = Vector3(1.0f, 0.0f, 0.0f);
    bs.angularVelocity = Vector3(0.0f, 0.0f, 1.0f);
    // v_point = v_cm + omega x r = (1,0,0) + (0,0,1) x (0,1,0) = (1,0,0) + (-1,0,0) = (0,0,0)
    Vector3 v = bs.velocityAtPoint(Vector3(0.0f, 1.0f, 0.0f));
    EXPECT_NEAR(v.x, 0.0f, kEpsilon);
    EXPECT_NEAR(v.y, 0.0f, kEpsilon);
    EXPECT_NEAR(v.z, 0.0f, kEpsilon);
}

TEST(Momentum, LinearMomentum) {
    Vector3 p = linearMomentum(Vector3(2.0f, 3.0f, 4.0f), 5.0f);
    EXPECT_NEAR(p.x, 10.0f, kEpsilon);
    EXPECT_NEAR(p.y, 15.0f, kEpsilon);
    EXPECT_NEAR(p.z, 20.0f, kEpsilon);
}

TEST(Momentum, AngularMomentum) {
    Matrix3 I = Matrix3::scale(2.0f, 3.0f, 4.0f);
    Vector3 L = angularMomentum(Vector3(1.0f, 1.0f, 1.0f), I);
    EXPECT_NEAR(L.x, 2.0f, kEpsilon);
    EXPECT_NEAR(L.y, 3.0f, kEpsilon);
    EXPECT_NEAR(L.z, 4.0f, kEpsilon);
}

TEST(Energy, KineticEnergy) {
    f32 ke = kineticEnergy(2.0f, Vector3(3.0f, 0.0f, 0.0f));
    // KE = 0.5 * 2 * 9 = 9
    EXPECT_NEAR(ke, 9.0f, kEpsilon);
}

TEST(Energy, RotationalKineticEnergy) {
    Matrix3 I = Matrix3::scale(2.0f, 2.0f, 2.0f);
    f32 ke = rotationalKineticEnergy(Vector3(1.0f, 0.0f, 0.0f), I);
    // KE = 0.5 * (1,0,0) * (2,0,0) = 0.5 * 2 = 1
    EXPECT_NEAR(ke, 1.0f, kEpsilon);
}

TEST(Energy, TotalKineticEnergy) {
    Matrix3 I = Matrix3::scale(2.0f, 2.0f, 2.0f);
    f32 ke = totalKineticEnergy(1.0f, Vector3(1.0f, 0.0f, 0.0f),
                                Vector3(1.0f, 0.0f, 0.0f), I);
    // translational: 0.5 * 1 * 1 = 0.5
    // rotational: 0.5 * 1 * 2 = 1.0
    EXPECT_NEAR(ke, 1.5f, kEpsilon);
}
