#include <gtest/gtest.h>
#include <primeon/dynamics/simulation.hpp>

using namespace primeon::math;

static constexpr f32 kGrav = 9.80665f;

TEST(Simulate, AllIntegrators) {
    MassProperties mass = MassProperties::fromMass(2.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kGrav, 0.0f));
    ParticleState state(Vector3(0.0f, 50.0f, 0.0f), Vector3(0.0f, -1.0f, 0.0f));
    for (u32 i = 0; i <= 4; ++i) {
        if (i == 2) continue;
        auto type = static_cast<IntegratorType>(i);
        ParticleState result = simulateParticle(state, forces, mass, 0.01f, type);
        EXPECT_TRUE(result.velocity.y < -1.0f) << "Integrator " << i;
        EXPECT_TRUE(result.position.y < 50.0f) << "Integrator " << i;
    }
}

TEST(Simulate, BodySemiImplicit) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    InertiaTensor inertia = InertiaTensor::solidSphere(1.0f, 1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kGrav, 0.0f));
    forces.addTorque(Vector3(0.0f, 0.0f, 1.0f));
    BodyState state;
    state.position = Vector3(0.0f, 10.0f, 0.0f);
    BodyState result = simulateBody(state, forces, mass, inertia, 0.01f,
                                     IntegratorType::SemiImplicitEuler);
    EXPECT_TRUE(result.position.y < 10.0f);
    EXPECT_TRUE(result.linearVelocity.y < 0.0f);
    EXPECT_TRUE(result.angularVelocity.z > 0.0f);
}

TEST(Simulate, FixedTimestep) {
    TimestepAccumulator acc(1.0f / 60.0f);
    f32 fixedDt = 1.0f / 60.0f;
    f32 dt1 = acc.step(0.005f);
    EXPECT_NEAR(dt1, 0.0f, kEpsilon);
    f32 dt2 = acc.step(0.02f);
    EXPECT_NEAR(dt2, fixedDt, kEpsilon);
    f32 dt3 = acc.step(0.01f);
    EXPECT_NEAR(dt3, fixedDt, kEpsilon);
}

TEST(Simulate, FixedStepsParticle) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kGrav, 0.0f));
    ParticleState state(Vector3(0.0f, 100.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    ParticleState result = simulateParticleFixedSteps(state, forces, mass, 1.0f, 1.0f / 60.0f);
    f32 expectedY = 100.0f - 0.5f * kGrav;
    EXPECT_NEAR(result.position.y, expectedY, 0.1f);
}

TEST(Simulate, FixedStepsBody) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    InertiaTensor inertia = InertiaTensor::solidSphere(1.0f, 1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kGrav, 0.0f));
    BodyState state;
    state.position = Vector3(0.0f, 100.0f, 0.0f);
    BodyState result = simulateBodyFixedSteps(state, forces, mass, inertia,
                                              1.0f, 1.0f / 60.0f);
    f32 expectedY = 100.0f - 0.5f * kGrav;
    EXPECT_NEAR(result.position.y, expectedY, 0.1f);
}

TEST(Simulate, BodyVelocityVerlet) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    InertiaTensor inertia = InertiaTensor::solidSphere(1.0f, 1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kGrav, 0.0f));
    forces.addTorque(Vector3(0.0f, 0.0f, 1.0f));
    BodyState state;
    state.position = Vector3(0.0f, 100.0f, 0.0f);
    for (int i = 0; i < 100; ++i)
        state = simulateBody(state, forces, mass, inertia, 0.01f,
                              IntegratorType::VelocityVerlet);
    f32 expectedY = 100.0f - 0.5f * kGrav;
    EXPECT_NEAR(state.position.y, expectedY, 0.01f);
}

TEST(Simulate, BodyRK4) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    InertiaTensor inertia = InertiaTensor::solidSphere(1.0f, 1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kGrav, 0.0f));
    BodyState state;
    state.position = Vector3(0.0f, 100.0f, 0.0f);
    for (int i = 0; i < 100; ++i)
        state = simulateBody(state, forces, mass, inertia, 0.01f,
                              IntegratorType::RK4);
    f32 expectedY = 100.0f - 0.5f * kGrav;
    EXPECT_NEAR(state.position.y, expectedY, 0.01f);
}

TEST(Simulate, BodyExplicitEuler) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    InertiaTensor inertia = InertiaTensor::solidSphere(1.0f, 1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kGrav, 0.0f));
    BodyState state;
    state.position = Vector3(0.0f, 100.0f, 0.0f);
    for (int i = 0; i < 100; ++i)
        state = simulateBody(state, forces, mass, inertia, 0.01f,
                              IntegratorType::ExplicitEuler);
    f32 expectedY = 100.0f - 0.5f * kGrav;
    EXPECT_NEAR(state.position.y, expectedY, 0.1f);
}

TEST(Simulate, ZeroGravity) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    ParticleState state(Vector3(5.0f, 5.0f, 5.0f), Vector3(1.0f, 2.0f, 3.0f));
    ParticleState result = simulateParticle(state, forces, mass, 1.0f,
                                             IntegratorType::SemiImplicitEuler);
    EXPECT_NEAR(result.position.x, 6.0f, kEpsilon);
    EXPECT_NEAR(result.position.y, 7.0f, kEpsilon);
    EXPECT_NEAR(result.position.z, 8.0f, kEpsilon);
}

TEST(Simulate, TinyTimestep) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kGrav, 0.0f));
    ParticleState state(Vector3(0.0f, 100.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    f32 dt = 1e-6f;
    for (int i = 0; i < 1000000; ++i)
        state = integrateSemiImplicitEuler(state, forces, mass, dt);
    f32 expectedY = 100.0f - 0.5f * kGrav;
    EXPECT_NEAR(state.position.y, expectedY, 0.5f);
}

TEST(Simulate, LargeTimestep) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kGrav, 0.0f));
    ParticleState state(Vector3(0.0f, 100.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    ParticleState result = simulateParticle(state, forces, mass, 10.0f,
                                             IntegratorType::SemiImplicitEuler);
    EXPECT_TRUE(result.velocity.y < 0.0f);
    EXPECT_TRUE(result.position.y < 100.0f);
}
