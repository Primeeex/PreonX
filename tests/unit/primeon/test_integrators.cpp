#include <gtest/gtest.h>
#include <primeon/dynamics/simulation.hpp>
#include <primeon/dynamics/energy/energy.hpp>

using namespace primeon::math;

static constexpr f32 kTestGravity = 9.80665f;

TEST(Integrator, ExplicitEuler_FreeFall) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kTestGravity, 0.0f));
    ParticleState state(Vector3(0.0f, 100.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    f32 dt = 0.01f;
    for (int i = 0; i < 100; ++i)
        state = integrateExplicitEuler(state, forces, mass, dt);
    EXPECT_NEAR(state.velocity.y, -kTestGravity, kLargeEpsilon);
    EXPECT_NEAR(state.position.y, 100.0f - 0.5f * kTestGravity, 0.1f);
}

TEST(Integrator, ExplicitEuler_ZeroForce) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    ParticleState state(Vector3(1.0f, 2.0f, 3.0f), Vector3(4.0f, 5.0f, 6.0f));
    ParticleState result = integrateExplicitEuler(state, forces, mass, 1.0f);
    EXPECT_NEAR(result.velocity.x, 4.0f, kEpsilon);
    EXPECT_NEAR(result.position.x, 5.0f, kEpsilon);
}

TEST(Integrator, ExplicitEuler_StaticMass) {
    MassProperties mass = makeStaticMass();
    ForceAccumulator forces;
    forces.addForce(Vector3(100.0f, 0.0f, 0.0f));
    ParticleState state(Vector3(1.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    ParticleState result = integrateExplicitEuler(state, forces, mass, 1.0f);
    EXPECT_NEAR(result.position.x, 1.0f, kEpsilon);
    EXPECT_TRUE(result.velocity.isZero());
}

TEST(Integrator, SemiImplicitEuler_FreeFall) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kTestGravity, 0.0f));
    ParticleState state(Vector3(0.0f, 100.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    f32 dt = 0.01f;
    for (int i = 0; i < 100; ++i)
        state = integrateSemiImplicitEuler(state, forces, mass, dt);
    EXPECT_NEAR(state.velocity.y, -kTestGravity, kLargeEpsilon);
    EXPECT_NEAR(state.position.y, 100.0f - 0.5f * kTestGravity, 0.1f);
}

TEST(Integrator, SemiImplicitEuler_ConstantVelocity) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    ParticleState state(Vector3(0.0f, 0.0f, 0.0f), Vector3(10.0f, 0.0f, 0.0f));
    f32 dt = 0.001f;
    for (int i = 0; i < 1000; ++i)
        state = integrateSemiImplicitEuler(state, forces, mass, dt);
    EXPECT_NEAR(state.position.x, 10.0f, 0.001f);
    EXPECT_NEAR(state.velocity.x, 10.0f, kEpsilon);
}

TEST(Integrator, Verlet_ZeroForce) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    ParticleState state(Vector3(5.0f, 0.0f, 0.0f), Vector3(4.0f, 0.0f, 0.0f));
    ParticleState result = integrateVerlet(state, forces, mass, 1.0f);
    EXPECT_NEAR(result.position.x, 6.0f, kEpsilon);
    EXPECT_NEAR(result.velocity.x, 5.0f, kEpsilon);
}

TEST(Integrator, Verlet_FreeFall) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kTestGravity, 0.0f));
    ParticleState state(Vector3(0.0f, 100.0f, 0.0f), Vector3(0.0f, 100.0f, 0.0f));
    f32 dt = 0.01f;
    for (int i = 0; i < 100; ++i)
        state = integrateVerlet(state, forces, mass, dt);
    f32 expectedY = 100.0f - 0.5f * kTestGravity;
    EXPECT_NEAR(state.position.y, expectedY, 0.1f);
}

TEST(Integrator, VelocityVerlet_FreeFall) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kTestGravity, 0.0f));
    ParticleState state(Vector3(0.0f, 100.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    f32 dt = 0.01f;
    for (int i = 0; i < 100; ++i)
        state = integrateVelocityVerlet(state, forces, mass, dt);
    EXPECT_NEAR(state.velocity.y, -kTestGravity, kLargeEpsilon);
    EXPECT_NEAR(state.position.y, 100.0f - 0.5f * kTestGravity, 0.01f);
}

TEST(Integrator, VelocityVerlet_ZeroForce) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    ParticleState state(Vector3(0.0f, 0.0f, 0.0f), Vector3(5.0f, 0.0f, 0.0f));
    ParticleState result = integrateVelocityVerlet(state, forces, mass, 1.0f);
    EXPECT_NEAR(result.position.x, 5.0f, kEpsilon);
    EXPECT_NEAR(result.velocity.x, 5.0f, kEpsilon);
}

TEST(Integrator, RK4_FreeFall) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kTestGravity, 0.0f));
    ParticleState state(Vector3(0.0f, 100.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    f32 dt = 0.01f;
    for (int i = 0; i < 100; ++i)
        state = integrateRK4(state, forces, mass, dt);
    EXPECT_NEAR(state.velocity.y, -kTestGravity, kLargeEpsilon);
    EXPECT_NEAR(state.position.y, 100.0f - 0.5f * kTestGravity, kLargeEpsilon);
}

TEST(Integrator, RK4_HighAccuracy) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kTestGravity, 0.0f));
    ParticleState state(Vector3(0.0f, 100.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    f32 dt = 0.1f;
    for (int i = 0; i < 10; ++i)
        state = integrateRK4(state, forces, mass, dt);
    EXPECT_NEAR(state.velocity.y, -kTestGravity, kLargeEpsilon);
    EXPECT_NEAR(state.position.y, 100.0f - 0.5f * kTestGravity, kLargeEpsilon);
}

TEST(Integrator, EnergyConservation_SemiImplicitEuler) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kTestGravity, 0.0f));
    ParticleState state(Vector3(0.0f, 100.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    f32 E0 = computeTotalEnergy(mass.mass, state);
    f32 dt = 0.001f;
    for (int i = 0; i < 100; ++i)
        state = integrateSemiImplicitEuler(state, forces, mass, dt);
    f32 E1 = computeTotalEnergy(mass.mass, state);
    EXPECT_NEAR(E0, E1, 0.1f);
}

TEST(Integrator, EnergyConservation_VelocityVerlet) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -kTestGravity, 0.0f));
    ParticleState state(Vector3(0.0f, 100.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    f32 E0 = computeTotalEnergy(mass.mass, state);
    f32 dt = 0.01f;
    for (int i = 0; i < 100; ++i)
        state = integrateVelocityVerlet(state, forces, mass, dt);
    f32 E1 = computeTotalEnergy(mass.mass, state);
    EXPECT_NEAR(E0, E1, 0.1f);
}
