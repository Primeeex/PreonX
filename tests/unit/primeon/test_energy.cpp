#include <gtest/gtest.h>
#include <primeon/dynamics/energy/energy.hpp>

using namespace primeon::math;

TEST(Energy, ComputeKineticEnergy) {
    f32 ke = computeKineticEnergy(2.0f, Vector3(3.0f, 4.0f, 0.0f));
    // v^2 = 9 + 16 = 25, KE = 0.5 * 2 * 25 = 25
    EXPECT_NEAR(ke, 25.0f, kEpsilon);
}

TEST(Energy, ComputeKineticEnergyZeroVelocity) {
    f32 ke = computeKineticEnergy(5.0f, Vector3(0.0f, 0.0f, 0.0f));
    EXPECT_NEAR(ke, 0.0f, kEpsilon);
}

TEST(Energy, ComputePotentialEnergy) {
    f32 pe = computePotentialEnergy(10.0f, 5.0f);
    // PE = m * g * h = 10 * 9.80665 * 5
    EXPECT_NEAR(pe, 10.0f * 9.80665f * 5.0f, kLargeEpsilon);
}

TEST(Energy, ComputePotentialEnergyNegativeHeight) {
    f32 pe = computePotentialEnergy(10.0f, -3.0f);
    EXPECT_TRUE(pe < 0.0f);
}

TEST(Energy, ComputeTotalEnergy) {
    f32 E = computeTotalEnergy(10.0f, Vector3(0.0f, 5.0f, 0.0f), Vector3(3.0f, 0.0f, 0.0f));
    // KE = 0.5 * 10 * 9 = 45
    // PE = 10 * 9.80665 * 5 = 490.3325
    // Total = 535.3325
    EXPECT_NEAR(E, 45.0f + 10.0f * 9.80665f * 5.0f, kLargeEpsilon);
}

TEST(Energy, ComputeTotalEnergyParticle) {
    ParticleState ps(Vector3(0.0f, 10.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    f32 E = computeTotalEnergy(1.0f, ps);
    // KE = 0, PE = 1 * 9.80665 * 10 = 98.0665
    EXPECT_NEAR(E, 98.0665f, kLargeEpsilon);
}

TEST(Energy, ComputeTotalEnergyBody) {
    BodyState bs;
    bs.position = Vector3(0.0f, 20.0f, 0.0f);
    bs.linearVelocity = Vector3(5.0f, 0.0f, 0.0f);
    f32 E = computeTotalEnergy(2.0f, bs);
    // KE = 0.5 * 2 * 25 = 25
    // PE = 2 * 9.80665 * 20 = 392.266
    EXPECT_NEAR(E, 25.0f + 2.0f * 9.80665f * 20.0f, kLargeEpsilon);
}

TEST(Energy, EnergyErrorZero) {
    f32 err = energyError(100.0f, 100.0f);
    EXPECT_NEAR(err, 0.0f, kEpsilon);
}

TEST(Energy, EnergyErrorNonZero) {
    f32 err = energyError(100.0f, 110.0f);
    EXPECT_NEAR(err, 0.1f, kEpsilon);
}

TEST(Energy, EnergyErrorZeroInitial) {
    f32 err = energyError(0.0f, 0.0f);
    EXPECT_NEAR(err, 0.0f, kEpsilon);
}

TEST(Energy, ConservationFreefall) {
    // Free-fall: total energy should be conserved
    f32 mass = 1.0f;
    Vector3 pos0(0.0f, 100.0f, 0.0f);
    Vector3 vel0(0.0f, 0.0f, 0.0f);
    f32 E0 = computeTotalEnergy(mass, pos0, vel0);

    // After 1 second of free fall: v = -g*t = -9.80665, y = 100 - 0.5*g*t^2
    f32 t = 1.0f;
    f32 g = 9.80665f;
    Vector3 pos1(0.0f, 100.0f - 0.5f * g * t * t, 0.0f);
    Vector3 vel1(0.0f, -g * t, 0.0f);
    f32 E1 = computeTotalEnergy(mass, pos1, vel1);

    EXPECT_NEAR(E0, E1, kLargeEpsilon);
}
