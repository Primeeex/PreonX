#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/dynamics/mass/mass.hpp"
#include "primeon/dynamics/forces/forces.hpp"
#include "primeon/dynamics/motion/motion.hpp"
#include "primeon/dynamics/integrators/integrators.hpp"

namespace primeon::math {

// ── Integrator Selection ─────────────────────────────────────────────────────

/// Identifies which integration method to use.
enum class IntegratorType : u32 {
    ExplicitEuler = 0,
    SemiImplicitEuler = 1,
    Verlet = 2,
    VelocityVerlet = 3,
    RK4 = 4
};

// ── Timestep Accumulator ─────────────────────────────────────────────────────

/// Manages fixed-timestep simulation with interpolation support.
/// The accumulator pattern decouples physics rate from render rate.
///
/// Usage:
///   TimestepAccumulator acc(1.0f / 60.0f);
///   while (frameTime > 0) {
///       f32 dt = acc.step(frameTime);
///       if (dt > 0) simulate(dt);
///       frameTime -= dt;
///   }
///   f32 alpha = acc.alpha(); // for interpolation between frames
struct TimestepAccumulator {
    f32 fixedDt;
    f32 accumulator = 0.0f;
    f32 lastAlpha = 0.0f;

    constexpr explicit TimestepAccumulator(f32 fixedDeltaTime) noexcept
        : fixedDt(fixedDeltaTime > kEpsilon ? fixedDeltaTime : 1.0f / 60.0f) {}

    /// Consumes frame time. Returns the fixed dt to simulate if >= fixedDt
    /// has accumulated, otherwise returns 0.
    [[nodiscard]] constexpr f32 step(f32 frameTime) noexcept {
        accumulator += frameTime;
        if (accumulator >= fixedDt) {
            accumulator -= fixedDt;
            lastAlpha = accumulator / fixedDt;
            return fixedDt;
        }
        lastAlpha = accumulator / fixedDt;
        return 0.0f;
    }

    /// Returns the interpolation alpha for rendering between physics steps.
    /// 0.0 = state at previous step, 1.0 = state at next step.
    [[nodiscard]] constexpr f32 alpha() const noexcept { return lastAlpha; }

    /// Resets the accumulator to zero.
    constexpr void reset() noexcept { accumulator = 0.0f; lastAlpha = 0.0f; }
};

// ── Particle Simulation Step ─────────────────────────────────────────────────

/// Advances a particle by one step using the specified integrator.
[[nodiscard]] inline ParticleState simulateParticle(
    const ParticleState& state,
    const ForceAccumulator& forces,
    const MassProperties& mass,
    f32 dt,
    IntegratorType integrator = IntegratorType::SemiImplicitEuler) noexcept
{
    switch (integrator) {
        case IntegratorType::ExplicitEuler:
            return integrateExplicitEuler(state, forces, mass, dt);
        case IntegratorType::SemiImplicitEuler:
            return integrateSemiImplicitEuler(state, forces, mass, dt);
        case IntegratorType::Verlet:
            return integrateVerlet(state, forces, mass, dt);
        case IntegratorType::VelocityVerlet:
            return integrateVelocityVerlet(state, forces, mass, dt);
        case IntegratorType::RK4:
            return integrateRK4(state, forces, mass, dt);
        default:
            return integrateSemiImplicitEuler(state, forces, mass, dt);
    }
}

// ── Body Simulation Step ─────────────────────────────────────────────────────

/// Advances a rigid body by one step using the specified integrator.
[[nodiscard]] inline BodyState simulateBody(
    const BodyState& state,
    const ForceAccumulator& forces,
    const MassProperties& mass,
    const InertiaTensor& inertia,
    f32 dt,
    IntegratorType integrator = IntegratorType::SemiImplicitEuler) noexcept
{
    switch (integrator) {
        case IntegratorType::ExplicitEuler:
            return integrateBodyExplicitEuler(state, forces, mass, inertia, dt);
        case IntegratorType::SemiImplicitEuler:
            return integrateBodySemiImplicitEuler(state, forces, mass, inertia, dt);
        case IntegratorType::Verlet:
            // Body Verlet uses semi-implicit for angular (quaternion requires velocity)
            return integrateBodySemiImplicitEuler(state, forces, mass, inertia, dt);
        case IntegratorType::VelocityVerlet:
            return integrateBodyVelocityVerlet(state, forces, mass, inertia, dt);
        case IntegratorType::RK4:
            return integrateBodyRK4(state, forces, mass, inertia, dt);
        default:
            return integrateBodySemiImplicitEuler(state, forces, mass, inertia, dt);
    }
}

// ── Multi-Step Simulation ────────────────────────────────────────────────────

/// Advances a particle for a given total time, sub-stepping with fixed dt.
/// Returns the final state after all sub-steps.
[[nodiscard]] inline ParticleState simulateParticleFixedSteps(
    ParticleState state,
    ForceAccumulator forces,
    MassProperties mass,
    f32 totalTime,
    f32 fixedDt,
    IntegratorType integrator = IntegratorType::SemiImplicitEuler) noexcept
{
    TimestepAccumulator acc(fixedDt);
    f32 remaining = totalTime;
    while (remaining > kEpsilon) {
        f32 dt = acc.step(remaining);
        if (dt > kEpsilon) {
            state = simulateParticle(state, forces, mass, dt, integrator);
        }
        remaining -= dt;
        if (dt < kEpsilon) break;
    }
    return state;
}

/// Advances a rigid body for a given total time, sub-stepping with fixed dt.
[[nodiscard]] inline BodyState simulateBodyFixedSteps(
    BodyState state,
    ForceAccumulator forces,
    MassProperties mass,
    InertiaTensor inertia,
    f32 totalTime,
    f32 fixedDt,
    IntegratorType integrator = IntegratorType::SemiImplicitEuler) noexcept
{
    TimestepAccumulator acc(fixedDt);
    f32 remaining = totalTime;
    while (remaining > kEpsilon) {
        f32 dt = acc.step(remaining);
        if (dt > kEpsilon) {
            state = simulateBody(state, forces, mass, inertia, dt, integrator);
        }
        remaining -= dt;
        if (dt < kEpsilon) break;
    }
    return state;
}

} // namespace primeon::math
