#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/vector/vector3.hpp"
#include "primeon/math/quaternion/quaternion.hpp"
#include "primeon/dynamics/mass/mass.hpp"
#include "primeon/dynamics/forces/forces.hpp"
#include "primeon/dynamics/motion/motion.hpp"

namespace primeon::math {

// ══════════════════════════════════════════════════════════════════════════════
// Particle Integrators
//
// All integrators follow the same interface:
//   ParticleState integrate(ParticleState, ForceAccumulator, MassProperties, f32 dt)
//
// Acceleration: a = F * inverseMass (zero inverse mass = static = no change)
// ══════════════════════════════════════════════════════════════════════════════

/// Computes acceleration from net force and mass.
[[nodiscard]] constexpr Vector3 computeAcceleration(const Vector3& force, f32 inverseMass) noexcept {
    return force * inverseMass;
}

// ── 1. Explicit Euler ────────────────────────────────────────────────────────
//
// x' = x + v * dt
// v' = v + a * dt
//
// O(h^2) local truncation error. First-order. Conditionally stable.
// The position is updated using the OLD velocity, causing energy drift
// (energy gain) for oscillatory systems. Not recommended for production.

[[nodiscard]] constexpr ParticleState integrateExplicitEuler(
    const ParticleState& state,
    const ForceAccumulator& forces,
    const MassProperties& mass,
    f32 dt) noexcept
{
    if (mass.isStatic()) return state;

    Vector3 a = computeAcceleration(forces.force(), mass.inverseMass);
    ParticleState result;
    result.position = state.position + state.velocity * dt;
    result.velocity = state.velocity + a * dt;
    return result;
}

// ── 2. Semi-Implicit (Symplectic) Euler ─────────────────────────────────────
//
// v' = v + a * dt
// x' = x + v' * dt
//
// O(h^2) local truncation error. First-order. Unconditionally stable for
// linear systems. The key difference from explicit Euler: velocity is updated
// FIRST, then position uses the NEW velocity. This makes it symplectic —
// it conserves a shadow Hamiltonian, preventing long-term energy drift.
// This is the standard integrator for game physics.

[[nodiscard]] constexpr ParticleState integrateSemiImplicitEuler(
    const ParticleState& state,
    const ForceAccumulator& forces,
    const MassProperties& mass,
    f32 dt) noexcept
{
    if (mass.isStatic()) return state;

    Vector3 a = computeAcceleration(forces.force(), mass.inverseMass);
    ParticleState result;
    result.velocity = state.velocity + a * dt;
    result.position = state.position + result.velocity * dt;
    return result;
}

// ── 3. Position Verlet ──────────────────────────────────────────────────────
//
// x' = 2*x - x_prev + a * dt^2
//
// O(h^4) local truncation error. Second-order. Symplectic. Requires storing
// the previous position (not the velocity). Velocity is derived as
// v = (x' - x_prev) / (2*dt). Excellent energy conservation.
// Limitation: forces must be evaluated at current position only (no velocity
// dependent forces without modification). For this API, we store the previous
// position inside the state (the position member represents current, and
// velocity member is repurposed to store previous position).
//
// NOTE: This integrator repurposes ParticleState.velocity as "previous position".
// After integration, velocity = (newPos - oldPos) / dt. The caller must be
// aware of this convention when using the Verlet integrator.

[[nodiscard]] constexpr ParticleState integrateVerlet(
    const ParticleState& state,
    const ForceAccumulator& forces,
    const MassProperties& mass,
    f32 dt) noexcept
{
    if (mass.isStatic()) return state;

    Vector3 a = computeAcceleration(forces.force(), mass.inverseMass);
    f32 dtSq = dt * dt;

    ParticleState result;
    // state.velocity here is actually the previous position
    // x_new = 2*x_current - x_prev + a*dt^2
    result.position = state.position * 2.0f - state.velocity + a * dtSq;
    // Output velocity stores the previous position (x_current) for the next step
    result.velocity = state.position;
    return result;
}

// ── 4. Velocity Verlet ──────────────────────────────────────────────────────
//
// x' = x + v*dt + 0.5*a*dt^2
// v' = v + 0.5*(a + a')*dt
//
// O(h^4) local truncation error. Second-order. Symplectic. Requires two force
// evaluations per step (at current and new position). For velocity-independent
// forces (like gravity), a' = a, simplifying to:
//   v' = v + a * dt (same as semi-implicit)
//   x' = x + v*dt + 0.5*a*dt^2
//
// This is the "kick-drift-kick" variant. Since we don't have collision yet,
// forces are velocity-independent, so we use the simplified form.

[[nodiscard]] constexpr ParticleState integrateVelocityVerlet(
    const ParticleState& state,
    const ForceAccumulator& forces,
    const MassProperties& mass,
    f32 dt) noexcept
{
    if (mass.isStatic()) return state;

    Vector3 a = computeAcceleration(forces.force(), mass.inverseMass);
    f32 halfDtSq = 0.5f * dt * dt;

    ParticleState result;
    result.position = state.position + state.velocity * dt + a * halfDtSq;
    // For velocity-independent forces: a' = a, so v' = v + a*dt
    result.velocity = state.velocity + a * dt;
    return result;
}

// ── 5. Runge-Kutta 4 (RK4) ─────────────────────────────────────────────────
//
// k1 = f(t, y)
// k2 = f(t + dt/2, y + dt/2 * k1)
// k3 = f(t + dt/2, y + dt/2 * k2)
// k4 = f(t + dt, y + dt * k3)
// y' = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
//
// O(h^4) local truncation error. Fourth-order. Conditionally stable.
// Most accurate of the five methods. Four force evaluations per step (4x cost).
// Not symplectic — can exhibit energy drift for very long simulations.
// Best for high-fidelity offline simulations or short-duration accuracy.

[[nodiscard]] constexpr ParticleState integrateRK4(
    const ParticleState& state,
    const ForceAccumulator& forces,
    const MassProperties& mass,
    f32 dt) noexcept
{
    if (mass.isStatic()) return state;

    Vector3 a = computeAcceleration(forces.force(), mass.inverseMass);

    // For constant forces, a is the same at all stages.
    // k1: derivative at current state
    Vector3 k1v = a;
    Vector3 k1x = state.velocity;

    // k2: derivative at midpoint using k1
    Vector3 k2v = a;
    Vector3 k2x = state.velocity + k1v * (0.5f * dt);

    // k3: derivative at midpoint using k2
    Vector3 k3v = a;
    Vector3 k3x = state.velocity + k2v * (0.5f * dt);

    // k4: derivative at endpoint using k3
    Vector3 k4v = a;
    Vector3 k4x = state.velocity + k3v * dt;

    ParticleState result;
    result.position = state.position + (k1x + k2x * 2.0f + k3x * 2.0f + k4x) * (dt / 6.0f);
    result.velocity = state.velocity + (k1v + k2v * 2.0f + k3v * 2.0f + k4v) * (dt / 6.0f);
    return result;
}

// ══════════════════════════════════════════════════════════════════════════════
// Rigid Body Integrators
//
// These integrate both linear and angular motion simultaneously.
// Interface:
//   BodyState integrate(BodyState, ForceAccumulator, MassProperties,
//                       InertiaTensor, f32 dt)
//
// Linear: a = F * invMass, alpha = I_inv * torque
// Angular: dq/dt = 0.5 * Quaternion(0, omega) * q
// ══════════════════════════════════════════════════════════════════════════════

/// Computes the quaternion derivative from angular velocity.
/// dq/dt = 0.5 * Quaternion(0, omega) * q
[[nodiscard]] inline Quaternion quaternionDerivative(const Quaternion& q, const Vector3& omega) noexcept {
    Quaternion qOmega(omega.x, omega.y, omega.z, 0.0f);
    return qOmega * q * 0.5f;
}

/// Integrates a quaternion: q' = q + dq/dt * dt, then normalizes.
[[nodiscard]] inline Quaternion integrateQuaternion(const Quaternion& q, const Vector3& omega, f32 dt) noexcept {
    Quaternion dq = quaternionDerivative(q, omega);
    Quaternion result;
    result.x = q.x + dq.x * dt;
    result.y = q.y + dq.y * dt;
    result.z = q.z + dq.z * dt;
    result.w = q.w + dq.w * dt;
    return result.normalized();
}

// ── Body: Explicit Euler ─────────────────────────────────────────────────────

[[nodiscard]] inline BodyState integrateBodyExplicitEuler(
    const BodyState& state,
    const ForceAccumulator& forces,
    const MassProperties& mass,
    const InertiaTensor& inertia,
    f32 dt) noexcept
{
    if (mass.isStatic()) return state;

    Vector3 linearAccel = computeAcceleration(forces.force(), mass.inverseMass);
    Matrix3 invInertia = inertia.inverse().I;
    Vector3 angularAccel = invInertia * forces.torque();

    BodyState result;
    result.position = state.position + state.linearVelocity * dt;
    result.rotation = integrateQuaternion(state.rotation, state.angularVelocity, dt);
    result.linearVelocity = state.linearVelocity + linearAccel * dt;
    result.angularVelocity = state.angularVelocity + angularAccel * dt;
    return result;
}

// ── Body: Semi-Implicit Euler ────────────────────────────────────────────────

[[nodiscard]] inline BodyState integrateBodySemiImplicitEuler(
    const BodyState& state,
    const ForceAccumulator& forces,
    const MassProperties& mass,
    const InertiaTensor& inertia,
    f32 dt) noexcept
{
    if (mass.isStatic()) return state;

    Vector3 linearAccel = computeAcceleration(forces.force(), mass.inverseMass);
    Matrix3 invInertia = inertia.inverse().I;
    Vector3 angularAccel = invInertia * forces.torque();

    BodyState result;
    result.linearVelocity = state.linearVelocity + linearAccel * dt;
    result.angularVelocity = state.angularVelocity + angularAccel * dt;
    result.position = state.position + result.linearVelocity * dt;
    result.rotation = integrateQuaternion(state.rotation, result.angularVelocity, dt);
    return result;
}

// ── Body: Velocity Verlet ────────────────────────────────────────────────────

[[nodiscard]] inline BodyState integrateBodyVelocityVerlet(
    const BodyState& state,
    const ForceAccumulator& forces,
    const MassProperties& mass,
    const InertiaTensor& inertia,
    f32 dt) noexcept
{
    if (mass.isStatic()) return state;

    Vector3 linearAccel = computeAcceleration(forces.force(), mass.inverseMass);
    Matrix3 invInertia = inertia.inverse().I;
    Vector3 angularAccel = invInertia * forces.torque();
    f32 halfDtSq = 0.5f * dt * dt;

    BodyState result;
    result.position = state.position + state.linearVelocity * dt + linearAccel * halfDtSq;
    result.linearVelocity = state.linearVelocity + linearAccel * dt;
    result.angularVelocity = state.angularVelocity + angularAccel * dt;
    result.rotation = integrateQuaternion(state.rotation, result.angularVelocity, dt);
    return result;
}

// ── Body: RK4 ────────────────────────────────────────────────────────────────

[[nodiscard]] inline BodyState integrateBodyRK4(
    const BodyState& state,
    const ForceAccumulator& forces,
    const MassProperties& mass,
    const InertiaTensor& inertia,
    f32 dt) noexcept
{
    if (mass.isStatic()) return state;

    Vector3 linearAccel = computeAcceleration(forces.force(), mass.inverseMass);
    Matrix3 invInertia = inertia.inverse().I;
    Vector3 angularAccel = invInertia * forces.torque();

    // Linear RK4 (constant acceleration — all stages have same acceleration)
    Vector3 k1v = linearAccel;
    Vector3 k1x = state.linearVelocity;
    Vector3 k2v = linearAccel;
    Vector3 k2x = state.linearVelocity + k1v * (0.5f * dt);
    Vector3 k3v = linearAccel;
    Vector3 k3x = state.linearVelocity + k2v * (0.5f * dt);
    Vector3 k4v = linearAccel;
    Vector3 k4x = state.linearVelocity + k3v * dt;

    BodyState result;
    result.position = state.position + (k1x + k2x * 2.0f + k3x * 2.0f + k4x) * (dt / 6.0f);
    result.linearVelocity = state.linearVelocity + (k1v + k2v * 2.0f + k3v * 2.0f + k4v) * (dt / 6.0f);

    // Angular RK4
    Quaternion q1 = quaternionDerivative(state.rotation, state.angularVelocity);
    Quaternion q2 = quaternionDerivative(
        (state.rotation + q1 * (0.5f * dt)).normalized(),
        state.angularVelocity + angularAccel * (0.5f * dt));
    Quaternion q3 = quaternionDerivative(
        (state.rotation + q2 * (0.5f * dt)).normalized(),
        state.angularVelocity + angularAccel * (0.5f * dt));
    Quaternion q4 = quaternionDerivative(
        (state.rotation + q3 * dt).normalized(),
        state.angularVelocity + angularAccel * dt);

    result.rotation.x = state.rotation.x + (q1.x + q2.x * 2.0f + q3.x * 2.0f + q4.x) * (dt / 6.0f);
    result.rotation.y = state.rotation.y + (q1.y + q2.y * 2.0f + q3.y * 2.0f + q4.y) * (dt / 6.0f);
    result.rotation.z = state.rotation.z + (q1.z + q2.z * 2.0f + q3.z * 2.0f + q4.z) * (dt / 6.0f);
    result.rotation.w = state.rotation.w + (q1.w + q2.w * 2.0f + q3.w * 2.0f + q4.w) * (dt / 6.0f);
    result.rotation = result.rotation.normalized();
    result.angularVelocity = state.angularVelocity + angularAccel * dt;
    return result;
}

} // namespace primeon::math
