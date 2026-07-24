#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/vector/vector3.hpp"
#include "primeon/dynamics/mass/mass.hpp"
#include "primeon/dynamics/forces/forces.hpp"
#include "primeon/dynamics/motion/motion.hpp"

namespace primeon::math {

// ── Energy Calculations ──────────────────────────────────────────────────────

/// Computes translational kinetic energy: KE = 0.5 * m * |v|^2.
/// Units: Joules (J).
[[nodiscard]] constexpr f32 computeKineticEnergy(f32 mass, const Vector3& velocity) noexcept {
    return 0.5f * mass * velocity.lengthSq();
}

/// Computes gravitational potential energy relative to a reference height.
/// PE = m * |g| * (y - y_ref). Uses Y component of position as height.
/// Units: Joules (J).
[[nodiscard]] constexpr f32 computePotentialEnergy(f32 mass, f32 height,
                                                    const Vector3& gravity = kGravityDown) noexcept {
    return mass * gravity.length() * height;
}

/// Computes gravitational potential energy for a body state.
/// Height is taken as the Y component of position.
[[nodiscard]] constexpr f32 computePotentialEnergy(f32 mass, const BodyState& body,
                                                    const Vector3& gravity = kGravityDown) noexcept {
    return computePotentialEnergy(mass, body.position.y, gravity);
}

/// Computes gravitational potential energy for a particle state.
[[nodiscard]] constexpr f32 computePotentialEnergy(f32 mass, const ParticleState& particle,
                                                    const Vector3& gravity = kGravityDown) noexcept {
    return computePotentialEnergy(mass, particle.position.y, gravity);
}

/// Computes total mechanical energy: E = KE + PE.
/// Units: Joules (J). Useful for energy conservation validation.
[[nodiscard]] constexpr f32 computeTotalEnergy(f32 mass, const Vector3& position,
                                               const Vector3& velocity,
                                               const Vector3& gravity = kGravityDown) noexcept {
    return computeKineticEnergy(mass, velocity) + computePotentialEnergy(mass, position.y, gravity);
}

/// Computes total mechanical energy for a particle.
[[nodiscard]] constexpr f32 computeTotalEnergy(f32 mass, const ParticleState& particle,
                                               const Vector3& gravity = kGravityDown) noexcept {
    return computeTotalEnergy(mass, particle.position, particle.velocity, gravity);
}

/// Computes total mechanical energy for a rigid body (translational + potential).
/// Does not include rotational KE (requires inertia tensor).
[[nodiscard]] constexpr f32 computeTotalEnergy(f32 mass, const BodyState& body,
                                               const Vector3& gravity = kGravityDown) noexcept {
    return computeTotalEnergy(mass, body.position, body.linearVelocity, gravity);
}

/// Computes the energy error as a fraction of initial energy.
/// Returns |E_current - E_initial| / |E_initial|.
/// Returns 0 if initial energy is zero (both zero = no error).
[[nodiscard]] constexpr f32 energyError(f32 initialEnergy, f32 currentEnergy) noexcept {
    f32 diff = abs(currentEnergy - initialEnergy);
    f32 ref = abs(initialEnergy);
    if (ref < kEpsilon) return (diff < kEpsilon) ? 0.0f : kLargeEpsilon;
    return diff / ref;
}

} // namespace primeon::math
