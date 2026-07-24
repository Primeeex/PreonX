#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/vector/vector3.hpp"

namespace primeon::math {

// ── Force ────────────────────────────────────────────────────────────────────

/// A constant force vector applied at a point (defaults to center of mass).
/// Units: Newtons (N = kg*m/s^2).
/// Point defaults to zero (center of mass); nonzero point produces torque.
struct Force {
    Vector3 vector = kVector3Zero;
    Vector3 point = kVector3Zero;

    constexpr Force() noexcept = default;
    constexpr Force(const Vector3& force) noexcept : vector(force) {}
    constexpr Force(const Vector3& force, const Vector3& at) noexcept : vector(force), point(at) {}

    constexpr Force operator+(const Force& o) const noexcept { return {vector + o.vector, point}; }
    constexpr Force& operator+=(const Force& o) noexcept { vector += o.vector; return *this; }

    constexpr bool operator==(const Force& o) const noexcept { return vector == o.vector && point == o.point; }
    constexpr bool operator!=(const Force& o) const noexcept { return !(*this == o); }
};

/// Represents an instantaneous change in velocity (impulse).
/// Units: kg*m/s (momentum units). Applied as delta_v = impulse * inverseMass.
struct Impulse {
    Vector3 linear = kVector3Zero;
    Vector3 angular = kVector3Zero;

    constexpr Impulse() noexcept = default;
    constexpr Impulse(const Vector3& linearImpulse) noexcept : linear(linearImpulse) {}
    constexpr Impulse(const Vector3& lin, const Vector3& ang) noexcept : linear(lin), angular(ang) {}

    constexpr Impulse operator+(const Impulse& o) const noexcept { return {linear + o.linear, angular + o.angular}; }
    constexpr Impulse& operator+=(const Impulse& o) noexcept { linear += o.linear; angular += o.angular; return *this; }
};

// ── Force Accumulator ────────────────────────────────────────────────────────

/// Accumulates forces and torques for a single body over a frame.
/// After all forces are added, read totalForce/totalTorque and call clear() for next frame.
struct ForceAccumulator {
    Vector3 totalForce = kVector3Zero;
    Vector3 totalTorque = kVector3Zero;

    constexpr ForceAccumulator() noexcept = default;

    /// Adds a force acting at a given offset from center of mass.
    /// If offset is zero, only linear force is accumulated.
    constexpr void addForce(const Force& force) noexcept {
        totalForce += force.vector;
        totalTorque += force.point.cross(force.vector);
    }

    /// Adds a pure linear force (at center of mass, no torque).
    constexpr void addForce(const Vector3& force) noexcept {
        totalForce += force;
    }

    /// Adds a pure torque (no linear force).
    constexpr void addTorque(const Vector3& torque) noexcept {
        totalTorque += torque;
    }

    /// Adds an impulse (instantaneous velocity change). Does not clear accumulator.
    /// Returns the delta-v to apply: deltaVelocity = impulse * inverseMass.
    [[nodiscard]] constexpr Vector3 applyImpulse(const Impulse& impulse, f32 inverseMass) const noexcept {
        return impulse.linear * inverseMass;
    }

    /// Returns the accumulated net force.
    [[nodiscard]] constexpr const Vector3& force() const noexcept { return totalForce; }

    /// Returns the accumulated net torque.
    [[nodiscard]] constexpr const Vector3& torque() const noexcept { return totalTorque; }

    /// Returns true if no forces or torques have been accumulated.
    [[nodiscard]] constexpr bool isZero() const noexcept {
        return totalForce.isZero() && totalTorque.isZero();
    }

    /// Resets all accumulated forces and torques to zero.
    constexpr void clear() noexcept { totalForce = kVector3Zero; totalTorque = kVector3Zero; }
};

// ── Gravity Helper ───────────────────────────────────────────────────────────

/// Standard gravity constants.
/// Default: Earth surface gravity, Y-up convention.
inline constexpr f32 kGravityMagnitude = 9.80665f;
inline constexpr Vector3 kGravityDown = {0.0f, -kGravityMagnitude, 0.0f};

/// Computes gravitational force on an object: F = m * g.
/// Returns force vector in Newtons.
[[nodiscard]] constexpr Vector3 gravitationalForce(f32 mass, const Vector3& gravity = kGravityDown) noexcept {
    return gravity * mass;
}

/// Computes gravitational potential energy: PE = m * g * h.
/// Height is measured along the gravity axis (Y for default gravity).
/// Returns energy in Joules.
[[nodiscard]] constexpr f32 gravitationalPotentialEnergy(f32 mass, f32 height, const Vector3& gravity = kGravityDown) noexcept {
    f32 gLen = gravity.length();
    return mass * gLen * height;
}

} // namespace primeon::math
