#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/vector/vector3.hpp"
#include "primeon/math/quaternion/quaternion.hpp"
#include "primeon/dynamics/mass/mass.hpp"

namespace primeon::math {

// ── Particle State ───────────────────────────────────────────────────────────

/// State of a point-mass particle. No rotation.
/// Units: position in meters, velocity in m/s, acceleration in m/s^2.
struct ParticleState {
    Vector3 position = kVector3Zero;
    Vector3 velocity = kVector3Zero;

    constexpr ParticleState() noexcept = default;
    constexpr ParticleState(const Vector3& pos, const Vector3& vel) noexcept
        : position(pos), velocity(vel) {}

    /// Applies a velocity impulse: v += impulse * inverseMass.
    constexpr void applyImpulse(const Vector3& impulse, f32 inverseMass) noexcept {
        velocity += impulse * inverseMass;
    }

    constexpr bool operator==(const ParticleState& o) const noexcept {
        return position == o.position && velocity == o.velocity;
    }
    constexpr bool operator!=(const ParticleState& o) const noexcept { return !(*this == o); }
};

// ── Body State ───────────────────────────────────────────────────────────────

/// State of a rigid body with orientation. Includes both linear and angular quantities.
/// Units: position in meters, velocity in m/s, orientation as quaternion,
/// angular velocity in rad/s.
struct BodyState {
    Vector3 position = kVector3Zero;
    Quaternion rotation = Quaternion::identity();
    Vector3 linearVelocity = kVector3Zero;
    Vector3 angularVelocity = kVector3Zero;

    constexpr BodyState() noexcept = default;
    constexpr BodyState(const Vector3& pos, const Quaternion& rot,
                        const Vector3& linVel, const Vector3& angVel) noexcept
        : position(pos), rotation(rot), linearVelocity(linVel), angularVelocity(angVel) {}

    /// Applies a linear impulse: v += impulse * inverseMass.
    constexpr void applyLinearImpulse(const Vector3& impulse, f32 inverseMass) noexcept {
        linearVelocity += impulse * inverseMass;
    }

    /// Applies an angular impulse: omega += I_inv * angularImpulse.
    constexpr void applyAngularImpulse(const Vector3& angularImpulse, const Matrix3& inverseInertia) noexcept {
        angularVelocity += inverseInertia * angularImpulse;
    }

    /// Returns the velocity of a point offset from center of mass.
    /// v_point = v_cm + omega x r
    [[nodiscard]] constexpr Vector3 velocityAtPoint(const Vector3& offset) const noexcept {
        return linearVelocity + angularVelocity.cross(offset);
    }

    constexpr bool operator==(const BodyState& o) const noexcept {
        return position == o.position && rotation == o.rotation &&
               linearVelocity == o.linearVelocity && angularVelocity == o.angularVelocity;
    }
    constexpr bool operator!=(const BodyState& o) const noexcept { return !(*this == o); }
};

// ── Derived Quantities ───────────────────────────────────────────────────────

/// Computes linear momentum: p = m * v.
[[nodiscard]] constexpr Vector3 linearMomentum(const Vector3& velocity, f32 mass) noexcept {
    return velocity * mass;
}

/// Computes angular momentum: L = I * omega.
[[nodiscard]] constexpr Vector3 angularMomentum(const Vector3& angularVelocity, const Matrix3& inertia) noexcept {
    return inertia * angularVelocity;
}

/// Computes kinetic energy for a particle: KE = 0.5 * m * v^2.
[[nodiscard]] constexpr f32 kineticEnergy(f32 mass, const Vector3& velocity) noexcept {
    return 0.5f * mass * velocity.lengthSq();
}

/// Computes rotational kinetic energy: KE_rot = 0.5 * omega^T * I * omega.
[[nodiscard]] constexpr f32 rotationalKineticEnergy(const Vector3& angularVelocity, const Matrix3& inertia) noexcept {
    return 0.5f * angularVelocity.dot(inertia * angularVelocity);
}

/// Computes total kinetic energy for a rigid body (translational + rotational).
[[nodiscard]] constexpr f32 totalKineticEnergy(f32 mass, const Vector3& velocity,
                                                const Vector3& angularVelocity,
                                                const Matrix3& inertia) noexcept {
    return kineticEnergy(mass, velocity) + rotationalKineticEnergy(angularVelocity, inertia);
}

} // namespace primeon::math
