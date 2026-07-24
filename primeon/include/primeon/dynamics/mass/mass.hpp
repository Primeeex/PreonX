#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/matrix/matrix3.hpp"
#include "primeon/math/vector/vector3.hpp"

namespace primeon::math {

// ── Mass Properties ──────────────────────────────────────────────────────────

/// Represents the inertial mass of an object. Mass must be non-negative.
/// A mass of zero represents an infinite-mass (static/immovable) object.
/// All units in kilograms (kg).
struct MassProperties {
    f32 mass = 1.0f;
    f32 inverseMass = 1.0f;

    constexpr MassProperties() noexcept = default;

    /// Constructs from mass. Clamps to [0, inf]. Zero mass = static object.
    [[nodiscard]] static constexpr MassProperties fromMass(f32 m) noexcept {
        MassProperties mp;
        mp.mass = (m < 0.0f) ? 0.0f : m;
        mp.inverseMass = (mp.mass > kEpsilon) ? (1.0f / mp.mass) : 0.0f;
        return mp;
    }

    /// Constructs from inverse mass directly. Useful for static objects (invMass=0).
    [[nodiscard]] static constexpr MassProperties fromInverseMass(f32 invM) noexcept {
        MassProperties mp;
        mp.inverseMass = (invM < 0.0f) ? 0.0f : invM;
        mp.mass = (mp.inverseMass > kEpsilon) ? (1.0f / mp.inverseMass) : 0.0f;
        return mp;
    }

    /// Returns true if this object is static (infinite mass, zero inverse mass).
    [[nodiscard]] constexpr bool isStatic() const noexcept { return inverseMass <= kEpsilon; }

    /// Returns true if this object is dynamic (finite mass, nonzero inverse mass).
    [[nodiscard]] constexpr bool isDynamic() const noexcept { return inverseMass > kEpsilon; }

    constexpr bool operator==(const MassProperties& o) const noexcept {
        return nearEqual(mass, o.mass);
    }
    constexpr bool operator!=(const MassProperties& o) const noexcept { return !(*this == o); }
};

/// Returns a static (infinite mass) mass properties.
[[nodiscard]] constexpr MassProperties makeStaticMass() noexcept {
    return MassProperties::fromMass(0.0f);
}

/// Returns a dynamic mass properties with the given mass in kg.
[[nodiscard]] constexpr MassProperties makeDynamicMass(f32 mass) noexcept {
    return MassProperties::fromMass(mass);
}

// ── Inertia Tensor Foundation ────────────────────────────────────────────────

/// A 3x3 symmetric positive-definite inertia tensor.
/// Units: kg*m^2. Relates angular velocity to angular momentum: L = I*omega.
/// For diagonal (axis-aligned) bodies, I is a diagonal matrix.
struct InertiaTensor {
    Matrix3 I = Matrix3::identity();

    constexpr InertiaTensor() noexcept = default;
    constexpr explicit InertiaTensor(const Matrix3& matrix) noexcept : I(matrix) {}

    /// Creates a diagonal inertia tensor (axis-aligned box/cylinder).
    [[nodiscard]] static constexpr InertiaTensor diagonal(f32 Ixx, f32 Iyy, f32 Izz) noexcept {
        return InertiaTensor(Matrix3::scale(Ixx, Iyy, Izz));
    }

    /// Creates an inertia tensor for a solid sphere of given mass and radius.
    /// I = (2/5) * m * r^2 * Identity
    [[nodiscard]] static constexpr InertiaTensor solidSphere(f32 mass, f32 radius) noexcept {
        f32 v = 0.4f * mass * radius * radius;
        return diagonal(v, v, v);
    }

    /// Creates an inertia tensor for a hollow sphere of given mass and radius.
    /// I = (2/3) * m * r^2 * Identity
    [[nodiscard]] static constexpr InertiaTensor hollowSphere(f32 mass, f32 radius) noexcept {
        f32 v = (2.0f / 3.0f) * mass * radius * radius;
        return diagonal(v, v, v);
    }

    /// Creates an inertia tensor for a solid box of given mass and dimensions (full extents).
    /// Ixx = (1/12) * m * (dy^2 + dz^2), etc.
    [[nodiscard]] static constexpr InertiaTensor solidBox(f32 mass, const Vector3& halfExtents) noexcept {
        f32 f = mass / 12.0f;
        f32 dx2 = 4.0f * halfExtents.x * halfExtents.x;
        f32 dy2 = 4.0f * halfExtents.y * halfExtents.y;
        f32 dz2 = 4.0f * halfExtents.z * halfExtents.z;
        return diagonal(f * (dy2 + dz2), f * (dx2 + dz2), f * (dx2 + dy2));
    }

    /// Transforms the inertia tensor by a rotation matrix: I' = R * I * R^T.
    [[nodiscard]] InertiaTensor rotated(const Matrix3& rotation) const noexcept {
        return InertiaTensor(rotation * I * rotation.transposed());
    }

    /// Returns the inverse of the inertia tensor. If singular, returns zero tensor.
    [[nodiscard]] InertiaTensor inverse() const noexcept {
        return InertiaTensor(I.inverse());
    }

    /// Applies the tensor to a vector: result = I * v.
    [[nodiscard]] constexpr Vector3 apply(const Vector3& v) const noexcept { return I * v; }

    constexpr bool operator==(const InertiaTensor& o) const noexcept { return I == o.I; }
    constexpr bool operator!=(const InertiaTensor& o) const noexcept { return !(*this == o); }
};

} // namespace primeon::math
