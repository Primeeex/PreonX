#pragma once

#include "primeon/math/scalar/scalar.hpp"

#include <cmath>

namespace primeon::math {

struct Vector3 {
    f32 x = 0.0f;
    f32 y = 0.0f;
    f32 z = 0.0f;

    constexpr Vector3() noexcept = default;
    constexpr Vector3(f32 x, f32 y, f32 z) noexcept : x(x), y(y), z(z) {}
    constexpr explicit Vector3(f32 scalar) noexcept : x(scalar), y(scalar), z(scalar) {}

    constexpr Vector3 operator+(const Vector3& v) const noexcept { return {x + v.x, y + v.y, z + v.z}; }
    constexpr Vector3 operator-(const Vector3& v) const noexcept { return {x - v.x, y - v.y, z - v.z}; }
    constexpr Vector3 operator*(f32 s) const noexcept { return {x * s, y * s, z * s}; }
    constexpr Vector3 operator/(f32 s) const noexcept { f32 inv = 1.0f / s; return {x * inv, y * inv, z * inv}; }
    constexpr Vector3 operator-() const noexcept { return {-x, -y, -z}; }

    constexpr Vector3& operator+=(const Vector3& v) noexcept { x += v.x; y += v.y; z += v.z; return *this; }
    constexpr Vector3& operator-=(const Vector3& v) noexcept { x -= v.x; y -= v.y; z -= v.z; return *this; }
    constexpr Vector3& operator*=(f32 s) noexcept { x *= s; y *= s; z *= s; return *this; }
    constexpr Vector3& operator/=(f32 s) noexcept { f32 inv = 1.0f / s; x *= inv; y *= inv; z *= inv; return *this; }

    constexpr bool operator==(const Vector3& v) const noexcept { return nearEqual(x, v.x) && nearEqual(y, v.y) && nearEqual(z, v.z); }
    constexpr bool operator!=(const Vector3& v) const noexcept { return !(*this == v); }

    /// Component access by index: 0=x, 1=y, 2=z.
    [[nodiscard]] constexpr f32 operator[](foundation::u32 i) const noexcept {
        return (i == 0) ? x : ((i == 1) ? y : z);
    }

    /// Returns the squared magnitude (avoids sqrt).
    [[nodiscard]] constexpr f32 lengthSq() const noexcept { return x * x + y * y + z * z; }

    /// Returns the magnitude (length).
    [[nodiscard]] constexpr f32 length() const noexcept { return std::sqrt(lengthSq()); }

    /// Returns a normalized copy. If length is near zero, returns zero vector.
    [[nodiscard]] Vector3 normalized() const noexcept {
        f32 len = length();
        if (len < kEpsilon) return {0.0f, 0.0f, 0.0f};
        f32 inv = 1.0f / len;
        return {x * inv, y * inv, z * inv};
    }

    /// Returns true if the vector is near zero length.
    [[nodiscard]] constexpr bool isZero() const noexcept { return lengthSq() <= kEpsilon * kEpsilon; }

    /// Returns the dot product of this and v.
    [[nodiscard]] constexpr f32 dot(const Vector3& v) const noexcept { return x * v.x + y * v.y + z * v.z; }

    /// Returns the cross product of this and v.
    [[nodiscard]] constexpr Vector3 cross(const Vector3& v) const noexcept {
        return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x};
    }

    /// Returns the distance to vector v.
    [[nodiscard]] constexpr f32 distanceTo(const Vector3& v) const noexcept { return (*this - v).length(); }

    /// Returns the squared distance to vector v.
    [[nodiscard]] constexpr f32 distanceSqTo(const Vector3& v) const noexcept { return (*this - v).lengthSq(); }

    /// Returns the angle in radians between this and v.
    [[nodiscard]] constexpr f32 angleTo(const Vector3& v) const noexcept {
        f32 lenProduct = length() * v.length();
        if (lenProduct < kEpsilon) return 0.0f;
        f32 d = dot(v) / lenProduct;
        d = clamp(d, -1.0f, 1.0f);
        return std::acos(d);
    }

    /// Returns a vector reflected across the given normal (assumed normalized).
    [[nodiscard]] constexpr Vector3 reflected(const Vector3& normal) const noexcept {
        return *this - normal * (2.0f * dot(normal));
    }

    /// Returns the projection of this vector onto v.
    [[nodiscard]] Vector3 projectedOnto(const Vector3& v) const noexcept {
        f32 lenSq = v.lengthSq();
        if (lenSq < kEpsilon) return {0.0f, 0.0f, 0.0f};
        return v * (dot(v) / lenSq);
    }

    /// Returns the component-wise multiply of this and v.
    [[nodiscard]] constexpr Vector3 cmul(const Vector3& v) const noexcept { return {x * v.x, y * v.y, z * v.z}; }

    /// Returns the component-wise minimum.
    [[nodiscard]] static constexpr Vector3 min(const Vector3& a, const Vector3& b) noexcept {
        return {math::min(a.x, b.x), math::min(a.y, b.y), math::min(a.z, b.z)};
    }

    /// Returns the component-wise maximum.
    [[nodiscard]] static constexpr Vector3 max(const Vector3& a, const Vector3& b) noexcept {
        return {math::max(a.x, b.x), math::max(a.y, b.y), math::max(a.z, b.z)};
    }

    /// Linear interpolation between a and b.
    [[nodiscard]] static constexpr Vector3 lerp(const Vector3& a, const Vector3& b, f32 t) noexcept {
        return {math::lerp(a.x, b.x, t), math::lerp(a.y, b.y, t), math::lerp(a.z, b.z, t)};
    }

    /// Returns the absolute value of each component.
    [[nodiscard]] constexpr Vector3 abs() const noexcept {
        return {math::abs(x), math::abs(y), math::abs(z)};
    }
};

/// Scalar * Vector.
[[nodiscard]] constexpr Vector3 operator*(f32 s, const Vector3& v) noexcept { return v * s; }

// ── Common axis vectors ──────────────────────────────────────────────────────

inline constexpr Vector3 kVector3Zero = {0.0f, 0.0f, 0.0f};
inline constexpr Vector3 kVector3One = {1.0f, 1.0f, 1.0f};
inline constexpr Vector3 kVector3UnitX = {1.0f, 0.0f, 0.0f};
inline constexpr Vector3 kVector3UnitY = {0.0f, 1.0f, 0.0f};
inline constexpr Vector3 kVector3UnitZ = {0.0f, 0.0f, 1.0f};

} // namespace primeon::math
