#pragma once

#include "primeon/math/scalar/scalar.hpp"

#include <cmath>

namespace primeon::math {

struct Vector2 {
    f32 x = 0.0f;
    f32 y = 0.0f;

    constexpr Vector2() noexcept = default;
    constexpr Vector2(f32 x, f32 y) noexcept : x(x), y(y) {}
    constexpr explicit Vector2(f32 scalar) noexcept : x(scalar), y(scalar) {}

    constexpr Vector2 operator+(const Vector2& v) const noexcept { return {x + v.x, y + v.y}; }
    constexpr Vector2 operator-(const Vector2& v) const noexcept { return {x - v.x, y - v.y}; }
    constexpr Vector2 operator*(f32 s) const noexcept { return {x * s, y * s}; }
    constexpr Vector2 operator/(f32 s) const noexcept { f32 inv = 1.0f / s; return {x * inv, y * inv}; }
    constexpr Vector2 operator-() const noexcept { return {-x, -y}; }

    constexpr Vector2& operator+=(const Vector2& v) noexcept { x += v.x; y += v.y; return *this; }
    constexpr Vector2& operator-=(const Vector2& v) noexcept { x -= v.x; y -= v.y; return *this; }
    constexpr Vector2& operator*=(f32 s) noexcept { x *= s; y *= s; return *this; }
    constexpr Vector2& operator/=(f32 s) noexcept { f32 inv = 1.0f / s; x *= inv; y *= inv; return *this; }

    constexpr bool operator==(const Vector2& v) const noexcept { return nearEqual(x, v.x) && nearEqual(y, v.y); }
    constexpr bool operator!=(const Vector2& v) const noexcept { return !(*this == v); }

    /// Returns the squared magnitude (avoids sqrt).
    [[nodiscard]] constexpr f32 lengthSq() const noexcept { return x * x + y * y; }

    /// Returns the magnitude (length).
    [[nodiscard]] constexpr f32 length() const noexcept { return std::sqrt(lengthSq()); }

    /// Returns a normalized copy. If length is near zero, returns zero vector.
    [[nodiscard]] Vector2 normalized() const noexcept {
        f32 len = length();
        if (len < kEpsilon) return {0.0f, 0.0f};
        f32 inv = 1.0f / len;
        return {x * inv, y * inv};
    }

    /// Returns true if the vector is near zero length.
    [[nodiscard]] constexpr bool isZero() const noexcept { return lengthSq() <= kEpsilon * kEpsilon; }

    /// Returns the dot product of this and v.
    [[nodiscard]] constexpr f32 dot(const Vector2& v) const noexcept { return x * v.x + y * v.y; }

    /// Returns the 2D cross product (scalar z-component of the 3D cross product).
    [[nodiscard]] constexpr f32 cross(const Vector2& v) const noexcept { return x * v.y - y * v.x; }

    /// Returns the perpendicular vector (-y, x).
    [[nodiscard]] constexpr Vector2 perpendicular() const noexcept { return {-y, x}; }

    /// Returns the distance to vector v.
    [[nodiscard]] constexpr f32 distanceTo(const Vector2& v) const noexcept { return (*this - v).length(); }

    /// Returns the squared distance to vector v.
    [[nodiscard]] constexpr f32 distanceSqTo(const Vector2& v) const noexcept { return (*this - v).lengthSq(); }

    /// Returns the angle in radians between this and v.
    [[nodiscard]] constexpr f32 angleTo(const Vector2& v) const noexcept {
        f32 lenProduct = length() * v.length();
        if (lenProduct < kEpsilon) return 0.0f;
        f32 d = dot(v) / lenProduct;
        d = clamp(d, -1.0f, 1.0f);
        return std::acos(d);
    }

    /// Returns a vector reflected across the given normal.
    [[nodiscard]] Vector2 reflected(const Vector2& normal) const noexcept {
        return *this - normal * (2.0f * dot(normal));
    }

    /// Returns the projection of this vector onto v.
    [[nodiscard]] Vector2 projectedOnto(const Vector2& v) const noexcept {
        f32 lenSq = v.lengthSq();
        if (lenSq < kEpsilon) return {0.0f, 0.0f};
        return v * (dot(v) / lenSq);
    }

    /// Returns the component-wise minimum.
    [[nodiscard]] static constexpr Vector2 min(const Vector2& a, const Vector2& b) noexcept {
        return {math::min(a.x, b.x), math::min(a.y, b.y)};
    }

    /// Returns the component-wise maximum.
    [[nodiscard]] static constexpr Vector2 max(const Vector2& a, const Vector2& b) noexcept {
        return {math::max(a.x, b.x), math::max(a.y, b.y)};
    }

    /// Linear interpolation between a and b.
    [[nodiscard]] static constexpr Vector2 lerp(const Vector2& a, const Vector2& b, f32 t) noexcept {
        return {math::lerp(a.x, b.x, t), math::lerp(a.y, b.y, t)};
    }
};

/// Scalar * Vector.
[[nodiscard]] constexpr Vector2 operator*(f32 s, const Vector2& v) noexcept { return v * s; }

// ── Common axis vectors ──────────────────────────────────────────────────────

inline constexpr Vector2 kVector2Zero = {0.0f, 0.0f};
inline constexpr Vector2 kVector2One = {1.0f, 1.0f};
inline constexpr Vector2 kVector2UnitX = {1.0f, 0.0f};
inline constexpr Vector2 kVector2UnitY = {0.0f, 1.0f};

} // namespace primeon::math
