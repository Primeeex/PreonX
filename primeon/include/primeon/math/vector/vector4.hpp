#pragma once

#include "primeon/math/scalar/scalar.hpp"

#include <cmath>

namespace primeon::math {

struct Vector4 {
    f32 x = 0.0f;
    f32 y = 0.0f;
    f32 z = 0.0f;
    f32 w = 0.0f;

    constexpr Vector4() noexcept = default;
    constexpr Vector4(f32 x, f32 y, f32 z, f32 w) noexcept : x(x), y(y), z(z), w(w) {}
    constexpr explicit Vector4(f32 scalar) noexcept : x(scalar), y(scalar), z(scalar), w(scalar) {}

    constexpr Vector4 operator+(const Vector4& v) const noexcept { return {x + v.x, y + v.y, z + v.z, w + v.w}; }
    constexpr Vector4 operator-(const Vector4& v) const noexcept { return {x - v.x, y - v.y, z - v.z, w - v.w}; }
    constexpr Vector4 operator*(f32 s) const noexcept { return {x * s, y * s, z * s, w * s}; }
    constexpr Vector4 operator/(f32 s) const noexcept { f32 inv = 1.0f / s; return {x * inv, y * inv, z * inv, w * inv}; }
    constexpr Vector4 operator-() const noexcept { return {-x, -y, -z, -w}; }

    constexpr Vector4& operator+=(const Vector4& v) noexcept { x += v.x; y += v.y; z += v.z; w += v.w; return *this; }
    constexpr Vector4& operator-=(const Vector4& v) noexcept { x -= v.x; y -= v.y; z -= v.z; w -= v.w; return *this; }
    constexpr Vector4& operator*=(f32 s) noexcept { x *= s; y *= s; z *= s; w *= s; return *this; }
    constexpr Vector4& operator/=(f32 s) noexcept { f32 inv = 1.0f / s; x *= inv; y *= inv; z *= inv; w *= inv; return *this; }

    constexpr bool operator==(const Vector4& v) const noexcept { return nearEqual(x, v.x) && nearEqual(y, v.y) && nearEqual(z, v.z) && nearEqual(w, v.w); }
    constexpr bool operator!=(const Vector4& v) const noexcept { return !(*this == v); }

    /// Returns the squared magnitude (avoids sqrt).
    [[nodiscard]] constexpr f32 lengthSq() const noexcept { return x * x + y * y + z * z + w * w; }

    /// Returns the magnitude (length).
    [[nodiscard]] constexpr f32 length() const noexcept { return std::sqrt(lengthSq()); }

    /// Returns a normalized copy. If length is near zero, returns zero vector.
    [[nodiscard]] Vector4 normalized() const noexcept {
        f32 len = length();
        if (len < kEpsilon) return {0.0f, 0.0f, 0.0f, 0.0f};
        f32 inv = 1.0f / len;
        return {x * inv, y * inv, z * inv, w * inv};
    }

    /// Returns true if the vector is near zero length.
    [[nodiscard]] constexpr bool isZero() const noexcept { return lengthSq() <= kEpsilon * kEpsilon; }

    /// Returns the dot product of this and v.
    [[nodiscard]] constexpr f32 dot(const Vector4& v) const noexcept { return x * v.x + y * v.y + z * v.z + w * v.w; }

    /// Returns the distance to vector v.
    [[nodiscard]] constexpr f32 distanceTo(const Vector4& v) const noexcept { return (*this - v).length(); }

    /// Returns the squared distance to vector v.
    [[nodiscard]] constexpr f32 distanceSqTo(const Vector4& v) const noexcept { return (*this - v).lengthSq(); }
};

/// Scalar * Vector.
[[nodiscard]] constexpr Vector4 operator*(f32 s, const Vector4& v) noexcept { return v * s; }

} // namespace primeon::math
