#pragma once

#include "foundation/core/types.hpp"

#include <cmath>
#include <limits>

namespace primeon::math {

using foundation::f32;
using foundation::u32;

// ── Mathematical Constants ───────────────────────────────────────────────────

inline constexpr f32 kPi = 3.14159265358979323846f;
inline constexpr f32 kTwoPi = 2.0f * kPi;
inline constexpr f32 kHalfPi = 0.5f * kPi;
inline constexpr f32 kQuarterPi = 0.25f * kPi;
inline constexpr f32 kInvPi = 1.0f / kPi;
inline constexpr f32 kSqrt2 = 1.41421356237309504880f;
inline constexpr f32 kSqrt3 = 1.73205080756887729352f;
inline constexpr f32 kInvSqrt2 = 1.0f / kSqrt2;
inline constexpr f32 kInvSqrt3 = 1.0f / kSqrt3;

inline constexpr f32 kDegToRad = kPi / 180.0f;
inline constexpr f32 kRadToDeg = 180.0f / kPi;

// ── Tolerance Constants ──────────────────────────────────────────────────────

inline constexpr f32 kEpsilon = 1e-6f;
inline constexpr f32 kLargeEpsilon = 1e-4f;
inline constexpr f32 kVerySmall = 1e-20f;

// ── Scalar Operations ────────────────────────────────────────────────────────

/// Returns the absolute value of x.
[[nodiscard]] constexpr f32 abs(f32 x) noexcept {
    return std::fabs(x);
}

/// Returns the square of x.
[[nodiscard]] constexpr f32 square(f32 x) noexcept {
    return x * x;
}

/// Returns the cube of x.
[[nodiscard]] constexpr f32 cube(f32 x) noexcept {
    return x * x * x;
}

/// Returns the sign of x: -1, 0, or +1.
[[nodiscard]] constexpr f32 sign(f32 x) noexcept {
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

/// Returns the minimum of a and b.
[[nodiscard]] constexpr f32 min(f32 a, f32 b) noexcept {
    return (a < b) ? a : b;
}

/// Returns the maximum of a and b.
[[nodiscard]] constexpr f32 max(f32 a, f32 b) noexcept {
    return (a > b) ? a : b;
}

/// Returns x clamped to [lo, hi].
[[nodiscard]] constexpr f32 clamp(f32 x, f32 lo, f32 hi) noexcept {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

/// Returns x clamped to [0, 1].
[[nodiscard]] constexpr f32 saturate(f32 x) noexcept {
    return clamp(x, 0.0f, 1.0f);
}

/// Linear interpolation: returns (1-t)*a + t*b.
[[nodiscard]] constexpr f32 lerp(f32 a, f32 b, f32 t) noexcept {
    return a + t * (b - a);
}

/// Returns the maximum of abs(a) and abs(b).
[[nodiscard]] constexpr f32 maxAbs(f32 a, f32 b) noexcept {
    f32 aa = abs(a);
    f32 bb = abs(b);
    return (aa > bb) ? aa : bb;
}

/// Returns true if |a - b| <= epsilon.
[[nodiscard]] constexpr bool nearEqual(f32 a, f32 b, f32 epsilon = kEpsilon) noexcept {
    return abs(a - b) <= epsilon;
}

/// Returns true if |x| <= epsilon.
[[nodiscard]] constexpr bool nearZero(f32 x, f32 epsilon = kEpsilon) noexcept {
    return abs(x) <= epsilon;
}

/// Converts degrees to radians.
[[nodiscard]] constexpr f32 degToRad(f32 degrees) noexcept {
    return degrees * kDegToRad;
}

/// Converts radians to degrees.
[[nodiscard]] constexpr f32 radToDeg(f32 radians) noexcept {
    return radians * kRadToDeg;
}

/// Returns the next power of two >= x. Returns 1 for x=0.
[[nodiscard]] constexpr foundation::u32 nextPowerOfTwo(foundation::u32 x) noexcept {
    if (x == 0) return 1;
    x--;
    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;
    x++;
    return x;
}

/// Returns true if x is a power of two.
[[nodiscard]] constexpr bool isPowerOfTwo(foundation::u32 x) noexcept {
    return x != 0 && (x & (x - 1)) == 0;
}

/// Returns x modulo n, always positive.
[[nodiscard]] constexpr f32 wrap(f32 x, f32 lo, f32 hi) noexcept {
    f32 range = hi - lo;
    return lo + (x - lo) - range * std::floor((x - lo) / range);
}

/// Smoothstep interpolation (Hermite).
[[nodiscard]] constexpr f32 smoothstep(f32 edge0, f32 edge1, f32 x) noexcept {
    f32 t = clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

} // namespace primeon::math
