#pragma once

#include "primeon/math/vector/vector3.hpp"
#include "primeon/math/matrix/matrix3.hpp"
#include "primeon/math/matrix/matrix4.hpp"

#include <cmath>

namespace primeon::math {

struct Quaternion {
    f32 x = 0.0f;
    f32 y = 0.0f;
    f32 z = 0.0f;
    f32 w = 1.0f;

    constexpr Quaternion() noexcept = default;
    constexpr Quaternion(f32 x, f32 y, f32 z, f32 w) noexcept : x(x), y(y), z(z), w(w) {}

    /// Returns the identity quaternion (no rotation).
    [[nodiscard]] static constexpr Quaternion identity() noexcept { return {0.0f, 0.0f, 0.0f, 1.0f}; }

    /// Constructs from axis and angle (radians).
    [[nodiscard]] static Quaternion fromAxisAngle(const Vector3& axis, f32 radians) noexcept {
        Vector3 n = axis.normalized();
        f32 half = radians * 0.5f;
        f32 s = std::sin(half);
        f32 c = std::cos(half);
        return {n.x * s, n.y * s, n.z * s, c};
    }

    /// Constructs from Euler angles (radians): pitch (X), yaw (Y), roll (Z).
    [[nodiscard]] static Quaternion fromEuler(f32 pitch, f32 yaw, f32 roll) noexcept {
        f32 cp = std::cos(pitch * 0.5f);
        f32 sp = std::sin(pitch * 0.5f);
        f32 cy = std::cos(yaw * 0.5f);
        f32 sy = std::sin(yaw * 0.5f);
        f32 cr = std::cos(roll * 0.5f);
        f32 sr = std::sin(roll * 0.5f);
        return {sp * cy * cr - cp * sy * sr,
                cp * sy * cr + sp * cy * sr,
                cp * cy * sr - sp * sy * cr,
                cp * cy * cr + sp * sy * sr};
    }

    /// Constructs a rotation that looks from 'from' towards 'to', with 'up' hint.
    [[nodiscard]] static Quaternion lookRotation(const Vector3& forward, const Vector3& up) noexcept {
        Vector3 f = forward.normalized();
        Vector3 r = f.cross(up).normalized();
        Vector3 u = r.cross(f);

        Matrix3 m(r.x, u.x, f.x,
                  r.y, u.y, f.y,
                  r.z, u.z, f.z);
        return fromMatrix(m);
    }

    /// Constructs from a 3x3 rotation matrix.
    [[nodiscard]] static Quaternion fromMatrix(const Matrix3& m) noexcept {
        f32 trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
        Quaternion q;

        if (trace > 0.0f) {
            f32 s = 0.5f / std::sqrt(trace + 1.0f);
            q.w = 0.25f / s;
            q.x = (m.m[2][1] - m.m[1][2]) * s;
            q.y = (m.m[0][2] - m.m[2][0]) * s;
            q.z = (m.m[1][0] - m.m[0][1]) * s;
        } else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
            f32 s = 2.0f * std::sqrt(1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2]);
            q.w = (m.m[2][1] - m.m[1][2]) / s;
            q.x = 0.25f * s;
            q.y = (m.m[0][1] + m.m[1][0]) / s;
            q.z = (m.m[0][2] + m.m[2][0]) / s;
        } else if (m.m[1][1] > m.m[2][2]) {
            f32 s = 2.0f * std::sqrt(1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2]);
            q.w = (m.m[0][2] - m.m[2][0]) / s;
            q.x = (m.m[0][1] + m.m[1][0]) / s;
            q.y = 0.25f * s;
            q.z = (m.m[1][2] + m.m[2][1]) / s;
        } else {
            f32 s = 2.0f * std::sqrt(1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1]);
            q.w = (m.m[1][0] - m.m[0][1]) / s;
            q.x = (m.m[0][2] + m.m[2][0]) / s;
            q.y = (m.m[1][2] + m.m[2][1]) / s;
            q.z = 0.25f * s;
        }
        return q;
    }

    /// Returns the squared length.
    [[nodiscard]] constexpr f32 lengthSq() const noexcept { return x * x + y * y + z * z + w * w; }

    /// Returns the length.
    [[nodiscard]] constexpr f32 length() const noexcept { return std::sqrt(lengthSq()); }

    /// Returns a normalized copy. If near-zero, returns identity.
    [[nodiscard]] Quaternion normalized() const noexcept {
        f32 len = length();
        if (len < kEpsilon) return identity();
        f32 inv = 1.0f / len;
        return {x * inv, y * inv, z * inv, w * inv};
    }

    /// Returns the conjugate (negates imaginary parts).
    [[nodiscard]] constexpr Quaternion conjugate() const noexcept { return {-x, -y, -z, w}; }

    /// Returns the inverse. Assumes unit quaternion; otherwise normalizes first.
    [[nodiscard]] Quaternion inverse() const noexcept { return conjugate().normalized(); }

    /// Returns the dot product.
    [[nodiscard]] constexpr f32 dot(const Quaternion& q) const noexcept {
        return x * q.x + y * q.y + z * q.z + w * q.w;
    }

    /// Rotates a vector by this quaternion.
    [[nodiscard]] Vector3 rotate(const Vector3& v) const noexcept {
        Vector3 qv(x, y, z);
        Vector3 uv = qv.cross(v);
        Vector3 uuv = qv.cross(uv);
        return v + (uv * w + uuv) * 2.0f;
    }

    /// Extracts the axis and angle (radians) of rotation.
    void toAxisAngle(Vector3& outAxis, f32& outAngle) const noexcept {
        Quaternion q = normalized();
        if (q.w < 0.0f) q = {-q.x, -q.y, -q.z, -q.w};
        outAngle = 2.0f * std::acos(clamp(q.w, -1.0f, 1.0f));
        f32 s = std::sqrt(1.0f - q.w * q.w);
        if (s < kEpsilon) {
            outAxis = kVector3UnitX;
        } else {
            outAxis = {q.x / s, q.y / s, q.z / s};
        }
    }

    /// Extracts Euler angles (radians): pitch (X), yaw (Y), roll (Z).
    Vector3 toEuler() const noexcept {
        Quaternion q = normalized();
        f32 sinr = 2.0f * (q.w * q.x + q.y * q.z);
        f32 cosr = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
        f32 roll = std::atan2(sinr, cosr);

        f32 sinp = 2.0f * (q.w * q.y - q.z * q.x);
        f32 pitch;
        if (std::abs(sinp) >= 1.0f) {
            pitch = std::copysign(kHalfPi, sinp);
        } else {
            pitch = std::asin(sinp);
        }

        f32 siny = 2.0f * (q.w * q.z + q.x * q.y);
        f32 cosy = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
        f32 yaw = std::atan2(siny, cosy);

        return {roll, pitch, yaw};
    }

    /// Returns the forward direction (Z+) after rotation.
    [[nodiscard]] Vector3 forward() const noexcept { return rotate(kVector3UnitZ); }

    /// Returns the up direction (Y+) after rotation.
    [[nodiscard]] Vector3 up() const noexcept { return rotate(kVector3UnitY); }

    /// Returns the right direction (X+) after rotation.
    [[nodiscard]] Vector3 right() const noexcept { return rotate(kVector3UnitX); }

    /// Converts to a 3x3 rotation matrix.
    [[nodiscard]] Matrix3 toMatrix3() const noexcept {
        Quaternion q = normalized();
        f32 xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        f32 xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
        f32 wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
        return {1.0f - 2.0f * (yy + zz), 2.0f * (xy - wz),       2.0f * (xz + wy),
                2.0f * (xy + wz),       1.0f - 2.0f * (xx + zz), 2.0f * (yz - wx),
                2.0f * (xz - wy),       2.0f * (yz + wx),       1.0f - 2.0f * (xx + yy)};
    }

    /// Converts to a 4x4 rotation matrix.
    [[nodiscard]] Matrix4 toMatrix4() const noexcept {
        Matrix3 m3 = toMatrix3();
        Matrix4 m{};
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                m.m[r][c] = m3.m[r][c];
        return m;
    }

    // ── Operators ──────────────────────────────────────────────────────────────

    /// Quaternion * Quaternion (composition).
    constexpr Quaternion operator*(const Quaternion& q) const noexcept {
        return {w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x,
                w * q.z + x * q.y - y * q.x + z * q.w,
                w * q.w - x * q.x - y * q.y - z * q.z};
    }

    constexpr Quaternion& operator*=(const Quaternion& q) noexcept { *this = *this * q; return *this; }

    constexpr Quaternion operator-() const noexcept { return {-x, -y, -z, -w}; }

    constexpr Quaternion operator+(const Quaternion& q) const noexcept {
        return {x + q.x, y + q.y, z + q.z, w + q.w};
    }

    constexpr Quaternion operator-(const Quaternion& q) const noexcept {
        return {x - q.x, y - q.y, z - q.z, w - q.w};
    }

    constexpr bool operator==(const Quaternion& q) const noexcept {
        return nearEqual(x, q.x) && nearEqual(y, q.y) && nearEqual(z, q.z) && nearEqual(w, q.w);
    }

    constexpr bool operator!=(const Quaternion& q) const noexcept { return !(*this == q); }
};

// ── Free functions (declared after struct so they can reference member operators) ─

/// Quaternion * scalar.
[[nodiscard]] constexpr Quaternion operator*(const Quaternion& q, f32 s) noexcept {
    return {q.x * s, q.y * s, q.z * s, q.w * s};
}

/// Scalar * Quaternion.
[[nodiscard]] constexpr Quaternion operator*(f32 s, const Quaternion& q) noexcept {
    return {s * q.x, s * q.y, s * q.z, s * q.w};
}

/// Spherical linear interpolation between a and b.
/// t=0 returns a, t=1 returns b.
[[nodiscard]] inline Quaternion slerp(const Quaternion& a, const Quaternion& b, f32 t) noexcept {
    Quaternion q1 = a.normalized();
    Quaternion q2 = b.normalized();
    f32 d = q1.dot(q2);

    if (d < 0.0f) {
        q2 = -q2;
        d = -d;
    }

    if (d > 0.9995f) {
        return (q1 + q2 * t - q1 * t).normalized();
    }

    f32 theta = std::acos(clamp(d, -1.0f, 1.0f));
    f32 sinTheta = std::sin(theta);
    f32 wa = std::sin((1.0f - t) * theta) / sinTheta;
    f32 wb = std::sin(t * theta) / sinTheta;
    return {wa * q1.x + wb * q2.x, wa * q1.y + wb * q2.y,
            wa * q1.z + wb * q2.z, wa * q1.w + wb * q2.w};
}

/// Member-style slerp helper.
[[nodiscard]] inline Quaternion slerpTo(const Quaternion& from, const Quaternion& to, f32 t) noexcept {
    return slerp(from, to, t);
}

} // namespace primeon::math
