#pragma once

#include "primeon/math/vector/vector4.hpp"
#include "primeon/math/vector/vector3.hpp"
#include "primeon/math/matrix/matrix3.hpp"

namespace primeon::math {

struct Matrix4 {
    f32 m[4][4] = {};

    constexpr Matrix4() noexcept {
        m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f; m[0][3] = 0.0f;
        m[1][0] = 0.0f; m[1][1] = 1.0f; m[1][2] = 0.0f; m[1][3] = 0.0f;
        m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = 1.0f; m[2][3] = 0.0f;
        m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f;
    }

    [[nodiscard]] static constexpr Matrix4 identity() noexcept { return {}; }

    /// Returns a translation matrix.
    [[nodiscard]] static constexpr Matrix4 translation(const Vector3& t) noexcept {
        Matrix4 r{};
        r.m[0][3] = t.x;
        r.m[1][3] = t.y;
        r.m[2][3] = t.z;
        return r;
    }

    /// Returns a translation matrix from components.
    [[nodiscard]] static constexpr Matrix4 translation(f32 x, f32 y, f32 z) noexcept {
        return translation({x, y, z});
    }

    /// Returns a scale matrix.
    [[nodiscard]] static constexpr Matrix4 scale(const Vector3& s) noexcept {
        Matrix4 r{};
        r.m[0][0] = s.x;
        r.m[1][1] = s.y;
        r.m[2][2] = s.z;
        return r;
    }

    /// Returns a scale matrix from components.
    [[nodiscard]] static constexpr Matrix4 scale(f32 x, f32 y, f32 z) noexcept {
        return scale({x, y, z});
    }

    /// Returns a combined rotation matrix from Euler angles (radians).
    [[nodiscard]] static Matrix4 rotationXYZ(f32 pitch, f32 yaw, f32 roll) noexcept {
        return rotationX(pitch) * rotationY(yaw) * rotationZ(roll);
    }

    /// Returns a rotation matrix around the X axis.
    [[nodiscard]] static Matrix4 rotationX(f32 radians) noexcept {
        Matrix4 r{};
        f32 c = std::cos(radians);
        f32 s = std::sin(radians);
        r.m[1][1] =  c; r.m[1][2] = s;
        r.m[2][1] = -s; r.m[2][2] = c;
        return r;
    }

    /// Returns a rotation matrix around the Y axis.
    [[nodiscard]] static Matrix4 rotationY(f32 radians) noexcept {
        Matrix4 r{};
        f32 c = std::cos(radians);
        f32 s = std::sin(radians);
        r.m[0][0] =  c; r.m[0][2] = -s;
        r.m[2][0] =  s; r.m[2][2] =  c;
        return r;
    }

    /// Returns a rotation matrix around the Z axis.
    [[nodiscard]] static Matrix4 rotationZ(f32 radians) noexcept {
        Matrix4 r{};
        f32 c = std::cos(radians);
        f32 s = std::sin(radians);
        r.m[0][0] =  c; r.m[0][1] = s;
        r.m[1][0] = -s; r.m[1][1] = c;
        return r;
    }

    /// Returns the transpose.
    [[nodiscard]] constexpr Matrix4 transposed() const noexcept {
        Matrix4 r{};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                r.m[i][j] = m[j][i];
        return r;
    }

    /// Returns the determinant.
    [[nodiscard]] constexpr f32 determinant() const noexcept {
        return m[0][0] * (m[1][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2])
                        - m[1][2] * (m[2][1] * m[3][3] - m[2][3] * m[3][1])
                        + m[1][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]))
             - m[0][1] * (m[1][0] * (m[2][2] * m[3][3] - m[2][3] * m[3][2])
                        - m[1][2] * (m[2][0] * m[3][3] - m[2][3] * m[3][0])
                        + m[1][3] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]))
             + m[0][2] * (m[1][0] * (m[2][1] * m[3][3] - m[2][3] * m[3][1])
                        - m[1][1] * (m[2][0] * m[3][3] - m[2][3] * m[3][0])
                        + m[1][3] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]))
             - m[0][3] * (m[1][0] * (m[2][1] * m[3][2] - m[2][2] * m[3][1])
                        - m[1][1] * (m[2][0] * m[3][2] - m[2][2] * m[3][0])
                        + m[1][2] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]));
    }

    /// Returns the inverse. If singular, returns identity.
    [[nodiscard]] Matrix4 inverse() const noexcept {
        f32 det = determinant();
        if (abs(det) < kEpsilon) return identity();
        f32 inv = 1.0f / det;

        Matrix4 r{};
        r.m[0][0] =  (m[1][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2])
                     - m[1][2] * (m[2][1] * m[3][3] - m[2][3] * m[3][1])
                     + m[1][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1])) * inv;
        r.m[0][1] = -(m[0][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2])
                     - m[0][2] * (m[2][1] * m[3][3] - m[2][3] * m[3][1])
                     + m[0][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1])) * inv;
        r.m[0][2] =  (m[0][1] * (m[1][2] * m[3][3] - m[1][3] * m[3][2])
                     - m[0][2] * (m[1][1] * m[3][3] - m[1][3] * m[3][1])
                     + m[0][3] * (m[1][1] * m[3][2] - m[1][2] * m[3][1])) * inv;
        r.m[0][3] = -(m[0][1] * (m[1][2] * m[2][3] - m[1][3] * m[2][2])
                     - m[0][2] * (m[1][1] * m[2][3] - m[1][3] * m[2][1])
                     + m[0][3] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])) * inv;

        r.m[1][0] = -(m[1][0] * (m[2][2] * m[3][3] - m[2][3] * m[3][2])
                     - m[1][2] * (m[2][0] * m[3][3] - m[2][3] * m[3][0])
                     + m[1][3] * (m[2][0] * m[3][2] - m[2][2] * m[3][0])) * inv;
        r.m[1][1] =  (m[0][0] * (m[2][2] * m[3][3] - m[2][3] * m[3][2])
                     - m[0][2] * (m[2][0] * m[3][3] - m[2][3] * m[3][0])
                     + m[0][3] * (m[2][0] * m[3][2] - m[2][2] * m[3][0])) * inv;
        r.m[1][2] = -(m[0][0] * (m[1][2] * m[3][3] - m[1][3] * m[3][2])
                     - m[0][2] * (m[1][0] * m[3][3] - m[1][3] * m[3][0])
                     + m[0][3] * (m[1][0] * m[3][2] - m[1][2] * m[3][0])) * inv;
        r.m[1][3] =  (m[0][0] * (m[1][2] * m[2][3] - m[1][3] * m[2][2])
                     - m[0][2] * (m[1][0] * m[2][3] - m[1][3] * m[2][0])
                     + m[0][3] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])) * inv;

        r.m[2][0] =  (m[1][0] * (m[2][1] * m[3][3] - m[2][3] * m[3][1])
                     - m[1][1] * (m[2][0] * m[3][3] - m[2][3] * m[3][0])
                     + m[1][3] * (m[2][0] * m[3][1] - m[2][1] * m[3][0])) * inv;
        r.m[2][1] = -(m[0][0] * (m[2][1] * m[3][3] - m[2][3] * m[3][1])
                     - m[0][1] * (m[2][0] * m[3][3] - m[2][3] * m[3][0])
                     + m[0][3] * (m[2][0] * m[3][1] - m[2][1] * m[3][0])) * inv;
        r.m[2][2] =  (m[0][0] * (m[1][1] * m[3][3] - m[1][3] * m[3][1])
                     - m[0][1] * (m[1][0] * m[3][3] - m[1][3] * m[3][0])
                     + m[0][3] * (m[1][0] * m[3][1] - m[1][1] * m[3][0])) * inv;
        r.m[2][3] = -(m[0][0] * (m[1][1] * m[2][3] - m[1][3] * m[2][1])
                     - m[0][1] * (m[1][0] * m[2][3] - m[1][3] * m[2][0])
                     + m[0][3] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) * inv;

        r.m[3][0] = -(m[1][0] * (m[2][1] * m[3][2] - m[2][2] * m[3][1])
                     - m[1][1] * (m[2][0] * m[3][2] - m[2][2] * m[3][0])
                     + m[1][2] * (m[2][0] * m[3][1] - m[2][1] * m[3][0])) * inv;
        r.m[3][1] =  (m[0][0] * (m[2][1] * m[3][2] - m[2][2] * m[3][1])
                     - m[0][1] * (m[2][0] * m[3][2] - m[2][2] * m[3][0])
                     + m[0][2] * (m[2][0] * m[3][1] - m[2][1] * m[3][0])) * inv;
        r.m[3][2] = -(m[0][0] * (m[1][1] * m[3][2] - m[1][2] * m[3][1])
                     - m[0][1] * (m[1][0] * m[3][2] - m[1][2] * m[3][0])
                     + m[0][2] * (m[1][0] * m[3][1] - m[1][1] * m[3][0])) * inv;
        r.m[3][3] =  (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                     - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                     + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) * inv;

        return r;
    }

    /// Matrix * Matrix multiplication.
    [[nodiscard]] constexpr Matrix4 operator*(const Matrix4& o) const noexcept {
        Matrix4 r{};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                r.m[i][j] = 0.0f;
                for (int k = 0; k < 4; ++k)
                    r.m[i][j] += m[i][k] * o.m[k][j];
            }
        return r;
    }

    /// Transforms a Vector4.
    [[nodiscard]] constexpr Vector4 operator*(const Vector4& v) const noexcept {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * v.w,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * v.w,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * v.w,
                m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3] * v.w};
    }

    /// Transforms a point (w=1).
    [[nodiscard]] Vector3 transformPoint(const Vector3& p) const noexcept {
        Vector4 h(p.x, p.y, p.z, 1.0f);
        Vector4 r = (*this) * h;
        f32 invW = (abs(r.w) < kEpsilon) ? 1.0f : 1.0f / r.w;
        return {r.x * invW, r.y * invW, r.z * invW};
    }

    /// Transforms a direction (w=0).
    [[nodiscard]] constexpr Vector3 transformDirection(const Vector3& d) const noexcept {
        return {m[0][0] * d.x + m[0][1] * d.y + m[0][2] * d.z,
                m[1][0] * d.x + m[1][1] * d.y + m[1][2] * d.z,
                m[2][0] * d.x + m[2][1] * d.y + m[2][2] * d.z};
    }

    /// Returns the upper-left 3x3 submatrix.
    [[nodiscard]] constexpr Matrix3 upper3x3() const noexcept {
        return {m[0][0], m[0][1], m[0][2],
                m[1][0], m[1][1], m[1][2],
                m[2][0], m[2][1], m[2][2]};
    }

    /// Returns the translation component.
    [[nodiscard]] constexpr Vector3 translation() const noexcept {
        return {m[0][3], m[1][3], m[2][3]};
    }

    constexpr bool operator==(const Matrix4& o) const noexcept {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                if (!nearEqual(m[i][j], o.m[i][j])) return false;
        return true;
    }

    constexpr bool operator!=(const Matrix4& o) const noexcept { return !(*this == o); }

    /// Component access by (row, col).
    [[nodiscard]] constexpr f32& operator()(int row, int col) noexcept { return m[row][col]; }
    [[nodiscard]] constexpr const f32& operator()(int row, int col) const noexcept { return m[row][col]; }
};

} // namespace primeon::math
