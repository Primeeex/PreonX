#pragma once

#include "primeon/math/vector/vector3.hpp"

namespace primeon::math {

struct Matrix3 {
    f32 m[3][3] = {};

    constexpr Matrix3() noexcept {
        m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f;
        m[1][0] = 0.0f; m[1][1] = 1.0f; m[1][2] = 0.0f;
        m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = 1.0f;
    }

    constexpr Matrix3(f32 m00, f32 m01, f32 m02,
                      f32 m10, f32 m11, f32 m12,
                      f32 m20, f32 m21, f32 m22) noexcept {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
    }

    /// Construct from three column vectors.
    constexpr Matrix3(const Vector3& col0, const Vector3& col1, const Vector3& col2) noexcept {
        m[0][0] = col0.x; m[0][1] = col1.x; m[0][2] = col2.x;
        m[1][0] = col0.y; m[1][1] = col1.y; m[1][2] = col2.y;
        m[2][0] = col0.z; m[2][1] = col1.z; m[2][2] = col2.z;
    }

    [[nodiscard]] static constexpr Matrix3 identity() noexcept { return {}; }

    /// Returns a rotation matrix around the X axis.
    [[nodiscard]] static Matrix3 rotationX(f32 radians) noexcept {
        f32 c = std::cos(radians);
        f32 s = std::sin(radians);
        return {1.0f, 0.0f, 0.0f,
                0.0f,   c,   s,
                0.0f,  -s,   c};
    }

    /// Returns a rotation matrix around the Y axis.
    [[nodiscard]] static Matrix3 rotationY(f32 radians) noexcept {
        f32 c = std::cos(radians);
        f32 s = std::sin(radians);
        return { c, 0.0f, -s,
                0.0f, 1.0f, 0.0f,
                  s, 0.0f,   c};
    }

    /// Returns a rotation matrix around the Z axis.
    [[nodiscard]] static Matrix3 rotationZ(f32 radians) noexcept {
        f32 c = std::cos(radians);
        f32 s = std::sin(radians);
        return {  c,   s, 0.0f,
                 -s,   c, 0.0f,
                0.0f, 0.0f, 1.0f};
    }

    /// Returns a scale matrix.
    [[nodiscard]] static constexpr Matrix3 scale(f32 sx, f32 sy, f32 sz) noexcept {
        return {sx, 0.0f, 0.0f,
                0.0f, sy, 0.0f,
                0.0f, 0.0f, sz};
    }

    /// Returns the transpose.
    [[nodiscard]] constexpr Matrix3 transposed() const noexcept {
        return {m[0][0], m[1][0], m[2][0],
                m[0][1], m[1][1], m[2][1],
                m[0][2], m[1][2], m[2][2]};
    }

    /// Returns the determinant.
    [[nodiscard]] constexpr f32 determinant() const noexcept {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    /// Returns the inverse. If singular, returns identity.
    [[nodiscard]] Matrix3 inverse() const noexcept {
        f32 det = determinant();
        if (abs(det) < kEpsilon) return identity();
        f32 inv = 1.0f / det;

        return {
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv,
            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv,
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv
        };
    }

    /// Matrix * Matrix multiplication.
    [[nodiscard]] constexpr Matrix3 operator*(const Matrix3& o) const noexcept {
        Matrix3 result{};
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) {
                result.m[r][c] = 0.0f;
                for (int k = 0; k < 3; ++k)
                    result.m[r][c] += m[r][k] * o.m[k][c];
            }
        return result;
    }

    /// Matrix * Vector transformation.
    [[nodiscard]] constexpr Vector3 operator*(const Vector3& v) const noexcept {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }

    constexpr bool operator==(const Matrix3& o) const noexcept {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                if (!nearEqual(m[r][c], o.m[r][c])) return false;
        return true;
    }

    constexpr bool operator!=(const Matrix3& o) const noexcept { return !(*this == o); }

    /// Component access by (row, col).
    [[nodiscard]] constexpr f32& operator()(int row, int col) noexcept { return m[row][col]; }
    [[nodiscard]] constexpr const f32& operator()(int row, int col) const noexcept { return m[row][col]; }

    /// Returns a column vector.
    [[nodiscard]] constexpr Vector3 column(foundation::u32 i) const noexcept {
        return {m[0][i], m[1][i], m[2][i]};
    }

    /// Returns a row vector.
    [[nodiscard]] constexpr Vector3 row(foundation::u32 i) const noexcept {
        return {m[i][0], m[i][1], m[i][2]};
    }
};

} // namespace primeon::math
