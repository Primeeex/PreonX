#pragma once

#include "primeon/math/vector/vector2.hpp"

namespace primeon::math {

struct Matrix2 {
    f32 m[2][2] = {};

    constexpr Matrix2() noexcept {
        m[0][0] = 1.0f; m[0][1] = 0.0f;
        m[1][0] = 0.0f; m[1][1] = 1.0f;
    }

    constexpr Matrix2(f32 m00, f32 m01, f32 m10, f32 m11) noexcept {
        m[0][0] = m00; m[0][1] = m01;
        m[1][0] = m10; m[1][1] = m11;
    }

    /// Returns the identity matrix.
    [[nodiscard]] static constexpr Matrix2 identity() noexcept { return {}; }

    /// Returns a rotation matrix for the given angle in radians.
    [[nodiscard]] static Matrix2 rotation(f32 radians) noexcept {
        f32 c = std::cos(radians);
        f32 s = std::sin(radians);
        return {c, -s, s, c};
    }

    /// Returns a scale matrix.
    [[nodiscard]] static constexpr Matrix2 scale(f32 sx, f32 sy) noexcept {
        return {sx, 0.0f, 0.0f, sy};
    }

    /// Returns the transpose.
    [[nodiscard]] constexpr Matrix2 transposed() const noexcept {
        return {m[0][0], m[1][0], m[0][1], m[1][1]};
    }

    /// Returns the determinant.
    [[nodiscard]] constexpr f32 determinant() const noexcept {
        return m[0][0] * m[1][1] - m[0][1] * m[1][0];
    }

    /// Returns the inverse. If singular, returns identity.
    [[nodiscard]] Matrix2 inverse() const noexcept {
        f32 det = determinant();
        if (abs(det) < kEpsilon) return identity();
        f32 inv = 1.0f / det;
        return {m[1][1] * inv, -m[0][1] * inv, -m[1][0] * inv, m[0][0] * inv};
    }

    /// Matrix * Matrix multiplication.
    [[nodiscard]] constexpr Matrix2 operator*(const Matrix2& o) const noexcept {
        return {
            m[0][0] * o.m[0][0] + m[0][1] * o.m[1][0],
            m[0][0] * o.m[0][1] + m[0][1] * o.m[1][1],
            m[1][0] * o.m[0][0] + m[1][1] * o.m[1][0],
            m[1][0] * o.m[0][1] + m[1][1] * o.m[1][1]
        };
    }

    /// Matrix * Vector transformation.
    [[nodiscard]] constexpr Vector2 operator*(const Vector2& v) const noexcept {
        return {m[0][0] * v.x + m[0][1] * v.y, m[1][0] * v.x + m[1][1] * v.y};
    }

    constexpr bool operator==(const Matrix2& o) const noexcept {
        for (int r = 0; r < 2; ++r)
            for (int c = 0; c < 2; ++c)
                if (!nearEqual(m[r][c], o.m[r][c])) return false;
        return true;
    }

    constexpr bool operator!=(const Matrix2& o) const noexcept { return !(*this == o); }

    /// Component access by (row, col).
    [[nodiscard]] constexpr f32& operator()(int row, int col) noexcept { return m[row][col]; }
    [[nodiscard]] constexpr const f32& operator()(int row, int col) const noexcept { return m[row][col]; }
};

} // namespace primeon::math
