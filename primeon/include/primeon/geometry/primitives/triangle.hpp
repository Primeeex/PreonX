#pragma once

#include "primeon/math/vector/vector3.hpp"

namespace primeon::math {

struct Triangle {
    Vector3 a = kVector3Zero;
    Vector3 b = kVector3UnitX;
    Vector3 c = kVector3UnitY;

    constexpr Triangle() noexcept = default;
    constexpr Triangle(const Vector3& a, const Vector3& b, const Vector3& c) noexcept
        : a(a), b(b), c(c) {}

    /// Returns the normal of the triangle (assumes counter-clockwise winding).
    [[nodiscard]] Vector3 normal() const noexcept {
        return (b - a).cross(c - a).normalized();
    }

    /// Returns the area of the triangle.
    [[nodiscard]] f32 area() const noexcept {
        return 0.5f * (b - a).cross(c - a).length();
    }

    /// Returns the centroid (center of mass).
    [[nodiscard]] constexpr Vector3 centroid() const noexcept {
        return (a + b + c) * (1.0f / 3.0f);
    }

    /// Returns a point given barycentric coordinates (u, v, w where u+v+w=1).
    [[nodiscard]] constexpr Vector3 pointAtBarycentric(f32 u, f32 v, f32 w) const noexcept {
        return a * u + b * v + c * w;
    }
};

} // namespace primeon::math
