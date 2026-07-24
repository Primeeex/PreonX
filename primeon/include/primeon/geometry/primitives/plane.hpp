#pragma once

#include "primeon/math/vector/vector3.hpp"

namespace primeon::math {

struct Plane {
    Vector3 normal = kVector3UnitY;
    f32 distance = 0.0f;

    constexpr Plane() noexcept = default;
    Plane(const Vector3& normal, f32 distance) noexcept
        : normal(normal.normalized()), distance(distance) {}

    /// Constructs from a normal and a point on the plane.
    static Plane fromPointNormal(const Vector3& point, const Vector3& normal) noexcept {
        Vector3 n = normal.normalized();
        return {n, n.dot(point)};
    }

    /// Constructs from three points (counter-clockwise winding).
    static Plane fromThreePoints(const Vector3& a, const Vector3& b, const Vector3& c) noexcept {
        Vector3 n = (b - a).cross(c - a).normalized();
        return {n, n.dot(a)};
    }

    /// Returns the signed distance from a point to this plane.
    [[nodiscard]] constexpr f32 distanceToPoint(const Vector3& point) const noexcept {
        return normal.dot(point) - distance;
    }

    /// Returns the closest point on the plane to the given point.
    [[nodiscard]] Vector3 closestPoint(const Vector3& point) const noexcept {
        return point - normal * distanceToPoint(point);
    }
};

} // namespace primeon::math
