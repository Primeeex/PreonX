#pragma once

#include "primeon/math/vector/vector3.hpp"

namespace primeon::math {

struct Sphere {
    Vector3 center = kVector3Zero;
    f32 radius = 1.0f;

    constexpr Sphere() noexcept = default;
    constexpr Sphere(const Vector3& center, f32 radius) noexcept : center(center), radius(radius) {}

    /// Returns the squared radius (avoids sqrt).
    [[nodiscard]] constexpr f32 radiusSq() const noexcept { return radius * radius; }

    /// Returns true if the point is inside or on the surface of the sphere.
    [[nodiscard]] constexpr bool containsPoint(const Vector3& point) const noexcept {
        return center.distanceSqTo(point) <= radiusSq();
    }
};

} // namespace primeon::math
