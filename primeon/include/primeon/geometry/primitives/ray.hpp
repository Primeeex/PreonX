#pragma once

#include "primeon/math/vector/vector3.hpp"

namespace primeon::math {

struct Ray {
    Vector3 origin = kVector3Zero;
    Vector3 direction = kVector3UnitZ;

    constexpr Ray() noexcept = default;
    Ray(const Vector3& origin, const Vector3& direction) noexcept
        : origin(origin), direction(direction.normalized()) {}

    /// Returns the point at parameter t along the ray.
    [[nodiscard]] constexpr Vector3 pointAt(f32 t) const noexcept {
        return origin + direction * t;
    }
};

} // namespace primeon::math
