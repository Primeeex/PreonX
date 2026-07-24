#pragma once

#include "primeon/math/vector/vector3.hpp"

namespace primeon::math {

struct Line {
    Vector3 origin = kVector3Zero;
    Vector3 direction = kVector3UnitZ;

    constexpr Line() noexcept = default;
    constexpr Line(const Vector3& origin, const Vector3& direction) noexcept
        : origin(origin), direction(direction.normalized()) {}
};

} // namespace primeon::math
