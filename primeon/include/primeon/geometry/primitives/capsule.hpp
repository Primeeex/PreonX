#pragma once

#include "primeon/math/vector/vector3.hpp"

namespace primeon::math {

struct Capsule {
    Vector3 start = kVector3Zero;
    Vector3 end = kVector3UnitY;
    f32 radius = 0.5f;

    constexpr Capsule() noexcept = default;
    constexpr Capsule(const Vector3& start, const Vector3& end, f32 radius) noexcept
        : start(start), end(end), radius(radius) {}

    /// Returns the axis (from start to end).
    [[nodiscard]] constexpr Vector3 axis() const noexcept { return end - start; }

    /// Returns the length of the capsule's central segment.
    [[nodiscard]] constexpr f32 length() const noexcept { return axis().length(); }

    /// Returns the height (total including end caps).
    [[nodiscard]] constexpr f32 height() const noexcept { return length() + 2.0f * radius; }

    /// Returns the midpoint of the capsule's central segment.
    [[nodiscard]] constexpr Vector3 midpoint() const noexcept { return (start + end) * 0.5f; }

    /// Returns the squared radius.
    [[nodiscard]] constexpr f32 radiusSq() const noexcept { return radius * radius; }
};

} // namespace primeon::math
