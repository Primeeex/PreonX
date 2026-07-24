#pragma once

#include "primeon/math/vector/vector3.hpp"

namespace primeon::math {

struct Segment {
    Vector3 start = kVector3Zero;
    Vector3 end = kVector3UnitZ;

    constexpr Segment() noexcept = default;
    constexpr Segment(const Vector3& start, const Vector3& end) noexcept
        : start(start), end(end) {}

    /// Returns the direction vector (not normalized).
    [[nodiscard]] constexpr Vector3 direction() const noexcept { return end - start; }

    /// Returns the length of the segment.
    [[nodiscard]] constexpr f32 length() const noexcept { return direction().length(); }

    /// Returns the squared length of the segment.
    [[nodiscard]] constexpr f32 lengthSq() const noexcept { return direction().lengthSq(); }

    /// Returns the midpoint.
    [[nodiscard]] constexpr Vector3 midpoint() const noexcept { return (start + end) * 0.5f; }

    /// Returns the point at parameter t in [0, 1].
    [[nodiscard]] constexpr Vector3 pointAt(f32 t) const noexcept {
        return start + direction() * t;
    }
};

} // namespace primeon::math
