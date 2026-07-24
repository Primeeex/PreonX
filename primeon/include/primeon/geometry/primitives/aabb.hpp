#pragma once

#include "primeon/math/vector/vector3.hpp"

namespace primeon::math {

struct AABB {
    Vector3 min = kVector3Zero;
    Vector3 max = kVector3One;

    constexpr AABB() noexcept = default;
    constexpr AABB(const Vector3& min, const Vector3& max) noexcept : min(min), max(max) {}

    /// Constructs from center and half-extents.
    [[nodiscard]] static AABB fromCenterHalfExtents(const Vector3& center, const Vector3& halfExtents) noexcept {
        return {center - halfExtents, center + halfExtents};
    }

    /// Returns the center of the AABB.
    [[nodiscard]] constexpr Vector3 center() const noexcept { return (min + max) * 0.5f; }

    /// Returns the half-extents.
    [[nodiscard]] constexpr Vector3 halfExtents() const noexcept { return (max - min) * 0.5f; }

    /// Returns the size (full extents).
    [[nodiscard]] constexpr Vector3 size() const noexcept { return max - min; }

    /// Returns the surface area.
    [[nodiscard]] f32 surfaceArea() const noexcept {
        Vector3 s = size();
        return 2.0f * (s.x * s.y + s.y * s.z + s.z * s.x);
    }

    /// Returns the volume.
    [[nodiscard]] constexpr f32 volume() const noexcept {
        Vector3 s = size();
        return s.x * s.y * s.z;
    }

    /// Returns true if the point is inside or on the surface.
    [[nodiscard]] constexpr bool containsPoint(const Vector3& point) const noexcept {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }

    /// Returns the closest point on the AABB to the given point.
    [[nodiscard]] constexpr Vector3 closestPoint(const Vector3& point) const noexcept {
        return {clamp(point.x, min.x, max.x),
                clamp(point.y, min.y, max.y),
                clamp(point.z, min.z, max.z)};
    }

    /// Expands this AABB to include the given point.
    void expandToInclude(const Vector3& point) noexcept {
        min = Vector3::min(min, point);
        max = Vector3::max(max, point);
    }

    /// Expands this AABB to include another AABB.
    void expandToInclude(const AABB& other) noexcept {
        min = Vector3::min(min, other.min);
        max = Vector3::max(max, other.max);
    }

    /// Returns an AABB that includes both this and other.
    [[nodiscard]] AABB merged(const AABB& other) const noexcept {
        return {Vector3::min(min, other.min), Vector3::max(max, other.max)};
    }
};

} // namespace primeon::math
