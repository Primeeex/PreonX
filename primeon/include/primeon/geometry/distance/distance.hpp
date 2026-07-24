#pragma once

#include "primeon/math/vector/vector3.hpp"
#include "primeon/math/scalar/scalar.hpp"
#include "primeon/geometry/primitives/ray.hpp"
#include "primeon/geometry/primitives/segment.hpp"
#include "primeon/geometry/primitives/plane.hpp"
#include "primeon/geometry/primitives/sphere.hpp"
#include "primeon/geometry/primitives/aabb.hpp"
#include "primeon/geometry/primitives/capsule.hpp"

namespace primeon::math::dist {

/// Returns the squared distance between two points.
[[nodiscard]] constexpr f32 pointPointSq(const Vector3& a, const Vector3& b) noexcept {
    return a.distanceSqTo(b);
}

/// Returns the distance between two points.
[[nodiscard]] constexpr f32 pointPoint(const Vector3& a, const Vector3& b) noexcept {
    return a.distanceTo(b);
}

/// Returns the squared distance from a point to a line.
[[nodiscard]] f32 pointLineSq(const Vector3& point, const Vector3& lineOrigin, const Vector3& lineDir) noexcept {
    Vector3 v = point - lineOrigin;
    f32 t = v.dot(lineDir);
    Vector3 closest = lineOrigin + lineDir * t;
    return point.distanceSqTo(closest);
}

/// Returns the squared distance from a point to a segment.
[[nodiscard]] f32 pointSegmentSq(const Vector3& point, const Vector3& segStart, const Vector3& segEnd) noexcept {
    Vector3 ab = segEnd - segStart;
    Vector3 ap = point - segStart;
    f32 abLenSq = ab.lengthSq();
    if (abLenSq < kEpsilon) return ap.lengthSq();
    f32 t = clamp(ap.dot(ab) / abLenSq, 0.0f, 1.0f);
    Vector3 closest = segStart + ab * t;
    return point.distanceSqTo(closest);
}

/// Returns the signed distance from a point to a plane.
[[nodiscard]] constexpr f32 pointPlane(const Vector3& point, const Plane& plane) noexcept {
    return plane.distanceToPoint(point);
}

/// Returns the squared distance from a point to a sphere.
[[nodiscard]] constexpr f32 pointSphereSq(const Vector3& point, const Sphere& sphere) noexcept {
    f32 d = point.distanceTo(sphere.center);
    f32 diff = d - sphere.radius;
    return (diff > 0.0f) ? diff * diff : 0.0f;
}

/// Returns the squared distance from a point to an AABB.
[[nodiscard]] constexpr f32 pointAABBSq(const Vector3& point, const AABB& aabb) noexcept {
    Vector3 closest = aabb.closestPoint(point);
    return point.distanceSqTo(closest);
}

/// Returns the squared distance from a point to a capsule (segment + radius).
[[nodiscard]] f32 pointCapsuleSq(const Vector3& point, const Capsule& capsule) noexcept {
    f32 segDistSq = pointSegmentSq(point, capsule.start, capsule.end);
    f32 segDist = std::sqrt(segDistSq);
    f32 diff = segDist - capsule.radius;
    return (diff > 0.0f) ? diff * diff : 0.0f;
}

} // namespace primeon::math::dist
