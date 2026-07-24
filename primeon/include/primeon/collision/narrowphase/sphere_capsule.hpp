#pragma once

#include "primeon/collision/contact.hpp"
#include "primeon/geometry/primitives/sphere.hpp"
#include "primeon/geometry/primitives/capsule.hpp"

namespace primeon::math {

/// Closest point on capsule segment to a given point.
[[nodiscard]] inline Vector3 closestPointOnSegment(const Vector3& p,
                                                    const Vector3& a, const Vector3& b) noexcept {
    Vector3 ab = b - a;
    f32 lenSq = ab.lengthSq();
    if (lenSq < kEpsilon) return a;
    f32 t = clamp((p - a).dot(ab) / lenSq, 0.0f, 1.0f);
    return a + ab * t;
}

/// Sphere vs Capsule narrowphase test.
[[nodiscard]] inline CollisionResult sphereCapsule(const Sphere& s, const Capsule& c) noexcept {
    CollisionResult result;
    Vector3 closest = closestPointOnSegment(s.center, c.start, c.end);
    Vector3 diff = s.center - closest;
    f32 distSq = diff.lengthSq();
    f32 radiusSum = s.radius + c.radius;

    if (distSq >= radiusSum * radiusSum) return result;

    f32 dist = std::sqrt(distSq);
    result.colliding = true;

    if (dist > kEpsilon) {
        result.manifold.normal = diff / dist;
    } else {
        Vector3 axis = c.axis();
        f32 axisLen = axis.length();
        if (axisLen > kEpsilon) result.manifold.normal = axis / axisLen;
        else result.manifold.normal = kVector3UnitY;
    }
    result.manifold.contacts[0].point = closest + result.manifold.normal * c.radius;
    result.manifold.contacts[0].penetration = radiusSum - dist;
    result.manifold.contactCount = 1;
    return result;
}

} // namespace primeon::math
