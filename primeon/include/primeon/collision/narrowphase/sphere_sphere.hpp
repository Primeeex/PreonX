#pragma once

#include "primeon/collision/contact.hpp"
#include "primeon/geometry/primitives/sphere.hpp"

namespace primeon::math {

/// Sphere vs Sphere narrowphase test.
/// Returns collision result with contact manifold.
[[nodiscard]] inline CollisionResult sphereSphere(const Sphere& a, const Sphere& b) noexcept {
    CollisionResult result;
    Vector3 diff = a.center - b.center;
    f32 distSq = diff.lengthSq();
    f32 radiusSum = a.radius + b.radius;

    if (distSq >= radiusSum * radiusSum) return result;

    f32 dist = std::sqrt(distSq);
    result.colliding = true;

    if (dist > kEpsilon) {
        result.manifold.normal = diff / dist;
    } else {
        result.manifold.normal = kVector3UnitY;
    }
    result.manifold.contacts[0].point = b.center + result.manifold.normal * b.radius;
    result.manifold.contacts[0].penetration = radiusSum - dist;
    result.manifold.contactCount = 1;
    return result;
}

} // namespace primeon::math
