#pragma once

#include "primeon/collision/contact.hpp"
#include "primeon/geometry/primitives/sphere.hpp"
#include "primeon/geometry/primitives/plane.hpp"

namespace primeon::math {

/// Sphere vs Plane narrowphase test.
/// Plane normal points away from the solid side.
[[nodiscard]] inline CollisionResult spherePlane(const Sphere& s, const Plane& p) noexcept {
    CollisionResult result;
    f32 dist = p.distanceToPoint(s.center);

    if (dist >= s.radius) return result;

    result.colliding = true;
    result.manifold.normal = p.normal;
    result.manifold.contacts[0].point = s.center - p.normal * dist;
    result.manifold.contacts[0].penetration = s.radius - dist;
    result.manifold.contactCount = 1;
    return result;
}

} // namespace primeon::math
