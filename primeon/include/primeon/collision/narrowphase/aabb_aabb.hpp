#pragma once

#include "primeon/collision/contact.hpp"
#include "primeon/geometry/primitives/aabb.hpp"

namespace primeon::math {

/// AABB vs AABB narrowphase test (interval overlap on 3 axes).
[[nodiscard]] inline CollisionResult aabbAABB(const AABB& a, const AABB& b) noexcept {
    CollisionResult result;

    f32 overlapX = min(a.max.x, b.max.x) - max(a.min.x, b.min.x);
    f32 overlapY = min(a.max.y, b.max.y) - max(a.min.y, b.min.y);
    f32 overlapZ = min(a.max.z, b.max.z) - max(a.min.z, b.min.z);

    if (overlapX <= 0.0f || overlapY <= 0.0f || overlapZ <= 0.0f) return result;

    result.colliding = true;

    Vector3 centers = a.center() - b.center();
    if (overlapX <= overlapY && overlapX <= overlapZ) {
        result.manifold.normal = (centers.x >= 0.0f) ? kVector3UnitX : Vector3(-1.0f, 0.0f, 0.0f);
        result.manifold.contacts[0].penetration = overlapX;
    } else if (overlapY <= overlapZ) {
        result.manifold.normal = (centers.y >= 0.0f) ? kVector3UnitY : Vector3(0.0f, -1.0f, 0.0f);
        result.manifold.contacts[0].penetration = overlapY;
    } else {
        result.manifold.normal = (centers.z >= 0.0f) ? kVector3UnitZ : Vector3(0.0f, 0.0f, -1.0f);
        result.manifold.contacts[0].penetration = overlapZ;
    }

    result.manifold.contacts[0].point = (a.center() + b.center()) * 0.5f;
    result.manifold.contactCount = 1;
    return result;
}

} // namespace primeon::math
