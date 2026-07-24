#pragma once

#include "primeon/collision/contact.hpp"
#include "primeon/geometry/primitives/sphere.hpp"
#include "primeon/geometry/primitives/aabb.hpp"

namespace primeon::math {

/// Sphere vs AABB narrowphase test.
[[nodiscard]] inline CollisionResult sphereAABB(const Sphere& s, const AABB& a) noexcept {
    CollisionResult result;
    Vector3 closest = a.closestPoint(s.center);
    Vector3 diff = s.center - closest;
    f32 distSq = diff.lengthSq();

    if (distSq >= s.radiusSq()) return result;

    f32 dist = std::sqrt(distSq);
    result.colliding = true;

    if (dist > kEpsilon) {
        result.manifold.normal = diff / dist;
    } else {
        Vector3 center = a.center();
        Vector3 d = s.center - center;
        Vector3 he = a.halfExtents();
        f32 nx = (d.x >= 0.0f) ? 1.0f : -1.0f;
        f32 ny = (d.y >= 0.0f) ? 1.0f : -1.0f;
        f32 nz = (d.z >= 0.0f) ? 1.0f : -1.0f;
        f32 px = he.x - abs(d.x);
        f32 py = he.y - abs(d.y);
        f32 pz = he.z - abs(d.z);
        if (px < py && px < pz) result.manifold.normal = Vector3(nx, 0.0f, 0.0f);
        else if (py < pz) result.manifold.normal = Vector3(0.0f, ny, 0.0f);
        else result.manifold.normal = Vector3(0.0f, 0.0f, nz);
        dist = s.radius;
    }

    result.manifold.contacts[0].point = closest;
    result.manifold.contacts[0].penetration = s.radius - dist;
    result.manifold.contactCount = 1;
    return result;
}

} // namespace primeon::math
