#pragma once

#include "primeon/collision/contact.hpp"
#include "primeon/geometry/primitives/obb.hpp"
#include "primeon/collision/narrowphase/aabb_aabb.hpp"
#include "primeon/collision/algorithms/sat.hpp"

namespace primeon::math {

/// OBB vs OBB narrowphase test using SAT with 15 axes.
/// Axes: 3 from A, 3 from B, 9 edge cross products.
[[nodiscard]] inline CollisionResult obbOBB(const OBB& a, const OBB& b) noexcept {
    CollisionResult result;

    Vector3 axesA[3] = {a.orientation.right(), a.orientation.up(), a.orientation.forward()};
    Vector3 axesB[3] = {b.orientation.right(), b.orientation.up(), b.orientation.forward()};
    Vector3 d = b.center - a.center;

    f32 minOverlap = std::numeric_limits<f32>::max();
    Vector3 minAxis = kVector3Zero;

    auto testAxis = [&](const Vector3& axis) -> bool {
        f32 lenSq = axis.lengthSq();
        if (lenSq < kEpsilon) return true;
        Vector3 n = axis / std::sqrt(lenSq);
        f32 overlap = satOverlapOBB(a, n, axesA, b, axesB);
        if (overlap <= 0.0f) return false;
        if (overlap < minOverlap) {
            minOverlap = overlap;
            minAxis = n;
        }
        return true;
    };

    for (int i = 0; i < 3; ++i)
        if (!testAxis(axesA[i])) return result;
    for (int i = 0; i < 3; ++i)
        if (!testAxis(axesB[i])) return result;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (!testAxis(axesA[i].cross(axesB[j]))) return result;

    if (d.dot(minAxis) < 0.0f) minAxis = -minAxis;

    result.colliding = true;
    result.manifold.normal = minAxis;
    result.manifold.contacts[0].point = (a.center + b.center) * 0.5f;
    result.manifold.contacts[0].penetration = minOverlap;
    result.manifold.contactCount = 1;
    return result;
}

} // namespace primeon::math
