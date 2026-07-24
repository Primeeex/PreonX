#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/vector/vector3.hpp"
#include "primeon/math/quaternion/quaternion.hpp"
#include "primeon/geometry/primitives/obb.hpp"

namespace primeon::math {

// ── SAT (Separating Axis Theorem) ────────────────────────────────────────────
//
// Two convex shapes do NOT overlap iff there exists an axis along which their
// projections don't overlap. For OBB-OBB, 15 candidate axes are tested:
//   3 from A's face normals
//   3 from B's face normals
//   9 from edge A × edge B cross products
//
// For each axis, the projection of each OBB is:
//   extent = |axesA[0]·axis|*heA.x + |axesA[1]·axis|*heA.y + |axesA[2]·axis|*heA.z
//
// The overlap is: radiusA + radiusB - |centerDifference·axis|
//
// The axis with minimum positive overlap gives the contact normal and depth.

/// Projects an OBB onto an axis and returns the half-extent of the projection.
[[nodiscard]] constexpr f32 projectOBBExtent(const OBB& o, const Vector3 axes[3],
                                              const Vector3& axis) noexcept {
    return abs(axes[0].dot(axis)) * o.halfExtents.x
         + abs(axes[1].dot(axis)) * o.halfExtents.y
         + abs(axes[2].dot(axis)) * o.halfExtents.z;
}

/// Tests one SAT axis for OBB-OBB overlap. Returns the overlap distance
/// (negative = separating). This does not account for center offset.
[[nodiscard]] inline f32 satOverlapOBB(const OBB& a, const Vector3& axis,
                                         const Vector3 axesA[3],
                                         const OBB& b,
                                         const Vector3 axesB[3]) noexcept {
    f32 ra = projectOBBExtent(a, axesA, axis);
    f32 rb = projectOBBExtent(b, axesB, axis);
    Vector3 d = b.center - a.center;
    f32 dist = abs(d.dot(axis));
    return ra + rb - dist;
}

/// Tests all 15 SAT axes for OBB-OBB. Returns overlap on the minimum axis
/// and sets outNormal to the separating axis direction.
/// Returns negative if separating.
[[nodiscard]] inline f32 satOBBFull(const OBB& a, const OBB& b, Vector3& outNormal) noexcept {
    Vector3 axesA[3] = {a.orientation.right(), a.orientation.up(), a.orientation.forward()};
    Vector3 axesB[3] = {b.orientation.right(), b.orientation.up(), b.orientation.forward()};

    f32 minOverlap = std::numeric_limits<f32>::max();
    Vector3 minAxis = kVector3Zero;

    auto testAxis = [&](const Vector3& axis) -> bool {
        f32 lenSq = axis.lengthSq();
        if (lenSq < kEpsilon) return true;
        Vector3 n = axis / std::sqrt(lenSq);
        f32 overlap = satOverlapOBB(a, n, axesA, b, axesB);
        if (overlap <= 0.0f) { outNormal = n; return false; }
        if (overlap < minOverlap) { minOverlap = overlap; minAxis = n; }
        return true;
    };

    for (int i = 0; i < 3; ++i)
        if (!testAxis(axesA[i])) return -1.0f;
    for (int i = 0; i < 3; ++i)
        if (!testAxis(axesB[i])) return -1.0f;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (!testAxis(axesA[i].cross(axesB[j]))) return -1.0f;

    outNormal = minAxis;
    return minOverlap;
}

} // namespace primeon::math
