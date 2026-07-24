#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/vector/vector3.hpp"
#include "primeon/math/quaternion/quaternion.hpp"
#include "primeon/geometry/primitives/sphere.hpp"
#include "primeon/geometry/primitives/aabb.hpp"
#include "primeon/geometry/primitives/obb.hpp"
#include "primeon/geometry/primitives/capsule.hpp"
#include "primeon/geometry/primitives/plane.hpp"
#include "primeon/geometry/primitives/triangle.hpp"

namespace primeon::math {

// ── Support Mappings ─────────────────────────────────────────────────────────
//
// Each support function returns the point on the shape farthest in direction d.
// These are used by GJK and EPA to query the Minkowski difference implicitly:
//   support_{A⊖B}(d) = support_A(d) - support_B(-d)

/// Support point of a sphere: center + radius * normalize(d).
[[nodiscard]] inline Vector3 supportSphere(const Sphere& s, const Vector3& d) noexcept {
    f32 len = d.length();
    if (len < kEpsilon) return s.center;
    return s.center + d * (s.radius / len);
}

/// Support point of an AABB: the corner most in direction d.
[[nodiscard]] inline Vector3 supportAABB(const AABB& a, const Vector3& d) noexcept {
    return Vector3(
        (d.x >= 0.0f) ? a.max.x : a.min.x,
        (d.y >= 0.0f) ? a.max.y : a.min.y,
        (d.z >= 0.0f) ? a.max.z : a.min.z
    );
}

/// Support point of an OBB: transform d to local space, find AABB support, transform back.
[[nodiscard]] inline Vector3 supportOBB(const OBB& o, const Vector3& d) noexcept {
    Quaternion qInv = o.orientation.conjugate();
    Vector3 localD = qInv.rotate(d);
    Vector3 localSupport(
        (localD.x >= 0.0f) ? o.halfExtents.x : -o.halfExtents.x,
        (localD.y >= 0.0f) ? o.halfExtents.y : -o.halfExtents.y,
        (localD.z >= 0.0f) ? o.halfExtents.z : -o.halfExtents.z
    );
    return o.center + o.orientation.rotate(localSupport);
}

/// Support point of a capsule: support on the sphere centered at the farthest endpoint.
[[nodiscard]] inline Vector3 supportCapsule(const Capsule& c, const Vector3& d) noexcept {
    f32 len = d.length();
    if (len < kEpsilon) return c.start;
    Vector3 dir = d / len;
    f32 dotStart = c.start.dot(dir);
    f32 dotEnd = c.end.dot(dir);
    Vector3 base = (dotEnd >= dotStart) ? c.end : c.start;
    return base + dir * c.radius;
}

/// Support point of a triangle: the vertex farthest in direction d.
[[nodiscard]] inline Vector3 supportTriangle(const Triangle& t, const Vector3& d) noexcept {
    f32 da = t.a.dot(d);
    f32 db = t.b.dot(d);
    f32 dc = t.c.dot(d);
    if (da >= db && da >= dc) return t.a;
    if (db >= dc) return t.b;
    return t.c;
}

/// Support point of a convex hull: the vertex farthest in direction d.
/// vertices must be non-empty.
[[nodiscard]] inline Vector3 supportConvexHull(const Vector3* vertices, u32 count, const Vector3& d) noexcept {
    u32 best = 0;
    f32 bestDot = vertices[0].dot(d);
    for (u32 i = 1; i < count; ++i) {
        f32 dot = vertices[i].dot(d);
        if (dot > bestDot) {
            bestDot = dot;
            best = i;
        }
    }
    return vertices[best];
}

// ── Generic Support Dispatch ──────────────────────────────────────────────────

/// Dispatches support to the correct shape function via overload.
[[nodiscard]] inline Vector3 supportPrimitive(const Sphere& s, const Vector3& d) noexcept {
    return supportSphere(s, d);
}
[[nodiscard]] inline Vector3 supportPrimitive(const AABB& a, const Vector3& d) noexcept {
    return supportAABB(a, d);
}
[[nodiscard]] inline Vector3 supportPrimitive(const OBB& o, const Vector3& d) noexcept {
    return supportOBB(o, d);
}
[[nodiscard]] inline Vector3 supportPrimitive(const Capsule& c, const Vector3& d) noexcept {
    return supportCapsule(c, d);
}
[[nodiscard]] inline Vector3 supportPrimitive(const Triangle& t, const Vector3& d) noexcept {
    return supportTriangle(t, d);
}

// ── Minkowski Difference Support ─────────────────────────────────────────────

/// Support of the Minkowski difference A ⊖ B in direction d.
/// s_{A⊖B}(d) = s_A(d) - s_B(-d).
template <typename FuncA, typename FuncB>
[[nodiscard]] inline Vector3 supportMinkowski(
    FuncA supportA, FuncB supportB, const Vector3& d) noexcept
{
    return supportA(d) - supportB(-d);
}

} // namespace primeon::math
