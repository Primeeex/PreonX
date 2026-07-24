#pragma once

#include "primeon/collision/contact.hpp"
#include "primeon/collision/shapes/support.hpp"
#include "primeon/collision/narrowphase/sphere_sphere.hpp"
#include "primeon/collision/narrowphase/sphere_plane.hpp"
#include "primeon/collision/narrowphase/sphere_aabb.hpp"
#include "primeon/collision/narrowphase/sphere_capsule.hpp"
#include "primeon/collision/narrowphase/aabb_aabb.hpp"
#include "primeon/collision/narrowphase/capsule_capsule.hpp"
#include "primeon/collision/narrowphase/obb_obb.hpp"
#include "primeon/collision/narrowphase/raycast.hpp"
#include "primeon/collision/algorithms/gjk.hpp"
#include "primeon/collision/algorithms/epa.hpp"
#include "primeon/geometry/primitives/sphere.hpp"
#include "primeon/geometry/primitives/aabb.hpp"
#include "primeon/geometry/primitives/obb.hpp"
#include "primeon/geometry/primitives/capsule.hpp"
#include "primeon/geometry/primitives/triangle.hpp"
#include "primeon/geometry/primitives/plane.hpp"
#include "primeon/geometry/primitives/ray.hpp"
#include "primeon/math/vector/vector3.hpp"

namespace primeon::collision {

// ── Unified Collision Query ──────────────────────────────────────────────────
//
// Stateless dispatch functions. All queries are free functions that operate
// on geometry primitives directly.
//
// Supported query types:
//   - intersects(A, B)        → boolean overlap test
//   - contact(A, B)           → CollisionResult
//   - raycast(ray, A)         → RayHit
//
// All narrowphase functions live in primeon::math; we alias them here.

// ── Sphere queries ───────────────────────────────────────────────────────────

[[nodiscard]] inline bool intersects(const math::Sphere& a, const math::Sphere& b) noexcept {
    return math::sphereSphere(a, b).colliding;
}

[[nodiscard]] inline bool intersects(const math::Sphere& s, const math::AABB& b) noexcept {
    return math::sphereAABB(s, b).colliding;
}

[[nodiscard]] inline bool intersects(const math::Sphere& s, const math::Plane& p) noexcept {
    return math::spherePlane(s, p).colliding;
}

[[nodiscard]] inline bool intersects(const math::Sphere& s, const math::Capsule& c) noexcept {
    return math::sphereCapsule(s, c).colliding;
}

[[nodiscard]] inline bool intersects(const math::Sphere& s, const math::OBB& o) noexcept {
    math::Vector3 local = o.orientation.conjugate().rotate(s.center - o.center);
    math::AABB localBox{-o.halfExtents, o.halfExtents};
    math::Sphere localSphere{local, s.radius};
    return math::sphereAABB(localSphere, localBox).colliding;
}

// ── AABB queries ─────────────────────────────────────────────────────────────

[[nodiscard]] inline bool intersects(const math::AABB& a, const math::AABB& b) noexcept {
    return math::aabbAABB(a, b).colliding;
}

[[nodiscard]] inline bool intersects(const math::AABB& a, const math::Plane& p) noexcept {
    math::Vector3 center = (a.min + a.max) * 0.5f;
    math::Vector3 half = (a.max - a.min) * 0.5f;
    f32 r = std::abs(p.normal.dot(half));
    return math::spherePlane(math::Sphere{center, r}, p).colliding;
}

[[nodiscard]] inline bool intersects(const math::AABB& a, const math::Capsule& c) noexcept {
    math::Vector3 closest = math::closestPointOnSegment(c.start, c.end, a.center());
    return math::sphereAABB(math::Sphere{closest, c.radius}, a).colliding;
}

// ── OBB queries ──────────────────────────────────────────────────────────────

[[nodiscard]] inline bool intersects(const math::OBB& a, const math::OBB& b) noexcept {
    return math::obbOBB(a, b).colliding;
}

[[nodiscard]] inline bool intersects(const math::OBB& o, const math::Plane& p) noexcept {
    math::Vector3 axes[3] = {o.orientation.right(), o.orientation.up(), o.orientation.forward()};
    f32 r = 0.0f;
    for (int i = 0; i < 3; ++i)
        r += std::abs(p.normal.dot(axes[i])) * ((&o.halfExtents.x)[i]);
    return math::spherePlane(math::Sphere{o.center, r}, p).colliding;
}

// ── Capsule queries ──────────────────────────────────────────────────────────

[[nodiscard]] inline bool intersects(const math::Capsule& a, const math::Capsule& b) noexcept {
    return math::capsuleCapsule(a, b).colliding;
}

[[nodiscard]] inline bool intersects(const math::Capsule& c, const math::Plane& p) noexcept {
    f32 d0 = p.distanceToPoint(c.start);
    f32 d1 = p.distanceToPoint(c.end);
    if (d0 * d1 < 0.0f) return true;
    f32 minD = std::min(std::abs(d0), std::abs(d1));
    return minD <= c.radius;
}

// ── Contact dispatch ─────────────────────────────────────────────────────────

[[nodiscard]] inline math::CollisionResult contact(const math::Sphere& a, const math::Sphere& b) noexcept {
    return math::sphereSphere(a, b);
}

[[nodiscard]] inline math::CollisionResult contact(const math::Sphere& s, const math::Plane& p) noexcept {
    return math::spherePlane(s, p);
}

[[nodiscard]] inline math::CollisionResult contact(const math::Sphere& s, const math::AABB& b) noexcept {
    return math::sphereAABB(s, b);
}

[[nodiscard]] inline math::CollisionResult contact(const math::Sphere& s, const math::Capsule& c) noexcept {
    return math::sphereCapsule(s, c);
}

[[nodiscard]] inline math::CollisionResult contact(const math::AABB& a, const math::AABB& b) noexcept {
    return math::aabbAABB(a, b);
}

[[nodiscard]] inline math::CollisionResult contact(const math::Capsule& a, const math::Capsule& b) noexcept {
    return math::capsuleCapsule(a, b);
}

[[nodiscard]] inline math::CollisionResult contact(const math::OBB& a, const math::OBB& b) noexcept {
    return math::obbOBB(a, b);
}

// ── GJK/EPA fallback contact for unsupported pairs ──────────────────────────

template <typename ShapeA, typename ShapeB>
[[nodiscard]] inline math::CollisionResult contactGJK(const ShapeA& a, const ShapeB& b) noexcept {
    using namespace math;
    auto supportFunc = [&](const Vector3& dir) -> Vector3 {
        return math::supportPrimitive(a, dir) - math::supportPrimitive(b, -dir);
    };

    GJKSimplex simplex;
    Vector3 d(1.0f, 0.0f, 0.0f);
    Vector3 w = supportFunc(d);
    if (w.lengthSq() < kEpsilon) {
        CollisionResult r;
        r.colliding = true;
        r.manifold.normal = d;
        r.manifold.contacts[0].penetration = 0.0f;
        r.manifold.contacts[0].point = (a.center + b.center) * 0.5f;
        r.manifold.contactCount = 1;
        return r;
    }
    simplex.add(w);
    d = -w;

    for (u32 i = 0; i < kGJKMaxIterations; ++i) {
        Vector3 aPt = supportFunc(d);
        if (aPt.dot(d) < kGJKEpsilon) {
            CollisionResult r;
            r.colliding = false;
            return r;
        }
        simplex.add(aPt);
        switch (simplex.count) {
            case 2: d = gjk_detail::solveLine(simplex); break;
            case 3: d = gjk_detail::solveTriangle(simplex); break;
            case 4: d = gjk_detail::solveTetrahedron(simplex); break;
            default: {
                CollisionResult r;
                r.colliding = false;
                return r;
            }
        }
        if (d.lengthSq() < kGJKEpsilon * kGJKEpsilon) break;
    }

    EPAResult epaResult = epa(simplex, supportFunc);
    CollisionResult r;
    r.colliding = epaResult.converged;
    if (epaResult.converged) {
        r.manifold.normal = epaResult.normal;
        r.manifold.contacts[0].penetration = epaResult.depth;
        r.manifold.contacts[0].point = epaResult.contactPoint;
        r.manifold.contactCount = 1;
    }
    return r;
}

// ── Raycast dispatch ─────────────────────────────────────────────────────────

[[nodiscard]] inline math::RayHit raycast(const math::Ray& r, const math::Sphere& s) noexcept {
    return math::raySphere(r, s);
}

[[nodiscard]] inline math::RayHit raycast(const math::Ray& r, const math::Plane& p) noexcept {
    return math::rayPlane(r, p);
}

[[nodiscard]] inline math::RayHit raycast(const math::Ray& r, const math::AABB& a) noexcept {
    return math::rayAABB(r, a);
}

[[nodiscard]] inline math::RayHit raycast(const math::Ray& r, const math::Capsule& c) noexcept {
    return math::rayCapsule(r, c);
}

[[nodiscard]] inline math::RayHit raycast(const math::Ray& r, const math::OBB& o) noexcept {
    return math::rayOBB(r, o);
}

[[nodiscard]] inline math::RayHit raycast(const math::Ray& r, const math::Triangle& t) noexcept {
    return math::rayTriangle(r, t);
}

} // namespace primeon::collision
