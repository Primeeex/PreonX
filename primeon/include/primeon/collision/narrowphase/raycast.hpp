#pragma once

#include "primeon/collision/contact.hpp"
#include "primeon/geometry/primitives/ray.hpp"
#include "primeon/geometry/primitives/sphere.hpp"
#include "primeon/geometry/primitives/aabb.hpp"
#include "primeon/geometry/primitives/plane.hpp"
#include "primeon/geometry/primitives/capsule.hpp"
#include "primeon/geometry/primitives/triangle.hpp"
#include "primeon/geometry/primitives/obb.hpp"
#include "primeon/collision/narrowphase/sphere_capsule.hpp"

namespace primeon::math {

// ── Ray vs Sphere ────────────────────────────────────────────────────────────

[[nodiscard]] inline RayHit raySphere(const Ray& r, const Sphere& s) noexcept {
    RayHit result;
    Vector3 oc = r.origin - s.center;
    f32 a = r.direction.dot(r.direction);
    f32 b = 2.0f * oc.dot(r.direction);
    f32 c = oc.dot(oc) - s.radiusSq();
    f32 discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f) return result;

    f32 sqrtD = std::sqrt(discriminant);
    f32 t = (-b - sqrtD) / (2.0f * a);
    if (t < 0.0f) {
        t = (-b + sqrtD) / (2.0f * a);
        if (t < 0.0f) return result;
    }

    result.hit = true;
    result.distance = t;
    result.point = r.pointAt(t);
    result.normal = (result.point - s.center) / s.radius;
    return result;
}

// ── Ray vs Plane ─────────────────────────────────────────────────────────────

[[nodiscard]] inline RayHit rayPlane(const Ray& r, const Plane& p) noexcept {
    RayHit result;
    f32 denom = p.normal.dot(r.direction);
    if (abs(denom) < kEpsilon) return result;

    f32 t = (p.distance - p.normal.dot(r.origin)) / denom;
    if (t < 0.0f) return result;

    result.hit = true;
    result.distance = t;
    result.point = r.pointAt(t);
    result.normal = (denom < 0.0f) ? p.normal : -p.normal;
    return result;
}

// ── Ray vs AABB ──────────────────────────────────────────────────────────────

[[nodiscard]] inline RayHit rayAABB(const Ray& r, const AABB& a) noexcept {
    RayHit result;
    f32 tmin = -std::numeric_limits<f32>::max();
    f32 tmax = std::numeric_limits<f32>::max();
    Vector3 normal;
    Vector3 absDir(
        abs(r.direction.x) < kEpsilon ? kEpsilon : r.direction.x,
        abs(r.direction.y) < kEpsilon ? kEpsilon : r.direction.y,
        abs(r.direction.z) < kEpsilon ? kEpsilon : r.direction.z
    );

    for (int i = 0; i < 3; ++i) {
        f32 orig = (i == 0) ? r.origin.x : ((i == 1) ? r.origin.y : r.origin.z);
        f32 lo = (i == 0) ? a.min.x : ((i == 1) ? a.min.y : a.min.z);
        f32 hi = (i == 0) ? a.max.x : ((i == 1) ? a.max.y : a.max.z);

        f32 absD = (i == 0) ? absDir.x : ((i == 1) ? absDir.y : absDir.z);
        f32 invD = 1.0f / absD;
        f32 t1 = (lo - orig) * invD;
        f32 t2 = (hi - orig) * invD;

        Vector3 n(0.0f);
        if (i == 0) n.x = -1.0f;
        else if (i == 1) n.y = -1.0f;
        else n.z = -1.0f;

        if (t1 > t2) {
            f32 tmp = t1; t1 = t2; t2 = tmp;
            n = -n;
        }

        if (t1 > tmin) { tmin = t1; normal = n; }
        if (t2 < tmax) tmax = t2;
        if (tmin > tmax) return result;
    }

    if (tmin < 0.0f) {
        if (tmax < 0.0f) return result;
        tmin = 0.0f;
    }

    result.hit = true;
    result.distance = tmin;
    result.point = r.pointAt(tmin);
    result.normal = normal;
    return result;
}

// ── Ray vs Capsule ───────────────────────────────────────────────────────────

[[nodiscard]] inline RayHit rayCapsule(const Ray& r, const Capsule& c) noexcept {
    RayHit best;
    RayHit hitSphere = raySphere(r, Sphere(c.start, c.radius));
    RayHit hitSphere2 = raySphere(r, Sphere(c.end, c.radius));

    if (hitSphere.hit && (!best.hit || hitSphere.distance < best.distance))
        best = hitSphere;
    if (hitSphere2.hit && (!best.hit || hitSphere2.distance < best.distance))
        best = hitSphere2;

    Vector3 ab = c.axis();
    f32 lenSq = ab.lengthSq();
    if (lenSq > kEpsilon) {
        Vector3 ao = r.origin - c.start;
        f32 d = ab.dot(r.direction);
        f32 e = ab.dot(ao);
        f32 f = r.direction.dot(r.direction);
        f32 g = ao.dot(ao);

        f32 denom = f * lenSq - d * d;
        if (abs(denom) > kEpsilon) {
            f32 t = clamp((d * e - lenSq * g) / denom, 0.0f, 1.0f);
            f32 s = clamp((d * e + f * e - d * g) / denom, 0.0f, 1.0f);
            Vector3 closest = c.start + ab * s;
            Vector3 diff = r.origin + r.direction * t - closest;
            f32 distSq = diff.lengthSq();
            if (distSq <= c.radiusSq()) {
                f32 rayT = t - std::sqrt(c.radiusSq() - distSq);
                if (rayT >= 0.0f && (!best.hit || rayT < best.distance)) {
                    best.hit = true;
                    best.distance = rayT;
                    best.point = r.pointAt(rayT);
                    best.normal = (best.point - closest).normalized();
                }
            }
        }
    }

    return best;
}

// ── Ray vs OBB ───────────────────────────────────────────────────────────────

[[nodiscard]] inline RayHit rayOBB(const Ray& r, const OBB& o) noexcept {
    Quaternion qInv = o.orientation.conjugate();
    Vector3 localOrigin = qInv.rotate(r.origin - o.center);
    Vector3 localDir = qInv.rotate(r.direction);
    AABB localBox(-o.halfExtents, o.halfExtents);
    Ray localRay(localOrigin, localDir);
    RayHit localHit = rayAABB(localRay, localBox);

    if (!localHit.hit) return localHit;

    localHit.point = o.center + o.orientation.rotate(localHit.point);
    localHit.normal = o.orientation.rotate(localHit.normal);
    return localHit;
}

// ── Ray vs Triangle ──────────────────────────────────────────────────────────

[[nodiscard]] inline RayHit rayTriangle(const Ray& r, const Triangle& t) noexcept {
    RayHit result;
    Vector3 ab = t.b - t.a;
    Vector3 ac = t.c - t.a;
    Vector3 pvec = r.direction.cross(ac);
    f32 det = ab.dot(pvec);

    if (abs(det) < kEpsilon) return result;

    f32 invDet = 1.0f / det;
    Vector3 tvec = r.origin - t.a;
    f32 u = tvec.dot(pvec) * invDet;
    if (u < 0.0f || u > 1.0f) return result;

    Vector3 qvec = tvec.cross(ab);
    f32 v = r.direction.dot(qvec) * invDet;
    if (v < 0.0f || u + v > 1.0f) return result;

    f32 tVal = ac.dot(qvec) * invDet;
    if (tVal < 0.0f) return result;

    result.hit = true;
    result.distance = tVal;
    result.point = r.pointAt(tVal);
    result.normal = t.normal();
    if (result.normal.dot(r.direction) > 0.0f) result.normal = -result.normal;
    return result;
}

} // namespace primeon::math
