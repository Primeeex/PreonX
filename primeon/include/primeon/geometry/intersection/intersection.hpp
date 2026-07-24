#pragma once

#include "primeon/math/vector/vector3.hpp"
#include "primeon/math/quaternion/quaternion.hpp"
#include "primeon/math/scalar/scalar.hpp"
#include "primeon/geometry/primitives/ray.hpp"
#include "primeon/geometry/primitives/plane.hpp"
#include "primeon/geometry/primitives/sphere.hpp"
#include "primeon/geometry/primitives/aabb.hpp"
#include "primeon/geometry/primitives/segment.hpp"

namespace primeon::math::intersect {

/// Result of a ray intersection test.
struct HitResult {
    bool hit = false;
    f32 distance = 0.0f;
    Vector3 point = kVector3Zero;
    Vector3 normal = kVector3Zero;
};

/// Ray vs sphere intersection.
[[nodiscard]] inline HitResult raySphere(const Ray& ray, const Sphere& sphere) noexcept {
    Vector3 oc = ray.origin - sphere.center;
    f32 a = ray.direction.dot(ray.direction);
    f32 b = 2.0f * oc.dot(ray.direction);
    f32 c = oc.dot(oc) - sphere.radius * sphere.radius;
    f32 discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f) return {};

    f32 sqrtD = std::sqrt(discriminant);
    f32 t = (-b - sqrtD) / (2.0f * a);

    if (t < 0.0f) {
        t = (-b + sqrtD) / (2.0f * a);
        if (t < 0.0f) return {};
    }

    Vector3 p = ray.pointAt(t);
    return {true, t, p, (p - sphere.center).normalized()};
}

/// Ray vs plane intersection. Returns hit if t >= 0.
[[nodiscard]] inline HitResult rayPlane(const Ray& ray, const Plane& plane) noexcept {
    f32 denom = plane.normal.dot(ray.direction);
    if (abs(denom) < kEpsilon) return {};

    f32 t = (plane.distance - plane.normal.dot(ray.origin)) / denom;
    if (t < 0.0f) return {};

    Vector3 p = ray.pointAt(t);
    return {true, t, p, plane.normal};
}

/// Ray vs AABB intersection (Slab method).
[[nodiscard]] inline HitResult rayAABB(const Ray& ray, const AABB& aabb) noexcept {
    f32 tmin = -std::numeric_limits<f32>::max();
    f32 tmax = std::numeric_limits<f32>::max();

    Vector3 invDir = {1.0f / (abs(ray.direction.x) < kEpsilon ? kEpsilon : ray.direction.x),
                      1.0f / (abs(ray.direction.y) < kEpsilon ? kEpsilon : ray.direction.y),
                      1.0f / (abs(ray.direction.z) < kEpsilon ? kEpsilon : ray.direction.z)};

    Vector3 origins[2] = {aabb.min, aabb.max};
    Vector3 normal = kVector3Zero;

    for (int i = 0; i < 3; ++i) {
        f32 o[2] = {(origins[0][i] - ray.origin[i]) * invDir[i],
                     (origins[1][i] - ray.origin[i]) * invDir[i]};

        Vector3 axisNormal = kVector3Zero;
        if (i == 0) axisNormal = kVector3UnitX;
        else if (i == 1) axisNormal = kVector3UnitY;
        else axisNormal = kVector3UnitZ;

        if (o[0] > o[1]) {
            std::swap(o[0], o[1]);
            axisNormal = -axisNormal;
        }

        if (o[0] > tmin) { tmin = o[0]; normal = axisNormal; }
        if (o[1] < tmax) { tmax = o[1]; }

        if (tmin > tmax || tmax < 0.0f) return {};
    }

    if (tmin < 0.0f) tmin = tmax;
    if (tmin < 0.0f) return {};

    Vector3 p = ray.pointAt(tmin);
    return {true, tmin, p, normal};
}

/// Sphere vs sphere intersection.
[[nodiscard]] constexpr bool sphereSphere(const Sphere& a, const Sphere& b) noexcept {
    f32 distSq = a.center.distanceSqTo(b.center);
    f32 radiusSum = a.radius + b.radius;
    return distSq <= radiusSum * radiusSum;
}

/// AABB vs AABB intersection.
[[nodiscard]] constexpr bool aabbAABB(const AABB& a, const AABB& b) noexcept {
    return a.min.x <= b.max.x && a.max.x >= b.min.x &&
           a.min.y <= b.max.y && a.max.y >= b.min.y &&
           a.min.z <= b.max.z && a.max.z >= b.min.z;
}

/// Point inside AABB.
[[nodiscard]] constexpr bool pointInsideAABB(const Vector3& point, const AABB& aabb) noexcept {
    return aabb.containsPoint(point);
}

/// Point inside sphere.
[[nodiscard]] constexpr bool pointInsideSphere(const Vector3& point, const Sphere& sphere) noexcept {
    return sphere.containsPoint(point);
}

/// Point inside OBB (transforms point to OBB local space, then tests AABB).
[[nodiscard]] inline bool pointInsideOBB(const Vector3& point, const Vector3& center,
                                         const Vector3& halfExtents, const Quaternion& orientation) noexcept {
    Vector3 local = orientation.inverse().rotate(point - center);
    return abs(local.x) <= halfExtents.x &&
           abs(local.y) <= halfExtents.y &&
           abs(local.z) <= halfExtents.z;
}

/// Segment vs sphere intersection.
[[nodiscard]] inline bool segmentSphere(const Segment& seg, const Sphere& sphere) noexcept {
    Vector3 d = seg.end - seg.start;
    Vector3 f = seg.start - sphere.center;
    f32 a = d.dot(d);
    f32 b = 2.0f * f.dot(d);
    f32 c = f.dot(f) - sphere.radius * sphere.radius;
    f32 discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f) return false;

    f32 sqrtD = std::sqrt(discriminant);
    f32 t1 = (-b - sqrtD) / (2.0f * a);
    f32 t2 = (-b + sqrtD) / (2.0f * a);

    return (t1 >= 0.0f && t1 <= 1.0f) || (t2 >= 0.0f && t2 <= 1.0f) ||
           (t1 < 0.0f && t2 > 1.0f);
}

} // namespace primeon::math::intersect
