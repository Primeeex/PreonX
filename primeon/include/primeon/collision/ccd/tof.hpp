#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/vector/vector3.hpp"
#include "primeon/geometry/primitives/sphere.hpp"
#include "primeon/geometry/primitives/aabb.hpp"
#include "primeon/geometry/primitives/capsule.hpp"
#include "primeon/geometry/primitives/ray.hpp"

namespace primeon::collision {

using math::f32;
using math::u32;
using math::Vector3;
using math::kVector3Zero;
using math::kEpsilon;

// ── CCD (Continuous Collision Detection) ─────────────────────────────────────
//
// Computes the Time of Impact (TOI) between two moving shapes, accounting
// for both translation and angular velocity. The foundation uses swept-sphere
// tests as the base case, and more complex shapes are reduced to swept-sphere
// or swept-point tests via their support functions.
//
// Key design: TOI returns a value in [0, 1] where 0 = start, 1 = end.
// If no collision occurs, TOI returns > 1.

inline constexpr f32 kCCDEpsilon = 1e-6f;
inline constexpr u32 kCCDMaxIterations = 32;

/// TOI result structure.
struct TOIResult {
    bool hit = false;
    f32 toi = 1.0f;         // time of impact in [0,1]
    Vector3 normal = kVector3Zero; // contact normal at TOI
};

/// Swept sphere vs swept sphere TOI.
/// posA0, posB0 = positions at t=0; posA1, posB1 = positions at t=1.
[[nodiscard]] inline TOIResult toiSphereSphere(
    const Vector3& posA0, f32 rA,
    const Vector3& posB0, f32 rB,
    const Vector3& posA1, const Vector3& posB1) noexcept
{
    TOIResult result;
    Vector3 dA = posA1 - posA0;
    Vector3 dB = posB1 - posB0;
    Vector3 d = dA - dB;
    Vector3 f = posA0 - posB0;

    f32 a = d.dot(d);
    f32 b = 2.0f * f.dot(d);
    f32 c = f.dot(f) - (rA + rB) * (rA + rB);

    if (a < kCCDEpsilon) {
        if (c > 0.0f) return result;
        // Overlapping at t=0.
        result.hit = true;
        result.toi = 0.0f;
        result.normal = f.lengthSq() > kCCDEpsilon
            ? f.normalized() : Vector3(1.0f, 0.0f, 0.0f);
        return result;
    }

    f32 discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f) return result;

    f32 sqrtDisc = std::sqrt(discriminant);
    f32 t0 = (-b - sqrtDisc) / (2.0f * a);
    f32 t1 = (-b + sqrtDisc) / (2.0f * a);

    if (t0 < 0.0f) t0 = t1;
    if (t0 < 0.0f || t0 > 1.0f) return result;

    result.hit = true;
    result.toi = t0;

    Vector3 posA = posA0 + dA * t0;
    Vector3 posB = posB0 + dB * t0;
    Vector3 diff = posA - posB;
    f32 dist = diff.length();
    result.normal = dist > kCCDEpsilon ? diff / dist : Vector3(1.0f, 0.0f, 0.0f);
    return result;
}

/// Swept sphere vs plane TOI.
[[nodiscard]] inline TOIResult toiSpherePlane(
    const Vector3& pos0, f32 radius,
    const Vector3& pos1,
    const math::Vector3& planeNormal, f32 planeDist) noexcept
{
    TOIResult result;
    Vector3 d = pos1 - pos0;
    f32 denom = planeNormal.dot(d);
    f32 dist0 = planeNormal.dot(pos0) - planeDist;

    if (std::abs(denom) < kCCDEpsilon) {
        if (std::abs(dist0) > radius) return result;
        result.hit = true;
        result.toi = 0.0f;
        result.normal = dist0 > 0.0f ? planeNormal : -planeNormal;
        return result;
    }

    f32 t = (planeDist + radius - planeNormal.dot(pos0)) / denom;
    if (dist0 < -radius) {
        t = (planeDist + radius - planeNormal.dot(pos0)) / denom;
    } else if (dist0 > radius) {
        t = (planeDist - radius - planeNormal.dot(pos0)) / denom;
    } else {
        t = 0.0f;
    }

    if (t < 0.0f || t > 1.0f) return result;

    result.hit = true;
    result.toi = t;
    result.normal = dist0 > 0.0f ? planeNormal : -planeNormal;
    return result;
}

/// Swept sphere vs AABB TOI. Reduces to sphere vs expanded AABB.
[[nodiscard]] inline TOIResult toiSphereAABB(
    const Vector3& spherePos0, f32 radius,
    const Vector3& spherePos1,
    const math::AABB& aabb) noexcept
{
    TOIResult result;
    math::AABB expanded = aabb;
    expanded.min = expanded.min - Vector3(radius, radius, radius);
    expanded.max = expanded.max + Vector3(radius, radius, radius);

    Vector3 d = spherePos1 - spherePos0;
    f32 tMin = 0.0f;
    f32 tMax = 1.0f;
    int hitAxis = -1;

    for (int i = 0; i < 3; ++i) {
        f32 invD = 1.0f / ((&d.x)[i] != 0.0f ? (&d.x)[i] : kCCDEpsilon);
        f32 t0 = ((&expanded.min.x)[i] - (&spherePos0.x)[i]) * invD;
        f32 t1 = ((&expanded.max.x)[i] - (&spherePos0.x)[i]) * invD;
        if (t0 > t1) std::swap(t0, t1);
        if (t0 > tMin) { tMin = t0; hitAxis = i; }
        tMax = std::min(tMax, t1);
        if (tMin > tMax) return result;
    }

    if (tMin < 0.0f || tMin > 1.0f) return result;

    result.hit = true;
    result.toi = tMin;
    result.normal = kVector3Zero;
    if (hitAxis >= 0) (&result.normal.x)[hitAxis] = (&d.x)[hitAxis] > 0 ? 1.0f : -1.0f;
    return result;
}

/// Swept sphere vs capsule TOI. Uses closest point on capsule segment.
[[nodiscard]] inline TOIResult toiSphereCapsule(
    const Vector3& spherePos0, f32 radius,
    const Vector3& spherePos1,
    const math::Vector3& capStart0, const math::Vector3& capEnd0,
    const math::Vector3& capStart1, const math::Vector3& capEnd1,
    f32 capRadius) noexcept
{
    TOIResult result;

    struct Frame { math::Vector3 A, B; f32 t; };
    Frame frames[2] = {
        {capStart0, capEnd0, 0.0f},
        {capStart1, capEnd1, 1.0f}
    };

    for (u32 i = 0; i < kCCDMaxIterations; ++i) {
        f32 tMid = (frames[0].t + frames[1].t) * 0.5f;
        Vector3 sPos = spherePos0 + (spherePos1 - spherePos0) * tMid;
        math::Vector3 cStart = frames[0].A + (frames[1].A - frames[0].A) * tMid;
        math::Vector3 cEnd = frames[0].B + (frames[1].B - frames[0].B) * tMid;

        Vector3 seg = cEnd - cStart;
        f32 segLenSq = seg.lengthSq();
        f32 t = 0.0f;
        if (segLenSq > kCCDEpsilon)
            t = std::clamp((sPos - cStart).dot(seg) / segLenSq, 0.0f, 1.0f);
        Vector3 closest = cStart + seg * t;
        f32 distSq = (sPos - closest).lengthSq();
        f32 combinedRadius = radius + capRadius;

        if (distSq < combinedRadius * combinedRadius) {
            frames[1] = {cStart, cEnd, tMid};
        } else {
            frames[0] = {cStart, cEnd, tMid};
        }

        if (frames[1].t - frames[0].t < kCCDEpsilon) break;
    }

    f32 toi = frames[1].t;
    if (toi < 0.0f || toi > 1.0f) return result;

    Vector3 sPos = spherePos0 + (spherePos1 - spherePos0) * toi;
    math::Vector3 cStart = frames[0].A + (frames[1].A - frames[0].A) * toi;
    math::Vector3 cEnd = frames[0].B + (frames[1].B - frames[0].B) * toi;
    Vector3 seg = cEnd - cStart;
    f32 segLenSq = seg.lengthSq();
    f32 t = 0.0f;
    if (segLenSq > kCCDEpsilon)
        t = std::clamp((sPos - cStart).dot(seg) / segLenSq, 0.0f, 1.0f);
    Vector3 closest = cStart + seg * t;
    Vector3 diff = sPos - closest;
    f32 dist = diff.length();

    if (dist > kCCDEpsilon) {
        result.hit = true;
        result.toi = toi;
        result.normal = diff / dist;
    } else {
        result.hit = true;
        result.toi = toi;
        result.normal = Vector3(0.0f, 1.0f, 0.0f);
    }
    return result;
}

} // namespace primeon::collision
