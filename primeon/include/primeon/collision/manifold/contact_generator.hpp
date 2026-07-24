#pragma once

#include "primeon/collision/contact.hpp"
#include "primeon/collision/narrowphase/sphere_capsule.hpp"
#include "primeon/collision/algorithms/sat.hpp"
#include "primeon/geometry/primitives/sphere.hpp"
#include "primeon/geometry/primitives/aabb.hpp"
#include "primeon/geometry/primitives/obb.hpp"
#include "primeon/geometry/primitives/capsule.hpp"
#include "primeon/geometry/primitives/plane.hpp"

namespace primeon::math {

// ── Contact Point Generation ──────────────────────────────────────────────
//
// Each generator produces a ContactManifold from two overlapping shapes.
// Simple pairs (sphere-*) produce 1 contact point.
// Box-box pairs use reference face clipping to produce up to 4 points.
// All generators set feature IDs for contact persistence.

// ── Sutherland-Hodgman polygon clipping helper ────────────────────────────

/// Clips a polygon against a half-plane defined by axis and value.
/// Keeps points on the keepAbove side. Returns the number of output points.
inline u32 clipPolygonHalfPlane(Vector3* poly, u32 count,
                                 int axis, f32 planeVal, bool keepAbove) noexcept {
    if (count == 0) return 0;
    Vector3 tmp[8];
    u32 n = count;
    for (u32 i = 0; i < n; ++i) tmp[i] = poly[i];
    u32 outCount = 0;
    for (u32 i = 0; i < n; ++i) {
        u32 next = (i + 1) % n;
        f32 d0 = (&tmp[i].x)[axis] - planeVal;
        f32 d1 = (&tmp[next].x)[axis] - planeVal;
        bool inside0 = keepAbove ? (d0 >= -kEpsilon) : (d0 <= kEpsilon);
        bool inside1 = keepAbove ? (d1 >= -kEpsilon) : (d1 <= kEpsilon);
        if (inside0) {
            poly[outCount++] = tmp[i];
            if (!inside1 && outCount < 8) {
                f32 denom = d0 - d1;
                if (abs(denom) > kEpsilon) {
                    f32 t = d0 / denom;
                    poly[outCount++] = tmp[i] + (tmp[next] - tmp[i]) * t;
                }
            }
        } else if (inside1 && outCount < 8) {
            f32 denom = d0 - d1;
            if (abs(denom) > kEpsilon) {
                f32 t = d0 / denom;
                poly[outCount++] = tmp[i] + (tmp[next] - tmp[i]) * t;
            }
        }
    }
    return outCount;
}

// ── Sphere-Sphere ──────────────────────────────────────────────────────────

[[nodiscard]] inline CollisionResult generateSphereSphere(
    const Sphere& a, const Sphere& b, u32 idA = 0, u32 idB = 0) noexcept
{
    CollisionResult result;
    Vector3 diff = a.center - b.center;
    f32 distSq = diff.lengthSq();
    f32 radiusSum = a.radius + b.radius;
    if (distSq >= radiusSum * radiusSum) return result;

    f32 dist = std::sqrt(distSq);
    result.colliding = true;
    result.manifold.bodyIDA = idA;
    result.manifold.bodyIDB = idB;

    if (dist > kEpsilon)
        result.manifold.normal = diff / dist;
    else
        result.manifold.normal = kVector3UnitY;

    ContactPoint cp;
    cp.point = b.center + result.manifold.normal * b.radius;
    cp.penetration = radiusSum - dist;
    cp.featureA = ContactFeature(FeatureType::Vertex, 0, 0);
    cp.featureB = ContactFeature(FeatureType::Vertex, 0, 0);
    cp.computeID();
    result.manifold.addContact(cp);
    result.manifold.computeTangents();
    return result;
}

// ── Sphere-Plane ───────────────────────────────────────────────────────────

[[nodiscard]] inline CollisionResult generateSpherePlane(
    const Sphere& s, const Plane& p, u32 idA = 0, u32 idB = 0) noexcept
{
    CollisionResult result;
    f32 dist = p.distanceToPoint(s.center);
    if (dist >= s.radius) return result;

    result.colliding = true;
    result.manifold.normal = p.normal;
    result.manifold.bodyIDA = idA;
    result.manifold.bodyIDB = idB;

    ContactPoint cp;
    cp.point = s.center - p.normal * dist;
    cp.penetration = s.radius - dist;
    cp.featureA = ContactFeature(FeatureType::Vertex, 0, 0);
    cp.featureB = ContactFeature(FeatureType::Face, 0, 0);
    cp.computeID();
    result.manifold.addContact(cp);
    result.manifold.computeTangents();
    return result;
}

// ── Sphere-AABB ────────────────────────────────────────────────────────────

[[nodiscard]] inline CollisionResult generateSphereAABB(
    const Sphere& s, const AABB& a, u32 idA = 0, u32 idB = 0) noexcept
{
    CollisionResult result;
    Vector3 closest = a.closestPoint(s.center);
    Vector3 diff = s.center - closest;
    f32 distSq = diff.lengthSq();
    if (distSq >= s.radiusSq()) return result;

    f32 dist = std::sqrt(distSq);
    result.colliding = true;
    result.manifold.bodyIDA = idA;
    result.manifold.bodyIDB = idB;

    if (dist > kEpsilon) {
        result.manifold.normal = diff / dist;
    } else {
        Vector3 d = s.center - a.center();
        Vector3 he = a.halfExtents();
        f32 px = he.x - abs(d.x);
        f32 py = he.y - abs(d.y);
        f32 pz = he.z - abs(d.z);
        f32 nx = (d.x >= 0.0f) ? 1.0f : -1.0f;
        f32 ny = (d.y >= 0.0f) ? 1.0f : -1.0f;
        f32 nz = (d.z >= 0.0f) ? 1.0f : -1.0f;
        f32 minClearance = min(px, min(py, pz));
        if (px < py && px < pz) result.manifold.normal = Vector3(nx, 0.0f, 0.0f);
        else if (py < pz) result.manifold.normal = Vector3(0.0f, ny, 0.0f);
        else result.manifold.normal = Vector3(0.0f, 0.0f, nz);
        dist = -minClearance;
    }

    ContactPoint cp;
    cp.point = closest;
    cp.penetration = s.radius - dist;
    cp.featureA = ContactFeature(FeatureType::Vertex, 0, 0);
    cp.featureB = ContactFeature(FeatureType::Face, 0, 0);
    cp.computeID();
    result.manifold.addContact(cp);
    result.manifold.computeTangents();
    return result;
}

// ── Sphere-Capsule ─────────────────────────────────────────────────────────

[[nodiscard]] inline CollisionResult generateSphereCapsule(
    const Sphere& s, const Capsule& c, u32 idA = 0, u32 idB = 0) noexcept
{
    CollisionResult result;
    Vector3 closest = closestPointOnSegment(s.center, c.start, c.end);
    Vector3 diff = s.center - closest;
    f32 distSq = diff.lengthSq();
    f32 radiusSum = s.radius + c.radius;
    if (distSq >= radiusSum * radiusSum) return result;

    f32 dist = std::sqrt(distSq);
    result.colliding = true;
    result.manifold.bodyIDA = idA;
    result.manifold.bodyIDB = idB;

    if (dist > kEpsilon) {
        result.manifold.normal = diff / dist;
    } else {
        Vector3 axis = c.axis();
        f32 axisLen = axis.length();
        if (axisLen > kEpsilon) result.manifold.normal = axis / axisLen;
        else result.manifold.normal = kVector3UnitY;
    }

    ContactPoint cp;
    cp.point = closest + result.manifold.normal * c.radius;
    cp.penetration = radiusSum - dist;
    cp.featureA = ContactFeature(FeatureType::Vertex, 0, 0);
    cp.featureB = ContactFeature(FeatureType::Edge, 0, 0);
    cp.computeID();
    result.manifold.addContact(cp);
    result.manifold.computeTangents();
    return result;
}

// ── AABB-AABB ──────────────────────────────────────────────────────────────

[[nodiscard]] inline CollisionResult generateAABBAABB(
    const AABB& a, const AABB& b, u32 idA = 0, u32 idB = 0) noexcept
{
    CollisionResult result;

    f32 overlapX = min(a.max.x, b.max.x) - max(a.min.x, b.min.x);
    f32 overlapY = min(a.max.y, b.max.y) - max(a.min.y, b.min.y);
    f32 overlapZ = min(a.max.z, b.max.z) - max(a.min.z, b.min.z);

    if (overlapX <= 0.0f || overlapY <= 0.0f || overlapZ <= 0.0f) return result;

    result.colliding = true;
    result.manifold.bodyIDA = idA;
    result.manifold.bodyIDB = idB;

    Vector3 centers = a.center() - b.center();
    int refAxis;
    f32 overlap;
    if (overlapX <= overlapY && overlapX <= overlapZ) {
        refAxis = 0; overlap = overlapX;
        result.manifold.normal = (centers.x >= 0.0f) ? kVector3UnitX : Vector3(-1.0f, 0.0f, 0.0f);
    } else if (overlapY <= overlapZ) {
        refAxis = 1; overlap = overlapY;
        result.manifold.normal = (centers.y >= 0.0f) ? kVector3UnitY : Vector3(0.0f, -1.0f, 0.0f);
    } else {
        refAxis = 2; overlap = overlapZ;
        result.manifold.normal = (centers.z >= 0.0f) ? kVector3UnitZ : Vector3(0.0f, 0.0f, -1.0f);
    }

    // Reference face: the face of A facing the contact normal
    // Incident face: the face of B most anti-aligned with the contact normal
    int a1 = (refAxis + 1) % 3;
    int a2 = (refAxis + 2) % 3;
    f32 refSign = (&result.manifold.normal.x)[refAxis] > 0.0f ? 1.0f : -1.0f;

    // Incident face center on B: project B onto the most anti-aligned face
    f32 incSign = -refSign;
    Vector3 incCenter = b.center();
    (&incCenter.x)[refAxis] = (incSign > 0.0f) ? (&b.max.x)[refAxis] : (&b.min.x)[refAxis];

    // Generate 4 corners of the incident face
    Vector3 bHE = b.halfExtents();
    f32 incHE1 = (&bHE.x)[a1];
    f32 incHE2 = (&bHE.x)[a2];
    Vector3 poly[8];
    for (int i = 0; i < 4; ++i) {
        poly[i] = incCenter;
        (&poly[i].x)[a1] += ((i & 1) ? 1.0f : -1.0f) * incHE1;
        (&poly[i].x)[a2] += ((i & 2) ? 1.0f : -1.0f) * incHE2;
    }

    // Clip against 4 side planes of reference face
    Vector3 aHE = a.halfExtents();
    Vector3 aCen = a.center();
    f32 refHE1 = (&aHE.x)[a1];
    f32 refHE2 = (&aHE.x)[a2];
    f32 refCenterA1 = (&aCen.x)[a1];
    f32 refCenterA2 = (&aCen.x)[a2];

    u32 count = clipPolygonHalfPlane(poly, 4, a1, refCenterA1 - refHE1, true);
    if (count > 0) count = clipPolygonHalfPlane(poly, count, a1, refCenterA1 + refHE1, false);
    if (count > 0) count = clipPolygonHalfPlane(poly, count, a2, refCenterA2 - refHE2, true);
    if (count > 0) count = clipPolygonHalfPlane(poly, count, a2, refCenterA2 + refHE2, false);

    // Project surviving points onto the reference face plane
    f32 refFacePos = (&aCen.x)[refAxis] + refSign * ((&aHE.x)[refAxis]);
    ContactFeature refFeature(FeatureType::Face, static_cast<u32>(refAxis), static_cast<u32>(refAxis));
    ContactFeature incFeature(FeatureType::Face, static_cast<u32>(refAxis), static_cast<u32>(refAxis));

    if (count == 0) {
        ContactPoint cp;
        cp.point = (a.center() + b.center()) * 0.5f;
        cp.penetration = overlap;
        cp.featureA = refFeature;
        cp.featureB = incFeature;
        cp.computeID();
        result.manifold.addContact(cp);
    } else {
        for (u32 i = 0; i < count && i < kMaxContactPoints; ++i) {
            ContactPoint cp;
            Vector3 p = poly[i];
            (&p.x)[refAxis] = refFacePos;
            cp.point = p;
            cp.penetration = overlap;
            cp.featureA = refFeature;
            cp.featureB = incFeature;
            cp.computeID();
            result.manifold.addContact(cp);
        }
    }

    result.manifold.computeTangents();
    return result;
}

// ── Capsule-Capsule ────────────────────────────────────────────────────────

[[nodiscard]] inline CollisionResult generateCapsuleCapsule(
    const Capsule& a, const Capsule& b, u32 idA = 0, u32 idB = 0) noexcept
{
    CollisionResult result;

    Vector3 d1 = a.axis();
    Vector3 d2 = b.axis();
    Vector3 r = a.start - b.start;
    f32 d1d1 = d1.lengthSq();
    f32 d2d2 = d2.lengthSq();
    f32 d1d2 = d1.dot(d2);
    f32 d1r = d1.dot(r);
    f32 d2r = d2.dot(r);
    f32 denom = d1d1 * d2d2 - d1d2 * d1d2;

    f32 s = 0.0f;
    f32 t = 0.0f;

    if (denom > kEpsilon) {
        s = clamp((d1d2 * d2r - d2d2 * d1r) / denom, 0.0f, 1.0f);
        t = clamp((d1d1 * d2r - d1d2 * d1r) / denom, 0.0f, 1.0f);
    } else {
        f32 tb0 = d1.dot(b.start - a.start) / d1d1;
        f32 tb1 = d1.dot(b.end - a.start) / d1d1;
        if (tb0 > tb1) std::swap(tb0, tb1);
        f32 lo = std::max(0.0f, tb0);
        f32 hi = std::min(1.0f, tb1);
        if (lo <= hi) {
            s = (lo + hi) * 0.5f;
            f32 t_on_b = d2.dot((a.start + d1 * s) - b.start) / d2d2;
            t = clamp(t_on_b, 0.0f, 1.0f);
        }
    }

    Vector3 pA = a.start + d1 * s;
    Vector3 pB = b.start + d2 * t;
    Vector3 diff = pA - pB;
    f32 distSq = diff.lengthSq();
    f32 radiusSum = a.radius + b.radius;

    if (distSq >= radiusSum * radiusSum) return result;

    f32 dist = std::sqrt(distSq);
    result.colliding = true;
    result.manifold.bodyIDA = idA;
    result.manifold.bodyIDB = idB;

    if (dist > kEpsilon) {
        result.manifold.normal = diff / dist;
    } else {
        Vector3 axis = d1.cross(d2);
        f32 axisLen = axis.length();
        if (axisLen > kEpsilon) result.manifold.normal = axis / axisLen;
        else result.manifold.normal = kVector3UnitY;
    }

    u32 featureA = (s >= 1.0f) ? 1 : 0;
    u32 featureB = (t >= 1.0f) ? 1 : 0;
    FeatureType ftypeA = (s <= kEpsilon || s >= 1.0f - kEpsilon) ? FeatureType::Vertex : FeatureType::Edge;
    FeatureType ftypeB = (t <= kEpsilon || t >= 1.0f - kEpsilon) ? FeatureType::Vertex : FeatureType::Edge;

    ContactPoint cp;
    cp.point = pB + result.manifold.normal * b.radius;
    cp.penetration = radiusSum - dist;
    cp.featureA = ContactFeature(ftypeA, featureA, featureA);
    cp.featureB = ContactFeature(ftypeB, featureB, featureB);
    cp.computeID();
    result.manifold.addContact(cp);
    result.manifold.computeTangents();
    return result;
}

// ── OBB-OBB ────────────────────────────────────────────────────────────────

[[nodiscard]] inline CollisionResult generateOBBOBB(
    const OBB& a, const OBB& b, u32 idA = 0, u32 idB = 0) noexcept
{
    CollisionResult result;

    Vector3 axesA[3] = {a.orientation.right(), a.orientation.up(), a.orientation.forward()};
    Vector3 axesB[3] = {b.orientation.right(), b.orientation.up(), b.orientation.forward()};
    Vector3 d = b.center - a.center;

    f32 minOverlap = std::numeric_limits<f32>::max();
    Vector3 minAxis = kVector3Zero;
    int minAxisIndex = -1;

    auto testAxis = [&](const Vector3& axis, int idx) -> bool {
        f32 lenSq = axis.lengthSq();
        if (lenSq < kEpsilon) return true;
        Vector3 n = axis / std::sqrt(lenSq);
        f32 overlap = satOverlapOBB(a, n, axesA, b, axesB);
        if (overlap <= 0.0f) return false;
        if (overlap < minOverlap) {
            minOverlap = overlap;
            minAxis = n;
            minAxisIndex = idx;
        }
        return true;
    };

    for (int i = 0; i < 3; ++i)
        if (!testAxis(axesA[i], i)) return result;
    for (int i = 0; i < 3; ++i)
        if (!testAxis(axesB[i], 3 + i)) return result;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (!testAxis(axesA[i].cross(axesB[j]), 6 + i * 3 + j)) return result;

    if (d.dot(minAxis) < 0.0f) minAxis = -minAxis;

    result.colliding = true;
    result.manifold.normal = minAxis;
    result.manifold.bodyIDA = idA;
    result.manifold.bodyIDB = idB;

    ContactFeature featA(FeatureType::Face, static_cast<u32>(minAxisIndex < 3 ? minAxisIndex : 0), 0);
    ContactFeature featB(FeatureType::Face, static_cast<u32>(minAxisIndex >= 3 && minAxisIndex < 6 ? minAxisIndex - 3 : 0), 0);

    // For face-face contacts, generate up to 4 contact points via clipping
    // Simplified: use the 8 corners of B projected onto A's reference face
    if (minAxisIndex < 6) {
        // Face-axis contact: clip incident face corners
        [[maybe_unused]] int refIdx = (minAxisIndex < 3) ? minAxisIndex : (minAxisIndex - 3);
        const OBB& refOBB = (minAxisIndex < 3) ? a : b;
        const OBB& incOBB = (minAxisIndex < 3) ? b : a;
        [[maybe_unused]] const Vector3* refAxes = (minAxisIndex < 3) ? axesA : axesB;
        const Vector3* incAxes = (minAxisIndex < 3) ? axesB : axesA;

        // Incident face: find the face most anti-aligned with the normal
        int incFaceIdx = 0;
        f32 bestDot = 1.0f;
        for (int i = 0; i < 3; ++i) {
            f32 dot = incAxes[i].dot(minAxis);
            if (dot < bestDot) { bestDot = dot; incFaceIdx = i; }
        }

        // Get 4 corners of incident face
        Vector3 incCorners[8];
        incOBB.getCorners(incCorners);

        // Filter to the 4 corners on the incident face
        // The face normal direction determines which 4 corners
        Vector3 faceNormal = incAxes[incFaceIdx];
        f32 faceOffset = faceNormal.dot(incOBB.center) + incOBB.halfExtents[incFaceIdx];

        Vector3 faceCorners[4];
        u32 faceCount = 0;
        for (int i = 0; i < 8 && faceCount < 4; ++i) {
            f32 proj = faceNormal.dot(incCorners[i]);
            if (abs(proj - faceOffset) < kEpsilon) {
                faceCorners[faceCount++] = incCorners[i];
            }
        }

        if (faceCount >= 3) {
            // Project corners onto reference face plane
            f32 refDot = minAxis.dot(refOBB.center);
            for (u32 i = 0; i < faceCount; ++i) {
                f32 dist = minAxis.dot(faceCorners[i]) - refDot;
                faceCorners[i] -= minAxis * dist;
            }

            // Keep up to 4 points
            for (u32 i = 0; i < faceCount && i < kMaxContactPoints; ++i) {
                ContactPoint cp;
                cp.point = faceCorners[i];
                cp.penetration = minOverlap;
                cp.featureA = featA;
                cp.featureB = featB;
                cp.computeID();
                result.manifold.addContact(cp);
            }
        }
    }

    if (result.manifold.contactCount == 0) {
        ContactPoint cp;
        cp.point = (a.center + b.center) * 0.5f;
        cp.penetration = minOverlap;
        cp.featureA = featA;
        cp.featureB = featB;
        cp.computeID();
        result.manifold.addContact(cp);
    }

    result.manifold.computeTangents();
    return result;
}

} // namespace primeon::math
