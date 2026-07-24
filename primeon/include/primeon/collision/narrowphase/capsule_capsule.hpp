#pragma once

#include "primeon/collision/contact.hpp"
#include "primeon/geometry/primitives/capsule.hpp"

namespace primeon::math {

/// Capsule vs Capsule narrowphase test.
/// Reduces to segment-segment distance, then compares against combined radii.
[[nodiscard]] inline CollisionResult capsuleCapsule(const Capsule& a, const Capsule& b) noexcept {
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
        // Parallel segments. Project B's endpoints onto A's line, then clamp the overlap.
        f32 tb0 = d1.dot(b.start - a.start) / d1d1;
        f32 tb1 = d1.dot(b.end - a.start) / d1d1;
        if (tb0 > tb1) std::swap(tb0, tb1);
        f32 lo = std::max(0.0f, tb0);
        f32 hi = std::min(1.0f, tb1);
        if (lo <= hi) {
            s = (lo + hi) * 0.5f;
            f32 t_on_b = d2.dot((a.start + d1 * s) - b.start) / d2d2;
            t = clamp(t_on_b, 0.0f, 1.0f);
        } else {
            f32 bestDistSq = std::numeric_limits<f32>::max();
            auto tryPair = [&](f32 sa, float tb) {
                Vector3 pa = a.start + d1 * sa;
                Vector3 pb = b.start + d2 * tb;
                f32 dsq = (pa - pb).lengthSq();
                if (dsq < bestDistSq) { bestDistSq = dsq; s = sa; t = tb; }
            };
            tryPair(0.0f, clamp(d2r / d2d2, 0.0f, 1.0f));
            tryPair(1.0f, clamp(d2.dot(b.end - a.start) / d2d2, 0.0f, 1.0f));
            tryPair(clamp(d1r / d1d1, 0.0f, 1.0f), 0.0f);
            tryPair(clamp(d1.dot(b.end - a.start) / d1d1, 0.0f, 1.0f), 1.0f);
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

    if (dist > kEpsilon) {
        result.manifold.normal = diff / dist;
    } else {
        Vector3 axis = d1.cross(d2);
        f32 axisLen = axis.length();
        if (axisLen > kEpsilon) result.manifold.normal = axis / axisLen;
        else result.manifold.normal = kVector3UnitY;
    }
    result.manifold.contacts[0].point = pB + result.manifold.normal * b.radius;
    result.manifold.contacts[0].penetration = radiusSum - dist;
    result.manifold.contactCount = 1;
    return result;
}

} // namespace primeon::math
