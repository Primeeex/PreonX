#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/vector/vector3.hpp"
#include "primeon/collision/algorithms/gjk.hpp"
#include "primeon/collision/contact.hpp"

namespace primeon::math {

// ── EPA (Expanding Polytope Algorithm) ───────────────────────────────────────
//
// After GJK detects intersection, EPA computes the penetration vector
// (normal × depth) by expanding a polytope around the origin.
//
// Starting from the GJK tetrahedron, it iteratively:
// 1. Finds the face closest to the origin.
// 2. Queries the support point beyond that face.
// 3. Removes all faces visible from the new point.
// 4. Adds new faces connecting the new point to exposed edges.
//
// Converges when the new support point projects within tolerance of the
// closest face. Falls back to face normal × depth if max iterations reached.

inline constexpr u32 kEPAMaxIterations = 64;
inline constexpr f32 kEPAEpsilon = 1e-4f;
inline constexpr u32 kEPAMaxFaces = 128;
inline constexpr u32 kEPAMaxVertices = 128;

/// A face of the EPA polytope (indices into a vertex array).
struct EPAFace {
    u32 vertices[3] = {};
    Vector3 normal = kVector3Zero;
    f32 distance = 0.0f;

    constexpr EPAFace() noexcept = default;
    constexpr EPAFace(u32 v0, u32 v1, u32 v2, const Vector3& n, f32 d) noexcept
        : vertices{v0, v1, v2}, normal(n), distance(d) {}
};

/// An edge of the EPA polytope (for boundary extraction).
struct EPAEdge {
    u32 v0 = 0;
    u32 v1 = 0;

    constexpr EPAEdge() noexcept = default;
    constexpr EPAEdge(u32 a, u32 b) noexcept : v0(a), v1(b) {}
};

namespace epa_detail {

/// Compute face normal and distance from origin.
[[nodiscard]] inline bool computeFace(const Vector3* verts, u32 v0, u32 v1, u32 v2,
                                        Vector3& outNormal, f32& outDist) noexcept {
    Vector3 ab = verts[v1] - verts[v0];
    Vector3 ac = verts[v2] - verts[v0];
    outNormal = ab.cross(ac);
    f32 len = outNormal.length();
    if (len < kEpsilon) return false;
    outNormal = outNormal / len;
    outDist = verts[v0].dot(outNormal);
    if (outDist < 0.0f) {
        outNormal = -outNormal;
        outDist = -outDist;
    }
    return true;
}

/// Check if a face is visible from a point (point is on the positive / outward side).
[[nodiscard]] inline bool isFaceVisible(const Vector3& point,
                                         const Vector3& normal, f32 dist) noexcept {
    return point.dot(normal) - dist > kEPAEpsilon;
}

/// Find edge silhouette: edges visible from the new point but adjacent to a non-visible face.
inline void findSilhouette(const EPAFace* faces, u32 faceCount, const bool* visible,
                            [[maybe_unused]] const Vector3* verts, [[maybe_unused]] u32 newIdx,
                            EPAEdge* silhouette, u32& silhouetteCount) noexcept {
    silhouetteCount = 0;
    for (u32 i = 0; i < faceCount; ++i) {
        if (!visible[i]) continue;
        const EPAFace& f = faces[i];
        for (int e = 0; e < 3; ++e) {
            u32 v0 = f.vertices[e];
            u32 v1 = f.vertices[(e + 1) % 3];
            bool adjVisible = false;
            for (u32 j = 0; j < faceCount; ++j) {
                if (j == i || !visible[j]) continue;
                const EPAFace& f2 = faces[j];
                for (int e2 = 0; e2 < 3; ++e2) {
                    if (f2.vertices[e2] == v1 && f2.vertices[(e2 + 1) % 3] == v0) {
                        adjVisible = true;
                        break;
                    }
                }
                if (adjVisible) break;
            }
            if (!adjVisible) {
                silhouette[silhouetteCount++] = EPAEdge(v0, v1);
            }
        }
    }
}

} // namespace epa_detail

/// EPA result structure.
struct EPAResult {
    bool converged = false;
    Vector3 normal = kVector3Zero;
    f32 depth = 0.0f;
    Vector3 contactPoint = kVector3Zero;
};

/// Runs EPA starting from a GJK simplex that is known to contain the origin.
/// supportFunc has the same signature as for GJK.
template <typename SupportFunc>
[[nodiscard]] inline EPAResult epa(const GJKSimplex& gjkSimplex,
                                    SupportFunc support) noexcept {
    EPAResult result;
    if (gjkSimplex.count < 4) return result;

    Vector3 verts[kEPAMaxVertices];
    EPAFace faces[kEPAMaxFaces];
    bool visible[kEPAMaxFaces];
    u32 vertCount = 0;
    u32 faceCount = 0;

    for (u32 i = 0; i < gjkSimplex.count && i < kEPAMaxVertices; ++i)
        verts[vertCount++] = gjkSimplex.vertices[i];

    auto addFace = [&](u32 v0, u32 v1, u32 v2) -> bool {
        if (faceCount >= kEPAMaxFaces) return false;
        Vector3 n;
        f32 d;
        if (!epa_detail::computeFace(verts, v0, v1, v2, n, d)) return false;
        faces[faceCount++] = EPAFace(v0, v1, v2, n, d);
        return true;
    };

    addFace(0, 1, 2);
    addFace(0, 2, 3);
    addFace(0, 3, 1);
    addFace(1, 3, 2);

    for (u32 iter = 0; iter < kEPAMaxIterations; ++iter) {
        u32 closestFace = 0;
        f32 minDist = faces[0].distance;
        for (u32 i = 1; i < faceCount; ++i) {
            if (faces[i].distance < minDist) {
                minDist = faces[i].distance;
                closestFace = i;
            }
        }

        Vector3 searchDir = faces[closestFace].normal;
        Vector3 newPoint = support(searchDir);

        f32 newProj = newPoint.dot(searchDir);
        if (newProj - minDist < kEPAEpsilon) {
            result.converged = true;
            result.normal = searchDir;
            result.depth = minDist;
            result.contactPoint = searchDir * minDist;
            return result;
        }

        if (vertCount >= kEPAMaxVertices) {
            result.converged = false;
            result.normal = searchDir;
            result.depth = minDist;
            result.contactPoint = searchDir * minDist;
            return result;
        }
        u32 newIdx = vertCount++;
        verts[newIdx] = newPoint;

        for (u32 i = 0; i < faceCount; ++i)
            visible[i] = epa_detail::isFaceVisible(newPoint, faces[i].normal, faces[i].distance);

        EPAEdge silhouette[kEPAMaxFaces * 3];
        u32 silhouetteCount = 0;
        epa_detail::findSilhouette(faces, faceCount, visible, verts,
                                    newIdx, silhouette, silhouetteCount);

        u32 writeIdx = 0;
        for (u32 i = 0; i < faceCount; ++i) {
            if (!visible[i]) faces[writeIdx++] = faces[i];
        }
        faceCount = writeIdx;

        for (u32 i = 0; i < silhouetteCount && faceCount < kEPAMaxFaces; ++i) {
            addFace(newIdx, silhouette[i].v0, silhouette[i].v1);
        }
    }

    u32 closestFace = 0;
    f32 minDist = faces[0].distance;
    for (u32 i = 1; i < faceCount; ++i) {
        if (faces[i].distance < minDist) {
            minDist = faces[i].distance;
            closestFace = i;
        }
    }
    result.converged = false;
    result.normal = faces[closestFace].normal;
    result.depth = minDist;
    result.contactPoint = faces[closestFace].normal * minDist;
    return result;
}

} // namespace primeon::math
