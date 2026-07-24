#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/vector/vector3.hpp"

namespace primeon::math {

// ── GJK (Gilbert-Johnson-Keerthi) ────────────────────────────────────────────
//
// Determines whether two convex shapes intersect by operating on their
// Minkowski difference implicitly via support mappings.
//
// The simplex evolves: point → line → triangle → tetrahedron (3D).
// At each step, the closest feature of the simplex to the origin is computed,
// and a new search direction is chosen toward the origin.
//
// Convergence: O(1) per iteration, typically 10-30 iterations.
// Each iteration costs 1 support function evaluation.

inline constexpr u32 kGJKMaxIterations = 64;
inline constexpr f32 kGJKEpsilon = 1e-6f;

/// A simplex vertex in 3D (up to 4 points for a tetrahedron).
struct GJKSimplex {
    Vector3 vertices[4] = {};
    u32 count = 0;

    constexpr void reset() noexcept { count = 0; }

    constexpr void add(const Vector3& v) noexcept {
        if (count < 4) vertices[count++] = v;
    }

    /// Remove vertex at index by shifting remaining vertices down.
    constexpr void remove(u32 idx) noexcept {
        for (u32 i = idx; i + 1 < count; ++i)
            vertices[i] = vertices[i + 1];
        --count;
    }
};

namespace gjk_detail {

/// Solve closest point on a line (2-simplex) to the origin.
/// Returns the search direction.
[[nodiscard]] inline Vector3 solveLine(GJKSimplex& s) noexcept {
    Vector3 a = s.vertices[1];
    Vector3 b = s.vertices[0];
    Vector3 ab = b - a;
    f32 d = ab.dot(-a);
    if (d <= kGJKEpsilon) {
        s.vertices[0] = a;
        s.count = 1;
        return -a;
    }
    f32 lenSq = ab.lengthSq();
    if (lenSq <= kGJKEpsilon) {
        s.vertices[0] = a;
        s.count = 1;
        return -a;
    }
    if (d >= lenSq) {
        s.vertices[0] = b;
        s.count = 1;
        return -b;
    }
    s.vertices[0] = a + ab * (d / lenSq);
    s.count = 2;
    return ab.cross(-a).cross(ab);
}

/// Solve closest point on a triangle (3-simplex) to the origin.
/// Returns the search direction.
[[nodiscard]] inline Vector3 solveTriangle(GJKSimplex& s) noexcept {
    Vector3 a = s.vertices[2];
    Vector3 b = s.vertices[1];
    Vector3 c = s.vertices[0];
    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 ao = -a;

    f32 d1 = ab.dot(ao);
    f32 d2 = ac.dot(ao);
    if (d1 <= kGJKEpsilon && d2 <= kGJKEpsilon) {
        s.vertices[0] = a;
        s.count = 1;
        return ao;
    }

    Vector3 bc = c - b;
    f32 d3 = bc.dot(-b);
    if (d3 <= kGJKEpsilon && d1 <= d3 + kGJKEpsilon) {
        s.vertices[0] = b;
        s.count = 1;
        return -b;
    }

    f32 vc = d1 * d3 - d2 * d1;
    if (vc <= kGJKEpsilon && d1 > kGJKEpsilon && d3 > kGJKEpsilon) {
        s.vertices[0] = b;
        s.vertices[1] = a;
        s.count = 2;
        return -b;
    }

    f32 d4 = ac.dot(b - a);
    if (d4 <= kGJKEpsilon && d2 <= d4 + kGJKEpsilon) {
        s.vertices[0] = c;
        s.count = 1;
        return -c;
    }

    f32 vb = d4 * d2 - d1 * d4;
    if (vb <= kGJKEpsilon && d2 > kGJKEpsilon && d4 > kGJKEpsilon) {
        s.vertices[0] = c;
        s.vertices[1] = a;
        s.count = 2;
        return -c;
    }

    f32 va = ab.dot(ac) * d3 - d4 * d1;
    if (va <= kGJKEpsilon && (d3 - d1) > kGJKEpsilon) {
        s.vertices[0] = b;
        s.vertices[1] = c;
        s.count = 2;
        return -(b + (c - b) * (d3 / (d3 - d1)));
    }

    f32 denom = va + vb + vc;
    if (denom <= kGJKEpsilon) {
        s.vertices[0] = a;
        s.vertices[1] = b;
        s.vertices[2] = c;
        s.count = 3;
        Vector3 n = ab.cross(ac);
        return n.cross(ab);
    }

    f32 u = va / denom;
    f32 v = vb / denom;
    s.vertices[0] = a + ab * u + ac * v;
    s.count = 3;
    if (s.vertices[0].lengthSq() < kGJKEpsilon * kGJKEpsilon) return kVector3Zero;
    return ab.cross(ac);
}

/// Solve closest point on a tetrahedron (4-simplex) to the origin.
/// Returns the search direction.
[[nodiscard]] inline Vector3 solveTetrahedron(GJKSimplex& s) noexcept {
    Vector3 a = s.vertices[3];
    Vector3 b = s.vertices[2];
    Vector3 c = s.vertices[1];
    Vector3 d = s.vertices[0];

    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 ad = d - a;
    Vector3 ao = -a;

    Vector3 abc = ab.cross(ac);
    Vector3 acd = ac.cross(ad);
    Vector3 adb = ad.cross(ab);

    f32 abcSign = abc.dot(ao);
    f32 acdSign = acd.dot(ao);
    f32 adbSign = adb.dot(ao);

    if (abcSign > kGJKEpsilon) {
        if (acdSign > kGJKEpsilon) {
            s.vertices[0] = a; s.vertices[1] = d; s.vertices[2] = c;
            s.count = 3;
            return solveTriangle(s);
        }
        if (adbSign > kGJKEpsilon) {
            s.vertices[0] = b; s.vertices[1] = a; s.vertices[2] = d;
            s.count = 3;
            return solveTriangle(s);
        }
        s.vertices[0] = b; s.vertices[1] = c; s.vertices[2] = a;
        s.count = 3;
        return solveTriangle(s);
    }

    if (acdSign > kGJKEpsilon) {
        if (adbSign > kGJKEpsilon) {
            s.vertices[0] = c; s.vertices[1] = d; s.vertices[2] = a;
            s.count = 3;
            return solveTriangle(s);
        }
        s.vertices[0] = c; s.vertices[1] = d; s.vertices[2] = a;
        s.count = 3;
        return solveTriangle(s);
    }

    if (adbSign > kGJKEpsilon) {
        s.vertices[0] = b; s.vertices[1] = a; s.vertices[2] = d;
        s.count = 3;
        return solveTriangle(s);
    }

    return kVector3Zero;
}

} // namespace gjk_detail

/// GJK intersection test.
/// supportFunc is a callable: Vector3 supportFunc(const Vector3& dir) -> Vector3
/// that returns the support point of the Minkowski difference in the given direction.
/// Returns true if the shapes intersect.
template <typename SupportFunc>
[[nodiscard]] inline bool gjkIntersect(SupportFunc support) noexcept {
    GJKSimplex simplex;
    Vector3 d(1.0f, 0.0f, 0.0f);

    Vector3 w = support(d);
    if (w.lengthSq() < kGJKEpsilon) return true;
    simplex.add(w);
    d = -w;

    for (u32 i = 0; i < kGJKMaxIterations; ++i) {
        Vector3 a = support(d);
        if (a.dot(d) < kGJKEpsilon) return false;
        simplex.add(a);

        switch (simplex.count) {
            case 2: d = gjk_detail::solveLine(simplex); break;
            case 3: d = gjk_detail::solveTriangle(simplex); break;
            case 4: d = gjk_detail::solveTetrahedron(simplex); break;
            default: return false;
        }

        if (d.lengthSq() < kGJKEpsilon * kGJKEpsilon) return true;
    }
    return false;
}

} // namespace primeon::math
