#pragma once

#include "primeon/constraints/contact_constraint.hpp"

namespace primeon::math {

// ── Jacobian Structures ────────────────────────────────────────────────────
//
// Jacobians map body-space impulses to constraint-space velocity changes.
// For a contact constraint with normal direction n and contact radii rA, rB:
//
//   J = [n^T, (rA × n)^T, -n^T, -(rB × n)^T]
//
// The effective mass is: K = J * M^{-1} * J^T (scalar for single normal constraint)
//
// For friction (tangent direction t):
//   J = [t^T, (rA × t)^T, -t^T, -(rB × t)^T]

/// Linear Jacobian component: maps impulse to linear velocity change.
/// J_linear_A = n, J_linear_B = -n
struct JacobianLinear {
    Vector3 a = kVector3Zero;  // impulse-to-velocity for body A
    Vector3 b = kVector3Zero;  // impulse-to-velocity for body B (usually -a)

    constexpr JacobianLinear() noexcept = default;
    constexpr JacobianLinear(const Vector3& direction) noexcept
        : a(direction), b(-direction) {}
};

/// Angular Jacobian component: maps impulse to angular velocity change.
/// J_angular_A = rA × n, J_angular_B = -(rB × n)
struct JacobianAngular {
    Vector3 a = kVector3Zero;  // rA × direction for body A
    Vector3 b = kVector3Zero;  // -(rB × direction) for body B

    constexpr JacobianAngular() noexcept = default;

    /// Compute from contact radii and direction.
    constexpr JacobianAngular(const Vector3& rA, const Vector3& rB,
                               const Vector3& direction) noexcept
        : a(rA.cross(direction))
        , b(-(rB.cross(direction))) {}
};

/// Full Jacobian for a single constraint (one row of the J matrix).
struct Jacobian {
    JacobianLinear linear;
    JacobianAngular angular;

    constexpr Jacobian() noexcept = default;

    /// Build normal Jacobian from contact geometry.
    static constexpr Jacobian normal(const Vector3& rA, const Vector3& rB,
                                      const Vector3& n) noexcept {
        Jacobian j;
        j.linear = JacobianLinear(n);
        j.angular = JacobianAngular(rA, rB, n);
        return j;
    }

    /// Build tangent Jacobian from contact geometry.
    static constexpr Jacobian tangent(const Vector3& rA, const Vector3& rB,
                                       const Vector3& t) noexcept {
        Jacobian j;
        j.linear = JacobianLinear(t);
        j.angular = JacobianAngular(rA, rB, t);
        return j;
    }

    /// Compute K = J * M^{-1} * J^T for a single body pair.
    /// Returns the effective mass for this constraint.
    [[nodiscard]] constexpr f32 effectiveMass(
        const InverseMassData& mA, const InverseMassData& mB) const noexcept
    {
        // Linear contribution
        f32 kLinear = (linear.a * mA.inverseMass).dot(linear.a)
                    + (linear.b * mB.inverseMass).dot(linear.b);

        // Angular contribution (assumes diagonal inverse inertia)
        Vector3 angA = {angular.a.x * mA.inverseInertiaDiag.x,
                        angular.a.y * mA.inverseInertiaDiag.y,
                        angular.a.z * mA.inverseInertiaDiag.z};
        Vector3 angB = {angular.b.x * mB.inverseInertiaDiag.x,
                        angular.b.y * mB.inverseInertiaDiag.y,
                        angular.b.z * mB.inverseInertiaDiag.z};
        f32 kAngular = angA.dot(angular.a) + angB.dot(angular.b);

        f32 k = kLinear + kAngular;
        return k > kEpsilon ? 1.0f / k : 0.0f;
    }
};

// ── Jacobian Pair ──────────────────────────────────────────────────────────
/// Complete Jacobian pair for a contact: one normal + two tangent (U, V).
struct ContactJacobianPair {
    Jacobian normal;        // normal direction Jacobian
    Jacobian tangentU;      // first tangent direction Jacobian
    Jacobian tangentV;      // second tangent direction Jacobian

    // Effective masses (cached)
    f32 normalMass = 0.0f;
    f32 tangentUMass = 0.0f;
    f32 tangentVMass = 0.0f;

    constexpr ContactJacobianPair() noexcept = default;

    /// Build from contact constraint.
    static ContactJacobianPair fromConstraint(const ContactConstraint& cc,
                                               const Vector3& tangentU,
                                               const Vector3& tangentV) noexcept {
        ContactJacobianPair jp;

        // Contact radii (from body centers to contact point — simplified)
        Vector3 rA = cc.pointA;
        Vector3 rB = cc.pointB;

        jp.normal = Jacobian::normal(rA, rB, cc.normal);
        jp.tangentU = Jacobian::tangent(rA, rB, tangentU);
        jp.tangentV = Jacobian::tangent(rA, rB, tangentV);

        jp.normalMass = jp.normal.effectiveMass(cc.massA, cc.massB);
        jp.tangentUMass = jp.tangentU.effectiveMass(cc.massA, cc.massB);
        jp.tangentVMass = jp.tangentV.effectiveMass(cc.massA, cc.massB);

        return jp;
    }

    /// Compute bias term for position correction.
    /// bias = baumgarte / dt * max(0, penetration + slop)
    [[nodiscard]] constexpr f32 computeBias(f32 penetration, f32 slop,
                                             f32 baumgarte, f32 dt) const noexcept {
        f32 correction = max(0.0f, penetration + slop);
        return (baumgarte / dt) * correction;
    }
};

// ── Jacobian Matrix (for batch solving) ────────────────────────────────────
/// Stores Jacobians for multiple constraints in a flat array for batch processing.
/// For future use when building the full solver.
struct JacobianMatrix {
    static constexpr u32 kMaxConstraints = 256;

    Jacobian rows[kMaxConstraints];
    f32 effectiveMass[kMaxConstraints] = {};
    u32 count = 0;

    constexpr JacobianMatrix() noexcept = default;

    void reset() noexcept { count = 0; }

    [[nodiscard]] bool add(const Jacobian& j, f32 mass) noexcept {
        if (count >= kMaxConstraints) return false;
        rows[count] = j;
        effectiveMass[count] = mass;
        ++count;
        return true;
    }

    /// Get the velocity error for constraint i.
    [[nodiscard]] constexpr f32 velocityError(
        u32 i, const Vector3& vA, const Vector3& wA,
        const Vector3& vB, const Vector3& wB) const noexcept
    {
        if (i >= count) return 0.0f;
        const Jacobian& j = rows[i];
        return j.linear.a.dot(vA) + j.angular.a.dot(wA)
             + j.linear.b.dot(vB) + j.angular.b.dot(wB);
    }
};

} // namespace primeon::math
