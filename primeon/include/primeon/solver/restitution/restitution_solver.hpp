#pragma once

#include "primeon/constraints/contact_constraint.hpp"
#include "primeon/solver/solver_config.hpp"

namespace primeon::math {

// ── Restitution Solver ──────────────────────────────────────────────────
//
// Computes and applies restitution (bounce) impulses at contact points.
// Restitution is only applied during the first frame of contact (impact)
// when the approach velocity exceeds a threshold.
//
// For sustained contacts (resting), restitution is disabled to prevent jitter.
//
// Newton's law: v_sep = -e * v_app
// where e = coefficient of restitution, v_app = approach velocity

/// Computes the restitution velocity bias for a contact.
/// Returns the target separation velocity (positive = separating).
/// Returns 0 if approach velocity is below threshold (resting contact).
[[nodiscard]] inline f32 computeRestitutionBias(
    const Vector3& velA, const Vector3& angVelA,
    const Vector3& velB, const Vector3& angVelB,
    const Vector3& pointA, const Vector3& pointB,
    const Vector3& normal,
    f32 restitution,
    f32 threshold) noexcept
{
    // Relative velocity at contact point along normal
    Vector3 rA = pointA;
    Vector3 rB = pointB;
    Vector3 vA = velA + angVelA.cross(rA);
    Vector3 vB = velB + angVelB.cross(rB);
    f32 normalVelocity = (vA - vB).dot(normal);

    // Only apply restitution for approaching contacts (negative normal velocity)
    if (normalVelocity >= 0.0f) return 0.0f;

    // Check threshold: only bounce if impact is fast enough
    if (abs(normalVelocity) < threshold) return 0.0f;

    // Target separation velocity
    return -restitution * normalVelocity;
}

/// Applies a restitution impulse to correct velocity toward the target.
/// The impulse is bounded to avoid over-correction.
inline void applyRestitutionImpulse(
    Vector3& velA, Vector3& angVelA,
    Vector3& velB, Vector3& angVelB,
    const InverseMassData& massA, const InverseMassData& massB,
    const Vector3& pointA, const Vector3& pointB,
    const Vector3& normal,
    f32 normalMass,
    f32 targetVelocity) noexcept
{
    if (normalMass < kEpsilon) return;

    // Current relative normal velocity
    Vector3 rA = pointA;
    Vector3 rB = pointB;
    Vector3 vA = velA + angVelA.cross(rA);
    Vector3 vB = velB + angVelB.cross(rB);
    f32 currentVelocity = (vA - vB).dot(normal);

    // Velocity error: how far from target
    f32 error = currentVelocity - targetVelocity;

    // Impulse magnitude (clamped to prevent reversing direction)
    f32 impulse = -error * normalMass;

    // Apply to body A
    if (massA.inverseMass > kEpsilon) {
        velA += normal * (impulse * massA.inverseMass);
        angVelA += Vector3{
            massA.inverseInertiaDiag.x * rA.cross(normal).x * impulse,
            massA.inverseInertiaDiag.y * rA.cross(normal).y * impulse,
            massA.inverseInertiaDiag.z * rA.cross(normal).z * impulse
        };
    }

    // Apply to body B (opposite)
    if (massB.inverseMass > kEpsilon) {
        velB -= normal * (impulse * massB.inverseMass);
        angVelB -= Vector3{
            massB.inverseInertiaDiag.x * rB.cross(normal).x * impulse,
            massB.inverseInertiaDiag.y * rB.cross(normal).y * impulse,
            massB.inverseInertiaDiag.z * rB.cross(normal).z * impulse
        };
    }
}

/// Solves restitution for a single contact constraint.
/// Modifies velocityBias of the constraint for use in the normal impulse solve.
inline void solveRestitution(
    Vector3& velA, Vector3& angVelA,
    Vector3& velB, Vector3& angVelB,
    ContactConstraint& cc,
    const SolverConfig& config) noexcept
{
    f32 restitution = (cc.restitution > kEpsilon) ? cc.restitution : config.defaultRestitution;
    if (restitution < kEpsilon) return;

    f32 bias = computeRestitutionBias(
        velA, angVelA, velB, angVelB,
        cc.pointA, cc.pointB, cc.normal,
        restitution, config.restitutionThreshold);

    // Set velocity bias for the normal impulse solve
    cc.velocityBias = bias;
}

} // namespace primeon::math
