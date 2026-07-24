#pragma once

#include "primeon/constraints/contact_constraint.hpp"
#include "primeon/solver/solver_config.hpp"

namespace primeon::math {

// ── Friction Solver ─────────────────────────────────────────────────────
//
// Computes and applies friction impulses at contact points.
// Two friction directions (tangentU, tangentV) are solved independently,
// then projected onto the Coulomb friction cone.
//
// Coulomb model: |f_t| <= mu * f_n
// where f_n is the accumulated normal impulse at the contact.

/// Applies a single friction impulse along a tangent direction.
/// Returns the actual impulse applied (after Coulomb clamping).
[[nodiscard]] inline f32 applyFrictionImpulse(
    Vector3& velA, Vector3& angVelA,
    Vector3& velB, Vector3& angVelB,
    const InverseMassData& massA, const InverseMassData& massB,
    const Vector3& pointA, const Vector3& pointB,
    const Vector3& tangent,
    f32 tangentMass,
    f32 maxFrictionImpulse,
    f32& accumulatedImpulse) noexcept
{
    // Contact radii (from body center to contact point)
    Vector3 rA = pointA;
    Vector3 rB = pointB;

    // Relative tangential velocity at contact point
    Vector3 vA = velA + angVelA.cross(rA);
    Vector3 vB = velB + angVelB.cross(rB);
    Vector3 vRel = vA - vB;
    f32 tangentVelocity = vRel.dot(tangent);

    // Raw friction impulse
    f32 impulse = -tangentVelocity * tangentMass;

    // Accumulate and clamp to Coulomb cone
    f32 newAccumulated = accumulatedImpulse + impulse;
    newAccumulated = clamp(newAccumulated, -maxFrictionImpulse, maxFrictionImpulse);
    impulse = newAccumulated - accumulatedImpulse;
    accumulatedImpulse = newAccumulated;

    // Apply impulse to body A
    if (massA.inverseMass > kEpsilon) {
        velA += tangent * (impulse * massA.inverseMass);
        angVelA += Vector3{
            massA.inverseInertiaDiag.x * rA.cross(tangent).x * impulse,
            massA.inverseInertiaDiag.y * rA.cross(tangent).y * impulse,
            massA.inverseInertiaDiag.z * rA.cross(tangent).z * impulse
        };
    }

    // Apply impulse to body B (opposite direction)
    if (massB.inverseMass > kEpsilon) {
        velB -= tangent * (impulse * massB.inverseMass);
        angVelB -= Vector3{
            massB.inverseInertiaDiag.x * rB.cross(tangent).x * impulse,
            massB.inverseInertiaDiag.y * rB.cross(tangent).y * impulse,
            massB.inverseInertiaDiag.z * rB.cross(tangent).z * impulse
        };
    }

    return impulse;
}

/// Solves friction for a single contact point (both tangent directions).
/// Uses the Coulomb cone model with accumulated normal impulse as the bound.
inline void solveContactFriction(
    Vector3& velA, Vector3& angVelA,
    Vector3& velB, Vector3& angVelB,
    const FrictionConstraint& fc,
    f32 accumulatedNormalImpulse,
    const SolverConfig& config) noexcept
{
    if (config.defaultFriction < kEpsilon && fc.friction < kEpsilon) return;

    f32 mu = (fc.friction > kEpsilon) ? fc.friction : config.defaultFriction;

    // Coulomb cone bound: |f_t| <= mu * f_n
    f32 maxFrictionImpulse = mu * abs(accumulatedNormalImpulse);

    // Solve U direction
    if (fc.tangent.lengthSq() > kEpsilon) {
        (void)applyFrictionImpulse(
            velA, angVelA, velB, angVelB,
            fc.massA, fc.massB,
            fc.pointA, fc.pointB,
            fc.tangent, fc.tangentMass,
            maxFrictionImpulse,
            const_cast<f32&>(fc.accumulatedFrictionImpulse));
    }
}

/// Solves friction for a contact with both tangent U and V directions.
inline void solveContactFrictionUV(
    Vector3& velA, Vector3& angVelA,
    Vector3& velB, Vector3& angVelB,
    const Vector3& pointA, const Vector3& pointB,
    const InverseMassData& massA, const InverseMassData& massB,
    const Vector3& tangentU, const Vector3& tangentV,
    f32 tangentUMass, f32 tangentVMass,
    f32 friction,
    f32 accumulatedNormalImpulse,
    f32& accumFrictionU, f32& accumFrictionV,
    const SolverConfig& config) noexcept
{
    f32 mu = (friction > kEpsilon) ? friction : config.defaultFriction;
    f32 maxFrictionImpulse = mu * abs(accumulatedNormalImpulse);

    // Solve U direction
    if (tangentUMass > kEpsilon && tangentU.lengthSq() > kEpsilon) {
        (void)applyFrictionImpulse(
            velA, angVelA, velB, angVelB,
            massA, massB, pointA, pointB,
            tangentU, tangentUMass, maxFrictionImpulse, accumFrictionU);
    }

    // Solve V direction
    if (tangentVMass > kEpsilon && tangentV.lengthSq() > kEpsilon) {
        (void)applyFrictionImpulse(
            velA, angVelA, velB, angVelB,
            massA, massB, pointA, pointB,
            tangentV, tangentVMass, maxFrictionImpulse, accumFrictionV);
    }
}

} // namespace primeon::math
