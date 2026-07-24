#pragma once

#include "primeon/collision/contact.hpp"
#include "primeon/math/vector/vector3.hpp"
#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/quaternion/quaternion.hpp"

namespace primeon::math {

// ── Contact Constraint ─────────────────────────────────────────────────────
//
// A contact constraint enforces non-penetration between two bodies at a
// contact point. It stores all data needed by the future solver:
// - Body indices and mass properties (inverse mass, inverse inertia)
// - Contact point, normal, and penetration depth
// - Accumulated normal impulse (for warm starting and clamping)
// - Baumgarte bias for position correction
// - Restitution target for bounce

/// Inverse mass properties for a body (cached for solver efficiency).
struct InverseMassData {
    f32 inverseMass = 0.0f;
    Vector3 inverseInertiaDiag = kVector3Zero;  // diagonal of inverse inertia in body frame
    Quaternion inverseInertiaOrientation = Quaternion::identity();  // body orientation for rotation
};

/// Contact constraint between two bodies.
/// Data only — no solving logic.
struct ContactConstraint {
    // Body identification
    u32 bodyIDA = 0;
    u32 bodyIDB = 0;

    // Mass properties (cached from body at constraint creation)
    InverseMassData massA;
    InverseMassData massB;

    // Contact geometry (world space)
    Vector3 pointA = kVector3Zero;   // contact point on body A
    Vector3 pointB = kVector3Zero;   // contact point on body B
    Vector3 normal = kVector3Zero;   // from B to A
    f32 penetration = 0.0f;

    // Constraint metadata
    f32 restitution = 0.0f;          // coefficient of restitution [0,1]
    f32 friction = 0.0f;             // coefficient of friction [0,inf)
    f32 slop = 0.005f;               // position correction allowance
    f32 baumgarte = 0.2f;            // Baumgarte stabilization factor

    // Accumulated impulses (for warm starting)
    f32 accumulatedNormalImpulse = 0.0f;
    f32 lowerNormalImpulse = 0.0f;   // typically 0 (non-penetration)
    f32 upperNormalImpulse = std::numeric_limits<f32>::max();

    // Contact ID for persistence
    u32 contactID = 0;

    // Bias term for position correction (computed per-frame)
    f32 velocityBias = 0.0f;

    // Effective mass (computed per-frame by the solver)
    f32 normalMass = 0.0f;

    constexpr ContactConstraint() noexcept = default;

    /// Returns the relative velocity at the contact point along the normal.
    /// Positive = separating, negative = approaching.
    [[nodiscard]] constexpr f32 relativeVelocity(
        const Vector3& velA, const Vector3& velAngA,
        const Vector3& velB, const Vector3& velAngB) const noexcept
    {
        Vector3 rA = pointA;  // relative to body A center (simplified)
        Vector3 rB = pointB;  // relative to body B center (simplified)
        Vector3 vA = velA + velAngA.cross(rA);
        Vector3 vB = velB + velAngB.cross(rB);
        return (vA - vB).dot(normal);
    }
};

// ── Friction Constraint ────────────────────────────────────────────────────
//
// A friction constraint prevents tangential sliding at a contact point.
// Two friction constraints are needed per contact (U and V tangent directions).
// The friction impulse is bounded by the Coulomb cone: |f| <= mu * normalImpulse.

/// Friction constraint along one tangent direction.
struct FrictionConstraint {
    // Body identification
    u32 bodyIDA = 0;
    u32 bodyIDB = 0;

    // Mass properties
    InverseMassData massA;
    InverseMassData massB;

    // Contact geometry
    Vector3 pointA = kVector3Zero;
    Vector3 pointB = kVector3Zero;
    Vector3 tangent = kVector3Zero;   // tangent direction (U or V)

    // Friction coefficient
    f32 friction = 0.0f;

    // Accumulated impulse
    f32 accumulatedFrictionImpulse = 0.0f;

    // Effective mass
    f32 tangentMass = 0.0f;

    // Coulomb cone bound (set by solver from the paired normal constraint)
    f32 maxFrictionImpulse = 0.0f;

    // Reference to the paired contact constraint
    u32 contactID = 0;

    constexpr FrictionConstraint() noexcept = default;
};

// ── Restitution Constraint ─────────────────────────────────────────────────
//
// Controls bounce velocity at contact. Only active during the first frame
// of contact (impact phase). Once objects are in sustained contact,
// restitution is disabled to prevent jitter.

/// Restitution constraint (velocity-level).
struct RestitutionConstraint {
    // Body identification
    u32 bodyIDA = 0;
    u32 bodyIDB = 0;

    // Mass properties
    InverseMassData massA;
    InverseMassData massB;

    // Contact geometry
    Vector3 pointA = kVector3Zero;
    Vector3 pointB = kVector3Zero;
    Vector3 normal = kVector3Zero;

    // Restitution coefficient
    f32 restitution = 0.0f;

    // Minimum approach velocity threshold
    // (restitution only applied if approach speed exceeds this)
    f32 velocityThreshold = 1.0f;

    // The target separation velocity
    f32 targetVelocity = 0.0f;

    // Effective mass
    f32 normalMass = 0.0f;

    // Whether this constraint is active (first frame of contact)
    bool active = false;

    // Reference to the paired contact constraint
    u32 contactID = 0;

    constexpr RestitutionConstraint() noexcept = default;

    /// Compute target velocity from approach velocity and restitution coefficient.
    void computeTargetVelocity(f32 approachVelocity) noexcept {
        if (abs(approachVelocity) < velocityThreshold) {
            targetVelocity = 0.0f;
            active = false;
        } else {
            targetVelocity = -restitution * approachVelocity;
            active = true;
        }
    }
};

} // namespace primeon::math
