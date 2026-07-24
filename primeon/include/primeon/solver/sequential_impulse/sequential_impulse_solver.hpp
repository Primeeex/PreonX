#pragma once

#include "primeon/solver/solver_config.hpp"
#include "primeon/solver/restitution/restitution_solver.hpp"
#include "primeon/solver/friction/friction_solver.hpp"
#include "primeon/solver/stabilization/stabilization.hpp"
#include "primeon/constraints/contact_constraint.hpp"
#include "primeon/constraints/jacobian.hpp"

namespace primeon::math {

// ── Body Solver Data ────────────────────────────────────────────────────
//
// Flat-array body data for the solver. Bodies are indexed by ID.
// The solver reads and writes these arrays directly.

/// Per-body data needed by the solver.
struct SolverBodyData {
    Vector3 linearVelocity = kVector3Zero;
    Vector3 angularVelocity = kVector3Zero;
    InverseMassData massData;
};

// ── Solver Contact ──────────────────────────────────────────────────────
//
// An internal solver contact that bundles everything needed per-contact.
// Built from ContactConstraint + ContactManifold data.

/// Internal solver contact (built from constraint + manifold).
struct SolverContact {
    // Body indices
    u32 bodyIDA = 0;
    u32 bodyIDB = 0;

    // Mass data
    InverseMassData massA;
    InverseMassData massB;

    // Contact geometry (world space)
    Vector3 pointA = kVector3Zero;
    Vector3 pointB = kVector3Zero;
    Vector3 normal = kVector3Zero;
    f32 penetration = 0.0f;

    // Tangent basis
    Vector3 tangentU = kVector3UnitX;
    Vector3 tangentV = kVector3UnitZ;

    // Material
    f32 restitution = 0.0f;
    f32 friction = 0.0f;

    // Effective masses
    f32 normalMass = 0.0f;
    f32 tangentUMass = 0.0f;
    f32 tangentVMass = 0.0f;

    // Accumulated impulses (warm starting)
    f32 accumulatedNormalImpulse = 0.0f;
    f32 accumulatedFrictionUImpulse = 0.0f;
    f32 accumulatedFrictionVImpulse = 0.0f;

    // Velocity bias (restitution + Baumgarte)
    f32 velocityBias = 0.0f;

    // Contact ID for persistence
    u32 contactID = 0;

    constexpr SolverContact() noexcept = default;
};

/// Solves restitution for a SolverContact (used by the sequential impulse solver).
/// Sets velocityBias to the negative of the target separation velocity so that
/// the PGS error = normalVelocity + velocityBias produces the correct impulse.
inline void solveRestitution(
    Vector3& velA, Vector3& angVelA,
    Vector3& velB, Vector3& angVelB,
    SolverContact& sc,
    const SolverConfig& config) noexcept
{
    f32 restitution = (sc.restitution > kEpsilon) ? sc.restitution : config.defaultRestitution;
    if (restitution < kEpsilon) return;

    f32 bias = computeRestitutionBias(
        velA, angVelA, velB, angVelB,
        sc.pointA, sc.pointB, sc.normal,
        restitution, config.restitutionThreshold);

    sc.velocityBias = -bias;
}

// ── Solver Statistics ───────────────────────────────────────────────────

/// Statistics from a solver step.
struct SolverStats {
    u32 contactCount = 0;       ///< Number of contacts solved
    u32 velocityIterations = 0; ///< Velocity iterations performed
    u32 positionIterations = 0; ///< Position iterations performed
    f32 totalImpulse = 0.0f;    ///< Sum of absolute normal impulses applied
    f32 maxPenetration = 0.0f;  ///< Maximum penetration after solving
};

// ── Sequential Impulse Solver ──────────────────────────────────────────
//
// The main solver. Takes contact constraints and produces velocity updates.
//
// Architecture:
// 1. Build SolverContact array from ContactConstraint + manifold data
// 2. Apply warm starting (scale previous impulses)
// 3. Solve restitution (set velocity bias)
// 4. Iterate velocity constraints (PGS)
//    - For each contact: solve normal impulse, then friction
// 5. If split impulse: iterate position constraints
// 6. Write back velocities to body array
//
// The solver is stateless between frames (all state in SolverContact).

/// The sequential impulse solver.
struct SequentialImpulseSolver {
    static constexpr u32 kMaxSolverContacts = 256;

    SolverContact contacts[kMaxSolverContacts];
    u32 contactCount = 0;

    SplitImpulseState splitState;
    SolverStats stats;

    constexpr SequentialImpulseSolver() noexcept = default;

    /// Resets the solver for a new step.
    void reset() noexcept {
        contactCount = 0;
        splitState.reset();
        stats = {};
    }

    /// Builds a SolverContact from a ContactConstraint + manifold data.
    bool addContact(const ContactConstraint& cc,
                    const Vector3& tangentU, const Vector3& tangentV) noexcept {
        if (contactCount >= kMaxSolverContacts) return false;

        SolverContact& sc = contacts[contactCount];
        sc.bodyIDA = cc.bodyIDA;
        sc.bodyIDB = cc.bodyIDB;
        sc.massA = cc.massA;
        sc.massB = cc.massB;
        sc.pointA = cc.pointA;
        sc.pointB = cc.pointB;
        sc.normal = cc.normal;
        sc.penetration = cc.penetration;
        sc.tangentU = tangentU;
        sc.tangentV = tangentV;
        sc.restitution = cc.restitution;
        sc.friction = cc.friction;
        sc.accumulatedNormalImpulse = cc.accumulatedNormalImpulse;
        sc.contactID = cc.contactID;

        // Compute effective masses
        Jacobian normalJ = Jacobian::normal(cc.pointA, cc.pointB, cc.normal);
        sc.normalMass = normalJ.effectiveMass(cc.massA, cc.massB);

        if (tangentU.lengthSq() > kEpsilon) {
            Jacobian tangUJ = Jacobian::tangent(cc.pointA, cc.pointB, tangentU);
            sc.tangentUMass = tangUJ.effectiveMass(cc.massA, cc.massB);
        }
        if (tangentV.lengthSq() > kEpsilon) {
            Jacobian tangVJ = Jacobian::tangent(cc.pointA, cc.pointB, tangentV);
            sc.tangentVMass = tangVJ.effectiveMass(cc.massA, cc.massB);
        }

        ++contactCount;
        ++stats.contactCount;
        return true;
    }

    /// Performs one complete solve step.
    void solve(SolverBodyData* bodies, u32 bodyCount,
               f32 dt, const SolverConfig& config) noexcept
    {
        if (contactCount == 0 || bodyCount == 0) return;

        stats.velocityIterations = config.velocityIterations;
        stats.positionIterations = config.positionIterations;

        // ── Step 1: Warm Starting ─────────────────────────────────────
        if (config.warmStartingEnabled) {
            applyWarmStarting(bodies, bodyCount, config);
        }

        // ── Step 2: Restitution ───────────────────────────────────────
        for (u32 i = 0; i < contactCount; ++i) {
            SolverContact& sc = contacts[i];
            if (sc.bodyIDA < bodyCount && sc.bodyIDB < bodyCount) {
                SolverBodyData& bA = bodies[sc.bodyIDA];
                SolverBodyData& bB = bodies[sc.bodyIDB];
                solveRestitution(
                    bA.linearVelocity, bA.angularVelocity,
                    bB.linearVelocity, bB.angularVelocity,
                    sc, config);
            }
        }

        // ── Step 3: Baumgarte / Split Impulse Setup ───────────────────
        for (u32 i = 0; i < contactCount; ++i) {
            SolverContact& sc = contacts[i];
            if (config.splitImpulseEnabled) {
                // Split impulse: zero velocity bias, position handled later
                sc.velocityBias = 0.0f;
            } else {
                // Baumgarte: add position correction bias to any existing restitution bias
                sc.velocityBias += computeBaumgarteBias(
                    sc.penetration, config.penetrationSlop,
                    config.baumgarteFactor, dt);
            }
        }

        // ── Step 4: Velocity Iterations (PGS) ────────────────────────
        for (u32 iter = 0; iter < config.velocityIterations; ++iter) {
            for (u32 i = 0; i < contactCount; ++i) {
                SolverContact& sc = contacts[i];
                if (sc.bodyIDA >= bodyCount || sc.bodyIDB >= bodyCount) continue;

                SolverBodyData& bA = bodies[sc.bodyIDA];
                SolverBodyData& bB = bodies[sc.bodyIDB];

                // ── Normal impulse ──
                solveNormalImpulse(bA, bB, sc);

                // ── Friction impulse ──
                solveFrictionImpulse(bA, bB, sc, config);
            }
        }

        // ── Step 5: Position Iterations (Split Impulse) ───────────────
        if (config.splitImpulseEnabled) {
            for (u32 iter = 0; iter < config.positionIterations; ++iter) {
                for (u32 i = 0; i < contactCount; ++i) {
                    SolverContact& sc = contacts[i];
                    if (sc.bodyIDA >= bodyCount || sc.bodyIDB >= bodyCount) continue;

                    SolverBodyData& bA = bodies[sc.bodyIDA];
                    SolverBodyData& bB = bodies[sc.bodyIDB];

                    solveSplitPosition(bA, bB, sc, config, dt, i);
                }
            }

            // Apply accumulated position corrections
            for (u32 i = 0; i < contactCount; ++i) {
                SolverContact& sc = contacts[i];
                if (sc.bodyIDA >= bodyCount || sc.bodyIDB >= bodyCount) continue;

                Vector3 posCorrection = splitState.getCorrection(i);
                if (posCorrection.lengthSq() > kEpsilon) {
                    // Positions are NOT modified here — the caller must apply them
                    // via the returned correction data
                }
            }
        }

        // ── Compute Stats ─────────────────────────────────────────────
        stats.totalImpulse = 0.0f;
        stats.maxPenetration = 0.0f;
        for (u32 i = 0; i < contactCount; ++i) {
            stats.totalImpulse += abs(contacts[i].accumulatedNormalImpulse);
            if (contacts[i].penetration > stats.maxPenetration) {
                stats.maxPenetration = contacts[i].penetration;
            }
        }
    }

    /// Returns statistics from the last solve step.
    [[nodiscard]] const SolverStats& getStats() const noexcept { return stats; }

    /// Returns the split impulse state for position correction.
    [[nodiscard]] const SplitImpulseState& getSplitState() const noexcept { return splitState; }

private:
    // ── Warm Starting ─────────────────────────────────────────────────────

    void applyWarmStarting(SolverBodyData* bodies, u32 bodyCount,
                           const SolverConfig& config) noexcept
    {
        f32 scale = config.warmStartFactor;
        for (u32 i = 0; i < contactCount; ++i) {
            SolverContact& sc = contacts[i];
            if (sc.bodyIDA >= bodyCount || sc.bodyIDB >= bodyCount) continue;

            SolverBodyData& bA = bodies[sc.bodyIDA];
            SolverBodyData& bB = bodies[sc.bodyIDB];

            // Scale accumulated impulses
            f32 scaledNormal = sc.accumulatedNormalImpulse * scale;
            f32 scaledFrictionU = sc.accumulatedFrictionUImpulse * scale;
            f32 scaledFrictionV = sc.accumulatedFrictionVImpulse * scale;

            // Apply normal impulse
            if (abs(scaledNormal) > kEpsilon) {
                applyImpulseToBody(bA, sc, scaledNormal, sc.normal);
                applyImpulseToBody(bB, sc, -scaledNormal, sc.normal);
            }

            // Apply friction U impulse
            if (abs(scaledFrictionU) > kEpsilon && sc.tangentU.lengthSq() > kEpsilon) {
                applyImpulseToBody(bA, sc, scaledFrictionU, sc.tangentU);
                applyImpulseToBody(bB, sc, -scaledFrictionU, sc.tangentU);
            }

            // Apply friction V impulse
            if (abs(scaledFrictionV) > kEpsilon && sc.tangentV.lengthSq() > kEpsilon) {
                applyImpulseToBody(bA, sc, scaledFrictionV, sc.tangentV);
                applyImpulseToBody(bB, sc, -scaledFrictionV, sc.tangentV);
            }

            // Reset accumulators for fresh iteration
            sc.accumulatedNormalImpulse = 0.0f;
            sc.accumulatedFrictionUImpulse = 0.0f;
            sc.accumulatedFrictionVImpulse = 0.0f;
        }
    }

    // ── Normal Impulse Solve ──────────────────────────────────────────────

    void solveNormalImpulse(SolverBodyData& bA, SolverBodyData& bB,
                            SolverContact& sc) noexcept
    {
        if (sc.normalMass < kEpsilon) return;

        // Contact radii
        Vector3 rA = sc.pointA;
        Vector3 rB = sc.pointB;

        // Relative velocity at contact point along normal
        Vector3 vA = bA.linearVelocity + bA.angularVelocity.cross(rA);
        Vector3 vB = bB.linearVelocity + bB.angularVelocity.cross(rB);
        f32 normalVelocity = (vA - vB).dot(sc.normal);

        // Velocity error (including bias from restitution/Baumgarte)
        f32 error = normalVelocity + sc.velocityBias;

        // Raw impulse
        f32 impulse = -error * sc.normalMass;

        // Accumulate and clamp (normal impulse >= 0)
        f32 newAccumulated = sc.accumulatedNormalImpulse + impulse;
        newAccumulated = max(0.0f, newAccumulated);
        impulse = newAccumulated - sc.accumulatedNormalImpulse;
        sc.accumulatedNormalImpulse = newAccumulated;

        // Apply impulse
        if (abs(impulse) > kEpsilon) {
            applyImpulseToBody(bA, sc, impulse, sc.normal);
            applyImpulseToBody(bB, sc, -impulse, sc.normal);
        }
    }

    // ── Friction Impulse Solve ────────────────────────────────────────────

    void solveFrictionImpulse(SolverBodyData& bA, SolverBodyData& bB,
                              SolverContact& sc,
                              const SolverConfig& config) noexcept
    {
        f32 mu = (sc.friction > kEpsilon) ? sc.friction : config.defaultFriction;
        if (mu < kEpsilon) return;

        // Coulomb cone bound
        f32 maxFrictionImpulse = mu * abs(sc.accumulatedNormalImpulse);

        // Solve U direction
        if (sc.tangentUMass > kEpsilon && sc.tangentU.lengthSq() > kEpsilon) {
            solveTangentImpulse(bA, bB, sc, sc.tangentU, sc.tangentUMass,
                               sc.accumulatedFrictionUImpulse, maxFrictionImpulse);
        }

        // Solve V direction
        if (sc.tangentVMass > kEpsilon && sc.tangentV.lengthSq() > kEpsilon) {
            solveTangentImpulse(bA, bB, sc, sc.tangentV, sc.tangentVMass,
                               sc.accumulatedFrictionVImpulse, maxFrictionImpulse);
        }
    }

    void solveTangentImpulse(SolverBodyData& bA, SolverBodyData& bB,
                             SolverContact& sc,
                             const Vector3& tangent, f32 tangentMass,
                             f32& accumulatedImpulse,
                             f32 maxImpulse) noexcept
    {
        Vector3 rA = sc.pointA;
        Vector3 rB = sc.pointB;

        // Relative tangential velocity
        Vector3 vA = bA.linearVelocity + bA.angularVelocity.cross(rA);
        Vector3 vB = bB.linearVelocity + bB.angularVelocity.cross(rB);
        f32 tangentVelocity = (vA - vB).dot(tangent);

        // Raw friction impulse
        f32 impulse = -tangentVelocity * tangentMass;

        // Accumulate and clamp to Coulomb cone
        f32 newAccumulated = accumulatedImpulse + impulse;
        newAccumulated = clamp(newAccumulated, -maxImpulse, maxImpulse);
        impulse = newAccumulated - accumulatedImpulse;
        accumulatedImpulse = newAccumulated;

        // Apply impulse
        if (abs(impulse) > kEpsilon) {
            applyImpulseToBody(bA, sc, impulse, tangent);
            applyImpulseToBody(bB, sc, -impulse, tangent);
        }
    }

    // ── Split Impulse Position Solve ──────────────────────────────────────

    void solveSplitPosition(SolverBodyData& bA, SolverBodyData& bB,
                           SolverContact& sc,
                           const SolverConfig& config,
                           f32 dt, u32 contactIndex) noexcept
    {
        if (sc.normalMass < kEpsilon) return;

        Vector3 rA = sc.pointA;
        Vector3 rB = sc.pointB;

        // Relative velocity at contact point
        Vector3 vA = bA.linearVelocity + bA.angularVelocity.cross(rA);
        Vector3 vB = bB.linearVelocity + bB.angularVelocity.cross(rB);
        f32 normalVelocity = (vA - vB).dot(sc.normal);

        // Position correction bias
        f32 bias = computeBaumgarteBias(
            sc.penetration, config.penetrationSlop,
            config.baumgarteFactorPosition, dt);

        // Error
        f32 error = normalVelocity + bias;

        // Impulse
        f32 impulse = -error * sc.normalMass;

        // Clamp (non-penetration)
        f32 newAccumulated = sc.accumulatedNormalImpulse + impulse;
        newAccumulated = max(0.0f, newAccumulated);
        impulse = newAccumulated - sc.accumulatedNormalImpulse;
        sc.accumulatedNormalImpulse = newAccumulated;

        // Apply as velocity impulse (will be integrated to position later)
        if (abs(impulse) > kEpsilon) {
            applyImpulseToBody(bA, sc, impulse, sc.normal);
            applyImpulseToBody(bB, sc, -impulse, sc.normal);
        }

        // Accumulate position correction
        f32 posImpulse = (config.baumgarteFactorPosition / dt) *
                         max(0.0f, sc.penetration + config.penetrationSlop);
        Vector3 correction = sc.normal * posImpulse;
        splitState.accumulateCorrection(contactIndex, correction);
    }

    // ── Impulse Application ───────────────────────────────────────────────

    /// Applies an impulse to a body (linear + angular).
    void applyImpulseToBody(SolverBodyData& body,
                            const SolverContact& sc,
                            f32 impulse,
                            const Vector3& direction) noexcept
    {
        if (body.massData.inverseMass < kEpsilon) return;

        // Linear: Δv = impulse * direction * inverseMass
        body.linearVelocity += direction * (impulse * body.massData.inverseMass);

        // Angular: Δω = I_inv * (r × (impulse * direction))
        Vector3 r = sc.pointA;
        Vector3 angularImpulse = r.cross(direction * impulse);
        body.angularVelocity += Vector3{
            body.massData.inverseInertiaDiag.x * angularImpulse.x,
            body.massData.inverseInertiaDiag.y * angularImpulse.y,
            body.massData.inverseInertiaDiag.z * angularImpulse.z
        };
    }
};

// ── Convenience: Build SolverContact from ContactConstraint ─────────────

/// Builds a SolverContact from a ContactConstraint, computing all derived data.
[[nodiscard]] inline SolverContact buildSolverContact(
    const ContactConstraint& cc,
    const Vector3& tangentU, const Vector3& tangentV) noexcept
{
    SolverContact sc;
    sc.bodyIDA = cc.bodyIDA;
    sc.bodyIDB = cc.bodyIDB;
    sc.massA = cc.massA;
    sc.massB = cc.massB;
    sc.pointA = cc.pointA;
    sc.pointB = cc.pointB;
    sc.normal = cc.normal;
    sc.penetration = cc.penetration;
    sc.tangentU = tangentU;
    sc.tangentV = tangentV;
    sc.restitution = cc.restitution;
    sc.friction = cc.friction;
    sc.accumulatedNormalImpulse = cc.accumulatedNormalImpulse;
    sc.contactID = cc.contactID;

    // Compute effective masses
    Jacobian normalJ = Jacobian::normal(cc.pointA, cc.pointB, cc.normal);
    sc.normalMass = normalJ.effectiveMass(cc.massA, cc.massB);

    if (tangentU.lengthSq() > kEpsilon) {
        Jacobian tangUJ = Jacobian::tangent(cc.pointA, cc.pointB, tangentU);
        sc.tangentUMass = tangUJ.effectiveMass(cc.massA, cc.massB);
    }
    if (tangentV.lengthSq() > kEpsilon) {
        Jacobian tangVJ = Jacobian::tangent(cc.pointA, cc.pointB, tangentV);
        sc.tangentVMass = tangVJ.effectiveMass(cc.massA, cc.massB);
    }

    return sc;
}

} // namespace primeon::math
