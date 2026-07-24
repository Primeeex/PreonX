#pragma once

#include "primeon/solver/solver_config.hpp"
#include "primeon/constraints/contact_constraint.hpp"

namespace primeon::math {

// ── Position Stabilization ──────────────────────────────────────────────
//
// Two methods for correcting penetration drift:
//
// 1. Baumgarte Stabilization:
//    Adds velocity bias = baumgarte/dt * max(0, penetration + slop)
//    Simple, effective, but adds energy to the system.
//
// 2. Split Impulse:
//    Separates velocity correction (real impulse) from position correction
//    (accumulated position impulse applied directly to positions).
//    No energy addition, more stable, but more complex.
//
// The solver applies one or both based on configuration.

// ── Baumgarte ───────────────────────────────────────────────────────────

/// Computes the Baumgarte position correction bias for a contact constraint.
/// bias = baumgarte / dt * max(0, penetration + slop)
///
/// This bias is added to the velocity constraint to drive bodies apart.
/// The baumgarte factor controls how aggressively positions are corrected:
///   - 0.1: Gentle, stable but slow drift correction
///   - 0.2: Default, good balance
///   - 0.3: Aggressive, fast correction but may cause jitter
[[nodiscard]] constexpr f32 computeBaumgarteBias(
    f32 penetration, f32 slop, f32 baumgarte, f32 dt) noexcept
{
    if (dt < kEpsilon) return 0.0f;
    f32 correction = max(0.0f, penetration + slop);
    return (baumgarte / dt) * correction;
}

/// Applies Baumgarte stabilization to a contact constraint.
/// Sets the velocityBias field based on penetration depth and config.
inline void applyBaumgarte(
    ContactConstraint& cc,
    const SolverConfig& config,
    f32 dt) noexcept
{
    cc.slop = config.penetrationSlop;
    cc.velocityBias = computeBaumgarteBias(
        cc.penetration, config.penetrationSlop,
        config.baumgarteFactor, dt);
}

// ── Split Impulse ───────────────────────────────────────────────────────

/// State for split impulse position correction.
/// Accumulates position-level impulses across iterations.
struct SplitImpulseState {
    static constexpr u32 kMaxContacts = 256;

    Vector3 positionCorrection[kMaxContacts] = {};  // accumulated position impulse per contact
    u32 count = 0;

    constexpr SplitImpulseState() noexcept = default;

    void reset() noexcept { count = 0; }

    /// Adds a contact for position tracking.
    void addContact(u32 index) noexcept {
        if (index < kMaxContacts) {
            positionCorrection[index] = kVector3Zero;
            if (index >= count) count = index + 1;
        }
    }

    /// Returns the position correction for a contact.
    [[nodiscard]] constexpr const Vector3& getCorrection(u32 index) const noexcept {
        return (index < kMaxContacts) ? positionCorrection[index] : kVector3Zero;
    }

    /// Adds to the position correction for a contact.
    void accumulateCorrection(u32 index, const Vector3& correction) noexcept {
        if (index < kMaxContacts) {
            positionCorrection[index] += correction;
        }
    }
};

/// Computes the split impulse position correction for a contact.
/// Returns the position-level impulse that should be applied directly to positions.
[[nodiscard]] inline Vector3 computeSplitImpulseCorrection(
    f32 penetration, f32 slop,
    const Vector3& normal,
    f32 baumgartePosition,
    f32 dt) noexcept
{
    if (dt < kEpsilon) return kVector3Zero;

    f32 correction = max(0.0f, penetration + slop);
    if (correction < kEpsilon) return kVector3Zero;

    // Position impulse magnitude
    f32 impulse = (baumgartePosition / dt) * correction;
    return normal * impulse;
}

/// Applies split impulse: separates velocity correction from position correction.
/// The velocity impulse corrects relative velocity.
/// The position impulse is accumulated and applied directly to positions later.
inline void applySplitImpulse(
    ContactConstraint& cc,
    [[maybe_unused]] Vector3& posA, [[maybe_unused]] Vector3& posB,
    [[maybe_unused]] const InverseMassData& massA, [[maybe_unused]] const InverseMassData& massB,
    const SolverConfig& config,
    f32 dt,
    SplitImpulseState& splitState,
    u32 contactIndex) noexcept
{
    if (!config.splitImpulseEnabled) {
        // Fall back to Baumgarte
        applyBaumgarte(cc, config, dt);
        return;
    }

    splitState.addContact(contactIndex);

    // Check if penetration exceeds split impulse threshold
    if (cc.penetration < config.splitImpulsePenetrationThreshold) {
        // Severe penetration: use Baumgarte for velocity correction
        applyBaumgarte(cc, config, dt);
        return;
    }

    // Compute position correction
    Vector3 correction = computeSplitImpulseCorrection(
        cc.penetration, config.penetrationSlop,
        cc.normal, config.baumgarteFactorPosition, dt);

    // Accumulate position correction
    splitState.accumulateCorrection(contactIndex, correction);

    // Set zero velocity bias (position correction handled separately)
    cc.velocityBias = 0.0f;
}

/// Applies accumulated split impulse position corrections to body positions.
inline void applySplitImpulsePositions(
    Vector3& posA, Vector3& posB,
    const InverseMassData& massA, const InverseMassData& massB,
    const Vector3& correction) noexcept
{
    f32 totalInverseMass = massA.inverseMass + massB.inverseMass;
    if (totalInverseMass < kEpsilon) return;

    f32 ratioA = massA.inverseMass / totalInverseMass;
    f32 ratioB = massB.inverseMass / totalInverseMass;

    posA += correction * ratioA;
    posB -= correction * ratioB;
}

} // namespace primeon::math
