#pragma once

#include "primeon/math/scalar/scalar.hpp"

namespace primeon::math {

// ── Friction Model ───────────────────────────────────────────────────────

/// Friction model used by the solver.
enum class FrictionModel : u32 {
    SingleCoefficient = 0,  ///< Single μ for static and dynamic (default)
    StaticDynamic     = 1,  ///< Separate μ_s and μ_k with velocity threshold
};

// ── Solver Configuration ────────────────────────────────────────────────

/// Configuration for the sequential impulse solver.
/// All parameters have sensible defaults matching Box2D/Bullet conventions.
struct SolverConfig {
    // ── Iteration Counts ──────────────────────────────────────────────
    u32 velocityIterations = 8;       ///< Number of velocity-solve passes
    u32 positionIterations = 3;       ///< Number of position-correction passes (if split impulse)

    // ── Penetration / Baumgarte ───────────────────────────────────────
    f32 penetrationSlop = 0.005f;    ///< Allowed penetration before correction (meters)
    f32 baumgarteFactor = 0.2f;      ///< Position correction strength [0, 1]
    f32 baumgarteFactorPosition = 0.8f; ///< Position-only correction factor (split impulse)

    // ── Restitution ───────────────────────────────────────────────────
    f32 restitutionThreshold = 1.0f; ///< Minimum approach velocity for bounce (m/s)
    f32 defaultRestitution = 0.0f;   ///< Default coefficient of restitution
    f32 defaultFriction = 0.5f;      ///< Default coefficient of friction

    // ── Friction ──────────────────────────────────────────────────────
    FrictionModel frictionModel = FrictionModel::SingleCoefficient;
    f32 staticFriction = 0.6f;       ///< Static friction coefficient (if StaticDynamic)
    f32 dynamicFriction = 0.4f;      ///< Dynamic friction coefficient (if StaticDynamic)
    f32 frictionSleepThreshold = 0.01f; ///< Tangential velocity below this = static friction

    // ── Warm Starting ─────────────────────────────────────────────────
    bool warmStartingEnabled = true;  ///< Enable impulse warm starting
    f32 warmStartFactor = 1.0f;       ///< Scale factor for warm-started impulses [0, 1]

    // ── Split Impulse ─────────────────────────────────────────────────
    bool splitImpulseEnabled = false; ///< Enable split impulse (no energy addition)
    f32 splitImpulsePenetrationThreshold = -0.01f; ///< Position correction threshold

    // ── Limits ────────────────────────────────────────────────────────
    u32 maxContacts = 256;            ///< Maximum contacts per solve step
    f32 velocityThreshold = 0.001f;   ///< Velocity below this is treated as zero

    // ── Defaults ──────────────────────────────────────────────────────
    constexpr SolverConfig() noexcept = default;

    /// Preset: Game-quality (8 velocity iterations, Baumgarte).
    [[nodiscard]] static constexpr SolverConfig gameDefault() noexcept {
        SolverConfig cfg;
        cfg.velocityIterations = 8;
        cfg.positionIterations = 3;
        cfg.baumgarteFactor = 0.2f;
        cfg.warmStartingEnabled = true;
        cfg.splitImpulseEnabled = false;
        return cfg;
    }

    /// Preset: High-quality simulation (20 iterations, split impulse).
    [[nodiscard]] static constexpr SolverConfig simulation() noexcept {
        SolverConfig cfg;
        cfg.velocityIterations = 20;
        cfg.positionIterations = 10;
        cfg.baumgarteFactor = 0.1f;
        cfg.warmStartingEnabled = true;
        cfg.splitImpulseEnabled = true;
        return cfg;
    }

    /// Preset: Fast/mobile (4 iterations, no warm start).
    [[nodiscard]] static constexpr SolverConfig fast() noexcept {
        SolverConfig cfg;
        cfg.velocityIterations = 4;
        cfg.positionIterations = 1;
        cfg.baumgarteFactor = 0.3f;
        cfg.warmStartingEnabled = false;
        cfg.splitImpulseEnabled = false;
        return cfg;
    }
};

} // namespace primeon::math
