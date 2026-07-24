#pragma once

#include "primeon/math/scalar/scalar.hpp"

namespace primeon::math {

// ── Simulation Statistics ──────────────────────────────────────────────────
//
// Per-frame statistics for profiling and debugging.

/// Statistics from a single simulation step.
struct SimulationStats {
    // Body counts
    u32 totalBodies     = 0;
    u32 activeBodies    = 0;
    u32 sleepingBodies  = 0;
    u32 staticBodies    = 0;
    u32 dynamicBodies   = 0;
    u32 kinematicBodies = 0;

    // Collision
    u32 broadphasePairs    = 0;
    u32 narrowphaseTests   = 0;
    u32 contactCount       = 0;
    u32 manifoldCount      = 0;

    // Islands
    u32 islandCount = 0;

    // Solver
    u32 velocityIterations = 0;
    u32 positionIterations = 0;
    f32 totalImpulse       = 0.0f;
    f32 maxPenetration     = 0.0f;

    constexpr SimulationStats() noexcept = default;
};

} // namespace primeon::math
