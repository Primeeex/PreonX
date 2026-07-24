#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/vector/vector3.hpp"
#include "primeon/world/rigid_body.hpp"
#include <vector>

namespace primeon::math {

// ── Sleep System ──────────────────────────────────────────────────────────
//
// Evaluates and manages sleep state for dynamic bodies.
//
// A body can sleep when:
// 1. Linear velocity magnitude < linearSleepThreshold
// 2. Angular velocity magnitude < angularSleepThreshold
// 3. Both conditions hold for sleepTime frames
//
// When any body in an island is woken, all bodies in that island are woken.
// When a force/impulse is applied to a sleeping body, it is woken.
// Static and kinematic bodies never sleep.

/// Configuration for the sleep system.
struct SleepConfig {
    f32 linearSleepThreshold  = 0.01f;   ///< m/s — below this, linear velocity is "at rest"
    f32 angularSleepThreshold = 0.01f;   ///< rad/s — below this, angular velocity is "at rest"
    u32 sleepTimeThreshold    = 60;      ///< frames below threshold before sleeping
    bool enabled = true;

    constexpr SleepConfig() noexcept = default;
};

/// Evaluates sleep state for a collection of bodies.
struct SleepSystem {
    SleepConfig config;

    constexpr SleepSystem() noexcept = default;
    explicit constexpr SleepSystem(SleepConfig cfg) noexcept : config(cfg) {}

    /// Returns true if a body is below sleep velocity thresholds.
    [[nodiscard]] bool isBodyAtRest(const RigidBody& body) const noexcept {
        if (!body.isDynamic() || !body.enabled) return false;

        f32 linSpeed = body.linearVelocity.length();
        f32 angSpeed = body.angularVelocity.length();

        return linSpeed < config.linearSleepThreshold &&
               angSpeed < config.angularSleepThreshold;
    }

    /// Updates sleep state for all bodies in a world.
    /// Should be called after the full simulation step.
    void update(std::vector<RigidBody>& bodies) noexcept {
        if (!config.enabled) return;

        for (auto& body : bodies) {
            if (body.id == kInvalidBody) continue;
            if (!body.isDynamic() || !body.enabled) continue;
            if (!body.allowSleep) {
                body.sleepState = SleepState::Awake;
                continue;
            }

            if (isBodyAtRest(body)) {
                if (body.sleepState == SleepState::Awake) {
                    body.sleepState = SleepState::CanSleep;
                    body.sleepTimer = 0;
                }

                if (body.sleepState == SleepState::CanSleep) {
                    ++body.sleepTimer;
                    if (body.sleepTimer >= config.sleepTimeThreshold) {
                        body.sleepState = SleepState::Sleeping;
                        body.linearVelocity = kVector3Zero;
                        body.angularVelocity = kVector3Zero;
                    }
                }
            } else {
                // Body is moving — reset sleep state
                body.sleepState = SleepState::Awake;
                body.sleepTimer = 0;
            }
        }
    }

    /// Wakes all bodies in the given list.
    void wakeAll(std::vector<RigidBody>& bodies) noexcept {
        for (auto& body : bodies) {
            if (body.id == kInvalidBody) continue;
            body.wake();
        }
    }

    /// Wakes a specific body and propagates wake to its island neighbors.
    /// contactPairs contains (bodyA, bodyB) pairs; all bodies connected to
    /// the woken body are also woken.
    void wakeBody(u32 bodyIndex,
                  std::vector<RigidBody>& bodies,
                  const std::pair<u32, u32>* contactPairs,
                  u32 contactCount) noexcept
    {
        if (bodyIndex >= bodies.size()) return;
        bodies[bodyIndex].wake();

        // BFS wake propagation
        std::vector<u32> stack;
        stack.push_back(bodyIndex);

        while (!stack.empty()) {
            u32 current = stack.back();
            stack.pop_back();

            for (u32 c = 0; c < contactCount; ++c) {
                u32 a = contactPairs[c].first;
                u32 b = contactPairs[c].second;

                if (a == current && b < bodies.size() && !bodies[b].isAwake()) {
                    bodies[b].wake();
                    stack.push_back(b);
                } else if (b == current && a < bodies.size() && !bodies[a].isAwake()) {
                    bodies[a].wake();
                    stack.push_back(a);
                }
            }
        }
    }
};

} // namespace primeon::math
