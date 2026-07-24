#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include <vector>
#include <algorithm>

namespace primeon::math {

// ── Body Manager ──────────────────────────────────────────────────────────
//
// Manages a pool of rigid bodies with stable IDs.
// Bodies are stored in a flat array for cache efficiency.
// Destruction marks slots as free; the free list is reused.
// Body pointers are stable until the next compaction.
//
// The manager does NOT run the simulation — it only stores bodies.
// The PhysicsWorld orchestrates simulation using the manager's data.

/// Statistics for the body manager.
struct BodyManagerStats {
    u32 totalBodies = 0;
    u32 activeBodies = 0;
    u32 sleepingBodies = 0;
    u32 staticBodies = 0;
    u32 dynamicBodies = 0;
    u32 kinematicBodies = 0;
};

/// Manages rigid body storage with stable IDs.
struct BodyManager {
    static constexpr u32 kMaxBodies = 1024;

    std::vector<RigidBody> bodies;
    std::vector<u32> freeList;
    u32 count = 0;

    BodyManager() noexcept {
        bodies.resize(kMaxBodies);
        freeList.reserve(kMaxBodies);
    }

    explicit BodyManager(u32 maxBodies) noexcept {
        bodies.resize(maxBodies);
        freeList.reserve(maxBodies);
    }

    /// Creates a body from a descriptor. Returns the body ID, or kInvalidBody on failure.
    [[nodiscard]] BodyID createBody(const BodyDescriptor& desc) noexcept {
        if (count >= bodies.size()) return kInvalidBody;

        BodyID id;
        if (!freeList.empty()) {
            id = freeList.back();
            freeList.pop_back();
        } else {
            id = count;
        }

        RigidBody& body = bodies[id];
        body = RigidBody{};
        body.id = id;
        body.type = desc.type;
        body.enabled = desc.enabled;
        body.position = desc.position;
        body.rotation = desc.rotation;
        body.material = desc.material;
        body.linearDamping = desc.linearDamping;
        body.angularDamping = desc.angularDamping;
        body.gravityScale = desc.gravityScale;
        body.allowSleep = desc.allowSleep;
        body.userData = desc.userData;
        body.collider = desc.collider;

        // Set mass
        if (desc.type == BodyType::Static || desc.type == BodyType::Kinematic) {
            body.massProperties = MassProperties::fromMass(0.0f);
        } else {
            body.massProperties = MassProperties::fromMass(desc.mass);
        }

        // Set inertia
        if (desc.type == BodyType::Dynamic) {
            body.inertia = desc.inertia;
            Matrix3 rot = desc.rotation.toMatrix3();
            body.inverseInertia = desc.inertia.rotated(rot).inverse();
        } else {
            body.inertia = InertiaTensor(Matrix3::identity());
            body.inverseInertia = InertiaTensor(Matrix3::identity());
        }

        // Set sleep state
        if (desc.type == BodyType::Dynamic) {
            body.sleepState = desc.awake ? SleepState::Awake : SleepState::Sleeping;
        } else {
            body.sleepState = SleepState::Awake;
        }

        ++count;
        return id;
    }

    /// Destroys a body by ID.
    void destroyBody(BodyID id) noexcept {
        if (id >= bodies.size()) return;
        RigidBody& body = bodies[id];
        body = RigidBody{};
        body.id = kInvalidBody;
        freeList.push_back(id);
        if (count > 0) --count;
    }

    /// Returns a mutable reference to a body.
    [[nodiscard]] RigidBody& getBody(BodyID id) noexcept { return bodies[id]; }

    /// Returns a const reference to a body.
    [[nodiscard]] const RigidBody& getBody(BodyID id) const noexcept { return bodies[id]; }

    /// Returns true if the body ID is valid and the body is active.
    [[nodiscard]] bool isValid(BodyID id) const noexcept {
        return id < bodies.size() && bodies[id].id != kInvalidBody;
    }

    /// Enables a body.
    void enableBody(BodyID id) noexcept {
        if (isValid(id)) bodies[id].enabled = true;
    }

    /// Disables a body.
    void disableBody(BodyID id) noexcept {
        if (isValid(id)) {
            bodies[id].enabled = false;
            bodies[id].putToSleep();
        }
    }

    /// Wakes a body.
    void wakeBody(BodyID id) noexcept {
        if (isValid(id)) bodies[id].wake();
    }

    /// Puts a body to sleep.
    void sleepBody(BodyID id) noexcept {
        if (isValid(id)) bodies[id].putToSleep();
    }

    /// Applies a force to a body.
    void applyForce(BodyID id, const Vector3& force) noexcept {
        if (isValid(id)) bodies[id].applyForce(force);
    }

    /// Applies a force at a world point.
    void applyForce(BodyID id, const Vector3& force, const Vector3& point) noexcept {
        if (isValid(id)) bodies[id].applyForce(force, point);
    }

    /// Applies a torque.
    void applyTorque(BodyID id, const Vector3& torque) noexcept {
        if (isValid(id)) bodies[id].applyTorque(torque);
    }

    /// Applies an impulse.
    void applyImpulse(BodyID id, const Vector3& impulse) noexcept {
        if (isValid(id)) bodies[id].applyImpulse(impulse);
    }

    /// Applies an impulse at a world point.
    void applyImpulse(BodyID id, const Vector3& impulse, const Vector3& point) noexcept {
        if (isValid(id)) bodies[id].applyImpulse(impulse, point);
    }

    /// Clears all forces on all dynamic bodies.
    void clearForces() noexcept {
        for (u32 i = 0; i < bodies.size(); ++i) {
            if (bodies[i].id != kInvalidBody && bodies[i].isDynamic()) {
                bodies[i].clearForces();
            }
        }
    }

    /// Returns body manager statistics.
    [[nodiscard]] BodyManagerStats getStats() const noexcept {
        BodyManagerStats stats;
        for (u32 i = 0; i < bodies.size(); ++i) {
            const RigidBody& b = bodies[i];
            if (b.id == kInvalidBody) continue;

            ++stats.totalBodies;
            if (b.isStatic()) ++stats.staticBodies;
            else if (b.isDynamic()) ++stats.dynamicBodies;
            else if (b.isKinematic()) ++stats.kinematicBodies;

            if (b.isAwake()) ++stats.activeBodies;
            else if (b.isSleeping()) ++stats.sleepingBodies;
        }
        return stats;
    }

    /// Returns the number of live bodies.
    [[nodiscard]] u32 size() const noexcept { return count; }

    /// Returns true if no bodies exist.
    [[nodiscard]] bool empty() const noexcept { return count == 0; }
};

} // namespace primeon::math
