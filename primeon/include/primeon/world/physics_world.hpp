#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/vector/vector3.hpp"
#include "primeon/math/quaternion/quaternion.hpp"
#include "primeon/dynamics/mass/mass.hpp"
#include "primeon/dynamics/forces/forces.hpp"
#include "primeon/dynamics/motion/motion.hpp"
#include "primeon/dynamics/integrators/integrators.hpp"
#include "primeon/dynamics/simulation.hpp"
#include "primeon/geometry/primitives/sphere.hpp"
#include "primeon/geometry/primitives/aabb.hpp"
#include "primeon/geometry/primitives/obb.hpp"
#include "primeon/geometry/primitives/plane.hpp"
#include "primeon/collision/broadphase/dynamic_aabb_tree.hpp"
#include "primeon/collision/contact.hpp"
#include "primeon/collision/narrowphase/sphere_sphere.hpp"
#include "primeon/collision/narrowphase/sphere_aabb.hpp"
#include "primeon/collision/narrowphase/sphere_plane.hpp"
#include "primeon/collision/narrowphase/aabb_aabb.hpp"
#include "primeon/collision/narrowphase/obb_obb.hpp"
#include "primeon/constraints/contact_constraint.hpp"
#include "primeon/solver/sequential_impulse/sequential_impulse_solver.hpp"
#include "primeon/solver/solver_config.hpp"

#include "primeon/world/rigid_body.hpp"
#include "primeon/world/physics_material.hpp"
#include "primeon/world/body_manager.hpp"
#include "primeon/world/island.hpp"
#include "primeon/world/sleeping.hpp"
#include "primeon/world/callbacks.hpp"
#include "primeon/world/simulation_stats.hpp"

#include <vector>
#include <algorithm>
#include <cmath>
#include <utility>

namespace primeon::math {

using collision::DynamicAABBTree;

/// Extracts the diagonal of a Matrix3 as a Vector3.
[[nodiscard]] inline Vector3 matrixDiagonal(const Matrix3& m) noexcept {
    return {m.m[0][0], m.m[1][1], m.m[2][2]};
}

// ── Physics World ─────────────────────────────────────────────────────────
//
// The central simulation class. Owns body storage, broadphase, solver config,
// and orchestrates the full simulation pipeline.
//
// Pipeline (per step):
//   1. Accumulate external forces (gravity, user forces)
//   2. Integrate velocities (semi-implicit Euler)
//   3. Update broadphase AABBs
//   4. Detect collisions (broadphase query → narrowphase → manifolds)
//   5. Build islands (union-find on contact graph)
//   6. Solve constraints (sequential impulse solver)
//   7. Integrate positions
//   8. Sleep evaluation
//   9. Clear temporary state
//
// The world is NOT copyable or movable (due to callback pointers).

/// Configuration for the physics world.
struct WorldConfig {
    SolverConfig solver;
    SleepConfig sleep;
    Vector3 gravity = kGravityDown;
    f32 fixedDt = 1.0f / 60.0f;
    u32 maxBodies = 1024;
    u32 maxContacts = 1024;
    bool sleepingEnabled = true;

    constexpr WorldConfig() noexcept = default;

    static WorldConfig gameDefault() noexcept {
        WorldConfig cfg;
        cfg.solver = SolverConfig::gameDefault();
        cfg.fixedDt = 1.0f / 60.0f;
        return cfg;
    }

    static WorldConfig simulation() noexcept {
        WorldConfig cfg;
        cfg.solver = SolverConfig::simulation();
        cfg.fixedDt = 1.0f / 120.0f;
        cfg.sleepingEnabled = true;
        return cfg;
    }

    static WorldConfig fast() noexcept {
        WorldConfig cfg;
        cfg.solver = SolverConfig::fast();
        cfg.fixedDt = 1.0f / 30.0f;
        cfg.sleepingEnabled = false;
        return cfg;
    }
};

/// The physics world — central simulation orchestrator.
struct PhysicsWorld {
    // ── Configuration ─────────────────────────────────────────────────────
    WorldConfig config;

    // ── State ─────────────────────────────────────────────────────────────
    BodyManager bodyManager;
    DynamicAABBTree broadphase;
    IslandBuilder islandBuilder;
    SleepSystem sleepSystem;
    SequentialImpulseSolver solver;
    TimestepAccumulator timestepAccumulator;
    WorldCallbacks callbacks;
    SimulationStats stats;

    // ── Internal Data ─────────────────────────────────────────────────────
    std::vector<ContactManifold> manifolds;
    std::vector<std::pair<u32, u32>> contactPairs;
    std::vector<SolverBodyData> solverBodies;
    std::vector<ContactConstraint> contactConstraints;
    u32 frameCount = 0;

    // ── Construction ──────────────────────────────────────────────────────

    PhysicsWorld() noexcept
        : PhysicsWorld(WorldConfig::gameDefault()) {}

    explicit PhysicsWorld(const WorldConfig& cfg) noexcept
        : config(cfg)
        , bodyManager(cfg.maxBodies)
        , broadphase(cfg.maxBodies * 2)
        , islandBuilder(cfg.maxBodies)
        , sleepSystem(cfg.sleep)
        , timestepAccumulator(cfg.fixedDt)
    {
        manifolds.reserve(cfg.maxContacts / 4);
        contactPairs.reserve(cfg.maxContacts);
        solverBodies.resize(cfg.maxBodies);
        contactConstraints.reserve(cfg.maxContacts);
    }

    // ── Body Management ───────────────────────────────────────────────────

    /// Creates a body from a descriptor. Returns body ID.
    [[nodiscard]] BodyID createBody(const BodyDescriptor& desc) noexcept {
        BodyID id = bodyManager.createBody(desc);
        if (id == kInvalidBody) return kInvalidBody;

        RigidBody& body = bodyManager.getBody(id);
        body.gravityScale *= (config.gravity.length() > kEpsilon) ? 1.0f : 0.0f;

        // Insert into broadphase
        AABB aabb = computeAABB(body);
        body.broadphaseNode = broadphase.insert(id, aabb);

        // Callback
        if (callbacks.onBodyCreated) {
            callbacks.onBodyCreated(id, callbacks.userData);
        }

        return id;
    }

    /// Destroys a body by ID.
    void destroyBody(BodyID id) noexcept {
        if (!bodyManager.isValid(id)) return;

        RigidBody& body = bodyManager.getBody(id);

        // Remove from broadphase
        if (body.broadphaseNode >= 0) {
            broadphase.remove(body.broadphaseNode);
            body.broadphaseNode = -1;
        }

        // Callback
        if (callbacks.onBodyDestroyed) {
            callbacks.onBodyDestroyed(id, callbacks.userData);
        }

        bodyManager.destroyBody(id);
    }

    /// Returns a mutable reference to a body.
    [[nodiscard]] RigidBody& getBody(BodyID id) noexcept { return bodyManager.getBody(id); }

    /// Returns a const reference to a body.
    [[nodiscard]] const RigidBody& getBody(BodyID id) const noexcept { return bodyManager.getBody(id); }

    /// Returns true if the body ID is valid.
    [[nodiscard]] bool isValidBody(BodyID id) const noexcept { return bodyManager.isValid(id); }

    // ── Body Operations ───────────────────────────────────────────────────

    void enableBody(BodyID id) noexcept { bodyManager.enableBody(id); }
    void disableBody(BodyID id) noexcept { bodyManager.disableBody(id); }
    void wakeBody(BodyID id) noexcept { bodyManager.wakeBody(id); }
    void sleepBody(BodyID id) noexcept { bodyManager.sleepBody(id); }

    void applyForce(BodyID id, const Vector3& force) noexcept {
        bodyManager.applyForce(id, force);
    }

    void applyForce(BodyID id, const Vector3& force, const Vector3& point) noexcept {
        bodyManager.applyForce(id, force, point);
    }

    void applyTorque(BodyID id, const Vector3& torque) noexcept {
        bodyManager.applyTorque(id, torque);
    }

    void applyImpulse(BodyID id, const Vector3& impulse) noexcept {
        bodyManager.applyImpulse(id, impulse);
    }

    void applyImpulse(BodyID id, const Vector3& impulse, const Vector3& point) noexcept {
        bodyManager.applyImpulse(id, impulse, point);
    }

    void clearForces() noexcept { bodyManager.clearForces(); }

    // ── Simulation ────────────────────────────────────────────────────────

    /// Steps the simulation by a fixed timestep.
    void step(f32 frameTime) noexcept {
        f32 dt = timestepAccumulator.step(frameTime);
        if (dt < kEpsilon) return;

        // Run one or more fixed steps
        while (dt >= config.fixedDt - kEpsilon) {
            stepFixed(config.fixedDt);
            dt -= config.fixedDt;
            if (dt < kEpsilon) break;
        }
    }

    /// Steps the simulation by exactly one fixed timestep.
    void stepFixed(f32 dt) noexcept {
        // Reset stats
        stats = {};

        auto& bodies = bodyManager.bodies;
        u32 bodyCount = static_cast<u32>(bodies.size());

        // ══════════════════════════════════════════════════════════════════
        // Pipeline Step 1: Accumulate external forces
        // ══════════════════════════════════════════════════════════════════
        // Gravity is applied during velocity integration (inside integrateVelocity).

        // ══════════════════════════════════════════════════════════════════
        // Pipeline Step 2: Integrate velocities
        // ══════════════════════════════════════════════════════════════════
        for (u32 i = 0; i < bodyCount; ++i) {
            RigidBody& body = bodies[i];
            if (body.id == kInvalidBody) continue;
            body.integrateVelocity(dt);
            body.pendingForces.clear();
        }

        // ══════════════════════════════════════════════════════════════════
        // Pipeline Step 3: Update broadphase
        // ══════════════════════════════════════════════════════════════════
        for (u32 i = 0; i < bodyCount; ++i) {
            RigidBody& body = bodies[i];
            if (body.id == kInvalidBody || !body.enabled) continue;

            AABB aabb = computeAABB(body);
            if (body.broadphaseNode >= 0) {
                broadphase.remove(body.broadphaseNode);
                body.broadphaseNode = broadphase.insert(body.id, aabb);
            }
        }

        // ══════════════════════════════════════════════════════════════════
        // Pipeline Step 4: Detect collisions
        // ══════════════════════════════════════════════════════════════════
        manifolds.clear();
        contactPairs.clear();

        // Broadphase query
        broadphase.queryResults.clear();
        for (u32 i = 0; i < bodyCount; ++i) {
            const RigidBody& body = bodies[i];
            if (body.id == kInvalidBody || !body.enabled) continue;

            AABB aabb = computeAABB(body);
            broadphase.queryResults.clear();
            broadphase.queryAABB(aabb);

            // Process overlapping pairs
            for (u32 j = 0; j < broadphase.queryResults.size(); ++j) {
                u32 otherNodeIdx = broadphase.queryResults[j];
                if (otherNodeIdx == static_cast<u32>(body.broadphaseNode)) continue;

                // Find body ID from node
                u32 otherBodyID = kInvalidBody;
                for (u32 k = 0; k < bodyCount; ++k) {
                    if (bodies[k].broadphaseNode == static_cast<i32>(otherNodeIdx)) {
                        otherBodyID = k;
                        break;
                    }
                }
                if (otherBodyID == kInvalidBody || otherBodyID <= i) continue;

                const RigidBody& other = bodies[otherBodyID];
                if (!other.enabled) continue;

                // Skip static-static pairs
                if (body.isStatic() && other.isStatic()) continue;

                // Narrowphase: dispatch by collider shape pair
                CollisionResult cr;
                cr.colliding = false;

                ColliderType typeA = body.collider.type;
                ColliderType typeB = other.collider.type;

                // Plane always second (B) for plane-X pairs
                if (typeA == ColliderType::Plane && typeB != ColliderType::Plane) {
                    // Swap: test B vs A's plane
                    Plane planeA(body.collider.planeNormal, body.collider.planeDistance);
                    if (typeB == ColliderType::Sphere) {
                        Sphere sB(other.position, other.collider.sphereRadius);
                        cr = spherePlane(sB, planeA);
                    }
                } else if (typeB == ColliderType::Plane && typeA != ColliderType::Plane) {
                    Plane planeB(other.collider.planeNormal, other.collider.planeDistance);
                    if (typeA == ColliderType::Sphere) {
                        Sphere sA(body.position, body.collider.sphereRadius);
                        cr = spherePlane(sA, planeB);
                    } else if (typeA == ColliderType::Box) {
                        // Box vs Plane: use AABB approximation
                        AABB boxAABB = AABB::fromCenterHalfExtents(body.position, body.collider.boxHalfExtents);
                        Sphere planeSphere(other.position, 0.0f);
                        cr = sphereAABB(planeSphere, boxAABB);
                        if (cr.colliding) {
                            // Override with proper plane contact
                            f32 dist = body.position.dot(other.collider.planeNormal) - other.collider.planeDistance;
                            f32 penetration = body.collider.boxHalfExtents.y - dist;
                            if (penetration > 0.0f) {
                                cr.colliding = true;
                                cr.manifold.normal = other.collider.planeNormal;
                                cr.manifold.contactCount = 1;
                                cr.manifold.contacts[0].point = body.position - other.collider.planeNormal * dist;
                                cr.manifold.contacts[0].penetration = penetration;
                            } else {
                                cr.colliding = false;
                            }
                        }
                    }
                } else if (typeA == ColliderType::Sphere && typeB == ColliderType::Sphere) {
                    Sphere sA(body.position, body.collider.sphereRadius);
                    Sphere sB(other.position, other.collider.sphereRadius);
                    cr = sphereSphere(sA, sB);
                } else if (typeA == ColliderType::Box && typeB == ColliderType::Box) {
                    OBB obbA(body.position, body.collider.boxHalfExtents, body.rotation);
                    OBB obbB(other.position, other.collider.boxHalfExtents, other.rotation);
                    cr = obbOBB(obbA, obbB);
                } else if (typeA == ColliderType::Sphere && typeB == ColliderType::Box) {
                    Sphere sA(body.position, body.collider.sphereRadius);
                    AABB bB = AABB::fromCenterHalfExtents(other.position, other.collider.boxHalfExtents);
                    cr = sphereAABB(sA, bB);
                } else if (typeA == ColliderType::Box && typeB == ColliderType::Sphere) {
                    AABB bA = AABB::fromCenterHalfExtents(body.position, body.collider.boxHalfExtents);
                    Sphere sB(other.position, other.collider.sphereRadius);
                    cr = sphereAABB(sB, bA);
                    if (cr.colliding) {
                        cr.manifold.normal = -cr.manifold.normal;
                    }
                } else if (typeA == ColliderType::Sphere && typeB == ColliderType::Plane) {
                    Sphere sA(body.position, body.collider.sphereRadius);
                    Plane planeB(other.collider.planeNormal, other.collider.planeDistance);
                    cr = spherePlane(sA, planeB);
                } else {
                    // Fallback: sphere-sphere with 0.5 radius
                    Sphere sA(body.position, 0.5f);
                    Sphere sB(other.position, 0.5f);
                    cr = sphereSphere(sA, sB);
                }

                if (cr.colliding) {
                    contactPairs.emplace_back(i, otherBodyID);

                    cr.manifold.bodyIDA = i;
                    cr.manifold.bodyIDB = otherBodyID;

                    for (u32 pi = 0; pi < cr.manifold.contactCount; ++pi) {
                        ContactPoint& cp = cr.manifold.contacts[pi];
                        cp.tangentU = kVector3UnitX;
                        cp.tangentV = kVector3UnitZ;
                        cp.computeID();
                    }
                    cr.manifold.computeTangents();

                    manifolds.push_back(cr.manifold);
                    ++stats.narrowphaseTests;
                }
            }
        }

        stats.broadphasePairs = static_cast<u32>(contactPairs.size());
        stats.manifoldCount = static_cast<u32>(manifolds.size());

        // ══════════════════════════════════════════════════════════════════
        // Pipeline Step 5: Build islands
        // ══════════════════════════════════════════════════════════════════
        const auto& islands = islandBuilder.build(
            bodyCount,
            contactPairs.data(),
            static_cast<u32>(contactPairs.size()),
            bodies.data());
        stats.islandCount = static_cast<u32>(islands.size());

        // ══════════════════════════════════════════════════════════════════
        // Pipeline Step 6: Solve constraints
        // ══════════════════════════════════════════════════════════════════
        u32 totalContactCount = 0;

        for (u32 islandIdx = 0; islandIdx < islands.size(); ++islandIdx) {
            const Island& island = islands[islandIdx];

            // Skip sleeping islands
            if (!island.isAwake && config.sleepingEnabled) continue;

            // Build solver bodies for this island
            // Map body index to solver index
            std::vector<u32> solverBodyMap(bodyCount, 0xFFFFFFFF);
            u32 solverCount = 0;

            for (u32 bi = 0; bi < island.bodies.size(); ++bi) {
                u32 bodyIdx = island.bodies[bi];
                solverBodyMap[bodyIdx] = solverCount;

                const RigidBody& body = bodies[bodyIdx];
                SolverBodyData& sb = solverBodies[solverCount];
                sb.linearVelocity = body.linearVelocity;
                sb.angularVelocity = body.angularVelocity;
                sb.massData.inverseMass = body.massProperties.inverseMass;
                sb.massData.inverseInertiaDiag = matrixDiagonal(body.inverseInertia.I);
                ++solverCount;
            }

            // Build contact constraints for this island
            solver.reset();

            for (u32 ci = 0; ci < island.contacts.size(); ++ci) {
                u32 contactIdx = island.contacts[ci];
                if (contactIdx >= manifolds.size()) continue;

                const ContactManifold& manifold = manifolds[contactIdx];
                u32 solverA = solverBodyMap[manifold.bodyIDA];
                u32 solverB = solverBodyMap[manifold.bodyIDB];
                if (solverA == 0xFFFFFFFF || solverB == 0xFFFFFFFF) continue;

                const RigidBody& bodyA = bodies[manifold.bodyIDA];
                const RigidBody& bodyB = bodies[manifold.bodyIDB];

                for (u32 pi = 0; pi < manifold.contactCount; ++pi) {
                    const ContactPoint& cp = manifold.contacts[pi];

                    ContactConstraint cc;
                    cc.bodyIDA = solverA;
                    cc.bodyIDB = solverB;
                    cc.massA.inverseMass = bodyA.massProperties.inverseMass;
                    cc.massA.inverseInertiaDiag = matrixDiagonal(bodyA.inverseInertia.I);
                    cc.massA.inverseInertiaOrientation = bodyA.rotation;
                    cc.massB.inverseMass = bodyB.massProperties.inverseMass;
                    cc.massB.inverseInertiaDiag = matrixDiagonal(bodyB.inverseInertia.I);
                    cc.massB.inverseInertiaOrientation = bodyB.rotation;
                    cc.pointA = cp.point;
                    cc.pointB = cp.point;
                    cc.normal = manifold.normal;
                    cc.penetration = cp.penetration;
                    cc.contactID = cp.contactID;

                    // Combine materials
                    cc.restitution = bodyA.material.combinedRestitution(bodyB.material);
                    cc.friction = bodyA.material.combinedFriction(bodyB.material);

                    solver.addContact(cc, cp.tangentU, cp.tangentV);
                    ++totalContactCount;
                }
            }

            // Solve
            if (solver.contactCount > 0) {
                solver.solve(solverBodies.data(), solverCount, dt, config.solver);

                // Write back velocities
                for (u32 bi = 0; bi < island.bodies.size(); ++bi) {
                    u32 bodyIdx = island.bodies[bi];
                    u32 solverIdx = solverBodyMap[bodyIdx];
                    if (solverIdx == 0xFFFFFFFF) continue;

                    RigidBody& body = bodies[bodyIdx];
                    body.linearVelocity = solverBodies[solverIdx].linearVelocity;
                    body.angularVelocity = solverBodies[solverIdx].angularVelocity;
                }

                // Wake sleeping bodies in contact with awake bodies
                if (config.sleepingEnabled) {
                    for (u32 bi = 0; bi < island.bodies.size(); ++bi) {
                        u32 bodyIdx = island.bodies[bi];
                        if (bodies[bodyIdx].isSleeping()) {
                            bodies[bodyIdx].wake();
                        }
                    }
                }
            }
        }

        stats.contactCount = totalContactCount;
        stats.velocityIterations = config.solver.velocityIterations;
        stats.positionIterations = config.solver.positionIterations;

        // ══════════════════════════════════════════════════════════════════
        // Pipeline Step 7: Integrate positions
        // ══════════════════════════════════════════════════════════════════
        for (u32 i = 0; i < bodyCount; ++i) {
            RigidBody& body = bodies[i];
            if (body.id == kInvalidBody) continue;
            body.integratePosition(dt);
        }

        // ══════════════════════════════════════════════════════════════════
        // Pipeline Step 8: Sleep evaluation
        // ══════════════════════════════════════════════════════════════════
        if (config.sleepingEnabled) {
            sleepSystem.update(bodies);
        }

        // Update sleep stats
        for (u32 i = 0; i < bodyCount; ++i) {
            const RigidBody& body = bodies[i];
            if (body.id == kInvalidBody) continue;

            ++stats.totalBodies;
            if (body.isStatic()) ++stats.staticBodies;
            else if (body.isDynamic()) ++stats.dynamicBodies;
            else if (body.isKinematic()) ++stats.kinematicBodies;

            if (body.isAwake()) ++stats.activeBodies;
            else if (body.isSleeping()) ++stats.sleepingBodies;
        }

        // ══════════════════════════════════════════════════════════════════
        // Pipeline Step 9: Clear temporary state
        // ══════════════════════════════════════════════════════════════════
        // manifolds and contactPairs are cleared at the start of the next step.

        ++frameCount;
    }

    // ── Query ─────────────────────────────────────────────────────────────

    /// Returns the interpolation alpha for rendering.
    [[nodiscard]] f32 getAlpha() const noexcept { return timestepAccumulator.alpha(); }

    /// Returns the current simulation statistics.
    [[nodiscard]] const SimulationStats& getStats() const noexcept { return stats; }

    /// Returns body manager statistics.
    [[nodiscard]] BodyManagerStats getBodyStats() const noexcept { return bodyManager.getStats(); }

    /// Returns the number of live bodies.
    [[nodiscard]] u32 getBodyCount() const noexcept { return bodyManager.size(); }

    /// Returns the frame count.
    [[nodiscard]] u32 getFrameCount() const noexcept { return frameCount; }

    /// Returns a mutable reference to the world config.
    [[nodiscard]] WorldConfig& getConfig() noexcept { return config; }

    /// Returns a mutable reference to the callbacks.
    [[nodiscard]] WorldCallbacks& getCallbacks() noexcept { return callbacks; }

    // ── Internal Helpers ──────────────────────────────────────────────────

    /// Computes a world-space AABB for a body based on its collider shape.
    [[nodiscard]] static AABB computeAABB(const RigidBody& body) noexcept {
        switch (body.collider.type) {
            case ColliderType::Box: {
                return AABB::fromCenterHalfExtents(body.position, body.collider.boxHalfExtents);
            }
            case ColliderType::Sphere: {
                f32 r = body.collider.sphereRadius;
                return AABB(body.position - Vector3(r, r, r),
                            body.position + Vector3(r, r, r));
            }
            case ColliderType::Plane: {
                f32 big = 1000.0f;
                if (body.collider.planeNormal.y > 0.9f) {
                    return AABB(Vector3(-big, body.collider.planeDistance - 0.1f, -big),
                                Vector3(big, body.collider.planeDistance + 0.1f, big));
                }
                return AABB(body.position - Vector3(big, big, big),
                            body.position + Vector3(big, big, big));
            }
            default: {
                f32 radius = 0.5f;
                return AABB(body.position - Vector3(radius, radius, radius),
                            body.position + Vector3(radius, radius, radius));
            }
        }
    }

    /// Computes an approximate bounding radius for broadphase.
    [[nodiscard]] static f32 computeBoundingRadius(const RigidBody& body) noexcept {
        // Use a generous bounding radius based on mass properties
        // For spheres, this is the sphere radius; for boxes, use diagonal/2
        f32 massRadius = std::cbrt(1.0f / (body.massProperties.mass + kEpsilon));
        return massRadius * 1.5f + 0.1f;  // Conservative bound
    }

    /// Wakes all bodies.
    void wakeAll() noexcept {
        sleepSystem.wakeAll(bodyManager.bodies);
    }
};

} // namespace primeon::math
