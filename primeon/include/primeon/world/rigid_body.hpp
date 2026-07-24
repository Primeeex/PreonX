#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/vector/vector3.hpp"
#include "primeon/math/quaternion/quaternion.hpp"
#include "primeon/dynamics/mass/mass.hpp"
#include "primeon/dynamics/forces/forces.hpp"
#include "primeon/dynamics/motion/motion.hpp"
#include "primeon/world/physics_material.hpp"
#include "foundation/core/types.hpp"
#include <limits>

namespace primeon::math {

using foundation::i32;

// ── Body Type ──────────────────────────────────────────────────────────────

/// The type of a rigid body in the simulation.
enum class BodyType : u32 {
    Static    = 0,  ///< Infinite mass, never moves. Efficient: never integrated.
    Dynamic   = 1,  ///< Fully simulated: forces, impulses, collision response.
    Kinematic = 2,  ///< Moves via velocity only. Infinite mass to colliders.
};

// ── Sleep State ────────────────────────────────────────────────────────────

/// Sleep state of a body.
enum class SleepState : u32 {
    Awake     = 0,
    Sleeping  = 1,
    CanSleep  = 2,  ///< Below threshold, counting down
};

// ── Collider Shape ─────────────────────────────────────────────────────────

/// Type of collider shape attached to a body.
enum class ColliderType : u32 {
    None   = 0,
    Sphere = 1,
    Box    = 2,
    Plane  = 3,
};

/// Collider shape data (tagged union via struct fields).
struct ColliderData {
    ColliderType type = ColliderType::None;

    // Sphere: radius from center
    f32 sphereRadius = 0.5f;

    // Box: half-extents (local-space)
    Vector3 boxHalfExtents = {0.5f, 0.5f, 0.5f};

    // Plane: normal and signed distance from origin
    Vector3 planeNormal = {0.0f, 1.0f, 0.0f};
    f32 planeDistance = 0.0f;

    constexpr ColliderData() noexcept = default;

    [[nodiscard]] static constexpr ColliderData sphere(f32 radius = 0.5f) noexcept {
        ColliderData c;
        c.type = ColliderType::Sphere;
        c.sphereRadius = radius;
        return c;
    }

    [[nodiscard]] static constexpr ColliderData box(const Vector3& halfExtents = {0.5f, 0.5f, 0.5f}) noexcept {
        ColliderData c;
        c.type = ColliderType::Box;
        c.boxHalfExtents = halfExtents;
        return c;
    }

    [[nodiscard]] static constexpr ColliderData plane(const Vector3& normal = {0.0f, 1.0f, 0.0f},
                                                       f32 distance = 0.0f) noexcept {
        ColliderData c;
        c.type = ColliderType::Plane;
        c.planeNormal = normal;
        c.planeDistance = distance;
        return c;
    }
};

// ── Rigid Body ─────────────────────────────────────────────────────────────
//
// A complete rigid body with all properties needed for simulation.
// The body is a value type — the world owns storage and provides handles.
//
// Design principles:
// - No pointers or heap allocations
// - All state is explicit and inspectable
// - Material is stored by value (no shared ownership overhead)
// - Internal solver data (accumulated impulses) is separate from body state

/// Unique identifier for a rigid body within a PhysicsWorld.
using BodyID = u32;
inline constexpr BodyID kInvalidBody = std::numeric_limits<BodyID>::max();

/// Description for creating a rigid body.
struct BodyDescriptor {
    BodyType type = BodyType::Dynamic;
    Vector3 position = kVector3Zero;
    Quaternion rotation = Quaternion::identity();
    f32 mass = 1.0f;
    InertiaTensor inertia = InertiaTensor::solidBox(1.0f, Vector3(0.5f, 0.5f, 0.5f));
    PhysicsMaterial material = PhysicsMaterial::defaultMaterial();
    f32 linearDamping = 0.0f;
    f32 angularDamping = 0.05f;
    f32 gravityScale = 1.0f;
    bool awake = true;
    bool allowSleep = true;
    bool enabled = true;
    ColliderData collider;
    void* userData = nullptr;

    constexpr BodyDescriptor() noexcept = default;

    static BodyDescriptor staticBody(const Vector3& pos) noexcept {
        BodyDescriptor d;
        d.type = BodyType::Static;
        d.position = pos;
        d.mass = 0.0f;
        d.collider = ColliderData::plane({0.0f, 1.0f, 0.0f}, pos.y);
        return d;
    }

    static BodyDescriptor dynamicBody(const Vector3& pos, f32 mass = 1.0f) noexcept {
        BodyDescriptor d;
        d.type = BodyType::Dynamic;
        d.position = pos;
        d.mass = mass;
        d.inertia = InertiaTensor::solidBox(mass, Vector3(0.5f, 0.5f, 0.5f));
        d.collider = ColliderData::box({0.5f, 0.5f, 0.5f});
        return d;
    }

    static BodyDescriptor kinematicBody(const Vector3& pos) noexcept {
        BodyDescriptor d;
        d.type = BodyType::Kinematic;
        d.position = pos;
        d.mass = 0.0f;
        d.collider = ColliderData::box({0.5f, 0.5f, 0.5f});
        return d;
    }
};

/// A rigid body in the physics simulation.
struct RigidBody {
    // ── Identity ──────────────────────────────────────────────────────────
    BodyID id = kInvalidBody;

    // ── Type ──────────────────────────────────────────────────────────────
    BodyType type = BodyType::Dynamic;
    bool enabled = true;

    // ── Transform ─────────────────────────────────────────────────────────
    Vector3 position = kVector3Zero;
    Quaternion rotation = Quaternion::identity();

    // ── Velocity ──────────────────────────────────────────────────────────
    Vector3 linearVelocity = kVector3Zero;
    Vector3 angularVelocity = kVector3Zero;

    // ── Mass Properties ───────────────────────────────────────────────────
    MassProperties massProperties = MassProperties::fromMass(1.0f);
    InertiaTensor inertia = InertiaTensor::solidBox(1.0f, Vector3(0.5f, 0.5f, 0.5f));
    InertiaTensor inverseInertia = inertia.inverse();

    // ── Material ──────────────────────────────────────────────────────────
    PhysicsMaterial material;

    // ── Collider ──────────────────────────────────────────────────────────
    ColliderData collider;

    // ── Damping ───────────────────────────────────────────────────────────
    f32 linearDamping = 0.0f;
    f32 angularDamping = 0.05f;

    // ── Gravity ───────────────────────────────────────────────────────────
    f32 gravityScale = 1.0f;

    // ── Sleep ─────────────────────────────────────────────────────────────
    SleepState sleepState = SleepState::Awake;
    bool allowSleep = true;
    u32 sleepTimer = 0;

    // ── User Data ─────────────────────────────────────────────────────────
    void* userData = nullptr;

    // ── Broadphase ────────────────────────────────────────────────────────
    i32 broadphaseNode = -1;  // index into DynamicAABBTree

    constexpr RigidBody() noexcept = default;

    // ── Queries ───────────────────────────────────────────────────────────

    [[nodiscard]] bool isStatic() const noexcept { return type == BodyType::Static; }
    [[nodiscard]] bool isDynamic() const noexcept { return type == BodyType::Dynamic; }
    [[nodiscard]] bool isKinematic() const noexcept { return type == BodyType::Kinematic; }
    [[nodiscard]] bool isAwake() const noexcept { return sleepState == SleepState::Awake; }
    [[nodiscard]] bool isSleeping() const noexcept { return sleepState == SleepState::Sleeping; }
    [[nodiscard]] bool isSleepEnabled() const noexcept { return allowSleep && type == BodyType::Dynamic; }

    /// Velocity at a world-space offset from center of mass.
    [[nodiscard]] Vector3 velocityAtPoint(const Vector3& worldPoint) const noexcept {
        Vector3 r = worldPoint - position;
        return linearVelocity + angularVelocity.cross(r);
    }

    // ── Mass Updates ──────────────────────────────────────────────────────

    void setMass(f32 m) noexcept {
        massProperties = MassProperties::fromMass(m);
        // Recompute inverse inertia for dynamic bodies
        if (massProperties.isDynamic()) {
            Matrix3 rotMat = rotation.toMatrix3();
            inverseInertia = inertia.rotated(rotMat).inverse();
        } else {
            inverseInertia = InertiaTensor(Matrix3::identity());
        }
    }

    void setInertia(const InertiaTensor& tensor) noexcept {
        inertia = tensor;
        if (massProperties.isDynamic()) {
            Matrix3 rot = rotation.toMatrix3();
            inverseInertia = inertia.rotated(rot).inverse();
        } else {
            inverseInertia = InertiaTensor(Matrix3::identity());
        }
    }

    // ── Force Application ─────────────────────────────────────────────────

    void applyForce(const Vector3& force) noexcept {
        if (!isDynamic() || !isAwake()) return;
        pendingForces.addForce(force);
    }

    void applyForce(const Vector3& force, const Vector3& point) noexcept {
        if (!isDynamic() || !isAwake()) return;
        pendingForces.addForce(Force(force, point - position));
    }

    void applyTorque(const Vector3& torque) noexcept {
        if (!isDynamic() || !isAwake()) return;
        pendingForces.addTorque(torque);
    }

    void applyImpulse(const Vector3& impulse) noexcept {
        if (!isDynamic() || !isAwake()) return;
        linearVelocity += impulse * massProperties.inverseMass;
    }

    void applyImpulse(const Vector3& impulse, const Vector3& worldPoint) noexcept {
        if (!isDynamic() || !isAwake()) return;
        Vector3 r = worldPoint - position;
        linearVelocity += impulse * massProperties.inverseMass;
        angularVelocity += inverseInertia.I * r.cross(impulse);
    }

    void applyAngularImpulse(const Vector3& angularImpulse) noexcept {
        if (!isDynamic() || !isAwake()) return;
        angularVelocity += inverseInertia.I * angularImpulse;
    }

    void clearForces() noexcept {
        pendingForces.clear();
    }

    // ── Sleep ─────────────────────────────────────────────────────────────

    void wake() noexcept {
        if (sleepState != SleepState::Awake && type == BodyType::Dynamic) {
            sleepState = SleepState::Awake;
            sleepTimer = 0;
        }
    }

    void putToSleep() noexcept {
        if (type == BodyType::Dynamic) {
            sleepState = SleepState::Sleeping;
            linearVelocity = kVector3Zero;
            angularVelocity = kVector3Zero;
            sleepTimer = 0;
        }
    }

    // ── Internal (used by world) ──────────────────────────────────────────

    ForceAccumulator pendingForces;

    /// Applies accumulated forces as velocity changes (integrate velocities).
    void integrateVelocity(f32 dt) noexcept {
        if (!isDynamic() || !isAwake()) return;

        // Apply gravity
        if (gravityScale > kEpsilon) {
            pendingForces.addForce(kGravityDown * gravityScale);
        }

        // Linear: v += (F/m) * dt
        Vector3 acceleration = computeAcceleration(pendingForces.force(), massProperties.inverseMass);
        linearVelocity += acceleration * dt;

        // Angular: omega += (I_inv * tau) * dt
        Vector3 angularAcceleration = inverseInertia.I * pendingForces.torque();
        angularVelocity += angularAcceleration * dt;

        // Apply damping
        linearVelocity *= (1.0f - linearDamping * dt);
        angularVelocity *= (1.0f - angularDamping * dt);
    }

    /// Integrates position from velocity (semi-implicit Euler).
    void integratePosition(f32 dt) noexcept {
        if (!isDynamic() || !isAwake()) return;

        position += linearVelocity * dt;
        rotation = integrateQuaternion(rotation, angularVelocity, dt);
    }
};

} // namespace primeon::math
