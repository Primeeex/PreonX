#pragma once

#include "rendering/debug_renderer.hpp"
#include "input/input.hpp"
#include "rendering/camera.hpp"
#include <primeon/world/physics_world.hpp>
#include <cmath>

class Scene {
public:
    virtual ~Scene() = default;

    virtual void init() = 0;
    virtual void cleanup() = 0;
    virtual void update(float dt, bool paused, bool stepOnce) = 0;
    virtual void render(DebugRenderer& renderer, const Camera& camera) = 0;
    virtual void handleInput(const Input& input) = 0;

    [[nodiscard]] virtual const char* name() const = 0;
    [[nodiscard]] virtual const char* description() const = 0;

    [[nodiscard]] const primeon::math::PhysicsWorld& getWorld() const { return world_; }
    [[nodiscard]] primeon::math::PhysicsWorld& getWorld() { return world_; }

    void setGravityEnabled(bool enabled) {
        gravityEnabled_ = enabled;
        world_.config.gravity = enabled ? primeon::math::kGravityDown : primeon::math::kVector3Zero;
    }
    [[nodiscard]] bool isGravityEnabled() const { return gravityEnabled_; }

    void resolveGroundCollisions(float groundY, float halfExtent = 0.5f,
                                 float restitution = 0.3f, float friction = 0.7f) {
        using namespace primeon::math;
        auto& bodies = world_.bodyManager.bodies;
        for (u32 i = 0; i < static_cast<u32>(bodies.size()); ++i) {
            RigidBody& body = bodies[i];
            if (body.id == kInvalidBody || !body.isDynamic() || !body.enabled) continue;
            if (body.isSleeping()) continue;

            float bottom = body.position.y - halfExtent;
            if (bottom < groundY) {
                body.position.y = groundY + halfExtent;
                if (body.linearVelocity.y < 0.0f) {
                    body.linearVelocity.y = -body.linearVelocity.y * restitution;
                    if (std::abs(body.linearVelocity.y) < 0.3f) {
                        body.linearVelocity.y = 0.0f;
                    }
                }
                body.linearVelocity.x *= friction;
                body.angularVelocity *= 0.9f;
            }
        }
    }

    void resolveBodyCollisions(float halfExtent = 0.5f, float restitution = 0.1f,
                               float positionBaumgarte = 0.6f) {
        using namespace primeon::math;
        auto& bodies = world_.bodyManager.bodies;
        u32 n = static_cast<u32>(bodies.size());

        for (u32 i = 0; i < n; ++i) {
            RigidBody& a = bodies[i];
            if (a.id == kInvalidBody || !a.isDynamic() || !a.enabled) continue;

            for (u32 j = i + 1; j < n; ++j) {
                RigidBody& b = bodies[j];
                if (b.id == kInvalidBody || !b.isDynamic() || !b.enabled) continue;

                f32 dx = b.position.x - a.position.x;
                f32 dy = b.position.y - a.position.y;
                f32 overlapX = halfExtent + halfExtent - std::abs(dx);
                f32 overlapY = halfExtent + halfExtent - std::abs(dy);

                if (overlapX <= 0.0f || overlapY <= 0.0f) continue;

                Vector3 normal;
                f32 penetration;
                if (overlapX < overlapY) {
                    penetration = overlapX;
                    normal = {dx > 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f};
                } else {
                    penetration = overlapY;
                    normal = {0.0f, dy > 0.0f ? 1.0f : -1.0f, 0.0f};
                }

                f32 totalInvMass = a.massProperties.inverseMass + b.massProperties.inverseMass;
                if (totalInvMass < kEpsilon) continue;

                f32 corr = penetration * positionBaumgarte / totalInvMass;
                a.position.x -= normal.x * corr * a.massProperties.inverseMass;
                a.position.y -= normal.y * corr * a.massProperties.inverseMass;
                b.position.x += normal.x * corr * b.massProperties.inverseMass;
                b.position.y += normal.y * corr * b.massProperties.inverseMass;

                f32 relVelX = b.linearVelocity.x - a.linearVelocity.x;
                f32 relVelY = b.linearVelocity.y - a.linearVelocity.y;
                f32 velAlongNormal = relVelX * normal.x + relVelY * normal.y;

                if (velAlongNormal > 0.0f) continue;

                f32 jMag = -(1.0f + restitution) * velAlongNormal / totalInvMass;
                a.linearVelocity.x -= jMag * normal.x * a.massProperties.inverseMass;
                a.linearVelocity.y -= jMag * normal.y * a.massProperties.inverseMass;
                b.linearVelocity.x += jMag * normal.x * b.massProperties.inverseMass;
                b.linearVelocity.y += jMag * normal.y * b.massProperties.inverseMass;
            }
        }
    }

    [[nodiscard]] static float getRotationAngle(const primeon::math::Quaternion& q) {
        return 2.0f * std::atan2(q.z, q.w);
    }

protected:
    primeon::math::PhysicsWorld world_;
    bool gravityEnabled_ = true;
};
