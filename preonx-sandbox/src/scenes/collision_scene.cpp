#include "scenes/collision_scene.hpp"
#include <cstdio>

using namespace primeon::math;

void CollisionScene::init() {
    WorldConfig cfg;
    cfg.gravity = kGravityDown;
    cfg.fixedDt = 1.0f / 60.0f;
    cfg.sleepingEnabled = true;
    cfg.maxBodies = 16;
    cfg.solver.velocityIterations = 8;
    cfg.solver.positionIterations = 3;
    world_ = PhysicsWorld(cfg);

    ground_ = world_.createBody(BodyDescriptor::staticBody(Vector3(0.0f, -1.0f, 0.0f)));

    BodyDescriptor sphereADesc = BodyDescriptor::dynamicBody(Vector3(-3.0f, 5.0f, 0.0f), 1.0f);
    sphereADesc.collider = ColliderData::sphere(0.5f);
    sphereA_ = world_.createBody(sphereADesc);

    BodyDescriptor sphereBDesc = BodyDescriptor::dynamicBody(Vector3(3.0f, 8.0f, 0.0f), 1.0f);
    sphereBDesc.collider = ColliderData::sphere(0.5f);
    sphereB_ = world_.createBody(sphereBDesc);

    boxA_ = world_.createBody(BodyDescriptor::dynamicBody(Vector3(-1.0f, 12.0f, 0.0f), 1.5f));
    boxB_ = world_.createBody(BodyDescriptor::dynamicBody(Vector3(1.0f, 15.0f, 0.0f), 1.5f));
}

void CollisionScene::cleanup() {
    world_ = PhysicsWorld();
}

void CollisionScene::update(float dt, bool paused, bool stepOnce) {
    if (!paused || stepOnce) {
        world_.step(dt);
        resolveGroundCollisions(-1.0f);
        resolveBodyCollisions(0.5f, 0.3f);
    }
}

void CollisionScene::render(DebugRenderer& renderer, const Camera& /*camera*/) {
    renderer.drawFilledRectangle(
        {-50.0f, -2.0f, 0.0f}, {50.0f, -1.0f, 0.0f}, {0.4f, 0.4f, 0.4f});

    auto drawSphereBody = [&](BodyID id, const Vector3& color) {
        const auto& body = world_.getBody(id);
        renderer.drawCircle(body.position, 0.5f, color);
        renderer.drawFilledCircle(body.position, 0.5f, color);
    };

    drawSphereBody(sphereA_, {0.2f, 0.8f, 0.3f});
    drawSphereBody(sphereB_, {0.3f, 0.2f, 0.8f});

    auto drawBoxBody = [&](BodyID id, const Vector3& color) {
        const auto& body = world_.getBody(id);
        renderer.drawFilledRectangle(
            {body.position.x - 0.5f, body.position.y - 0.5f, 0.0f},
            {body.position.x + 0.5f, body.position.y + 0.5f, 0.0f},
            color);
        renderer.drawRectangle(
            {body.position.x - 0.5f, body.position.y - 0.5f, 0.0f},
            {body.position.x + 0.5f, body.position.y + 0.5f, 0.0f},
            {1.0f, 1.0f, 1.0f});
    };

    drawBoxBody(boxA_, {0.8f, 0.4f, 0.2f});
    drawBoxBody(boxB_, {0.8f, 0.2f, 0.6f});

    const auto& stats = world_.getStats();
    if (stats.contactCount > 0) {
        for (u32 i = 0; i < stats.contactCount; ++i) {
            renderer.drawPoint({0.0f, 0.0f, 0.0f}, 0.1f, {1.0f, 1.0f, 0.0f});
        }
    }
}

void CollisionScene::handleInput(const Input& /*input*/) {
}
