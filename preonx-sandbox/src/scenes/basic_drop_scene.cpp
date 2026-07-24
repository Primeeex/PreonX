#include "scenes/basic_drop_scene.hpp"
#include <cstdio>

using namespace primeon::math;

void BasicDropScene::init() {
    WorldConfig cfg;
    cfg.gravity = kGravityDown;
    cfg.fixedDt = 1.0f / 60.0f;
    cfg.sleepingEnabled = true;
    cfg.maxBodies = 16;
    world_ = PhysicsWorld(cfg);

    ground_ = world_.createBody(BodyDescriptor::staticBody(Vector3(0.0f, -1.0f, 0.0f)));
    box_ = world_.createBody(BodyDescriptor::dynamicBody(Vector3(0.0f, 10.0f, 0.0f), 2.0f));
}

void BasicDropScene::cleanup() {
    world_ = PhysicsWorld();
}

void BasicDropScene::update(float dt, bool paused, bool stepOnce) {
    if (!paused || stepOnce) {
        world_.step(dt);
        resolveGroundCollisions(-1.0f);
    }
}

void BasicDropScene::render(DebugRenderer& renderer, const Camera& /*camera*/) {
    const auto& boxBody = world_.getBody(box_);
    renderer.drawFilledRectangle(
        {boxBody.position.x - 0.5f, boxBody.position.y - 0.5f, 0.0f},
        {boxBody.position.x + 0.5f, boxBody.position.y + 0.5f, 0.0f},
        {0.2f, 0.6f, 1.0f});

    renderer.drawRectangle(
        {boxBody.position.x - 0.5f, boxBody.position.y - 0.5f, 0.0f},
        {boxBody.position.x + 0.5f, boxBody.position.y + 0.5f, 0.0f},
        {1.0f, 1.0f, 1.0f});

    if (boxBody.linearVelocity.lengthSq() > 0.001f) {
        Vector3 dir = boxBody.linearVelocity;
        float spd = std::sqrt(dir.lengthSq());
        if (spd > 0.01f) {
            dir = dir / spd;
        }
        renderer.drawArrow(boxBody.position, dir, spd * 0.1f, {1.0f, 0.3f, 0.0f});
    }

    renderer.drawFilledRectangle(
        {-50.0f, -2.0f, 0.0f},
        {50.0f, -1.0f, 0.0f},
        {0.4f, 0.4f, 0.4f});

    renderer.drawLine({-50.0f, -1.0f, 0.0f}, {50.0f, -1.0f, 0.0f}, {0.8f, 0.8f, 0.8f});
}

void BasicDropScene::handleInput(const Input& /*input*/) {
}
