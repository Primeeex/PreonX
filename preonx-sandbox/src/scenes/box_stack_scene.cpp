#include "scenes/box_stack_scene.hpp"
#include <cstdio>

using namespace primeon::math;

void BoxStackScene::configure(int count, float mass, float rest, float sp) {
    stackHeight_ = (count > kMaxStackHeight) ? kMaxStackHeight : count;
    boxMass_ = mass;
    restitution_ = rest;
    spacing_ = sp;
}

void BoxStackScene::init() {
    WorldConfig cfg;
    cfg.gravity = kGravityDown;
    cfg.fixedDt = 1.0f / 120.0f;
    cfg.sleepingEnabled = true;
    cfg.maxBodies = stackHeight_ + 8;
    cfg.solver.velocityIterations = 0;
    cfg.solver.positionIterations = 0;
    world_ = PhysicsWorld(cfg);

    ground_ = world_.createBody(BodyDescriptor::staticBody(Vector3(0.0f, -1.0f, 0.0f)));

    for (int i = 0; i < stackHeight_; ++i) {
        f32 y = static_cast<f32>(i) * spacing_ + 0.5f;
        boxes_[i] = world_.createBody(BodyDescriptor::dynamicBody(Vector3(0.0f, y, 0.0f), boxMass_));
    }
}

void BoxStackScene::cleanup() {
    world_ = PhysicsWorld();
}

void BoxStackScene::update(float dt, bool paused, bool stepOnce) {
    if (!paused || stepOnce) {
        world_.step(dt);
        resolveGroundCollisions(-1.0f, 0.45f, restitution_);
        resolveBodyCollisions(0.45f, restitution_);
    }
}

void BoxStackScene::render(DebugRenderer& renderer, const Camera& /*camera*/) {
    renderer.drawFilledRectangle(
        {-50.0f, -2.0f, 0.0f}, {50.0f, -1.0f, 0.0f}, {0.4f, 0.4f, 0.4f});

    for (int i = 0; i < stackHeight_; ++i) {
        const auto& body = world_.getBody(boxes_[i]);
        f32 hue = static_cast<f32>(i) / static_cast<f32>(stackHeight_);

        f32 angle = getRotationAngle(body.rotation);
        Vector3 color = {0.2f + hue * 0.7f, 0.8f - hue * 0.4f, 0.4f + hue * 0.5f};

        renderer.drawRotatedFilledRect(body.position.x, body.position.y, 0.45f, 0.45f,
                                       angle, color);
        renderer.drawRotatedRect(body.position.x, body.position.y, 0.45f, 0.45f,
                                 angle, {1.0f, 1.0f, 1.0f});
    }
}

void BoxStackScene::handleInput(const Input& /*input*/) {
}
