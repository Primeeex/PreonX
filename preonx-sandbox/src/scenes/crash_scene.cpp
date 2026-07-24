#include "scenes/crash_scene.hpp"
#include <cmath>
#include <algorithm>

using namespace primeon::math;

void CrashScene::configure(float projMass, float projSpeed, int count, float boxMass) {
    projectileMass_ = projMass;
    projectileSpeed_ = projSpeed;
    boxCount_ = std::min(count, kMaxBoxes);
    boxMass_ = boxMass;

    WorldConfig cfg;
    cfg.gravity = kGravityDown;
    cfg.fixedDt = 1.0f / 120.0f;
    cfg.sleepingEnabled = false;
    cfg.maxBodies = kMaxBoxes + 4;
    cfg.solver.velocityIterations = 8;
    cfg.solver.positionIterations = 3;
    world_ = PhysicsWorld(cfg);

    BodyDescriptor groundDesc = BodyDescriptor::staticBody(Vector3(0.0f, -1.0f, 0.0f));
    groundDesc.material = PhysicsMaterial(0.6f, 0.3f, 1.0f);
    world_.createBody(groundDesc);

    BodyDescriptor projDesc = BodyDescriptor::dynamicBody(Vector3(-18.0f, 2.5f, 0.0f), projMass);
    projDesc.collider = ColliderData::sphere(0.6f);
    projDesc.inertia = InertiaTensor::solidBox(projMass, Vector3(0.6f, 0.6f, 0.6f));
    projDesc.material = PhysicsMaterial(0.4f, 0.5f, 1.0f);
    projectile_ = world_.createBody(projDesc);

    float spacing = 1.4f;
    float startX = 2.0f;
    float angles[] = {0.0f, 15.0f, 30.0f, -20.0f, 45.0f, -10.0f, 35.0f, -30.0f,
                      25.0f, -15.0f, 40.0f, -25.0f, 10.0f, -35.0f, 20.0f, -40.0f};

    for (int i = 0; i < boxCount_; ++i) {
        float x = startX + static_cast<float>(i) * spacing;
        float angle = angles[i % 16] * 3.14159265f / 180.0f;

        BodyDescriptor desc = BodyDescriptor::dynamicBody(Vector3(x, 0.55f, 0.0f), boxMass);
        desc.inertia = InertiaTensor::solidBox(boxMass, Vector3(0.45f, 0.45f, 0.45f));
        desc.rotation = Quaternion::fromAxisAngle(Vector3(0.0f, 0.0f, 1.0f), angle);
        desc.material = PhysicsMaterial(0.5f, 0.4f, 1.0f);
        boxes_[i] = world_.createBody(desc);
    }

    auto& proj = world_.getBody(projectile_);
    proj.linearVelocity = Vector3(projectileSpeed_, 0.0f, 0.0f);
}

void CrashScene::init() {
    configure(20.0f, 30.0f, 8, 1.0f);
}

void CrashScene::cleanup() {
    world_ = PhysicsWorld();
}

void CrashScene::update(float dt, bool paused, bool stepOnce) {
    if (!paused || stepOnce) {
        world_.step(dt);
        resolveGroundCollisions(-1.0f, 0.5f, 0.35f);
        resolveBodyCollisions(0.5f, 0.5f);
    }
}

void CrashScene::render(DebugRenderer& renderer, const Camera& /*camera*/) {
    renderer.drawFilledRectangle(
        {-50.0f, -2.0f, 0.0f}, {50.0f, -1.0f, 0.0f}, {0.35f, 0.35f, 0.38f});
    renderer.drawLine({-50.0f, -1.0f, 0.0f}, {50.0f, -1.0f, 0.0f}, {0.6f, 0.6f, 0.65f});

    for (float x = -20.0f; x <= 20.0f; x += 2.0f) {
        renderer.drawLine({x, -1.0f, 0.0f}, {x, -1.0f, 0.0f}, {0.25f, 0.25f, 0.28f});
    }

    const auto& proj = world_.getBody(projectile_);
    float projAngle = getRotationAngle(proj.rotation);
    renderer.drawRotatedFilledRect(proj.position.x, proj.position.y, 0.6f, 0.4f, projAngle,
                                   {0.9f, 0.2f, 0.15f});
    renderer.drawRotatedRect(proj.position.x, proj.position.y, 0.6f, 0.4f, projAngle,
                             {1.0f, 0.4f, 0.3f});

    float spd = std::sqrt(proj.linearVelocity.x * proj.linearVelocity.x +
                          proj.linearVelocity.y * proj.linearVelocity.y);
    if (spd > 1.0f) {
        Vector3 dir = {proj.linearVelocity.x / spd, proj.linearVelocity.y / spd, 0.0f};
        renderer.drawArrow(proj.position, dir, std::min(spd * 0.1f, 3.0f), {1.0f, 0.5f, 0.0f});
    }

    for (int i = 0; i < boxCount_; ++i) {
        const auto& body = world_.getBody(boxes_[i]);
        float angle = getRotationAngle(body.rotation);

        float speed = std::sqrt(body.linearVelocity.x * body.linearVelocity.x +
                                body.linearVelocity.y * body.linearVelocity.y);
        float t = static_cast<float>(i) / static_cast<float>(boxCount_);
        float speedFactor = std::min(speed / 15.0f, 1.0f);

        Vector3 color;
        if (speedFactor > 0.3f) {
            color = {0.9f + speedFactor * 0.1f, 0.3f - speedFactor * 0.2f, 0.2f};
        } else {
            color = {0.2f + t * 0.5f, 0.7f - t * 0.2f, 0.3f + t * 0.5f};
        }

        renderer.drawRotatedFilledRect(body.position.x, body.position.y, 0.45f, 0.45f,
                                       angle, color);
        renderer.drawRotatedRect(body.position.x, body.position.y, 0.45f, 0.45f,
                                 angle, {1.0f, 1.0f, 1.0f});

        if (speed > 2.0f) {
            Vector3 velDir = {body.linearVelocity.x / speed, body.linearVelocity.y / speed, 0.0f};
            renderer.drawArrow(body.position, velDir, std::min(speed * 0.08f, 1.5f),
                               {1.0f, 0.7f, 0.0f});
        }
    }
}

void CrashScene::handleInput(const Input& input) {
    if (input.isKeyPressed(SDL_SCANCODE_R)) {
        cleanup();
        init();
    }
}
