#include "scenes/stress_scene.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>

using namespace primeon::math;

void StressScene::configure(int gx, int gy, int gz, float mass) {
    gridX_ = gx;
    gridY_ = gy;
    gridZ_ = gz;

    WorldConfig cfg;
    cfg.gravity = kGravityDown;
    cfg.fixedDt = 1.0f / 120.0f;
    cfg.sleepingEnabled = true;
    cfg.maxBodies = kMaxBodies + 16;
    cfg.solver.velocityIterations = 0;
    cfg.solver.positionIterations = 0;
    world_ = PhysicsWorld(cfg);

    ground_ = world_.createBody(BodyDescriptor::staticBody(Vector3(0.0f, -1.0f, 0.0f)));

    bodyCount_ = 0;
    float spacing = 1.05f;
    int limit = std::min(gridX_ * gridY_ * gridZ_, kMaxBodies);

    for (int y = 0; y < gridY_ && bodyCount_ < limit; ++y) {
        for (int x = 0; x < gridX_ && bodyCount_ < limit; ++x) {
            for (int z = 0; z < gridZ_ && bodyCount_ < limit; ++z) {
                float px = (static_cast<float>(x) - static_cast<float>(gridX_) * 0.5f) * spacing;
                float py = static_cast<float>(y) * spacing + 0.5f;
                float pz = (static_cast<float>(z) - static_cast<float>(gridZ_) * 0.5f) * spacing;

                float jitterX = (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) - 0.5f) * 0.05f;
                float jitterY = (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) - 0.5f) * 0.05f;

                BodyDescriptor desc = BodyDescriptor::dynamicBody(Vector3(px + jitterX, py + jitterY, pz), mass);
                desc.inertia = InertiaTensor::solidBox(mass, Vector3(0.4f, 0.4f, 0.4f));
                bodies_[bodyCount_++] = world_.createBody(desc);
            }
        }
    }

    spawnTime_ = 0.0f;
    spawnComplete_ = false;
    stepAccum_ = 0.0f;
    stepCount_ = 0;
    fpsAccum_ = 0.0f;
    fpsFrameCount_ = 0;
    lastFpsTime_ = std::chrono::steady_clock::now();
}

void StressScene::init() {
    configure(12, 8, 8, 1.0f);
}

void StressScene::cleanup() {
    world_ = PhysicsWorld();
    bodyCount_ = 0;
    spawnComplete_ = false;
}

void StressScene::update(float dt, bool paused, bool stepOnce) {
    auto t0 = std::chrono::steady_clock::now();

    if (!spawnComplete_) {
        spawnTime_ += dt;
        if (spawnTime_ > 1.0f) spawnComplete_ = true;
    }

    if (!paused || stepOnce) {
        world_.step(dt);
        resolveGroundCollisions(-1.0f, 0.4f, 0.0f, 1.0f);
        resolveBodyCollisions(0.4f, 0.05f);
    }

    auto t1 = std::chrono::steady_clock::now();
    float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
    stepAccum_ += ms;
    ++stepCount_;
    if (stepCount_ > 30) {
        avgStepMs_ = stepAccum_ / static_cast<float>(stepCount_);
        stepAccum_ = 0.0f;
        stepCount_ = 0;
    }

    ++fpsFrameCount_;
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - lastFpsTime_).count();
    if (elapsed >= 0.5f) {
        fps_ = static_cast<float>(fpsFrameCount_) / elapsed;
        fpsFrameCount_ = 0;
        lastFpsTime_ = now;
    }
}

void StressScene::render(DebugRenderer& renderer, const Camera& camera) {
    float halfX = static_cast<float>(gridX_) * 1.05f * 0.5f + 1.0f;
    float halfZ = static_cast<float>(gridZ_) * 1.05f * 0.5f + 1.0f;
    float gridSize = std::max(halfX, halfZ);

    for (float x = -gridSize; x <= gridSize; x += 2.0f) {
        renderer.drawLine({x, -1.0f, -gridSize}, {x, -1.0f, gridSize}, {0.25f, 0.25f, 0.28f});
    }
    for (float z = -gridSize; z <= gridSize; z += 2.0f) {
        renderer.drawLine({-gridSize, -1.0f, z}, {gridSize, -1.0f, z}, {0.25f, 0.25f, 0.28f});
    }

    renderer.drawFilledRectangle(
        {-50.0f, -2.0f, 0.0f}, {50.0f, -1.0f, 0.0f}, {0.3f, 0.3f, 0.32f});
    renderer.drawLine({-50.0f, -1.0f, 0.0f}, {50.0f, -1.0f, 0.0f}, {0.5f, 0.5f, 0.55f});

    for (int i = 0; i < bodyCount_; ++i) {
        const auto& body = world_.getBody(bodies_[i]);
        float angle = getRotationAngle(body.rotation);

        float heightFactor = body.position.y / (static_cast<float>(gridY_) * 1.05f + 0.5f);
        heightFactor = std::clamp(heightFactor, 0.0f, 1.0f);

        Vector3 color;
        if (heightFactor < 0.33f) {
            float t = heightFactor * 3.0f;
            color = {0.1f + t * 0.1f, 0.3f + t * 0.4f, 0.8f - t * 0.2f};
        } else if (heightFactor < 0.66f) {
            float t = (heightFactor - 0.33f) * 3.0f;
            color = {0.2f + t * 0.7f, 0.7f - t * 0.1f, 0.6f - t * 0.4f};
        } else {
            float t = (heightFactor - 0.66f) * 3.0f;
            color = {0.9f + t * 0.1f, 0.6f - t * 0.4f, 0.2f - t * 0.1f};
        }

        float speed = std::sqrt(body.linearVelocity.x * body.linearVelocity.x +
                                 body.linearVelocity.y * body.linearVelocity.y);
        if (speed > 2.0f) {
            float boost = std::min(speed / 20.0f, 0.5f);
            color.x = std::min(color.x + boost, 1.0f);
            color.z = std::max(color.z - boost * 0.3f, 0.0f);
        }

        renderer.drawRotatedFilledRect(body.position.x, body.position.y, 0.4f, 0.4f,
                                       angle, color);
        if (body.isSleeping()) {
            renderer.drawRotatedRect(body.position.x, body.position.y, 0.4f, 0.4f,
                                     angle, {0.4f, 0.4f, 0.5f});
        } else {
            renderer.drawRotatedRect(body.position.x, body.position.y, 0.4f, 0.4f,
                                     angle, {0.8f, 0.8f, 0.85f});
        }
    }

    for (int i = 0; i < bodyCount_; ++i) {
        const auto& body = world_.getBody(bodies_[i]);
        float spd = std::sqrt(body.linearVelocity.x * body.linearVelocity.x +
                              body.linearVelocity.y * body.linearVelocity.y);
        if (spd > 1.5f) {
            float vn = spd;
            Vector3 dir = {body.linearVelocity.x / vn, body.linearVelocity.y / vn, 0.0f};
            renderer.drawArrow(body.position, dir, std::min(vn * 0.15f, 2.0f), {1.0f, 0.6f, 0.0f});
        }
    }
}

void StressScene::handleInput(const Input& input) {
    if (input.isKeyPressed(SDL_SCANCODE_R)) {
        cleanup();
        init();
    }
}
