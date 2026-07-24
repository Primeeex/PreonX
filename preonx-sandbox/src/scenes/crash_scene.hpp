#pragma once

#include "scenes/scene.hpp"

class CrashScene : public Scene {
public:
    void init() override;
    void cleanup() override;
    void update(float dt, bool paused, bool stepOnce) override;
    void render(DebugRenderer& renderer, const Camera& camera) override;
    void handleInput(const Input& input) override;

    [[nodiscard]] const char* name() const override { return "Crash Test"; }
    [[nodiscard]] const char* description() const override { return "Projectile hits angled objects"; }

    void configure(float projMass, float projSpeed, int boxCount, float boxMass);

private:
    static constexpr int kMaxBoxes = 16;
    primeon::math::BodyID projectile_ = primeon::math::kInvalidBody;
    primeon::math::BodyID boxes_[kMaxBoxes] = {};
    int boxCount_ = 0;
    float projectileMass_ = 20.0f;
    float projectileSpeed_ = 25.0f;
    float boxMass_ = 1.0f;
};
