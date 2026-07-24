#pragma once

#include "scenes/scene.hpp"
#include <chrono>

class StressScene : public Scene {
public:
    void init() override;
    void cleanup() override;
    void update(float dt, bool paused, bool stepOnce) override;
    void render(DebugRenderer& renderer, const Camera& camera) override;
    void handleInput(const Input& input) override;

    [[nodiscard]] const char* name() const override { return "Stress Test"; }
    [[nodiscard]] const char* description() const override { return "Massive body simulation with performance stats"; }

    [[nodiscard]] float getAvgStepMs() const { return avgStepMs_; }
    [[nodiscard]] unsigned getBodyCount() const { return static_cast<unsigned>(bodyCount_); }
    [[nodiscard]] float getFPS() const { return fps_; }

    void configure(int gx, int gy, int gz, float mass);

private:
    static constexpr int kMaxBodies = 3000;
    primeon::math::BodyID ground_ = 0;
    primeon::math::BodyID bodies_[kMaxBodies] = {};
    int bodyCount_ = 0;
    int gridX_ = 12, gridY_ = 8, gridZ_ = 8;

    float avgStepMs_ = 0.0f;
    float stepAccum_ = 0.0f;
    int stepCount_ = 0;
    float fps_ = 0.0f;
    float fpsAccum_ = 0.0f;
    int fpsFrameCount_ = 0;
    std::chrono::steady_clock::time_point lastFpsTime_;

    float spawnTime_ = 0.0f;
    bool spawnComplete_ = false;
};
