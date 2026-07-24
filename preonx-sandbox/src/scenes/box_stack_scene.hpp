#pragma once

#include "scenes/scene.hpp"

class BoxStackScene : public Scene {
public:
    void init() override;
    void cleanup() override;
    void update(float dt, bool paused, bool stepOnce) override;
    void render(DebugRenderer& renderer, const Camera& camera) override;
    void handleInput(const Input& input) override;

    [[nodiscard]] const char* name() const override { return "Box Stack"; }
    [[nodiscard]] const char* description() const override { return "Stacked boxes testing solver stability"; }

    void configure(int count, float mass, float restitution, float spacing);

private:
    static constexpr int kMaxStackHeight = 32;
    int stackHeight_ = 12;
    float boxMass_ = 1.0f;
    float restitution_ = 0.0f;
    float spacing_ = 1.05f;
    primeon::math::BodyID ground_ = 0;
    primeon::math::BodyID boxes_[kMaxStackHeight] = {};
};
