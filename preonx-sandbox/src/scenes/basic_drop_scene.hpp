#pragma once

#include "scenes/scene.hpp"

class BasicDropScene : public Scene {
public:
    void init() override;
    void cleanup() override;
    void update(float dt, bool paused, bool stepOnce) override;
    void render(DebugRenderer& renderer, const Camera& camera) override;
    void handleInput(const Input& input) override;

    [[nodiscard]] const char* name() const override { return "Basic Drop"; }
    [[nodiscard]] const char* description() const override { return "Box falling under gravity with ground plane"; }

private:
    primeon::math::BodyID ground_ = 0;
    primeon::math::BodyID box_ = 0;
};
