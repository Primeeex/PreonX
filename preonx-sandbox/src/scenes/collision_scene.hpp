#pragma once

#include "scenes/scene.hpp"

class CollisionScene : public Scene {
public:
    void init() override;
    void cleanup() override;
    void update(float dt, bool paused, bool stepOnce) override;
    void render(DebugRenderer& renderer, const Camera& camera) override;
    void handleInput(const Input& input) override;

    [[nodiscard]] const char* name() const override { return "Collision"; }
    [[nodiscard]] const char* description() const override { return "All supported collision shapes"; }

private:
    primeon::math::BodyID ground_ = 0;
    primeon::math::BodyID sphereA_ = 0;
    primeon::math::BodyID sphereB_ = 0;
    primeon::math::BodyID boxA_ = 0;
    primeon::math::BodyID boxB_ = 0;
};
