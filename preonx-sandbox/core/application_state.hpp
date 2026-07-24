#pragma once

#include <primeon/math/scalar/scalar.hpp>
#include <string>

using primeon::math::f32;
using primeon::math::u32;

struct AppState {
    bool running = true;
    u32 windowWidth = 1280;
    u32 windowHeight = 720;
    u32 selectedScene = 0;
    u32 frameCount = 0;
    f32 totalTime = 0.0f;
    f32 fixedDt = 1.0f / 60.0f;
    f32 simulationSpeed = 1.0f;
    bool gravityEnabled = true;
    std::string currentSceneName;
};
