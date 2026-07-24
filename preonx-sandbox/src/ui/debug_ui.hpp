#pragma once

#include "scenes/scene_manager.hpp"
#include "input/input.hpp"
#include <imgui.h>

struct DebugUIData {
    bool paused = false;
    bool stepOnce = false;
    bool gravityEnabled = true;
    float fixedDt = 1.0f / 60.0f;
    float simulationSpeed = 1.0f;
    float fps = 0.0f;
    float frameTime = 0.0f;
    float physicsTime = 0.0f;
    float renderTime = 0.0f;
};

class DebugUI {
public:
    void init(void* window, void* glContext);
    void shutdown();
    void beginFrame();
    void endFrame();

    void renderPanel(DebugUIData& data, SceneManager& scenes);

    [[nodiscard]] bool wantsMouse() const { return io_ && io_->WantCaptureMouse; }
    [[nodiscard]] bool wantsKeyboard() const { return io_ && io_->WantCaptureKeyboard; }

private:
    ImGuiIO* io_ = nullptr;
};
