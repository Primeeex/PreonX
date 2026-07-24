#pragma once

#include "application_state.hpp"
#include "scenes/scene_manager.hpp"
#include "window/window.hpp"
#include "input/input.hpp"
#include "rendering/camera.hpp"
#include "rendering/debug_renderer.hpp"
#include "ui/debug_ui.hpp"

#include <memory>
#include <vector>
#include <string>

class Application {
public:
    Application();
    ~Application();

    void startup(int argc, char* argv[]);
    void run();
    void shutdown();

    [[nodiscard]] AppState& state() { return state_; }
    [[nodiscard]] const AppState& state() const { return state_; }
    [[nodiscard]] SceneManager& scenes() { return sceneManager_; }

private:
    void processArguments(int argc, char* argv[]);
    void handleEvents();
    void update(float dt);
    void render();

    AppState state_;
    Window window_;
    Input input_;
    Camera camera_;
    DebugRenderer renderer_;
    DebugUI debugUI_;
    SceneManager sceneManager_;

    bool paused_ = false;
    bool stepOnce_ = false;

    float physicsTime_ = 0.0f;
    float renderTime_ = 0.0f;
    float uiTime_ = 0.0f;
    float frameTime_ = 0.0f;
    int fps_ = 0;
    int fpsCounter_ = 0;
    float fpsAccumulator_ = 0.0f;
};
