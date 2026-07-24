#include "application.hpp"
#include "scenes/basic_drop_scene.hpp"
#include "scenes/box_stack_scene.hpp"
#include "scenes/collision_scene.hpp"
#include "scenes/stress_scene.hpp"
#include "scenes/crash_scene.hpp"
#include "rendering/opengl.hpp"
#include "config.hpp"

#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_opengl3.h>
#include <cstdio>
#include <chrono>
#include <cstring>

Application::Application() = default;
Application::~Application() = default;

void Application::startup(int argc, char* argv[]) {
    processArguments(argc, argv);

    SandboxConfig cfg = SandboxConfig::loadFrom("sandbox.cfg");

    if (!window_.create("PreonX Sandbox v0.3.0", state_.windowWidth, state_.windowHeight)) {
        state_.running = false;
        return;
    }

    glViewport(0, 0, static_cast<GLsizei>(state_.windowWidth), static_cast<GLsizei>(state_.windowHeight));
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_DEPTH_TEST);

    renderer_.init();
    camera_.init(static_cast<float>(state_.windowWidth), static_cast<float>(state_.windowHeight));
    debugUI_.init(window_.getHandle(), window_.getGLContext());

    state_.gravityEnabled = cfg.gravityEnabled;

    auto dropScene = std::make_unique<BasicDropScene>();
    auto boxScene = std::make_unique<BoxStackScene>();
    auto collScene = std::make_unique<CollisionScene>();
    auto stressScene = std::make_unique<StressScene>();
    auto crashScene = std::make_unique<CrashScene>();

    boxScene->configure(cfg.boxStackCount, cfg.boxStackMass, cfg.boxStackRestitution, cfg.boxStackSpacing);
    crashScene->configure(cfg.crashProjectileMass, cfg.crashProjectileSpeed, cfg.crashBoxCount, cfg.crashBoxMass);
    stressScene->configure(cfg.stressGridX, cfg.stressGridY, cfg.stressGridZ, cfg.stressMass);

    sceneManager_.addScene(std::move(dropScene));
    sceneManager_.addScene(std::move(boxScene));
    sceneManager_.addScene(std::move(collScene));
    sceneManager_.addScene(std::move(stressScene));
    sceneManager_.addScene(std::move(crashScene));

    sceneManager_.selectScene(state_.selectedScene);

    std::printf("PreonX Sandbox v0.3.0 started\n");
    std::printf("Loaded config: sandbox.cfg\n");
    sceneManager_.listAll();
}

void Application::processArguments(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--scene" && i + 1 < argc) {
            state_.selectedScene = static_cast<u32>(std::atoi(argv[++i]));
        } else if (arg == "--help" || arg == "-h") {
            std::printf("Usage: preonx_sandbox [options]\n");
            std::printf("  --scene <index>  Select scene by index\n");
            std::printf("  --help, -h       Show this help\n");
            state_.running = false;
            return;
        }
    }
}

void Application::handleEvents() {
    input_.beginFrame();

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            state_.running = false;
            return;
        }
        if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_RESIZED) {
            state_.windowWidth = static_cast<u32>(event.window.data1);
            state_.windowHeight = static_cast<u32>(event.window.data2);
            glViewport(0, 0, event.window.data1, event.window.data2);
            camera_.setViewport(static_cast<float>(state_.windowWidth), static_cast<float>(state_.windowHeight));
        }
        if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) {
            state_.running = false;
            return;
        }
        ImGui_ImplSDL2_ProcessEvent(&event);
        input_.processEvent(event);
    }

    if (!debugUI_.wantsMouse()) {
        if (input_.isMouseButtonDown(SDL_BUTTON_MIDDLE)) {
            float dx = static_cast<float>(input_.getMouseDeltaX());
            float dy = static_cast<float>(input_.getMouseDeltaY());
            camera_.pan(dx, dy);
        }
        int wheel = input_.getMouseWheel();
        if (wheel != 0) {
            camera_.zoom(wheel > 0 ? 1.1f : 0.9f);
        }
    }

    if (!debugUI_.wantsKeyboard()) {
        if (input_.isKeyPressed(SDL_SCANCODE_R)) {
            sceneManager_.resetScene();
        }
        if (input_.isKeyPressed(SDL_SCANCODE_SPACE)) {
            paused_ = !paused_;
        }
        if (input_.isKeyPressed(SDL_SCANCODE_N)) {
            stepOnce_ = true;
        }
        if (input_.isKeyPressed(SDL_SCANCODE_RIGHT)) {
            sceneManager_.nextScene();
        }
        if (input_.isKeyPressed(SDL_SCANCODE_LEFT)) {
            sceneManager_.prevScene();
        }
        if (input_.isKeyPressed(SDL_SCANCODE_HOME)) {
            camera_.reset();
        }
    }

    if (!window_.isOpen()) {
        state_.running = false;
    }
}

void Application::update(float dt) {
    state_.frameCount++;
    state_.totalTime += dt;

    Scene* scene = sceneManager_.currentScene();
    if (scene) {
        scene->handleInput(input_);

        bool step = stepOnce_;
        stepOnce_ = false;

        auto t0 = std::chrono::high_resolution_clock::now();
        scene->setGravityEnabled(state_.gravityEnabled);
        scene->update(dt * state_.simulationSpeed, paused_, step);
        auto t1 = std::chrono::high_resolution_clock::now();
        physicsTime_ = std::chrono::duration<float, std::milli>(t1 - t0).count();
    }

    state_.currentSceneName = sceneManager_.currentSceneName();
}

void Application::render() {
    auto t0 = std::chrono::high_resolution_clock::now();

    glClearColor(0.12f, 0.12f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float view[16], proj[16];
    camera_.getViewMatrix(view);
    camera_.getProjectionMatrix(proj);

    renderer_.beginFrame();

    Scene* scene = sceneManager_.currentScene();
    if (scene) {
        scene->render(renderer_, camera_);
    }

    renderer_.endFrame(view, proj);

    debugUI_.beginFrame();
    DebugUIData uiData;
    uiData.paused = paused_;
    uiData.gravityEnabled = state_.gravityEnabled;
    uiData.fixedDt = state_.fixedDt;
    uiData.simulationSpeed = state_.simulationSpeed;
    uiData.fps = static_cast<float>(fps_);
    uiData.frameTime = frameTime_;
    uiData.physicsTime = physicsTime_;
    uiData.renderTime = renderTime_;
    debugUI_.renderPanel(uiData, sceneManager_);
    paused_ = uiData.paused;
    state_.gravityEnabled = uiData.gravityEnabled;
    state_.fixedDt = uiData.fixedDt;
    state_.simulationSpeed = uiData.simulationSpeed;
    debugUI_.endFrame();

    SDL_GL_SwapWindow(window_.getHandle());

    auto t1 = std::chrono::high_resolution_clock::now();
    renderTime_ = std::chrono::duration<float, std::milli>(t1 - t0).count();
}

void Application::run() {
    using Clock = std::chrono::high_resolution_clock;
    auto lastTime = Clock::now();

    while (state_.running) {
        auto now = Clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;
        frameTime_ = dt * 1000.0f;

        fpsCounter_++;
        fpsAccumulator_ += dt;
        if (fpsAccumulator_ >= 1.0f) {
            fps_ = fpsCounter_;
            fpsCounter_ = 0;
            fpsAccumulator_ -= 1.0f;
        }

        handleEvents();
        if (!state_.running) break;

        float simDt = (dt < 0.1f) ? dt : 0.1f;
        update(simDt);
        render();
    }
}

void Application::shutdown() {
    if (auto* scene = sceneManager_.currentScene()) {
        scene->cleanup();
    }
    debugUI_.shutdown();
    renderer_.shutdown();
    window_.destroy();
    std::printf("Sandbox shut down after %u frames (%.1fs)\n", state_.frameCount, static_cast<double>(state_.totalTime));
}
