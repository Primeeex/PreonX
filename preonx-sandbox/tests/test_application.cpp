#include <gtest/gtest.h>
#include "application.hpp"
#include "scenes/scene_manager.hpp"
#include "scenes/basic_drop_scene.hpp"
#include "scenes/box_stack_scene.hpp"
#include "scenes/collision_scene.hpp"
#include "scenes/stress_scene.hpp"
#include "scenes/crash_scene.hpp"
#include "input/input.hpp"
#include "rendering/camera.hpp"
#include "rendering/debug_renderer.hpp"
#include "config.hpp"

using namespace primeon::math;

// ── Application ──────────────────────────────────────────────────────────────

TEST(Application, DefaultState) {
    AppState state;
    EXPECT_TRUE(state.running);
    EXPECT_EQ(state.windowWidth, 1280u);
    EXPECT_EQ(state.windowHeight, 720u);
    EXPECT_EQ(state.frameCount, 0u);
    EXPECT_TRUE(state.gravityEnabled);
}

// ── SceneManager ─────────────────────────────────────────────────────────────

TEST(SceneManager, AddAndCount) {
    SceneManager mgr;
    mgr.addScene(std::make_unique<BasicDropScene>());
    mgr.addScene(std::make_unique<BoxStackScene>());
    mgr.addScene(std::make_unique<CollisionScene>());
    mgr.addScene(std::make_unique<StressScene>());
    mgr.addScene(std::make_unique<CrashScene>());
    EXPECT_EQ(mgr.sceneCount(), 5u);
}

TEST(SceneManager, SelectScene) {
    SceneManager mgr;
    mgr.addScene(std::make_unique<BasicDropScene>());
    mgr.addScene(std::make_unique<BoxStackScene>());
    mgr.selectScene(1);
    EXPECT_EQ(mgr.currentSceneIndex(), 1u);
    EXPECT_STREQ(mgr.currentSceneName().c_str(), "Box Stack");
}

TEST(SceneManager, NextPrevScene) {
    SceneManager mgr;
    mgr.addScene(std::make_unique<BasicDropScene>());
    mgr.addScene(std::make_unique<BoxStackScene>());
    mgr.addScene(std::make_unique<CollisionScene>());
    mgr.selectScene(0);
    mgr.nextScene();
    EXPECT_EQ(mgr.currentSceneIndex(), 1u);
    mgr.nextScene();
    EXPECT_EQ(mgr.currentSceneIndex(), 2u);
    mgr.nextScene();
    EXPECT_EQ(mgr.currentSceneIndex(), 0u);
    mgr.prevScene();
    EXPECT_EQ(mgr.currentSceneIndex(), 2u);
}

TEST(SceneManager, ResetScene) {
    SceneManager mgr;
    mgr.addScene(std::make_unique<BasicDropScene>());
    mgr.selectScene(0);
    mgr.resetScene();
    EXPECT_EQ(mgr.currentSceneIndex(), 0u);
    EXPECT_NE(mgr.currentScene(), nullptr);
}

// ── Scene Physics ────────────────────────────────────────────────────────────

TEST(BasicDropScene, PhysicsUpdates) {
    BasicDropScene scene;
    scene.init();
    auto& world = scene.getWorld();
    f32 y0 = world.getBody(1).position.y;
    for (int i = 0; i < 60; ++i) {
        scene.update(1.0f / 60.0f, false, false);
    }
    f32 y1 = world.getBody(1).position.y;
    EXPECT_LT(y1, y0);
    scene.cleanup();
}

TEST(BoxStackScene, BodiesCreated) {
    BoxStackScene scene;
    scene.init();
    EXPECT_GT(scene.getWorld().getBodyCount(), 10u);
    scene.cleanup();
}

TEST(CollisionScene, SphereFalls) {
    CollisionScene scene;
    scene.init();
    f32 y0 = scene.getWorld().getBody(1).position.y;
    for (int i = 0; i < 60; ++i) {
        scene.update(1.0f / 60.0f, false, false);
    }
    f32 y1 = scene.getWorld().getBody(1).position.y;
    EXPECT_LT(y1, y0);
    scene.cleanup();
}

TEST(StressScene, ManyBodies) {
    StressScene scene;
    scene.init();
    EXPECT_GT(scene.getBodyCount(), 500u);
    scene.update(1.0f / 30.0f, false, false);
    EXPECT_GE(scene.getAvgStepMs(), 0.0f);
    scene.cleanup();
}

TEST(CrashScene, ProjectileLaunched) {
    CrashScene scene;
    scene.init();
    auto& world = scene.getWorld();
    f32 x0 = world.getBody(1).position.x;
    for (int i = 0; i < 30; ++i) {
        scene.update(1.0f / 60.0f, false, false);
    }
    f32 x1 = world.getBody(1).position.x;
    EXPECT_GT(x1, x0);
    scene.cleanup();
}

TEST(CrashScene, BoxesCreated) {
    CrashScene scene;
    scene.init();
    EXPECT_GT(scene.getWorld().getBodyCount(), 5u);
    scene.cleanup();
}

TEST(Config, DefaultValues) {
    SandboxConfig cfg;
    EXPECT_TRUE(cfg.gravityEnabled);
    EXPECT_FLOAT_EQ(cfg.gravityStrength, 9.81f);
    EXPECT_EQ(cfg.boxStackCount, 12);
    EXPECT_EQ(cfg.crashBoxCount, 8);
}

TEST(Config, LoadFile) {
    SandboxConfig cfg = SandboxConfig::loadFrom("sandbox.cfg");
    EXPECT_TRUE(cfg.gravityEnabled);
    EXPECT_FLOAT_EQ(cfg.gravityStrength, 9.81f);
    EXPECT_EQ(cfg.boxStackCount, 12);
    EXPECT_EQ(cfg.crashProjectileSpeed, 25.0f);
}

// ── Determinism ──────────────────────────────────────────────────────────────

TEST(BasicDropScene, DeterministicResults) {
    auto runSim = []() {
        BasicDropScene scene;
        scene.init();
        for (int i = 0; i < 120; ++i) {
            scene.update(1.0f / 60.0f, false, false);
        }
        f32 y = scene.getWorld().getBody(1).position.y;
        f32 vx = scene.getWorld().getBody(1).linearVelocity.x;
        scene.cleanup();
        return std::make_pair(y, vx);
    };

    auto [y1, vx1] = runSim();
    auto [y2, vx2] = runSim();
    EXPECT_FLOAT_EQ(y1, y2);
    EXPECT_FLOAT_EQ(vx1, vx2);
}

// ── Input ────────────────────────────────────────────────────────────────────

TEST(Input, DefaultState) {
    Input input;
    EXPECT_FALSE(input.isKeyDown(SDL_SCANCODE_SPACE));
    EXPECT_FALSE(input.isKeyPressed(SDL_SCANCODE_SPACE));
    EXPECT_FALSE(input.isMouseButtonDown(SDL_BUTTON_LEFT));
    EXPECT_EQ(input.getMouseX(), 0);
    EXPECT_EQ(input.getMouseY(), 0);
}

TEST(Input, KeyPressRelease) {
    Input input;
    input.beginFrame();
    SDL_Event event{};
    event.type = SDL_KEYDOWN;
    event.key.keysym.scancode = SDL_SCANCODE_A;
    input.processEvent(event);
    EXPECT_TRUE(input.isKeyPressed(SDL_SCANCODE_A));

    input.beginFrame();
    EXPECT_FALSE(input.isKeyPressed(SDL_SCANCODE_A));
    EXPECT_TRUE(input.isKeyDown(SDL_SCANCODE_A));

    event.type = SDL_KEYUP;
    input.processEvent(event);
    EXPECT_FALSE(input.isKeyDown(SDL_SCANCODE_A));
    EXPECT_TRUE(input.isKeyReleased(SDL_SCANCODE_A));

    input.beginFrame();
    EXPECT_FALSE(input.isKeyReleased(SDL_SCANCODE_A));
}

// ── Camera ───────────────────────────────────────────────────────────────────

TEST(Camera, DefaultView) {
    Camera cam;
    cam.init(1280.0f, 720.0f);
    EXPECT_FLOAT_EQ(cam.getZoom(), 50.0f);
}

TEST(Camera, Pan) {
    Camera cam;
    cam.init(1280.0f, 720.0f);
    float x0 = cam.getCenter().x;
    cam.pan(100.0f, 0.0f);
    EXPECT_NE(cam.getCenter().x, x0);
}

TEST(Camera, Zoom) {
    Camera cam;
    cam.init(1280.0f, 720.0f);
    float z0 = cam.getZoom();
    cam.zoom(2.0f);
    EXPECT_GT(cam.getZoom(), z0);
    cam.zoom(0.01f);
    EXPECT_GE(cam.getZoom(), 1.0f);
}

TEST(Camera, Reset) {
    Camera cam;
    cam.init(1280.0f, 720.0f);
    cam.pan(500.0f, 500.0f);
    cam.zoom(0.1f);
    cam.reset();
    EXPECT_FLOAT_EQ(cam.getZoom(), 50.0f);
}

TEST(Camera, ScreenToWorld) {
    Camera cam;
    cam.init(1280.0f, 720.0f);
    auto center = cam.screenToWorld(640.0f, 360.0f);
    EXPECT_NEAR(center.x, cam.getCenter().x, 0.01f);
    EXPECT_NEAR(center.y, cam.getCenter().y, 0.01f);
}
