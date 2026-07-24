# PreonX Sandbox — Technical Reference

## Overview

The PreonX Sandbox is an interactive physics visualization application. It creates an SDL2 window with an OpenGL 3.3 core context, renders physics simulations using a debug line/triangle renderer, and provides an ImGui-based UI for real-time control.

## Build System

The CMake build produces:

| Target | Type | Description |
|--------|------|-------------|
| `preonx_sandbox_lib` | Static library | All sandbox code (no `main()`), testable |
| `preonx_sandbox` | Executable | Links `preonx_sandbox_lib`, provides `main()` |
| `sandbox_tests` | Executable | GoogleTest binary, links `preonx_sandbox_lib` |
| `imgui_lib` | Static library | Dear ImGui v1.91.8 with SDL2+OpenGL3 backends |

External dependencies fetched via `FetchContent`:
- **Dear ImGui** v1.91.8 — UI rendering
- **GoogleTest** v1.14.0 — unit testing

System dependencies (found via `find_package`):
- **SDL2** — windowing, input, OpenGL context
- **OpenGL** — rendering

## Module Reference

### Window (`src/window/`)

Wraps SDL2 window creation with OpenGL 3.3 core context. Requests multisampling (4x).

```
Window::create(title, width, height)  → bool
Window::destroy()
Window::pollEvents()                  // dispatches resize/quit/ESC
Window::shouldClose()                 → bool
Window::swap()                        // glSwapBuffers
```

### Input (`src/input/`)

Frame-based input system with previous-frame tracking for press/release detection.

```
Input::beginFrame()           // copies current → previous state
Input::processEvent(event)    // handles SDL_KEYDOWN/UP, MOUSE*, WHEEL
Input::isKeyDown(scancode)    → bool  // currently held
Input::isKeyPressed(scancode) → bool  // went down this frame
Input::isKeyReleased(scancode)→ bool  // went up this frame
Input::getMouseX/Y()          → int
Input::getMouseDeltaX/Y()     → int
Input::getMouseWheel()        → int
```

### Camera (`src/rendering/`)

2D orthographic camera with pan (middle-mouse drag) and zoom (scroll wheel).

```
Camera::init(viewWidth, viewHeight)
Camera::pan(dx, dy)
Camera::zoom(factor)
Camera::reset()
Camera::getViewMatrix(float[16])
Camera::getProjectionMatrix(float[16])
Camera::screenToWorld(screenX, screenY) → Vector3
```

Default: centered at origin, 50 pixels/unit zoom, range [1, 1000].

### DebugRenderer (`src/rendering/`)

Batched renderer using two GL programs: one for `GL_LINES`, one for `GL_TRIANGLES`. Each frame:
1. Call `beginFrame()` — clears vertex buffers
2. Issue draw commands (lines, triangles, shapes)
3. Call `endFrame(view, proj)` — uploads and draws batches

Supported primitives:
- `drawLine(a, b, color)` — colored line segment
- `drawPoint(pos, size, color)` — cross marker
- `drawCircle(center, radius, color, segments)` — wireframe circle
- `drawFilledCircle(...)` — filled circle (semi-transparent)
- `drawRectangle(min, max, color)` — wireframe rectangle
- `drawFilledRectangle(...)` — filled rectangle
- `drawBox(center, halfExtents, color)` — wireframe axis-aligned box
- `drawPolygon(points, count, color)` — wireframe polygon
- `drawArrow(from, dir, length, color)` — line with arrowhead

Capacity: 65536 vertices per batch.

### DebugUI (`src/ui/`)

ImGui panel with four collapsible sections:

1. **Controls** — pause, step, reset, gravity toggle, fixed dt slider, speed slider
2. **Scene** — clickable scene list (shows all scenes by name)
3. **Physics** — body counts, contacts, islands, frame count from `PhysicsWorld`
4. **Performance** — FPS, frame time, physics time, render time

Mouse/keyboard input is suppressed when ImGui wants capture (`wantsMouse()`/`wantsKeyboard()`).

### SceneManager (`src/scenes/`)

Manages scene lifecycle with `init()`/`cleanup()` transitions.

```
SceneManager::addScene(unique_ptr<Scene>)
SceneManager::selectScene(index)  // cleanup old, init new
SceneManager::resetScene()        // cleanup + init current
SceneManager::nextScene() / prevScene()
SceneManager::sceneCount() → unsigned
SceneManager::sceneName(index) → const char*
```

### Scene (`src/scenes/`)

Base class. Each scene owns a `PhysicsWorld` and implements:

```cpp
class Scene {
    virtual void init() = 0;
    virtual void cleanup() = 0;
    virtual void update(float dt, bool paused, bool stepOnce) = 0;
    virtual void render(DebugRenderer& renderer, const Camera& camera) = 0;
    virtual void handleInput(const Input& input) = 0;
    virtual const char* name() const = 0;
    virtual const char* description() const = 0;
};
```

`cleanup()` resets the world via `world_ = PhysicsWorld()`.

## Built-in Scenes

### Basic Drop (`--scene 0`)
Single dynamic box falling under gravity with ground plane and velocity arrow. Demonstrates basic rigid body dynamics, gravity, and sleep states.

### Box Stack (`--scene 1`)
12 stacked boxes at 120 Hz fixed timestep (12 velocity iterations, 8 position iterations). Stress-tests solver stability and contact management.

### Collision (`--scene 2`)
Two spheres + two boxes dropping with contact visualization. Shows broadphase/narrowphase interaction and contact point rendering.

### Stress Test (`--scene 3`)
12×8×8 = 768 body grid with random sizes and masses. Measures average step time in milliseconds for performance benchmarking. Uses 120 Hz fixed timestep.

## Application Loop

```
Application::run()
  while running:
    window.pollEvents()
    input.beginFrame()
    while SDL_PollEvent:
      input.processEvent(event)
      debugUI.processEvent(event)
    handleInput()     // keyboard shortcuts (R, Space, N, arrows, Home)
    update()          // fixed timestep physics, scene update
    render()          // clear → camera → renderer → scene render → ImGui → swap
    updateTimings()   // FPS counter, frame/physics/render times
```

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `R` | Reset current scene |
| `Space` | Toggle pause |
| `N` | Step one frame (when paused) |
| `Left` | Previous scene |
| `Right` | Next scene |
| `Home` | Reset camera |
| `ESC` | Quit |

### Fixed Timestep

Physics runs at a configurable fixed timestep (default 1/60s). The application accumulates frame time and steps the simulation in fixed increments. When paused, `stepOnce` advances exactly one step.

## Creating a New Scene

1. Create header and source files in `src/scenes/`
2. Inherit from `Scene`, implement all pure virtuals
3. Register in `Application::startup()`
4. Add to `CMakeLists.txt` source list
5. Add tests in `tests/test_application.cpp`
6. The scene appears automatically in the ImGui scene list

## Test Suite

17 tests covering:

| Suite | Tests |
|-------|-------|
| Application | Default state validation |
| SceneManager | Add, select, next/prev, reset |
| BasicDropScene | Physics update, determinism |
| BoxStackScene | Body creation |
| CollisionScene | Sphere physics |
| StressScene | Body count, performance metrics |
| Input | Default state, key press/release lifecycle |
| Camera | Default view, pan, zoom, reset, screen-to-world |
