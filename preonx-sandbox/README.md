# PreonX Sandbox

Interactive physics visualization environment for the PreonX game engine ecosystem.

## Purpose

- **Development environment** for testing new engine features
- **Debugging tool** for visualizing physics, collision, and solver behavior
- **Profiling platform** for measuring engine performance
- **Reference implementation** demonstrating proper engine usage

Consumes PreonX through public APIs only. Never accesses engine internals.

## Dependencies

- **PreonX Engine** (`../`) — Foundation runtime and Primeon physics kernel
- **CMake 3.24+**
- **C++20 compiler** (GCC 14+ or Clang 17+)
- **SDL2** — windowing and input
- **OpenGL 3.3+** — debug rendering
- **Dear ImGui** (fetched automatically via CMake FetchContent)
- **GoogleTest** (fetched automatically for tests)

### Installing SDL2

```bash
# Fedora
sudo dnf install SDL2-devel mesa-libGL-devel

# Ubuntu/Debian
sudo apt install libsdl2-dev libgl-dev

# macOS (Homebrew)
brew install sdl2
```

## Building

```bash
cd preonx-sandbox
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
```

### Options

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release          # Optimized build
cmake -B build -DSANDBOX_BUILD_TESTS=OFF            # Skip tests
cmake -B build -DPREONX_SOURCE_DIR=/path/to/engine  # Custom engine path
```

## Running

```bash
./build/preonx_sandbox                   # Default scene (Basic Drop)
./build/preonx_sandbox --scene 0         # Basic Drop
./build/preonx_sandbox --scene 1         # Box Stack
./build/preonx_sandbox --scene 2         # Collision
./build/preonx_sandbox --scene 3         # Stress Test (768 bodies)
./build/preonx_sandbox --help            # Show all options
```

## Controls

| Key | Action |
|-----|--------|
| **R** | Reset current scene |
| **Space** | Pause / unpause simulation |
| **N** | Step one frame (when paused) |
| **Left/Right Arrow** | Switch scene |
| **Home** | Reset camera to origin |
| **Mouse Middle-Drag** | Pan camera |
| **Mouse Scroll** | Zoom in/out |
| **ESC** | Quit |

The ImGui panel provides interactive controls for pause, reset, gravity, simulation speed, and scene selection.

## Directory Layout

```text
preonx-sandbox/
├── CMakeLists.txt              Build system (SDL2, OpenGL, ImGui, GTest)
├── README.md                   This file
├── core/
│   └── application_state.hpp   Global state struct
├── src/
│   ├── main.cpp                Entry point
│   ├── application.hpp/cpp     Main loop orchestrator
│   ├── window/
│   │   └── window.hpp/cpp      SDL2 window + OpenGL 3.3 core context
│   ├── input/
│   │   └── input.hpp/cpp       Keyboard, mouse, wheel input
│   ├── rendering/
│   │   ├── opengl.hpp          Minimal GL 3.3 core wrapper
│   │   ├── camera.hpp/cpp      2D orthographic camera
│   │   └── debug_renderer.hpp/cpp  Batched line/triangle renderer
│   ├── scenes/
│   │   ├── scene.hpp           Scene base class
│   │   ├── scene_manager.hpp/cpp   Scene lifecycle management
│   │   ├── basic_drop_scene.*  Falling box under gravity
│   │   ├── box_stack_scene.*   Stacked boxes (solver stability)
│   │   ├── collision_scene.*   Multi-body collision visualization
│   │   └── stress_scene.*      768-body performance test
│   └── ui/
│       └── debug_ui.hpp/cpp    ImGui debug panel
├── tests/
│   └── test_application.cpp    Unit tests (17 tests)
└── docs/
    └── sandbox.md              Detailed sandbox documentation
```

## Architecture

```
preonx-sandbox
    ├── Application (main loop)
    │   ├── Window (SDL2 + OpenGL)
    │   ├── Input (keyboard, mouse)
    │   ├── Camera (2D orthographic)
    │   ├── DebugRenderer (GL_LINES + GL_TRIANGLES batching)
    │   ├── DebugUI (Dear ImGui panel)
    │   └── SceneManager
    │       ├── BasicDropScene
    │       ├── BoxStackScene
    │       ├── CollisionScene
    │       └── StressScene
    │
    └── PreonX (external dependency)
        ├── Foundation (types, containers, logging)
        └── Primeon (physics, collision, math)
```

All engine interaction goes through public API headers only:
- `<primeon/world/physics_world.hpp>`
- `<primeon/math/scalar/scalar.hpp>`
- `<primeon/math/vector/vector3.hpp>`

## Creating New Scenes

1. Create `src/scenes/my_scene.hpp` and `src/scenes/my_scene.cpp`
2. Inherit from `Scene`, implement virtual methods:

```cpp
class MyScene : public Scene {
public:
    void init() override;           // Create bodies, configure world
    void cleanup() override;        // Reset world
    void update(float dt, bool paused, bool stepOnce) override;
    void render(DebugRenderer& renderer, const Camera& camera) override;
    void handleInput(const Input& input) override;
    const char* name() const override { return "My Scene"; }
    const char* description() const override { return "Description"; }
};
```

3. Register in `application.cpp`:

```cpp
sceneManager_.addScene(std::make_unique<MyScene>());
```

4. Add to `CMakeLists.txt` source list and `tests/test_application.cpp`

## License

MIT License. See [LICENSE](LICENSE) for details.
