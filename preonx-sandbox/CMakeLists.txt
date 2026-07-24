# ── PreonX Sandbox ────────────────────────────────────────────────────────────
# Interactive development, debugging, and visualization environment for PreonX.

cmake_minimum_required(VERSION 3.24)

project(PreonXSandbox
    VERSION 0.3.0
    DESCRIPTION "Interactive physics visualization for the PreonX engine"
    LANGUAGES C CXX
)

# ── Standards ─────────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

# ── Compiler warnings ─────────────────────────────────────────────────────────
set(SANDBOX_WARNINGS
    -Wall -Wextra -Wpedantic
    -Wnon-virtual-dtor -Wold-style-cast
    -Wcast-align -Wunused -Woverloaded-virtual
    -Wconversion -Wsign-conversion -Wnull-dereference
    -Wdouble-promotion -Wformat=2 -Wimplicit-fallthrough
    -Wmisleading-indentation
)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    list(APPEND SANDBOX_WARNINGS -Wmost -Wconditional-uninitialized)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    list(APPEND SANDBOX_WARNINGS -Wduplicated-cond -Wduplicated-branches -Wlogical-op)
endif()
add_compile_options(${SANDBOX_WARNINGS})

# ── Locate PreonX engine ─────────────────────────────────────────────────────
set(PREONX_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.."
    CACHE PATH "Path to the PreonX engine root")

if(TARGET PreonX::Foundation AND TARGET PreonX::Primeon)
    message(STATUS "PreonX targets found in current build tree")
else()
    if(NOT EXISTS "${PREONX_SOURCE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR
            "PreonX engine not found at: ${PREONX_SOURCE_DIR}\n"
            "Set PREONX_SOURCE_DIR to the PreonX root directory.")
    endif()
    set(PREONX_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(PREONX_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
    set(PREONX_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(PREONX_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    add_subdirectory("${PREONX_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/preonx_engine")
endif()

# ── SDL2 ──────────────────────────────────────────────────────────────────────
find_package(SDL2 REQUIRED)

# ── OpenGL ────────────────────────────────────────────────────────────────────
find_package(OpenGL REQUIRED)

# ── Dear ImGui via FetchContent ───────────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.8
)
FetchContent_MakeAvailable(imgui)

set(IMGUI_DIR ${imgui_SOURCE_DIR})

add_library(imgui_lib STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui_lib PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)

target_link_libraries(imgui_lib PUBLIC
    SDL2::SDL2
    OpenGL::GL
)



# ── Sandbox library (testable, no main) ──────────────────────────────────────
add_library(preonx_sandbox_lib STATIC
    src/window/window.cpp
    src/input/input.cpp
    src/rendering/camera.cpp
    src/rendering/debug_renderer.cpp
    src/scenes/scene_manager.cpp
    src/scenes/basic_drop_scene.cpp
    src/scenes/box_stack_scene.cpp
    src/scenes/collision_scene.cpp
    src/scenes/stress_scene.cpp
    src/scenes/crash_scene.cpp
    src/ui/debug_ui.cpp
    src/application.cpp
)

target_include_directories(preonx_sandbox_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/core
)

target_compile_definitions(preonx_sandbox_lib
    PUBLIC
        GL_GLEXT_PROTOTYPES
)

target_link_libraries(preonx_sandbox_lib
    PUBLIC
        PreonX::Primeon
        PreonX::Foundation
        SDL2::SDL2
        OpenGL::GL
        imgui_lib
)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(preonx_sandbox src/main.cpp)
target_link_libraries(preonx_sandbox PRIVATE preonx_sandbox_lib)

# ── Tests ─────────────────────────────────────────────────────────────────────
option(SANDBOX_BUILD_TESTS "Build sandbox tests" ON)

if(SANDBOX_BUILD_TESTS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(BUILD_GTEST ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(sandbox_tests tests/test_application.cpp)
    target_link_libraries(sandbox_tests
        PRIVATE
            preonx_sandbox_lib
            GTest::gtest_main
    )
    target_include_directories(sandbox_tests
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
            ${CMAKE_CURRENT_SOURCE_DIR}/core
    )
    include(GoogleTest)
    gtest_discover_tests(sandbox_tests)
endif()
