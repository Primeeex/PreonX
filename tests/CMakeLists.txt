cmake_minimum_required(VERSION 3.24)

# ── Fetch GoogleTest ───────────────────────────────────────────────────────────
include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)

# Prevent overriding the parent project's compiler/linker settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(BUILD_GTEST ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

# Suppress warnings-as-errors for third-party code
if(PREONX_WARNINGS_AS_ERRORS)
    target_compile_options(gtest PRIVATE -Wno-error)
endif()

# ── Test sources ──────────────────────────────────────────────────────────────

add_executable(foundation_tests
    unit/core/test_assert.cpp
    unit/core/test_version.cpp
    unit/core/test_types.cpp
    unit/error/test_error_code.cpp
    unit/error/test_error.cpp
    unit/error/test_result.cpp
    unit/logging/test_logger.cpp
    unit/memory/test_allocator.cpp
    unit/containers/test_dynamic_array.cpp
    unit/time/test_clock.cpp
    unit/hashing/test_hash.cpp
    unit/filesystem/test_path.cpp
    unit/filesystem/test_file_system.cpp
    unit/threading/test_mutex.cpp
    unit/threading/test_thread.cpp
    unit/platform/test_platform.cpp
)

target_link_libraries(foundation_tests
    PRIVATE
        foundation
        GTest::gtest_main
)

target_include_directories(foundation_tests
    PRIVATE
        ${PREONX_SOURCE_DIR}
)

set_target_properties(foundation_tests PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

include(GoogleTest)
gtest_discover_tests(foundation_tests)

# ── Cambyses tests ────────────────────────────────────────────────────────────

add_executable(cambyses_tests
    unit/cambyses/test_entity.cpp
    unit/cambyses/test_component.cpp
    unit/cambyses/test_archetype.cpp
    unit/cambyses/test_query.cpp
    unit/cambyses/test_system.cpp
    unit/cambyses/test_event.cpp
    unit/cambyses/test_world.cpp
)

target_link_libraries(cambyses_tests
    PRIVATE
        cambyses
        foundation
        GTest::gtest_main
)

target_include_directories(cambyses_tests
    PRIVATE
        ${PREONX_SOURCE_DIR}
)

set_target_properties(cambyses_tests PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

gtest_discover_tests(cambyses_tests)

# ── Primeon tests ─────────────────────────────────────────────────────────────

add_executable(primeon_tests
    unit/primeon/test_scalar.cpp
    unit/primeon/test_vector2.cpp
    unit/primeon/test_vector3.cpp
    unit/primeon/test_vector4.cpp
    unit/primeon/test_matrix2.cpp
    unit/primeon/test_matrix3.cpp
    unit/primeon/test_matrix4.cpp
    unit/primeon/test_quaternion.cpp
    unit/primeon/test_transform.cpp
    unit/primeon/test_geometry.cpp
    unit/primeon/test_distance.cpp
    unit/primeon/test_intersection.cpp
    unit/primeon/test_mass.cpp
    unit/primeon/test_forces.cpp
    unit/primeon/test_motion.cpp
    unit/primeon/test_energy.cpp
    unit/primeon/test_integrators.cpp
    unit/primeon/test_simulation.cpp
    unit/primeon/test_collision.cpp
    unit/primeon/test_manifold.cpp
    unit/primeon/test_solver.cpp
    unit/primeon/test_world.cpp
)

target_link_libraries(primeon_tests
    PRIVATE
        primeon
        foundation
        GTest::gtest_main
)

target_include_directories(primeon_tests
    PRIVATE
        ${PREONX_SOURCE_DIR}
)

set_target_properties(primeon_tests PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

gtest_discover_tests(primeon_tests)
