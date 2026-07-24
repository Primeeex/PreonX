cmake_minimum_required(VERSION 3.24)

# ── Fetch Google Benchmark ────────────────────────────────────────────────────
include(FetchContent)

FetchContent_Declare(
    benchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG        v1.8.3
)

set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
set(BENCHMARK_DOWNLOAD_DEPENDENCIES ON CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_WERROR OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(benchmark)

if(PREONX_WARNINGS_AS_ERRORS)
    target_compile_options(benchmark PRIVATE -Wno-error)
endif()

# ── Benchmark sources ────────────────────────────────────────────────────────

add_executable(foundation_benchmarks
    micro/allocator_benchmark.cpp
    micro/array_benchmark.cpp
    micro/hash_benchmark.cpp
)

target_link_libraries(foundation_benchmarks
    PRIVATE
        foundation
        benchmark::benchmark
        benchmark::benchmark_main
)

target_include_directories(foundation_benchmarks
    PRIVATE
        ${PREONX_SOURCE_DIR}
)

set_target_properties(foundation_benchmarks PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# ── Cambyses benchmarks ──────────────────────────────────────────────────────

add_executable(cambyses_benchmarks
    micro/cambyses_benchmark.cpp
)

target_link_libraries(cambyses_benchmarks
    PRIVATE
        cambyses
        foundation
        benchmark::benchmark
        benchmark::benchmark_main
)

target_include_directories(cambyses_benchmarks
    PRIVATE
        ${PREONX_SOURCE_DIR}
)

set_target_properties(cambyses_benchmarks PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# ── Primeon benchmarks ────────────────────────────────────────────────────────

add_executable(primeon_benchmarks
    micro/primeon_benchmark.cpp
)

target_link_libraries(primeon_benchmarks
    PRIVATE
        primeon
        foundation
        benchmark::benchmark
        benchmark::benchmark_main
)

target_include_directories(primeon_benchmarks
    PRIVATE
        ${PREONX_SOURCE_DIR}
)

set_target_properties(primeon_benchmarks PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# ── Collision benchmarks ────────────────────────────────────────────────────

add_executable(collision_benchmarks
    micro/collision_benchmark.cpp
)

target_link_libraries(collision_benchmarks
    PRIVATE
        primeon
        foundation
        benchmark::benchmark
        benchmark::benchmark_main
)

target_include_directories(collision_benchmarks
    PRIVATE
        ${PREONX_SOURCE_DIR}
)

set_target_properties(collision_benchmarks PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# ── Solver benchmarks ──────────────────────────────────────────────────────

add_executable(solver_benchmarks
    micro/solver_benchmark.cpp
)

target_link_libraries(solver_benchmarks
    PRIVATE
        primeon
        foundation
        benchmark::benchmark
        benchmark::benchmark_main
)

target_include_directories(solver_benchmarks
    PRIVATE
        ${PREONX_SOURCE_DIR}
)

set_target_properties(solver_benchmarks PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# ── World benchmarks ────────────────────────────────────────────────────────

add_executable(world_benchmarks
    micro/world_benchmark.cpp
)

target_link_libraries(world_benchmarks
    PRIVATE
        primeon
        foundation
        benchmark::benchmark
        benchmark::benchmark_main
)

target_include_directories(world_benchmarks
    PRIVATE
        ${PREONX_SOURCE_DIR}
)

set_target_properties(world_benchmarks PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)
