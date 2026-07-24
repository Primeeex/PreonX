cmake_minimum_required(VERSION 3.24)

# ── Version ────────────────────────────────────────────────────────────────────
project(PreonX
    VERSION 0.1.0
    DESCRIPTION "A modular, high-performance game engine ecosystem"
    HOMEPAGE_URL "https://github.com/Primeeex/PreonX"
    LANGUAGES CXX
)

# ── Standard project settings ─────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Project-root variables (safe when used via add_subdirectory)
set(PREONX_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(PREONX_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# Default to Release if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

# ── Project options ────────────────────────────────────────────────────────────
option(PREONX_BUILD_TESTS      "Build unit and integration tests"     ON)
option(PREONX_BUILD_BENCHMARKS "Build benchmark suites"               OFF)
option(PREONX_BUILD_EXAMPLES   "Build example projects"               ON)
option(PREONX_BUILD_TOOLS      "Build developer tools"                OFF)
option(PREONX_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(PREONX_ENABLE_SANITIZERS  "Enable address and UB sanitizers"  OFF)

# ── Compiler warning configuration ────────────────────────────────────────────
set(PREONX_COMMON_WARNINGS
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wconversion
    -Wsign-conversion
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
    -Wimplicit-fallthrough
    -Wmisleading-indentation
)

set(PREONX_CLANG_WARNINGS
    -Wmost
    -Wconditional-uninitialized
    -Wshadow-all
    -Wdeprecated-volatile
    -Wbitwise-instead-of-logical
    -Wsingle-bit-bitfield-constant-conversion
)

set(PREONX_GCC_WARNINGS
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wuseless-cast
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(${PREONX_COMMON_WARNINGS} ${PREONX_CLANG_WARNINGS})
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(${PREONX_COMMON_WARNINGS} ${PREONX_GCC_WARNINGS})
endif()

if(PREONX_WARNINGS_AS_ERRORS)
    add_compile_options(-Werror)
endif()

# ── Sanitizers ─────────────────────────────────────────────────────────────────
if(PREONX_ENABLE_SANITIZERS)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

# ── Version header generation ─────────────────────────────────────────────────
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.hpp.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/version.hpp"
    @ONLY
)

# ── Include generated headers ─────────────────────────────────────────────────
include_directories("${CMAKE_CURRENT_BINARY_DIR}/generated")

# ── Global include directories ────────────────────────────────────────────────
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

# ── Subdirectories ─────────────────────────────────────────────────────────────
# Each subsystem is added as a subdirectory. In Stage 1, these directories
# contain only placeholder content. Actual library targets will be added
# in subsequent stages.

# Core subsystems (libraries will be added in later stages)
add_subdirectory(foundation)
add_subdirectory(primeon)
add_subdirectory(luminium)
add_subdirectory(cambyses)
add_subdirectory(cochise)
add_subdirectory(jubal)
add_subdirectory(integration)
add_subdirectory(sdk)

# Developer tools (Stage 9)
if(PREONX_BUILD_TOOLS)
    add_subdirectory(tools)
endif()

# Examples
if(PREONX_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Tests
if(PREONX_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Benchmarks
if(PREONX_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

# ── Export compile commands ────────────────────────────────────────────────────
# Useful for IDE integration and clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

message(STATUS "")
message(STATUS "PreonX ${PROJECT_VERSION} configuration:")
message(STATUS "  Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "  Compiler:          ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  C++ Standard:      ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build tests:       ${PREONX_BUILD_TESTS}")
message(STATUS "  Build benchmarks:  ${PREONX_BUILD_BENCHMARKS}")
message(STATUS "  Build examples:    ${PREONX_BUILD_EXAMPLES}")
message(STATUS "  Build tools:       ${PREONX_BUILD_TOOLS}")
message(STATUS "  Warnings as errors:${PREONX_WARNINGS_AS_ERRORS}")
message(STATUS "  Sanitizers:        ${PREONX_ENABLE_SANITIZERS}")
message(STATUS "")
