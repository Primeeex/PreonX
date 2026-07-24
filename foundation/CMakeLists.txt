cmake_minimum_required(VERSION 3.24)

# ── Foundation library ─────────────────────────────────────────────────────────

set(FOUNDATION_SOURCES
    src/foundation/core/assert.cpp
    src/foundation/error/error_code.cpp
    src/foundation/logging/logger.cpp
    src/foundation/logging/console_sink.cpp
    src/foundation/logging/log_level.cpp
    src/foundation/memory/allocator.cpp
    src/foundation/filesystem/path.cpp
    src/foundation/platform/platform.cpp
    src/foundation/threading/thread.cpp
)

add_library(foundation STATIC ${FOUNDATION_SOURCES})

target_include_directories(foundation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PREONX_SOURCE_DIR}>
        $<BUILD_INTERFACE:${PREONX_BINARY_DIR}/generated>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(foundation PUBLIC cxx_std_20)

set_target_properties(foundation PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# Threading support
find_package(Threads REQUIRED)
target_link_libraries(foundation PUBLIC Threads::Threads)

# Alias for use in other parts of the project
add_library(PreonX::Foundation ALIAS foundation)
