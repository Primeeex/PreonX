cmake_minimum_required(VERSION 3.24)

# ── Cambyses ECS library ──────────────────────────────────────────────────────

set(CAMBYSSES_SOURCES
    src/cambyses/component/component_registry.cpp
    src/cambyses/archetype/column.cpp
    src/cambyses/archetype/archetype.cpp
    src/cambyses/entity/entity_manager.cpp
    src/cambyses/serialization/serialization.cpp
    src/cambyses/world.cpp
)

add_library(cambyses STATIC ${CAMBYSSES_SOURCES})

target_include_directories(cambyses
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PREONX_SOURCE_DIR}>
        $<BUILD_INTERFACE:${PREONX_BINARY_DIR}/generated>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(cambyses
    PUBLIC
        PreonX::Foundation
)

target_compile_features(cambyses PUBLIC cxx_std_20)

set_target_properties(cambyses PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

add_library(PreonX::Cambyses ALIAS cambyses)
