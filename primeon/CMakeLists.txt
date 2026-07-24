cmake_minimum_required(VERSION 3.24)

# ── Primeon library (header-only math and geometry) ───────────────────────────
add_library(primeon INTERFACE)

target_include_directories(primeon
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PREONX_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(primeon INTERFACE foundation)

target_compile_features(primeon INTERFACE cxx_std_20)

# Alias for use in other parts of the project
add_library(PreonX::Primeon ALIAS primeon)
