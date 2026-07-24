#pragma once

#include "foundation/core/platform.hpp"
#include "foundation/core/types.hpp"

namespace foundation {

enum class Platform : u8 {
    Linux,
    Windows,
    macOS,
    Unknown,
};

enum class Arch : u8 {
    X86_64,
    ARM64,
    X86,
    Unknown,
};

enum class Compiler : u8 {
    Clang,
    GCC,
    MSVC,
    Unknown,
};

[[nodiscard]] constexpr Platform current_platform() noexcept {
#if defined(PREONX_PLATFORM_LINUX)
    return Platform::Linux;
#elif defined(PREONX_PLATFORM_WINDOWS)
    return Platform::Windows;
#elif defined(PREONX_PLATFORM_MACOS)
    return Platform::macOS;
#else
    return Platform::Unknown;
#endif
}

[[nodiscard]] constexpr Arch current_arch() noexcept {
#if defined(PREONX_ARCH_X86_64)
    return Arch::X86_64;
#elif defined(PREONX_ARCH_ARM64)
    return Arch::ARM64;
#elif defined(PREONX_ARCH_X86)
    return Arch::X86;
#else
    return Arch::Unknown;
#endif
}

[[nodiscard]] constexpr Compiler current_compiler() noexcept {
#if defined(PREONX_COMPILER_CLANG)
    return Compiler::Clang;
#elif defined(PREONX_COMPILER_GCC)
    return Compiler::GCC;
#elif defined(PREONX_COMPILER_MSVC)
    return Compiler::MSVC;
#else
    return Compiler::Unknown;
#endif
}

[[nodiscard]] const char* platform_name(Platform platform) noexcept;
[[nodiscard]] const char* arch_name(Arch arch) noexcept;
[[nodiscard]] const char* compiler_name(Compiler compiler) noexcept;

struct CpuInfo {
    u32 core_count;
    u32 thread_count;
    bool has_sse4_2;
    bool has_avx2;
    bool has_avx512;
    bool has_neon;
};

[[nodiscard]] CpuInfo detect_cpu_info() noexcept;

} // namespace foundation
