#include "foundation/platform/platform.hpp"

#include <thread>

namespace foundation {

const char* platform_name(Platform platform) noexcept {
    switch (platform) {
    case Platform::Linux:
        return "Linux";
    case Platform::Windows:
        return "Windows";
    case Platform::macOS:
        return "macOS";
    case Platform::Unknown:
        return "Unknown";
    }
    return "Unknown";
}

const char* arch_name(Arch arch) noexcept {
    switch (arch) {
    case Arch::X86_64:
        return "x86_64";
    case Arch::ARM64:
        return "ARM64";
    case Arch::X86:
        return "x86";
    case Arch::Unknown:
        return "Unknown";
    }
    return "Unknown";
}

const char* compiler_name(Compiler compiler) noexcept {
    switch (compiler) {
    case Compiler::Clang:
        return "Clang";
    case Compiler::GCC:
        return "GCC";
    case Compiler::MSVC:
        return "MSVC";
    case Compiler::Unknown:
        return "Unknown";
    }
    return "Unknown";
}

CpuInfo detect_cpu_info() noexcept {
    CpuInfo info{};
    info.core_count = std::thread::hardware_concurrency();
    info.thread_count = info.core_count;

#if defined(PREONX_ARCH_X86_64) || defined(PREONX_ARCH_X86)
    info.has_sse4_2 = true;  // Assume SSE4.2 for x86_64
    #if defined(__AVX2__)
    info.has_avx2 = true;
    #else
    info.has_avx2 = false;
    #endif
    #if defined(__AVX512F__)
    info.has_avx512 = true;
    #else
    info.has_avx512 = false;
    #endif
    info.has_neon = false;
#elif defined(PREONX_ARCH_ARM64)
    info.has_sse4_2 = false;
    info.has_avx2 = false;
    info.has_avx512 = false;
    info.has_neon = true;
#else
    info.has_sse4_2 = false;
    info.has_avx2 = false;
    info.has_avx512 = false;
    info.has_neon = false;
#endif

    return info;
}

} // namespace foundation
