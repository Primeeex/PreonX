#pragma once

#include "foundation/core/platform.hpp"

namespace foundation {

struct Version {
    int major;
    int minor;
    int patch;

    [[nodiscard]] constexpr bool operator==(const Version& other) const noexcept {
        return major == other.major && minor == other.minor && patch == other.patch;
    }

    [[nodiscard]] constexpr bool operator!=(const Version& other) const noexcept {
        return !(*this == other);
    }

    [[nodiscard]] constexpr bool operator<(const Version& other) const noexcept {
        if (major != other.major) return major < other.major;
        if (minor != other.minor) return minor < other.minor;
        return patch < other.patch;
    }

    [[nodiscard]] constexpr bool operator<=(const Version& other) const noexcept {
        return *this == other || *this < other;
    }

    [[nodiscard]] constexpr bool operator>(const Version& other) const noexcept {
        return !(*this <= other);
    }

    [[nodiscard]] constexpr bool operator>=(const Version& other) const noexcept {
        return !(*this < other);
    }
};

[[nodiscard]] constexpr Version library_version() noexcept {
    return {PREONX_VERSION_MAJOR, PREONX_VERSION_MINOR, PREONX_VERSION_PATCH};
}

[[nodiscard]] constexpr const char* library_version_string() noexcept {
    return PREONX_VERSION;
}

[[nodiscard]] constexpr const char* build_configuration() noexcept {
#ifdef NDEBUG
    return "Release";
#else
    return "Debug";
#endif
}

[[nodiscard]] constexpr const char* compiler_id() noexcept {
#if defined(__clang__)
    return "Clang " __clang_VERSION__;
#elif defined(__GNUC__)
    return "GCC " __VERSION__;
#elif defined(_MSC_VER)
    return "MSVC " _MSC_VER_STR
#else
    return "Unknown";
#endif
}

[[nodiscard]] constexpr const char* platform_name() noexcept {
#if defined(__linux__)
    return "Linux";
#elif defined(_WIN32)
    return "Windows";
#elif defined(__APPLE__)
    return "macOS";
#else
    return "Unknown";
#endif
}

[[nodiscard]] constexpr const char* architecture_name() noexcept {
#if defined(__x86_64__) || defined(_M_X64)
    return "x86_64";
#elif defined(__aarch64__) || defined(_M_ARM64)
    return "ARM64";
#elif defined(__i386__) || defined(_M_IX86)
    return "x86";
#else
    return "Unknown";
#endif
}

struct BuildInfo {
    Version version;
    const char* configuration;
    const char* compiler;
    const char* platform;
    const char* architecture;
};

[[nodiscard]] inline BuildInfo build_info() noexcept {
    return {library_version(), build_configuration(), compiler_id(), platform_name(), architecture_name()};
}

} // namespace foundation
