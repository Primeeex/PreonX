#pragma once

// ── Compiler detection ────────────────────────────────────────────────────────
#if defined(__clang__)
    #define PREONX_COMPILER_CLANG 1
    #define PREONX_COMPILER_NAME "Clang"
    #define PREONX_COMPILER_VERSION (__clang_major__ * 10000 + __clang_minor__ * 100 + __clang_patchlevel__)
#elif defined(__GNUC__)
    #define PREONX_COMPILER_GCC 1
    #define PREONX_COMPILER_NAME "GCC"
    #define PREONX_COMPILER_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#elif defined(_MSC_VER)
    #define PREONX_COMPILER_MSVC 1
    #define PREONX_COMPILER_NAME "MSVC"
    #define PREONX_COMPILER_VERSION _MSC_VER
#else
    #error "Unsupported compiler"
#endif

// ── Platform detection ────────────────────────────────────────────────────────
#if defined(__linux__)
    #define PREONX_PLATFORM_LINUX 1
    #define PREONX_PLATFORM_NAME "Linux"
#elif defined(_WIN32)
    #define PREONX_PLATFORM_WINDOWS 1
    #define PREONX_PLATFORM_NAME "Windows"
#elif defined(__APPLE__)
    #define PREONX_PLATFORM_MACOS 1
    #define PREONX_PLATFORM_NAME "macOS"
#else
    #error "Unsupported platform"
#endif

// ── Architecture detection ────────────────────────────────────────────────────
#if defined(__x86_64__) || defined(_M_X64)
    #define PREONX_ARCH_X86_64 1
    #define PREONX_ARCH_NAME "x86_64"
#elif defined(__aarch64__) || defined(_M_ARM64)
    #define PREONX_ARCH_ARM64 1
    #define PREONX_ARCH_NAME "ARM64"
#elif defined(__i386__) || defined(_M_IX86)
    #define PREONX_ARCH_X86 1
    #define PREONX_ARCH_NAME "x86"
#else
    #define PREONX_ARCH_UNKNOWN 1
    #define PREONX_ARCH_NAME "Unknown"
#endif

// ── Version macros ────────────────────────────────────────────────────────────
#ifndef PREONX_VERSION_MAJOR
    #define PREONX_VERSION_MAJOR 0
#endif
#ifndef PREONX_VERSION_MINOR
    #define PREONX_VERSION_MINOR 1
#endif
#ifndef PREONX_VERSION_PATCH
    #define PREONX_VERSION_PATCH 0
#endif
#ifndef PREONX_VERSION
    #define PREONX_VERSION "0.1.0"
#endif
#ifndef PREONX_VERSION_STRING
    #define PREONX_VERSION_STRING "0.1.0"
#endif

// ── Utility macros ────────────────────────────────────────────────────────────
#define PREONX_UNUSED(x) ((void)(x))

#if defined(__cplusplus) && __cplusplus >= 202002L
    #define PREONX_CONSTEXPR20 constexpr
#else
    #define PREONX_CONSTEXPR20
#endif

// Force inline
#if defined(PREONX_COMPILER_MSVC)
    #define PREONX_FORCE_INLINE __forceinline
#else
    #define PREONX_FORCE_INLINE inline __attribute__((always_inline))
#endif

// Likely/unlikely
#if defined(__cplusplus) && __cplusplus >= 202002L
    #define PREONX_LIKELY [[likely]]
    #define PREONX_UNLIKELY [[unlikely]]
#else
    #if defined(__GNUC__) || defined(__clang__)
        #define PREONX_LIKELY   __builtin_expect(!!(1), 1)
        #define PREONX_UNLIKELY __builtin_expect(!!(1), 0)
    #else
        #define PREONX_LIKELY
        #define PREONX_UNLIKELY
    #endif
#endif

// Deprecation
#if defined(PREONX_COMPILER_MSVC)
    #define PREONX_DEPRECATED(msg) __declspec(deprecated(msg))
#elif defined(__GNUC__) || defined(__clang__)
    #define PREONX_DEPRECATED(msg) __attribute__((deprecated(msg)))
#else
    #define PREONX_DEPRECATED(msg)
#endif
