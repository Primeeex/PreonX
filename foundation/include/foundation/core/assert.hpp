#pragma once

#include "foundation/core/platform.hpp"

#include <cstdio>
#include <cstdlib>

// ── Assertion macros ──────────────────────────────────────────────────────────

#if defined(NDEBUG)
    #define PREONX_ASSERT(expr) ((void)(expr))
    #define PREONX_ASSERT_MSG(expr, msg) ((void)(expr))
    #define PREONX_UNREACHABLE() ((void)0)
    #define PREONX_UNIMPLEMENTED() ((void)0)
#else
    #define PREONX_ASSERT(expr)                                                                             \
        do {                                                                                                \
            if (!(expr)) {                                                                                  \
                ::foundation::detail::assert_fail(#expr, __FILE__, __LINE__, __FUNCTION__);                 \
            }                                                                                               \
        } while (0)

    #define PREONX_ASSERT_MSG(expr, msg)                                                                    \
        do {                                                                                                \
            if (!(expr)) {                                                                                  \
                ::foundation::detail::assert_fail(#expr, __FILE__, __LINE__, __FUNCTION__, (msg));           \
            }                                                                                               \
        } while (0)

    #define PREONX_UNREACHABLE()                                                                            \
        do {                                                                                                \
            ::foundation::detail::assert_fail("Unreachable code reached", __FILE__, __LINE__,               \
                                              __FUNCTION__);                                                \
        } while (0)

    #define PREONX_UNIMPLEMENTED()                                                                          \
        do {                                                                                                \
            ::foundation::detail::assert_fail("Not yet implemented", __FILE__, __LINE__,                    \
                                              __FUNCTION__);                                                \
        } while (0)
#endif

// ── Debug-only checks ─────────────────────────────────────────────────────────
#if !defined(NDEBUG)
    #define PREONX_DEBUG_ONLY(code) code
#else
    #define PREONX_DEBUG_ONLY(code)
#endif

namespace foundation::detail {

[[noreturn]] void assert_fail(const char* expression, const char* file, int line, const char* function);
[[noreturn]] void assert_fail(const char* expression, const char* file, int line, const char* function,
                              const char* message);

} // namespace foundation::detail
