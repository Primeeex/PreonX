#include "foundation/core/assert.hpp"

#include <cstdio>
#include <cstdlib>

namespace foundation::detail {

void assert_fail(const char* expression, const char* file, int line, const char* function) {
    std::fprintf(stderr, "\n=== ASSERTION FAILED ===\n");
    std::fprintf(stderr, "  Expression: %s\n", expression);
    std::fprintf(stderr, "  File:       %s\n", file);
    std::fprintf(stderr, "  Line:       %d\n", line);
    std::fprintf(stderr, "  Function:   %s\n", function);
    std::fprintf(stderr, "========================\n\n");
    std::fflush(stderr);
    std::abort();
}

void assert_fail(const char* expression, const char* file, int line, const char* function, const char* message) {
    std::fprintf(stderr, "\n=== ASSERTION FAILED ===\n");
    std::fprintf(stderr, "  Expression: %s\n", expression);
    std::fprintf(stderr, "  Message:    %s\n", message);
    std::fprintf(stderr, "  File:       %s\n", file);
    std::fprintf(stderr, "  Line:       %d\n", line);
    std::fprintf(stderr, "  Function:   %s\n", function);
    std::fprintf(stderr, "========================\n\n");
    std::fflush(stderr);
    std::abort();
}

} // namespace foundation::detail
