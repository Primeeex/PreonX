#include <gtest/gtest.h>

#include "foundation/platform/platform.hpp"

namespace {

TEST(PlatformTest, CurrentPlatformIsValid) {
    foundation::Platform p = foundation::current_platform();
    EXPECT_NE(p, foundation::Platform::Unknown);
}

TEST(PlatformTest, CurrentArchIsValid) {
    foundation::Arch a = foundation::current_arch();
    EXPECT_NE(a, foundation::Arch::Unknown);
}

TEST(PlatformTest, CurrentCompilerIsValid) {
    foundation::Compiler c = foundation::current_compiler();
    EXPECT_NE(c, foundation::Compiler::Unknown);
}

TEST(PlatformTest, PlatformNames) {
    EXPECT_NE(foundation::platform_name(foundation::Platform::Linux), nullptr);
    EXPECT_NE(foundation::platform_name(foundation::Platform::Windows), nullptr);
    EXPECT_NE(foundation::platform_name(foundation::Platform::macOS), nullptr);
}

TEST(PlatformTest, ArchNames) {
    EXPECT_NE(foundation::arch_name(foundation::Arch::X86_64), nullptr);
    EXPECT_NE(foundation::arch_name(foundation::Arch::ARM64), nullptr);
    EXPECT_NE(foundation::arch_name(foundation::Arch::X86), nullptr);
}

TEST(PlatformTest, CompilerNames) {
    EXPECT_NE(foundation::compiler_name(foundation::Compiler::Clang), nullptr);
    EXPECT_NE(foundation::compiler_name(foundation::Compiler::GCC), nullptr);
    EXPECT_NE(foundation::compiler_name(foundation::Compiler::MSVC), nullptr);
}

TEST(PlatformTest, DetectCpuInfo) {
    foundation::CpuInfo info = foundation::detect_cpu_info();
    EXPECT_GT(info.core_count, 0u);
    EXPECT_GT(info.thread_count, 0u);
}

} // namespace
