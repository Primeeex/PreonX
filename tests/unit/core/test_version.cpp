#include <gtest/gtest.h>

#include "foundation/core/version.hpp"

namespace {

TEST(VersionTest, LibraryVersion) {
    foundation::Version v = foundation::library_version();
    EXPECT_EQ(v.major, PREONX_VERSION_MAJOR);
    EXPECT_EQ(v.minor, PREONX_VERSION_MINOR);
    EXPECT_EQ(v.patch, PREONX_VERSION_PATCH);
}

TEST(VersionTest, VersionString) {
    EXPECT_STREQ(foundation::library_version_string(), PREONX_VERSION);
}

TEST(VersionTest, BuildConfiguration) {
    const char* config = foundation::build_configuration();
    EXPECT_NE(config, nullptr);
    EXPECT_TRUE(config == std::string("Debug") || config == std::string("Release"));
}

TEST(VersionTest, CompilerId) {
    const char* compiler = foundation::compiler_id();
    EXPECT_NE(compiler, nullptr);
    EXPECT_TRUE(compiler[0] != '\0');
}

TEST(VersionTest, PlatformName) {
    const char* platform = foundation::platform_name();
    EXPECT_NE(platform, nullptr);
    EXPECT_TRUE(platform[0] != '\0');
}

TEST(VersionTest, ArchitectureName) {
    const char* arch = foundation::architecture_name();
    EXPECT_NE(arch, nullptr);
    EXPECT_TRUE(arch[0] != '\0');
}

TEST(VersionTest, BuildInfo) {
    foundation::BuildInfo info = foundation::build_info();
    EXPECT_EQ(info.version, foundation::library_version());
    EXPECT_NE(info.configuration, nullptr);
    EXPECT_NE(info.compiler, nullptr);
    EXPECT_NE(info.platform, nullptr);
    EXPECT_NE(info.architecture, nullptr);
}

TEST(VersionTest, ComparisonOperators) {
    foundation::Version a{1, 0, 0};
    foundation::Version b{1, 0, 0};
    foundation::Version c{1, 1, 0};
    foundation::Version d{2, 0, 0};

    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a != b);
    EXPECT_TRUE(a < c);
    EXPECT_TRUE(a <= b);
    EXPECT_FALSE(a > b);
    EXPECT_TRUE(d >= a);
    EXPECT_TRUE(c > a);
}

} // namespace
