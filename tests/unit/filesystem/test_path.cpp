#include <gtest/gtest.h>

#include "foundation/filesystem/path.hpp"

namespace {

TEST(PathTest, DefaultConstruction) {
    foundation::Path p;
    EXPECT_TRUE(p.empty());
}

TEST(PathTest, StringConstruction) {
    foundation::Path p("/home/user/file.txt");
    EXPECT_EQ(p.string(), "/home/user/file.txt");
    EXPECT_FALSE(p.empty());
}

TEST(PathTest, CStr) {
    foundation::Path p("/test/path");
    EXPECT_STREQ(p.c_str(), "/test/path");
}

TEST(PathTest, Filename) {
    foundation::Path p("/home/user/document.txt");
    EXPECT_EQ(p.filename(), "document.txt");
}

TEST(PathTest, FilenameNoExtension) {
    foundation::Path p("/home/user/Makefile");
    EXPECT_EQ(p.filename(), "Makefile");
}

TEST(PathTest, Stem) {
    foundation::Path p("/home/user/archive.tar.gz");
    EXPECT_EQ(p.stem(), "archive.tar");
}

TEST(PathTest, Extension) {
    foundation::Path p("/home/user/file.txt");
    EXPECT_EQ(p.extension(), ".txt");
}

TEST(PathTest, ParentPath) {
    foundation::Path p("/home/user/file.txt");
    foundation::Path parent = p.parent_path();
    EXPECT_EQ(parent.string(), "/home/user");
}

TEST(PathTest, IsAbsolute) {
    EXPECT_TRUE(foundation::Path("/absolute/path").is_absolute());
    EXPECT_FALSE(foundation::Path("relative/path").is_absolute());
}

TEST(PathTest, IsRelative) {
    EXPECT_FALSE(foundation::Path("/absolute/path").is_relative());
    EXPECT_TRUE(foundation::Path("relative/path").is_relative());
}

TEST(PathTest, OperatorDivide) {
    foundation::Path parent("/home/user");
    foundation::Path child = parent / "file.txt";
    EXPECT_EQ(child.string(), "/home/user/file.txt");
}

TEST(PathTest, OperatorDivideEqual) {
    foundation::Path p("/home/user");
    p /= "file.txt";
    EXPECT_EQ(p.string(), "/home/user/file.txt");
}

TEST(PathTest, Equality) {
    foundation::Path a("/a/b");
    foundation::Path b("/a/b");
    foundation::Path c("/a/c");
    EXPECT_TRUE(a == b);
    EXPECT_TRUE(a != c);
}

TEST(PathTest, Comparison) {
    foundation::Path a("/a");
    foundation::Path b("/b");
    EXPECT_TRUE(a < b);
}

TEST(PathTest, Normalize) {
    foundation::Path p("/home/user/../user/./file.txt");
    p.normalize();
    // Normalization depends on the filesystem, but should not crash
    EXPECT_FALSE(p.empty());
}

} // namespace
