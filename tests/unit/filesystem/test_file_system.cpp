#include <gtest/gtest.h>

#include "foundation/filesystem/path.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>

namespace {

class FileSystemTest : public ::testing::Test {
protected:
    void SetUp() override {
        temp_dir_ = "/tmp/preonx_test_" + std::to_string(reinterpret_cast<uintptr_t>(this));
        std::filesystem::create_directories(temp_dir_);
    }

    void TearDown() override {
        std::filesystem::remove_all(temp_dir_);
    }

    std::string temp_dir_;
};

TEST_F(FileSystemTest, Exists) {
    foundation::Path p(temp_dir_);
    auto result = foundation::FileSystem::exists(p);
    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(result.value());

    foundation::Path nonexistent(temp_dir_ + "/nonexistent.txt");
    auto result2 = foundation::FileSystem::exists(nonexistent);
    EXPECT_TRUE(result2.has_value());
    EXPECT_FALSE(result2.value());
}

TEST_F(FileSystemTest, IsFile) {
    foundation::Path file(temp_dir_ + "/test.txt");
    std::ofstream ofs(file.string());
    ofs << "test";
    ofs.close();

    auto result = foundation::FileSystem::is_file(file);
    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(result.value());

    auto dir_result = foundation::FileSystem::is_file(foundation::Path(temp_dir_));
    EXPECT_TRUE(dir_result.has_value());
    EXPECT_FALSE(dir_result.value());
}

TEST_F(FileSystemTest, IsDirectory) {
    foundation::Path dir(temp_dir_ + "/subdir");
    std::filesystem::create_directories(dir.string());

    auto result = foundation::FileSystem::is_directory(dir);
    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(result.value());
}

TEST_F(FileSystemTest, WriteAndReadText) {
    foundation::Path file(temp_dir_ + "/test.txt");
    auto write_result = foundation::FileSystem::write_text(file, "Hello, PreonX!");
    EXPECT_TRUE(write_result.has_value());

    auto read_result = foundation::FileSystem::read_text(file);
    EXPECT_TRUE(read_result.has_value());
    EXPECT_EQ(read_result.value(), "Hello, PreonX!");
}

TEST_F(FileSystemTest, WriteAndReadBinary) {
    foundation::Path file(temp_dir_ + "/test.bin");
    std::vector<foundation::u8> data = {0x01, 0x02, 0x03, 0xFF};
    auto write_result = foundation::FileSystem::write_binary(file, data.data(), data.size());
    EXPECT_TRUE(write_result.has_value());

    auto read_result = foundation::FileSystem::read_binary(file);
    EXPECT_TRUE(read_result.has_value());
    EXPECT_EQ(read_result.value(), data);
}

TEST_F(FileSystemTest, WriteTextOverwrite) {
    foundation::Path file(temp_dir_ + "/overwrite.txt");
    (void)foundation::FileSystem::write_text(file, "first");
    (void)foundation::FileSystem::write_text(file, "second");

    auto result = foundation::FileSystem::read_text(file);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), "second");
}

TEST_F(FileSystemTest, ReadNonexistent) {
    foundation::Path file(temp_dir_ + "/nonexistent.txt");
    auto result = foundation::FileSystem::read_text(file);
    EXPECT_TRUE(result.has_error());
}

TEST_F(FileSystemTest, CreateDirectory) {
    foundation::Path dir(temp_dir_ + "/newdir/subdir");
    auto result = foundation::FileSystem::create_directory(dir);
    EXPECT_TRUE(result.has_value());

    auto exists = foundation::FileSystem::is_directory(dir);
    EXPECT_TRUE(exists.has_value());
    EXPECT_TRUE(exists.value());
}

TEST_F(FileSystemTest, RemoveFile) {
    foundation::Path file(temp_dir_ + "/todelete.txt");
    (void)foundation::FileSystem::write_text(file, "delete me");

    auto remove_result = foundation::FileSystem::remove(file);
    EXPECT_TRUE(remove_result.has_value());

    auto exists = foundation::FileSystem::exists(file);
    EXPECT_TRUE(exists.has_value());
    EXPECT_FALSE(exists.value());
}

TEST_F(FileSystemTest, CurrentDirectory) {
    auto result = foundation::FileSystem::current_directory();
    EXPECT_TRUE(result.has_value());
    EXPECT_FALSE(result.value().empty());
}

TEST_F(FileSystemTest, ListDirectory) {
    foundation::Path file1(temp_dir_ + "/a.txt");
    foundation::Path file2(temp_dir_ + "/b.txt");
    (void)foundation::FileSystem::write_text(file1, "a");
    (void)foundation::FileSystem::write_text(file2, "b");

    auto result = foundation::FileSystem::list_directory(foundation::Path(temp_dir_));
    EXPECT_TRUE(result.has_value());
    EXPECT_GE(result.value().size(), 2u);
}

TEST_F(FileSystemTest, WriteTextCreatesParentDirectories) {
    foundation::Path file(temp_dir_ + "/deep/nested/path/file.txt");
    auto result = foundation::FileSystem::write_text(file, "deep write");
    EXPECT_TRUE(result.has_value());

    auto read_result = foundation::FileSystem::read_text(file);
    EXPECT_TRUE(read_result.has_value());
    EXPECT_EQ(read_result.value(), "deep write");
}

} // namespace
