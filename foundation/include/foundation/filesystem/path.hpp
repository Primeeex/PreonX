#pragma once

#include "foundation/core/types.hpp"
#include "foundation/error/error.hpp"
#include "foundation/error/result.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace foundation {

class Path {
public:
    Path() = default;

    Path(std::string_view path) : path_(path) {}

    Path(const char* path) : path_(path) {} // NOLINT(google-explicit-constructor)

    Path(const std::string& path) : path_(path) {} // NOLINT(google-explicit-constructor)

    [[nodiscard]] const std::string& string() const noexcept {
        return path_;
    }

    [[nodiscard]] const char* c_str() const noexcept {
        return path_.c_str();
    }

    [[nodiscard]] bool empty() const noexcept {
        return path_.empty();
    }

    [[nodiscard]] std::string_view filename() const noexcept;
    [[nodiscard]] std::string_view stem() const noexcept;
    [[nodiscard]] std::string_view extension() const noexcept;
    [[nodiscard]] Path parent_path() const;
    [[nodiscard]] bool is_absolute() const;
    [[nodiscard]] bool is_relative() const;

    Path operator/(std::string_view child) const;
    Path& operator/=(std::string_view child);

    bool operator==(const Path& other) const noexcept {
        return path_ == other.path_;
    }

    bool operator!=(const Path& other) const noexcept {
        return path_ != other.path_;
    }

    bool operator<(const Path& other) const noexcept {
        return path_ < other.path_;
    }

    friend std::ostream& operator<<(std::ostream& os, const Path& path) {
        return os << path.path_;
    }

    void normalize();

private:
    std::string path_;
};

class FileSystem {
public:
    /// Check if a path exists.
    [[nodiscard]] static Result<bool> exists(const Path& path);

    /// Check if a path is a regular file.
    [[nodiscard]] static Result<bool> is_file(const Path& path);

    /// Check if a path is a directory.
    [[nodiscard]] static Result<bool> is_directory(const Path& path);

    /// Read the entire contents of a file.
    [[nodiscard]] static Result<std::string> read_text(const Path& path);

    /// Read the entire contents of a binary file.
    [[nodiscard]] static Result<std::vector<u8>> read_binary(const Path& path);

    /// Write text to a file, creating or overwriting it.
    [[nodiscard]] static Result<void> write_text(const Path& path, std::string_view content);

    /// Write binary data to a file, creating or overwriting it.
    [[nodiscard]] static Result<void> write_binary(const Path& path, const u8* data, size_t size);

    /// Create a directory. Returns error if it already exists.
    [[nodiscard]] static Result<void> create_directory(const Path& path);

    /// Remove a file.
    [[nodiscard]] static Result<void> remove(const Path& path);

    /// Get the current working directory.
    [[nodiscard]] static Result<Path> current_directory();

    /// List files in a directory.
    [[nodiscard]] static Result<std::vector<Path>> list_directory(const Path& path);
};

} // namespace foundation
