#include "foundation/filesystem/path.hpp"

#include "foundation/core/platform.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>

namespace foundation {

std::string_view Path::filename() const noexcept {
    auto pos = path_.find_last_of("/\\");
    if (pos == std::string::npos) {
        return path_;
    }
    return std::string_view(path_).substr(pos + 1);
}

std::string_view Path::stem() const noexcept {
    auto fname = filename();
    auto pos = fname.find_last_of('.');
    if (pos == std::string_view::npos) {
        return fname;
    }
    return fname.substr(0, pos);
}

std::string_view Path::extension() const noexcept {
    auto fname = filename();
    auto pos = fname.find_last_of('.');
    if (pos == std::string_view::npos) {
        return {};
    }
    return fname.substr(pos);
}

Path Path::parent_path() const {
    auto pos = path_.find_last_of("/\\");
    if (pos == std::string::npos) {
        return {};
    }
    return Path(path_.substr(0, pos));
}

bool Path::is_absolute() const {
    return !path_.empty() && (path_[0] == '/');
}

bool Path::is_relative() const {
    return !is_absolute();
}

Path Path::operator/(std::string_view child) const {
    if (path_.empty()) {
        return Path(child);
    }
    Path result = *this;
    if (!path_.empty() && path_.back() != '/' && path_.back() != '\\') {
        result.path_ += '/';
    }
    result.path_ += child;
    return result;
}

Path& Path::operator/=(std::string_view child) {
    if (!path_.empty() && path_.back() != '/' && path_.back() != '\\') {
        path_ += '/';
    }
    path_ += child;
    return *this;
}

void Path::normalize() {
    std::filesystem::path p(path_);
    path_ = p.lexically_normal().string();
}

// ── FileSystem implementation ─────────────────────────────────────────────────

Result<bool> FileSystem::exists(const Path& path) {
    return std::filesystem::exists(path.string());
}

Result<bool> FileSystem::is_file(const Path& path) {
    return std::filesystem::is_regular_file(path.string());
}

Result<bool> FileSystem::is_directory(const Path& path) {
    return std::filesystem::is_directory(path.string());
}

Result<std::string> FileSystem::read_text(const Path& path) {
    std::ifstream file(path.string(), std::ios::in);
    if (!file.is_open()) {
        return make_error(ErrorCode::FileNotFound, std::string("Cannot open file: ") + path.string());
    }
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return content;
}

Result<std::vector<u8>> FileSystem::read_binary(const Path& path) {
    std::ifstream file(path.string(), std::ios::in | std::ios::binary);
    if (!file.is_open()) {
        return make_error(ErrorCode::FileNotFound, std::string("Cannot open file: ") + path.string());
    }
    file.seekg(0, std::ios::end);
    auto size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<u8> data(static_cast<size_t>(size));
    file.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}

Result<void> FileSystem::write_text(const Path& path, std::string_view content) {
    auto parent = path.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent.string());
    }
    std::ofstream file(path.string(), std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        return make_error(ErrorCode::WriteFailed, std::string("Cannot create file: ") + path.string());
    }
    file.write(content.data(), static_cast<std::streamsize>(content.size()));
    return Result<void>();
}

Result<void> FileSystem::write_binary(const Path& path, const u8* data, size_t size) {
    auto parent = path.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent.string());
    }
    std::ofstream file(path.string(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (!file.is_open()) {
        return make_error(ErrorCode::WriteFailed, std::string("Cannot create file: ") + path.string());
    }
    file.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
    return Result<void>();
}

Result<void> FileSystem::create_directory(const Path& path) {
    std::error_code ec;
    bool created = std::filesystem::create_directories(path.string(), ec);
    if (ec) {
        return make_error(ErrorCode::WriteFailed, ec.message());
    }
    PREONX_UNUSED(created);
    return Result<void>();
}

Result<void> FileSystem::remove(const Path& path) {
    std::error_code ec;
    bool removed = std::filesystem::remove(path.string(), ec);
    if (ec) {
        return make_error(ErrorCode::WriteFailed, ec.message());
    }
    PREONX_UNUSED(removed);
    return Result<void>();
}

Result<Path> FileSystem::current_directory() {
    return Path(std::filesystem::current_path().string());
}

Result<std::vector<Path>> FileSystem::list_directory(const Path& path) {
    std::error_code ec;
    std::vector<Path> result;
    for (const auto& entry : std::filesystem::directory_iterator(path.string(), ec)) {
        result.emplace_back(entry.path().string());
    }
    if (ec) {
        return make_error(ErrorCode::PathInvalid, ec.message());
    }
    return result;
}

} // namespace foundation
