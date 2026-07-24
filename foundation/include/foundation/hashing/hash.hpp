#pragma once

#include "foundation/core/types.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace foundation {

using HashValue = u64;

/// FNV-1a hash function — fast, deterministic, suitable for hash maps and content hashing.
namespace hash {

[[nodiscard]] inline HashValue fnv1a(const void* data, size_t size) noexcept {
    constexpr HashValue kFnvOffsetBasis = 14695981039346656037ULL;
    constexpr HashValue kFnvPrime = 1099511628211ULL;

    HashValue hash = kFnvOffsetBasis;
    const auto* bytes = static_cast<const u8*>(data);
    for (size_t i = 0; i < size; ++i) {
        hash ^= static_cast<HashValue>(bytes[i]);
        hash *= kFnvPrime;
    }
    return hash;
}

[[nodiscard]] inline HashValue hash_bytes(const void* data, size_t size) noexcept {
    return fnv1a(data, size);
}

[[nodiscard]] inline HashValue hash_string(std::string_view str) noexcept {
    return fnv1a(str.data(), str.size());
}

template <typename T>
[[nodiscard]] inline HashValue hash_value(const T& value) noexcept {
    return fnv1a(&value, sizeof(T));
}

/// Combine multiple hash values into one.
inline HashValue& combine(HashValue& seed, HashValue hash) noexcept {
    constexpr HashValue kCombinePrime = 1099511628211ULL;
    seed ^= hash;
    seed *= kCombinePrime;
    return seed;
}

[[nodiscard]] inline HashValue combine(std::initializer_list<HashValue> hashes) noexcept {
    HashValue seed = 0;
    for (HashValue h : hashes) {
        combine(seed, h);
    }
    return seed;
}

} // namespace hash

} // namespace foundation
