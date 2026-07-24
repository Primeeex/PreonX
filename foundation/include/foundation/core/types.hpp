#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

namespace foundation {

using i8 = std::int8_t;
using i16 = std::int16_t;
using i32 = std::int32_t;
using i64 = std::int64_t;

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;

using f32 = float;
using f64 = double;

using byte = std::byte;
using size_t = std::size_t;

using ptrdiff_t = std::ptrdiff_t;

inline constexpr u8 kMaxU8 = std::numeric_limits<u8>::max();
inline constexpr u16 kMaxU16 = std::numeric_limits<u16>::max();
inline constexpr u32 kMaxU32 = std::numeric_limits<u32>::max();
inline constexpr u64 kMaxU64 = std::numeric_limits<u64>::max();

inline constexpr i8 kMinI8 = std::numeric_limits<i8>::min();
inline constexpr i16 kMinI16 = std::numeric_limits<i16>::min();
inline constexpr i32 kMinI32 = std::numeric_limits<i32>::min();
inline constexpr i64 kMinI64 = std::numeric_limits<i64>::min();

inline constexpr i8 kMaxI8 = std::numeric_limits<i8>::max();
inline constexpr i16 kMaxI16 = std::numeric_limits<i16>::max();
inline constexpr i32 kMaxI32 = std::numeric_limits<i32>::max();
inline constexpr i64 kMaxI64 = std::numeric_limits<i64>::max();

inline constexpr f32 kEpsilonF32 = std::numeric_limits<f32>::epsilon();
inline constexpr f64 kEpsilonF64 = std::numeric_limits<f64>::epsilon();

inline constexpr size_t kNotFound = static_cast<size_t>(-1);

} // namespace foundation
