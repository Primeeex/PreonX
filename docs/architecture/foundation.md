# Foundation Library Architecture

Foundation is the lowest-level library in the PreonX engine ecosystem. Every other module depends on it. It has zero dependencies on other PreonX modules.

## Module Map

```
foundation/
├── core/           Platform detection, type aliases, assert macros, version info
├── containers/     DynamicArray<T> — the only custom container
├── error/          ErrorCode, Error, Result<T> — error handling without exceptions
├── filesystem/     Path, FileSystem — cross-platform file operations
├── hashing/        FNV-1a hash functions and hash combining utilities
├── logging/        Logger with sink architecture (console, file, custom)
├── memory/         Allocator interface, DefaultAllocator (posix_memalign / aligned_malloc)
├── platform/       Runtime platform detection (OS, arch, compiler, CPU info)
├── threading/      Mutex, SharedMutex, Thread wrappers
└── time/           Clock, Stopwatch, DeltaTimer
```

## Design Principles

### No Exceptions

Foundation does not use exceptions for normal runtime errors. Instead, it uses the `Result<T>` / `Error` / `ErrorCode` pattern:

```cpp
Result<std::string> read_config(const Path& path) {
    auto content = FileSystem::read_text(path);
    if (!content) {
        return content.error();  // propagate error
    }
    return content.value();      // propagate value
}
```

- `Result<T>` holds either a value of type `T` or an `Error`
- `Result<void>` is specialized to hold only success/error
- `Error` wraps an `ErrorCode` enum plus an optional human-readable message
- All `Result`-returning functions are marked `[[nodiscard]]`

### Custom Allocator Model

All heap allocations go through a `foundation::Allocator` interface:

```cpp
class Allocator {
public:
    virtual void* allocate(size_t size, size_t alignment) = 0;
    virtual void deallocate(void* ptr, size_t size, size_t alignment) = 0;
};
```

- `DefaultAllocator` uses `posix_memalign` (POSIX) or `_aligned_malloc` (Windows)
- Alignment is clamped to `alignof(std::max_align_t)` to satisfy platform requirements
- Tracks total bytes allocated and allocation count for diagnostics
- `DynamicArray<T>` accepts an `Allocator&` at construction; all other containers follow this pattern

### DynamicArray<T>

The sole custom container. A `std::vector`-like class with:

- Allocator-aware construction (`DynamicArray<T>(n, allocator)`)
- Trivially-copyable fast path: uses `memcpy`/`memmove` instead of element-wise copy/move
- Trivially-destructible fast path: skips destructor calls on clear/destruct
- No iterators beyond raw pointers (`data()`, `begin()`, `end()`)

### Thread Safety

- Logger is internally synchronized with a mutex
- `Mutex`, `SharedMutex`, `ScopedLock` wrap `std::mutex`, `std::shared_mutex`, and RAII lock guards
- `Thread` wraps `std::thread` with a name and running-state query

### Ownership Rules

- No singletons. `Logger::global()` is the sole exception (a static factory method, not a hidden global)
- No smart pointers in the foundation API — ownership is explicit via `Allocator&` or value semantics
- `Path` is a value type (wraps `std::string`)
- `FileSystem` is a stateless static class (all methods are `static`)

## Dependency Rules

```
Other modules ──→ foundation/
                  foundation/ ──→ (no PreonX deps)
```

- Foundation must never `#include` anything from `primeon/`, `luminium/`, etc.
- Foundation headers must compile standalone (self-contained includes)
- Use `#pragma once` for all headers
- Foundation may use STL, POSIX APIs, and platform-specific system calls

## Build Configuration

- Builds as a static library: `libfoundation.a`
- Exported CMake target: `Preonx::Foundation`
- C++20 required (`-std=c++20`)
- Warnings-as-errors enabled (`-Werror`)
- GoogleTest fetched via `FetchContent` for the test suite (v1.14.0)
- Tests discover via `gtest_discover_tests` (each death test runs in its own subprocess)
