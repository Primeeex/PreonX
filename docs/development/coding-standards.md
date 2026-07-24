# Coding Standards

This document defines the coding conventions for all PreonX code.

---

## C++

### Standard

- **C++20** (`-std=c++20`)
- No compiler-specific extensions beyond what is necessary for platform abstraction.
- Use `[[nodiscard]]`, `[[maybe_unused]]`, `[[deprecated]]` attributes where appropriate.
- Prefer `constexpr` and `consteval` for compile-time computation.
- Use `std::` types over C-style equivalents.

### Formatting

Managed by `.clang-format`. Key rules:

- **Indentation**: 4 spaces, no tabs.
- **Column limit**: 120 characters.
- **Braces**: Attach (K&R style).
- **Namespace indentation**: None.
- **Pointer/reference alignment**: Left (`int* ptr`, not `int *ptr`).
- **Include order**: project headers, third-party, system (grouped and sorted).

### Naming Conventions

| Element | Convention | Example |
|---------|-----------|---------|
| Namespaces | `lower_snake_case` | `preonx::foundation::memory` |
| Classes/Structs | `PascalCase` | `RigidBody`, `CollisionResult` |
| Functions/Methods | `PascalCase` | `GetMass()`, `SolveConstraints()` |
| Member variables | `snake_case_` (trailing underscore) | `mass_`, `velocity_` |
| Local variables | `snake_case` | `contact_point`, `delta_time` |
| Constants | `kPascalCase` | `kMaxBodies`, `kGravity` |
| Enums (type) | `PascalCase` | `CollisionType` |
| Enums (values) | `kPascalCase` | `kStatic`, `kDynamic` |
| Template params | `T`, `TInput`, `TStorage` | `TComponent` |
| Macros | `PREONX_UPPER_CASE` | `PREONX_ASSERT` |
| File names | `snake_case` | `rigid_body.cpp`, `collision_result.hpp` |
| Header guards | `#pragma once` | — |

### Include Order

1. Corresponding header (for `.cpp` files)
2. Project headers (sorted)
3. Third-party library headers
4. System/standard library headers

Example:

```cpp
#include "foundation/memory/allocator.hpp"

#include <algorithm>
#include <vector>

#include "foundation/containers/dynamic_array.hpp"
#include "foundation/logging/logger.hpp"

#include <gtest/gtest.h>
```

### Error Handling

- No exceptions for normal runtime errors — use the `Result<T>` / `Error` / `ErrorCode` pattern from `foundation/error/`.
- Assert with `PREONX_ASSERT` / `PREONX_UNREACHABLE` for invariant checks in debug builds.
- Use `[[nodiscard]]` on all functions that return `Result<T>`, `Error`, or `ErrorCode`.
- Propagate errors upward with `return result.error();`; do not log-and-return.

### Memory Management

- Prefer stack allocation and RAII over heap allocation.
- Use the `foundation::Allocator` interface for all heap allocations.
- Never use raw `new`/`delete` outside of allocator implementations.
- `DynamicArray<T>` is the project-wide replacement for `std::vector`.
- Pass `Allocator&` as a constructor parameter for types that allocate.

### Header Organization

- Use `#pragma once` for include guards.
- Forward-declare when possible; include only what you use.
- Keep headers self-contained (each header compiles independently).
- Avoid including headers in other headers unless necessary.

---

## Rust

### Edition

- **Rust 2021** edition.
- Minimum supported Rust version (MSRV): 1.75.

### Formatting

Managed by `rustfmt.toml`. Key rules:

- **Max width**: 120 characters.
- **Indent**: 4 spaces.
- **Imports**: grouped (std, external, local), sorted.

### Linting

Managed by `clippy.toml` and workspace lint configuration.

- `clippy::all` and `clippy::pedantic` are denied.
- `unsafe_code` is forbidden globally (use explicit `unsafe` blocks only in designated modules).
- `clippy::unwrap_used` and `clippy::expect_used` produce warnings.
- `clippy::panic` is denied.

### Module Organization

- One module per file.
- Use `pub(crate)` by default; `pub` only for the SDK surface.
- Module tree mirrors directory structure.
- Use `mod.rs` files sparingly; prefer file-per-module where practical.

### Naming Conventions

| Element | Convention | Example |
|---------|-----------|---------|
| Modules | `snake_case` | `rigid_body`, `collision` |
| Types (structs, enums) | `PascalCase` | `RigidBody`, `CollisionResult` |
| Functions/Methods | `snake_case` | `get_mass()`, `solve_constraints()` |
| Variables | `snake_case` | `contact_point`, `delta_time` |
| Constants | `SCREAMING_SNAKE_CASE` | `MAX_BODIES`, `GRAVITY` |
| Traits | `PascalCase` | `Serialize`, `Component` |
| Lifetimes | `'a`, `'input` | `'allocator` |
| Macros | `snake_case!()` | `preonx_assert!()` |
| Crates | `snake_case` | `preonx_foundation` |

### Ownership Guidelines

- Prefer owned types over borrowed in public APIs.
- Use references (`&T`, `&mut T`) for function parameters that don't take ownership.
- Use `Cow<'_, T>` when you might need to own or borrow.
- Minimize use of `Arc<Mutex<T>>`; prefer channels or message passing.
- Document all `unsafe` blocks with a `SAFETY` comment explaining the invariants.

---

## Python

### Standard

- **Python 3.11+**
- Type hints are required on all public functions.
- Use `from __future__ import annotations` for forward references.

### Formatting

Managed by `ruff format`. Key rules:

- **Line length**: 120 characters.
- **Quote style**: double quotes.
- **Indent**: 4 spaces.

### Linting

Managed by `ruff check`. Enabled rule sets:

- `E`, `W` — pycodestyle
- `F` — pyflakes
- `I` — isort
- `N` — pep8-naming
- `UP` — pyupgrade
- `B` — flake8-bugbear
- `SIM` — flake8-simplify
- `TCH` — flake8-type-checking
- `RUF` — Ruff-specific rules

### Type Checking

Managed by `mypy` with strict mode enabled.

### Package Management

- Use `uv` for dependency management.
- `pyproject.toml` is the single source of truth for Python project configuration.
- Scripts go in the `scripts/` directory.

### Naming Conventions

| Element | Convention | Example |
|---------|-----------|---------|
| Modules | `snake_case` | `format_checker.py` |
| Classes | `PascalCase` | `AssetImporter` |
| Functions/Methods | `snake_case` | `load_texture()` |
| Variables | `snake_case` | `asset_path` |
| Constants | `SCREAMING_SNAKE_CASE` | `MAX_RETRY_COUNT` |
| Private | `_leading_underscore` | `_internal_state` |

---

## General Principles

1. **Clarity over cleverness.** Write code that is easy to read and understand.
2. **Consistency.** Follow the established patterns in the surrounding code.
3. **Minimal API surface.** Expose only what is necessary.
4. **Document intent, not mechanics.** Comments should explain _why_, not _what_.
5. **Fail fast.** Use assertions and early returns to catch bugs early.
6. **Separate concerns.** Each module should have a single, well-defined responsibility.
7. **Test what matters.** Focus tests on behavior, not implementation details.
