# Development Guides

This directory contains guides for developing with and contributing to PreonX.

## Contents

| Document | Status | Description |
|----------|--------|-------------|
| [Coding Standards](coding-standards.md) | **Active** | C++, Rust, and Python coding conventions |
| Getting Started | Planned | First-time setup and build instructions |
| Build Guide | Planned | Comprehensive build instructions for all platforms |
| IDE Setup | Planned | CLion, VS Code, and other IDE configurations |
| Testing Guide | Planned | How to write and run tests |
| Profiling | Planned | Performance analysis workflows |
| Release Process | Planned | How releases are versioned and published |
| Git Workflow | Planned | Branching strategy and commit conventions |

## Quick Start (Current)

```bash
# Configure (Debug, tests enabled)
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DPREONX_BUILD_TESTS=ON

# Build
cmake --build build -j$(nproc)

# Run tests
./build/tests/foundation_tests
```
