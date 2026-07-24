# PreonX

**A modular, high-performance game engine ecosystem.**

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://isocpp.org/)
[![Rust](https://img.shields.io/badge/Rust-1.75+-orange.svg)](https://www.rust-lang.org/)
[![Build Status](https://github.com/Primeeex/PreonX/actions/workflows/ci.yml/badge.svg)](https://github.com/Primeeex/PreonX/actions)

---

## Overview

PreonX is an open-source, modular game engine ecosystem built for performance, flexibility, and long-term maintainability. It is designed around a layered architecture where each subsystem is an independent, well-defined component.

The engine is written primarily in C++20 with strategic use of Rust for safety-critical subsystems, and Python for tooling and scripting.

### Subsystems

| Module | Purpose | Language |
|--------|---------|----------|
| **foundation** | Core utilities, memory, platform abstraction | C++20 |
| **primeon** | Physics simulation kernel | C++20 |
| **luminium** | Rendering engine | C++20 |
| **cambyses** | Entity Component System | C++20 |
| **cochise** | Asset pipeline | C++20 / Rust |
| **jubal** | Audio system | C++20 / Rust |
| **integration** | Subsystem orchestration | C++20 |
| **sdk** | Public API surface | C++20 |
| **tools** | Editor, CLI, codecs, profiler | C++20 / Python |

## Building

### Prerequisites

- CMake 3.24 or later
- Clang 17+ or GCC 14+ (C++20 support required)
- Rust 1.75+ (for Rust components)
- Python 3.11+ with `uv` (for tooling)

### Quick Start

```bash
# Clone the repository
git clone https://github.com/Primeeex/PreonX.git
cd PreonX

# Configure with CMake
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build --parallel

# Run tests
ctest --test-dir build --output-on-failure
```

### Using the Python Environment

```bash
# Install uv if not already installed
pip install uv

# Set up the Python environment
uv sync

# Run formatting checks
uv run ruff format --check .
```

## Project Status

PreonX is in **active development** (Stage 9 — Physics World, Rigid Bodies & Simulation Pipeline). The foundation runtime, ECS, mathematics/geometry, collision detection, constraint solver, and physics world are fully implemented with 642 passing tests.

See the [ROADMAP.md](ROADMAP.md) for the full development plan.

## Documentation

Comprehensive documentation is available in the [docs/](docs/) directory:

- [Architecture](docs/architecture/) — System design and subsystem relationships
- [Development Guides](docs/development/) — Getting started, build instructions, workflows
- [Coding Standards](docs/development/coding-standards.md) — C++, Rust, and Python conventions
- [API Reference](docs/api/) — Public API documentation (forthcoming)

## Contributing

We welcome contributions. Please read [CONTRIBUTING.md](CONTRIBUTING.md) before submitting a pull request.

This project adheres to the [Code of Conduct](CODE_OF_CONDUCT.md).

## Security

For security-related issues, please see [SECURITY.md](SECURITY.md).

## License

PreonX is licensed under the [MIT License](LICENSE).
