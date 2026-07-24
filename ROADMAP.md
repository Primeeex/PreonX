# PreonX Development Roadmap

This document outlines the planned development stages for the PreonX engine ecosystem.

## Versioning Policy

PreonX follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html):

- **MAJOR** (X.0.0): Incompatible API changes.
- **MINOR** (0.X.0): New functionality in a backwards-compatible manner.
- **PATCH** (0.0.X): Backwards-compatible bug fixes.

During initial development (version 0.x.y), the API is not considered stable. Minor version bumps may contain breaking changes, documented in CHANGELOG.md.

---

## Stage 1 — Repository Foundation ✅

**Version: 0.1.0**

Project infrastructure, build system, CI/CD, documentation structure, and coding standards.

- Complete repository skeleton
- CMake build system with C++20
- Rust workspace configuration
- Python tooling with uv
- GitHub Actions CI
- Governance and documentation

## Stage 2 — Foundation Layer ✅

**Version: 0.2.0**

Core utilities and platform abstraction that all other subsystems depend on.

- Memory allocation framework
- Platform detection and abstraction
- Core math library (vectors, matrices, quaternions)
- Container types (dynamic arrays, hash maps, ring buffers)
- Logging infrastructure
- Error handling framework
- Threading primitives
- Filesystem abstraction
- Compile-time reflection utilities
- Serialization core

## Stage 3 — ECS Core ✅

**Version: 0.3.0**

Entity Component System implementation.

- Component storage (archetype-based)
- Entity management
- System execution framework
- Query system
- Event system
- World management
- Component registration

## Stage 4 — Physics Kernel ✅

**Version: 0.4.0**

Physics simulation and collision detection.

- Rigid body dynamics
- Collision detection (broadphase + narrowphase)
- Constraint solver
- Contact management
- Physics world integration with ECS
- Serialization of physics state

## Stage 5 — Rendering Engine

**Version: 0.5.0**

Graphics rendering pipeline.

- Renderer core and framegraph
- Material system
- Light types and shadow mapping
- Camera system
- Mesh and geometry handling
- Texture management
- Shader compilation pipeline
- Backend abstraction (Vulkan initial target)
- Post-processing framework

## Stage 6 — Asset Pipeline

**Version: 0.6.0**

Asset loading, caching, and management.

- Asset loading framework
- Format support (meshes, textures, audio)
- Asset caching
- Hot-reloading support
- Import pipeline
- Asset database

## Stage 7 — Audio System

**Version: 0.7.0**

Audio playback and spatial sound.

- Audio backend abstraction
- Mixer
- 3D spatial audio
- Audio effects
- Stream management
- Audio resource management

## Stage 8 — Integration Layer

**Version: 0.8.0**

Subsystem orchestration and lifecycle management.

- Engine initialization and shutdown
- Subsystem lifecycle management
- Update loop coordination
- Event bus integration
- Configuration management

## Stage 9 — SDK & Tools

**Version: 0.9.0**

Public API and developer tooling.

- Public SDK API surface
- Editor application
- CLI tools
- Codec implementations
- Profiler integration

## Stage 10 — Polish & Release

**Version: 1.0.0**

Documentation, examples, benchmarks, and release preparation.

- Comprehensive API documentation
- Example projects
- Performance benchmarks
- Cross-platform validation
- Release packaging
- Migration guide

---

## Philosophy

Each stage is designed to be **independently verifiable**. A stage is considered complete when:

1. All planned functionality is implemented.
2. Unit and integration tests pass.
3. Documentation is updated.
4. CI pipeline is green.
5. Performance baselines are established (where applicable).

This staged approach ensures that each layer is solid before building upon it, reducing technical debt and maintaining a high standard of quality throughout development.
