# Architecture

This directory contains documentation about the PreonX engine architecture.

## Contents

| Document | Status | Description |
|----------|--------|-------------|
| [Foundation](foundation.md) | **Implemented** | Foundation library architecture, design principles, dependency rules |
| [Cambyses](cambyses.md) | **Implemented** | Archetype-based ECS, entity lifecycle, component storage, queries |
| Overview | Planned | High-level system architecture |
| Subsystem Map | Planned | Visual diagram of subsystem dependencies |
| Data Flow | Planned | How data moves between subsystems |
| Memory Architecture | Planned | Engine-wide memory management strategy |
| Threading Model | Planned | Concurrency and parallelism design |
| Platform Abstraction | Planned | Cross-platform design decisions |
| Build System | Planned | CMake and Cargo architecture |

## Subsystem Dependency Graph (Current)

```
foundation/          ← Stage 2 (complete)
cambyses/            ← Stage 3 (complete) — depends on foundation only
primeon/             ← Stage 4 (not started)
luminium/            ← Stage 5 (not started)
cochise/             ← Stage 6 (not started)
jubal/               ← Stage 7 (not started)
integration/         ← Stage 8 (not started)
sdk/                 ← Stage 9 (not started)
tools/               ← Stage 10 (not started)
```

All future modules depend on `foundation/`. Foundation has zero intra-project dependencies.
