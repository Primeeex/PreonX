# Primeon — Physics Simulation Kernel

**Status**: Stage 4 (Math/Geometry) ✅ | Stage 5 (Dynamics) ✅ | Stage 6+ (Collision, Constraints) — Planned

## Purpose

`primeon` is the physics simulation subsystem. It provides math primitives, geometric operations, and rigid body dynamics for PreonX.

## Module Structure

```
primeon/include/primeon/
├── math/                          — Stage 4: Linear algebra foundations
│   ├── scalar/scalar.hpp          — f32 utilities, constants, interpolation
│   ├── vector/                    — Vector2, Vector3, Vector4
│   ├── matrix/                    — Matrix2, Matrix3, Matrix4
│   └── quaternion/quaternion.hpp  — Quaternion rotation
│
├── geometry/                      — Stage 4: Geometric primitives & queries
│   ├── primitives/                — AABB, Sphere, Capsule, Triangle, OBB, Plane, Ray, Segment
│   ├── intersection/              — Ray casting, overlap tests
│   └── distance/                  — Point-to-primitive distance
│
└── dynamics/                      — Stage 5: Physics simulation
    ├── mass/mass.hpp              — MassProperties, InertiaTensor
    ├── forces/forces.hpp          — Force, Impulse, ForceAccumulator, Gravity
    ├── motion/motion.hpp          — ParticleState, BodyState, derived quantities
    ├── energy/energy.hpp          — Kinetic, potential, total energy
    ├── integrators/integrators.hpp — 5 particle + 4 body integration methods
    └── simulation.hpp             — Integrator dispatch, fixed-timestep accumulator
```

## Dependencies

- `foundation` — Core types (f32, u32), memory, platform abstraction

## Design Principles

- **Header-only** — All code is `constexpr`/`inline`, zero link-time overhead
- **Value types** — No singletons, no hidden state, no implicit conversions
- **Explicit units** — Position in meters, velocity in m/s, force in Newtons
- **Testable** — Every function independently unit-testable

## Test Coverage

| Module | Tests |
|--------|-------|
| Math (scalar, vector, matrix, quaternion) | 163 |
| Geometry (primitives, intersection, distance) | included above |
| Dynamics (mass, forces, motion, energy, integrators, simulation) | 74 |
| **Total** | **237** |

## See Also

- [docs/physics/dynamics.md](../../docs/physics/dynamics.md) — Full dynamics layer documentation
- [docs/physics/README.md](../../docs/physics/README.md) — Physics overview
