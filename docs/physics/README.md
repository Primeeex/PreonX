# Physics

Documentation for the PreonX physics simulation system (`primeon`).

## Contents

- **[dynamics.md](dynamics.md)** — Motion, forces, mass, energy, numerical integration (Stage 5)
- **[collision.md](collision.md)** — Collision detection: narrowphase, broadphase, GJK/EPA, raycasting, CCD, manifolds, constraints, Jacobians (Stages 6–7)
- **[solver.md](solver.md)** — Sequential impulse solver: friction, restitution, warm starting, Baumgarte/split impulse stabilization (Stage 8)
- **[world.md](world.md)** — Physics world: simulation pipeline, rigid body management, islands, sleeping, callbacks (Stage 9)

## Architecture

```
                    ┌─────────────┐
                    │  primeon    │
                    └──────┬──────┘
                           │
              ┌────────────┼────────────┐
              │            │            │
         ┌────┴────┐ ┌────┴────┐ ┌────┴────┐
         │  math   │ │geometry │ │dynamics │
         └────┬────┘ └────┬────┘ └────┬────┘
              │            │            │
              └────────────┼────────────┘
                           │
                    ┌──────┴──────┐
                    │  foundation │
                    └─────────────┘
```

All `primeon` modules are header-only and depend only on `foundation`.
