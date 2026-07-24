# Sequential Impulse Solver

**Stage 8 — Constraint-Based Collision Response**

The solver transforms collision manifolds into physically correct body motion using projected Gauss-Seidel (PGS) iteration on contact constraints.

## Architecture

```
solver/
├── solver_config.hpp                          — SolverConfig, presets, friction model enum
├── sequential_impulse/
│   └── sequential_impulse_solver.hpp          — SolverBodyData, SolverContact, SolverStats, SequentialImpulseSolver
├── friction/
│   └── friction_solver.hpp                    — Coulomb cone friction impulse application
├── restitution/
│   └── restitution_solver.hpp                 — Newton's restitution with velocity threshold
└── stabilization/
    └── stabilization.hpp                      — Baumgarte stabilization, SplitImpulseState, split impulse
```

## Solve Pipeline

Each frame, `SequentialImpulseSolver::solve()` executes:

```
1. Warm Starting
   └─ Scale accumulated impulses from previous frame by warmStartFactor

2. Restitution
   └─ Set velocityBias = -(−e · v_approach) for impact contacts

3. Baumgarte / Split Impulse Setup
   └─ Add position-correction bias to velocityBias (or zero it for split impulse)

4. Velocity Iterations (PGS)
   └─ For each contact:
      a. Solve normal impulse (clamped ≥ 0)
      b. Solve friction impulse (clamped to Coulomb cone: |f_t| ≤ μ·f_n)

5. Position Iterations (split impulse only)
   └─ Accumulate position corrections without modifying velocities

6. Statistics
   └─ Total impulse, max penetration
```

## Key Types

### SolverBodyData

Flat-array body representation for the solver. Bodies are indexed by ID — no body struct is included.

```cpp
struct SolverBodyData {
    Vector3 linearVelocity;
    Vector3 angularVelocity;
    InverseMassData massData;  // inverseMass, inverseInertiaDiag
};
```

### SolverContact

Internal per-contact data built from `ContactConstraint`:

```cpp
struct SolverContact {
    u32 bodyIDA, bodyIDB;
    Vector3 normal, tangentU, tangentV;
    f32 normalMass, tangentUMass, tangentVMass;
    f32 accumulatedNormalImpulse;
    f32 accumulatedFrictionUImpulse, accumulatedFrictionVImpulse;
    f32 velocityBias;  // restitution + Baumgarte
};
```

### SolverConfig

```cpp
struct SolverConfig {
    u32 velocityIterations = 8;
    u32 positionIterations = 3;
    f32 penetrationSlop = 0.005f;
    f32 baumgarteFactor = 0.2f;
    f32 restitutionThreshold = 1.0f;
    bool warmStartingEnabled = true;
    bool splitImpulseEnabled = false;

    static SolverConfig gameDefault();   // 8 vel / 3 pos, Baumgarte
    static SolverConfig simulation();   // 20 vel / 10 pos, split impulse
    static SolverConfig fast();         // 4 vel / 1 pos, no warm start
};
```

## Friction Model

Coulomb cone: `|f_t| ≤ μ · f_n` where `f_n` is the accumulated normal impulse.

- Single coefficient by default (static = dynamic)
- Optional static/dynamic split via `FrictionModel::StaticDynamic`
- Two tangent directions (U, V) solved independently, then clamped

## Restitution

Newton's law: `v_sep = -e · v_app`

- Only active when approach velocity exceeds `restitutionThreshold` (default 1.0 m/s)
- Below threshold: treated as resting contact (no bounce)
- Applied as velocity bias before PGS iterations

## Warm Starting

Scales accumulated impulses from the previous frame by `warmStartFactor` (default 1.0) and applies them as initial impulses. Reduces iteration count needed for convergence.

## Position Correction

### Baumgarte Stabilization

Adds velocity bias: `bias = (baumgarte / dt) · max(0, penetration + slop)`

Simple and effective but adds energy to the system.

### Split Impulse

Separates velocity correction (real impulse) from position correction (accumulated position impulse applied directly to positions). No energy addition, more stable for stacking.

## Efficiency

- **Jacobian-based effective mass**: Pre-computed per contact, avoids redundant dot products
- **Flat arrays**: Cache-friendly iteration, no pointer chasing
- **Stateless between frames**: All state in SolverContact accumulators
- **Header-only**: Zero compilation overhead, inlining-friendly

## Benchmarks

12 benchmarks covering iteration counts (1/8/20), friction on/off, restitution, box stacks (3/8 bodies), large contact sets (32), warm starting effectiveness, and split impulse.

## Tests

36 tests across 6 suites: SolverConfig (4), Stabilization (5), Restitution (4), Friction (3), SequentialImpulseSolver (19), SolverContact (1). Tests cover sphere/box collisions, bouncing, resting contacts, friction, stacks, inclined planes, energy conservation, convergence, determinism, and split impulse.
