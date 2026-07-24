# Dynamics Layer

**Stage 5 — Motion, Forces & Numerical Integration**

The dynamics layer provides the physical simulation foundation for PreonX. It models particle and rigid body motion under forces, using explicit numerical integration methods. All types are value-oriented, header-only, and live in `primeon::math`.

## Architecture

```
dynamics/
├── mass/mass.hpp          — Mass properties and inertia tensors
├── forces/forces.hpp      — Force/impulse accumulation and gravity
├── motion/motion.hpp      — Particle and body state representations
├── energy/energy.hpp      — Kinetic, potential, and total energy
├── integrators/integrators.hpp — 5 particle + 4 body integration methods
└── simulation.hpp         — Integrator dispatch, fixed-timestep accumulator
```

**Dependency chain:** `mass` → `forces` → `motion` → `integrators` → `simulation`

All components depend only on Foundation (math primitives). There are no hidden globals, singletons, or implicit unit conversions.

## Units

| Quantity | Unit |
|----------|------|
| Position | meters (m) |
| Velocity | m/s |
| Acceleration | m/s² |
| Force | Newtons (N = kg·m/s²) |
| Mass | kilograms (kg) |
| Energy | Joules (J) |
| Angular velocity | rad/s |
| Torque | N·m |
| Inertia tensor | kg·m² |

---

## Mass Properties (`mass.hpp`)

`MassProperties` stores mass and its precomputed inverse. A zero inverse mass denotes a static (immovable) object.

```cpp
MassProperties mp = MassProperties::fromMass(5.0f);    // 5 kg dynamic
MassProperties stat = makeStaticMass();                  // infinite mass
MassProperties fromInv = MassProperties::fromInverseMass(0.5f); // 2 kg
```

**Key API:**
- `fromMass(f32)` — Constructs from mass; clamps negative to zero
- `fromInverseMass(f32)` — Constructs from inverse mass directly
- `isStatic()` / `isDynamic()` — Query mobility
- `inverseMass` — Precomputed `1/mass` (zero for static)

### Inertia Tensor

`InertiaTensor` wraps a `Matrix3` and provides factory methods for common shapes.

```cpp
InertiaTensor sphere = InertiaTensor::solidSphere(1.0f, 0.5f);   // I = 0.4*m*r²
InertiaTensor hollow = InertiaTensor::hollowSphere(1.0f, 0.5f);  // I = 0.667*m*r²
InertiaTensor box = InertiaTensor::solidBox(2.0f, {1, 0.5f, 0.5f});
InertiaTensor diag = InertiaTensor::diagonal(1.0f, 2.0f, 3.0f);
InertiaTensor inv = tensor.inverse();
```

---

## Forces & Impulses (`forces.hpp`)

`ForceAccumulator` collects forces and torques over a frame. Forces applied at an offset from the center of mass produce both linear force and torque.

```cpp
ForceAccumulator acc;
acc.addForce(Vector3(0, -9.80665f, 0));              // gravity (at COM)
acc.addForce(Force(pushForce, contactPoint));          // at offset → torque
acc.addTorque(Vector3(0, 0, 10.0f));                  // pure torque
acc.clear();                                           // reset for next frame
```

**`Force` struct:** `{ vector, point }` — force vector + application point (zero = COM).

**`Impulse` struct:** `{ linear, angular }` — instantaneous velocity change in kg·m/s.

**Gravity helpers:**
```cpp
Vector3 Fg = gravitationalForce(mass);                    // m * g
f32 PE = gravitationalPotentialEnergy(mass, height);      // m * g * h
```

Constants: `kGravityDown = (0, -9.80665, 0)`, `kGravityMagnitude = 9.80665`.

---

## Motion State (`motion.hpp`)

### ParticleState

```cpp
ParticleState state(position, velocity);
state.applyImpulse(impulse, inverseMass);
```

### BodyState

Full rigid body state with orientation:

```cpp
BodyState body;
body.position = Vector3(0, 10, 0);
body.rotation = Quaternion::fromAxisAngle(kVector3UnitY, 0.5f);
body.linearVelocity = Vector3(1, 0, 0);
body.angularVelocity = Vector3(0, 3.14f, 0);

body.applyLinearImpulse(impulse, inverseMass);
body.applyAngularImpulse(angImpulse, inverseInertia);
Vector3 velAtCorner = body.velocityAtPoint(offset);  // v + ω × r
```

### Derived Quantities

```cpp
Vector3 p = linearMomentum(velocity, mass);                    // p = mv
Vector3 L = angularMomentum(angularVelocity, inertia);         // L = Iω
f32 KE = kineticEnergy(mass, velocity);                        // ½mv²
f32 KErot = rotationalKineticEnergy(omega, inertia);           // ½ωᵀIω
f32 KEtotal = totalKineticEnergy(mass, v, omega, inertia);     // translational + rotational
```

---

## Numerical Integration (`integrators.hpp`)

Five integration methods for particles, four for rigid bodies. All share the same interface pattern:

```cpp
ParticleState next = integrate<Method>(state, forces, mass, dt);
BodyState next = integrateBody<Method>(state, forces, mass, inertia, dt);
```

### Particle Integrators

| Method | Order | Symplectic | Notes |
|--------|-------|------------|-------|
| `integrateExplicitEuler` | O(h²) | No | Position uses old velocity. Energy drift. |
| `integrateSemiImplicitEuler` | O(h²) | Yes | Velocity first, then position. Standard for games. |
| `integrateVerlet` | O(h⁴) | Yes | Position-only. Velocity field stores previous position. |
| `integrateVelocityVerlet` | O(h⁴) | Yes | Kick-drift-kick. Excellent accuracy. |
| `integrateRK4` | O(h⁴) | No | 4 force evaluations. Most accurate per step. |

#### Position Verlet Convention

The Verlet integrator repurposes `ParticleState.velocity` as the **previous position**. For a loop to work correctly, the caller must ensure the velocity field holds the prior-frame position:

```cpp
// Starting from rest: previous position = current position
ParticleState s(initialPos, initialPos);  // velocity = position
for (int i = 0; i < N; ++i)
    s = integrateVerlet(s, forces, mass, dt);
// s.position is the final position
```

#### Velocity Verlet (Simplified)

For velocity-independent forces (constant forces like gravity), the full velocity Verlet simplifies to:

```
x' = x + v·dt + ½·a·dt²
v' = v + a·dt
```

This is equivalent to semi-implicit Euler for position but uses the explicit analytical form.

### Body Integrators

| Method | Order | Notes |
|--------|-------|-------|
| `integrateBodyExplicitEuler` | O(h²) | Same issues as particle explicit Euler |
| `integrateBodySemiImplicitEuler` | O(h²) | Standard game physics choice |
| `integrateBodyVelocityVerlet` | O(h⁴) | Best accuracy for linear + angular |
| `integrateBodyRK4` | O(h⁴) | Full RK4 for both linear and angular |

**Angular integration** uses quaternion derivative:
```
dq/dt = ½ · Ω(ω) · q
```
where `Ω(ω)` is the quaternion form of angular velocity. The result is normalized each step to prevent drift.

---

## Simulation Dispatch (`simulation.hpp`)

### Integrator Selection

```cpp
enum class IntegratorType : u32 {
    ExplicitEuler = 0,
    SemiImplicitEuler = 1,
    Verlet = 2,         // Only for particles
    VelocityVerlet = 3,
    RK4 = 4
};
```

### Single-Step Dispatch

```cpp
ParticleState p = simulateParticle(state, forces, mass, dt,
                                    IntegratorType::SemiImplicitEuler);
BodyState b = simulateBody(state, forces, mass, inertia, dt,
                            IntegratorType::VelocityVerlet);
```

### Fixed-Timestep Sub-Stepping

`TimestepAccumulator` implements the classic fixed-timestep pattern, decoupling physics rate from frame rate:

```cpp
TimestepAccumulator acc(1.0f / 60.0f);  // 60 Hz physics

// In game loop:
f32 dt = acc.step(frameTime);
if (dt > 0) state = simulate(state, forces, mass, dt);
f32 alpha = acc.alpha();  // for interpolation between physics states
```

**`TimestepAccumulator` API:**
- `step(frameTime)` — Consumes frame time, returns fixed dt if enough accumulated
- `alpha()` — Interpolation factor [0,1] between physics frames
- `reset()` — Clears accumulator

### Multi-Step Simulation

Convenience functions that sub-step over a total time:

```cpp
ParticleState final = simulateParticleFixedSteps(
    initialState, forces, mass,
    totalTime, fixedDt,
    IntegratorType::SemiImplicitEuler);

BodyState final = simulateBodyFixedSteps(
    initialState, forces, mass, inertia,
    totalTime, fixedDt);
```

---

## Energy (`energy.hpp`)

Standalone energy computation functions (not tied to any integrator):

```cpp
f32 KE = computeKineticEnergy(mass, velocity);
f32 PE = computePotentialEnergy(mass, height);              // gravitational
f32 PE = computePotentialEnergy(mass, body);                // uses body.position.y
f32 TE = computeTotalEnergy(mass, position, velocity);      // KE + PE
f32 TE = computeTotalEnergy(mass, particle);                // ParticleState overload
f32 TE = computeTotalEnergy(mass, body);                    // BodyState overload
f32 err = energyError(initialEnergy, currentEnergy);        // |E₁ - E₀| / |E₀|
```

---

## Design Decisions

### Why Header-Only?

All dynamics code is `constexpr`/`inline` in headers. This enables:
- Compile-time evaluation where possible
- Zero-linker-overhead for the engine
- Easy testability (each function is independently unit-testable)

### Why Explicit Value Types?

No `PhysicsWorld` singleton, no `RigidBody*` pointers. Every function takes state in and returns state out. This makes the physics layer:
- Deterministic (no hidden state)
- Thread-safe (no shared mutable state)
- Testable (no setup/teardown for unit tests)

### Integrator Accuracy vs. Cost

| Method | Position Error (1s freefall, dt=1/60) | Cost |
|--------|---------------------------------------|------|
| Explicit Euler | ~0.08 m | 1 FLOP |
| Semi-Implicit Euler | ~0.08 m | 1 FLOP |
| Verlet | ~0.003 m | 1 FLOP |
| Velocity Verlet | ~0.003 m | 1 FLOP |
| RK4 | <0.001 m | 4 FLOPs |

For most game applications, **Semi-Implicit Euler** offers the best cost/accuracy tradeoff. Use **Velocity Verlet** or **RK4** for high-fidelity simulations.

### Float32 Precision

All types use `f32` (32-bit float). Accumulated error over millions of steps is expected and documented. For example, semi-implicit Euler with `dt=1e-6` over 1M steps (~1s) accumulates ~0.25 m of position error due to float32 rounding — this is inherent to single-precision arithmetic, not an integrator bug.

---

## Testing

237 unit tests across 6 test files:

| File | Tests | Coverage |
|------|-------|----------|
| `test_mass.cpp` | 14 | MassProperties + InertiaTensor factories |
| `test_forces.cpp` | 12 | Force, Impulse, ForceAccumulator, Gravity |
| `test_motion.cpp` | 13 | ParticleState, BodyState, Momentum, Energy |
| `test_energy.cpp` | 14 | Energy computation and conservation |
| `test_integrators.cpp` | 13 | All 5 particle integrators + energy conservation |
| `test_simulation.cpp` | 11 | Dispatch, fixed-timestep, edge cases |

## Benchmarks

14 micro-benchmarks in `benchmarks/micro/primeon_benchmark.cpp`:

| Benchmark | Typical Time |
|-----------|-------------|
| `BM_Integrator_ExplicitEuler` | ~3.6 ns |
| `BM_Integrator_SemiImplicitEuler` | ~4.6 ns |
| `BM_Integrator_Verlet` | ~8.0 ns |
| `BM_Integrator_VelocityVerlet` | ~5.0 ns |
| `BM_Integrator_RK4` | ~8.2 ns |
| `BM_BodySemiImplicitEuler` | ~20.7 ns |
| `BM_BodyRK4` | ~76.8 ns |
| `BM_SimulateParticle` | ~4.5 ns |
| `BM_SimulateBody` | ~20.5 ns |
| `BM_IntegrateQuaternion` | ~20.1 ns |
| `BM_ComputeKineticEnergy` | ~0.13 ns |
| `BM_ComputeTotalEnergy` | ~0.13 ns |
