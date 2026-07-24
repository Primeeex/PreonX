# Physics World

**Stage 9 — Simulation Runtime Integration**

The physics world orchestrates the full simulation pipeline: body management, broadphase, collision detection, island generation, constraint solving, integration, and sleep evaluation.

## Architecture

```
world/
├── rigid_body.hpp              — RigidBody, BodyType, SleepState, BodyDescriptor, BodyID
├── body_manager.hpp            — BodyManager, BodyManagerStats
├── physics_material.hpp        — PhysicsMaterial, MaterialCombine, presets
├── island.hpp                  — Island, UnionFind, IslandBuilder
├── sleeping.hpp                — SleepConfig, SleepSystem
├── callbacks.hpp               — WorldCallbacks, ContactEvent
├── simulation_stats.hpp        — SimulationStats
└── physics_world.hpp           — PhysicsWorld, WorldConfig
```

## WorldConfig

```cpp
struct WorldConfig {
    SolverConfig solver;
    SleepConfig sleep;
    Vector3 gravity = kGravityDown;
    f32 fixedDt = 1.0f / 60.0f;
    u32 maxBodies = 1024;
    u32 maxContacts = 1024;
    bool sleepingEnabled = true;

    static WorldConfig gameDefault();   // 1/60 fixedDt, 8 vel iters, Baumgarte
    static WorldConfig simulation();    // 1/120 fixedDt, 20 vel iters, split impulse
    static WorldConfig fast();          // 1/30 fixedDt, 4 vel iters, no sleep
};
```

## Simulation Pipeline

Each call to `step(frameTime)` accumulates frame time and runs one or more fixed substeps:

```
step(frameTime)
├── TimestepAccumulator accumulates time
└── while (accumulated >= fixedDt):
      └── stepFixed(fixedDt)
          │
          ├── 1. Integrate velocities (semi-implicit Euler)
          │     body.integrateVelocity(dt)  — applies gravity + accumulated forces
          │     body.pendingForces.clear()
          │
          ├── 2. Update broadphase AABBs
          │     For each active body: remove from tree, recompute AABB, reinsert
          │     Broadphase tree capacity = 2 * maxBodies (leaf + internal nodes)
          │
          ├── 3. Detect collisions
          │     Broadphase query (AABB overlap) → contact pairs
          │     Narrowphase (bounding sphere test) → ContactManifolds
          │
          ├── 4. Build islands (Union-Find)
          │     Contact graph → connected components
          │     Each island solved independently
          │
          ├── 5. Solve constraints
          │     Per island:
          │       ├─ Skip sleeping islands
          │       ├─ Build SolverBodyData array
          │       ├─ Build ContactConstraint array from manifolds
          │       ├─ Combine materials (friction, restitution)
          │       ├─ SequentialImpulseSolver::solve()
          │       ├─ Write back velocities to RigidBody
          │       └─ Wake sleeping bodies in contact
          │
          ├── 6. Integrate positions
          │     body.integratePosition(dt)  — v * dt applied to position
          │
          ├── 7. Sleep evaluation
          │     SleepSystem::update() — checks velocity thresholds + timer
          │
          └── 8. Update statistics
```

## RigidBody

Three body types: `Dynamic`, `Static`, `Kinematic`.

```cpp
struct RigidBody {
    BodyID id;
    BodyType type;
    SleepState sleepState;
    bool enabled;

    Vector3 position;
    Quaternion rotation;
    Vector3 linearVelocity;
    Vector3 angularVelocity;

    ForceAccumulator pendingForces;
    MassProperties massProperties;
    InverseMassMatrix inverseInertia;
    PhysicsMaterial material;

    f32 gravityScale;
    f32 sleepTimer;
    i32 broadphaseNode;

    // Methods
    void integrateVelocity(f32 dt);
    void integratePosition(f32 dt);
    void applyForce(const Vector3& force);
    void applyImpulse(const Vector3& impulse);
    void applyImpulse(const Vector3& impulse, const Vector3& contactPoint);
};
```

- **Dynamic**: Full simulation — gravity, forces, impulses, collisions
- **Static**: Infinite mass, immovable, collides with other bodies
- **Kinematic**: Moves by velocity only, not affected by forces/collisions

## BodyManager

Slot-based body storage with free-list recycling.

```cpp
struct BodyManager {
    std::vector<RigidBody> bodies;
    std::vector<u32> freeList;
    u32 capacity;

    BodyID createBody(const BodyDescriptor& desc);
    void destroyBody(BodyID id);
    RigidBody& getBody(BodyID id);
    u32 size() const;
};
```

Bodies are addressed by `BodyID` (integer index). Invalid bodies have `kInvalidBody = 0xFFFFFFFF`.

## Island System

Union-Find algorithm partitions bodies into independent groups based on the contact graph.

- Each contact pair links two bodies into the same island
- Islands are solved independently (smaller islands = less work)
- Sleeping islands skip the solver entirely
- Static bodies anchor islands but don't consume solver resources

## Sleep System

Bodies below velocity thresholds for a sustained time are put to sleep.

```cpp
struct SleepConfig {
    f32 linearSleepThreshold = 0.01f;
    f32 angularSleepThreshold = 0.01f;
    f32 sleepTimeThreshold = 2.0f;
};
```

- Velocity magnitude below threshold → increment sleep timer
- Timer exceeds threshold → body sleeps
- Any contact force / impulse → wake the body and all neighbors

## PhysicsMaterial

Material properties with configurable combine modes.

```cpp
struct PhysicsMaterial {
    f32 friction = 0.5f;
    f32 restitution = 0.0f;
    MaterialCombine combine = MaterialCombine::Average;

    static PhysicsMaterial defaultMaterial();
    static PhysicsMaterial ice();
    static PhysicsMaterial rubber();
    static PhysicsMaterial steel();
    static PhysicsMaterial bouncy();
};
```

Combine modes: `Average`, `Multiply`, `Minimum`, `Maximum`.

## Callbacks

Event hooks for body creation, destruction, and contacts.

```cpp
struct WorldCallbacks {
    void (*onBodyCreated)(BodyID id, void* userData);
    void (*onBodyDestroyed)(BodyID id, void* userData);
    void (*onContact)(const ContactEvent& event, void* userData);
    void* userData;
};

struct ContactEvent {
    BodyID bodyA, bodyB;
    Vector3 contactPoint;
    Vector3 contactNormal;
    f32 penetration;
    f32 impulse;
};
```

## SimulationStats

Per-step statistics collected automatically.

```cpp
struct SimulationStats {
    u32 totalBodies, dynamicBodies, staticBodies, kinematicBodies;
    u32 activeBodies, sleepingBodies;
    u32 contactCount, manifoldCount;
    u32 broadphasePairs;
    u32 islandCount;
    u32 narrowphaseTests;
    u32 velocityIterations, positionIterations;
};
```

## Efficiency

- **Header-only**: Zero compilation overhead, full inlining
- **AABB tree broadphase**: O(log n) insert/remove, O(log n + k) overlap queries
- **Island isolation**: Only awake, connected body groups are solved
- **Sleep culling**: Static and sleeping bodies skip integration and solver
- **2x tree capacity**: DynamicAABBTree pre-allocates `2 * maxBodies` nodes for leaf + internal node pairs

## Bug Fixes Applied During Stage 9

1. **DynamicAABBTree::balance()** — Parent pointer corruption during AVL rotations. In the `else` branches of both left and right rotations, the child moved from one subtree to another had its `parent` field left stale, and AABB/height recomputations used the wrong child indices (F↔G or D↔E swapped). Fixed by correcting parent assignments and swapping the references in AABB/height computations.

2. **DynamicAABBTree::allocateNode()** — Node pool exhaustion with capacity = maxBodies. Each `insert()` allocates 2 nodes (leaf + parent), but the free-list only had `maxBodies` slots. After `maxBodies/2` inserts, the pool was exhausted, causing failed inserts or node corruption. Fixed by initializing the tree with `2 * maxBodies` capacity.

## Benchmarks

27 benchmarks covering body creation (dynamic/static × 16/64/256/512), world steps (empty, no-contacts, sleeping), fixed timestep substeps, island building, single-body gravity, multi-step simulation, and statistics queries.

## Tests

51 tests across 9 suites: PhysicsMaterial (7), RigidBody (8), BodyManager (7), UnionFind (3), IslandBuilder (4), SleepSystem (5), Callbacks (2), PhysicsWorld (15), SimulationStats (1). Tests cover body creation/destruction, forces/impulses, gravity, static/kinematic behavior, sleeping transitions, island generation, broadphase integration, fixed timestep, configuration presets, and body enable/disable.
