# Collision Detection & Manifold Layer

**Stage 6–7 — Shape Intersection, Narrowphase, Broadphase, CCD, Manifolds & Constraints**

The collision layer provides stateless, header-only collision detection algorithms for PreonX. All functions live in `primeon::math` (narrowphase), `primeon::collision` (broadphase/queries), and operate on value-type geometry primitives. There are no collision managers, hidden caches, or OOP dispatch hierarchies.

## Architecture

```
collision/
├── contact.hpp                 — ContactFeature, ContactPoint, ContactManifold, ContactPair, CollisionResult, RayHit
├── shapes/
│   └── support.hpp             — Support mappings (GJK/EPA primitive)
├── narrowphase/
│   ├── sphere_sphere.hpp       — Sphere ↔ Sphere
│   ├── sphere_plane.hpp        — Sphere ↔ Plane
│   ├── sphere_aabb.hpp         — Sphere ↔ AABB
│   ├── sphere_capsule.hpp      — Sphere ↔ Capsule
│   ├── aabb_aabb.hpp           — AABB ↔ AABB
│   ├── obb_obb.hpp             — OBB ↔ OBB (SAT)
│   ├── capsule_capsule.hpp     — Capsule ↔ Capsule
│   └── raycast.hpp             — Ray vs all shapes
├── manifold/
│   ├── contact_generator.hpp   — Multi-point manifold generators for all shape pairs
│   └── contact_persistence.hpp — Manifold merging, feature matching, warm-starting
├── algorithms/
│   ├── sat.hpp                 — Separating Axis Theorem
│   ├── gjk.hpp                 — Gilbert-Johnson-Keerthi
│   └── epa.hpp                 — Expanding Polytope Algorithm
├── broadphase/
│   ├── sweep_and_prune.hpp     — SAP with sorted endpoints
│   └── dynamic_aabb_tree.hpp   — Balanced AABB tree
├── ccd/
│   └── tof.hpp                 — Time of Impact (continuous detection)
└── queries/
    └── collision_query.hpp     — Unified dispatch (intersects/contact/raycast)
```

```
constraints/
├── contact_constraint.hpp      — ContactConstraint, FrictionConstraint, RestitutionConstraint
└── jacobian.hpp                — Jacobian structures, effective mass, bias terms
```

**Dependency chain:** `contact` ← `narrowphase` ← `algorithms` ← `broadphase` ← `ccd` ← `queries`
**Stage 7 chain:** `contact` ← `manifold` ← `constraints` ← `jacobian`

---

## Contact Types (`contact.hpp`)

```cpp
enum class FeatureType : u32 { None, Face, Edge, Vertex };

struct ContactFeature {
    FeatureType type;
    u32 indexA, indexB;
};

struct ContactPoint {
    Vector3      point;
    f32          penetration;
    ContactFeature featureA, featureB;
    u32          contactID;
    f32          normalImpulse;      // accumulated for warm starting
    f32          frictionImpulseU, frictionImpulseV;
    Vector3      tangentU, tangentV; // friction tangent basis
    void computeID();
    bool hasSameID(const ContactPoint& other) const;
};

struct ContactManifold {
    ContactPoint contacts[4];
    u32          contactCount;
    u32          bodyIDA, bodyIDB;
    Vector3      normal;
    bool addContact(const ContactPoint& cp);
    void removeContact(u32 index);
    void clear();
    f32 maxPenetration() const;
    u32 deepestIndex() const;
    void computeTangents();
};

struct ContactPair {
    u32 bodyIDA, bodyIDB;
    ContactManifold manifold;
    bool matches(u32 a, u32 b) const;
};

struct CollisionResult {
    bool           colliding = false;
    ContactManifold manifold;
};

struct RayHit {
    bool    hit = false;
    f32     distance = std::numeric_limits<f32>::max();
    Vector3 point, normal;
};
```

---

## Narrowphase (`narrowphase/`)

All narrowphase functions are free functions in `primeon::math`. They take geometry primitives by const reference and return `CollisionResult` or `RayHit`.

```cpp
// Shape ↔ Shape
CollisionResult sphereSphere(const Sphere& a, const Sphere& b);
CollisionResult spherePlane(const Sphere& s, const Plane& p);
CollisionResult sphereAABB(const Sphere& s, const AABB& b);
CollisionResult sphereCapsule(const Sphere& s, const Capsule& c);
CollisionResult aabbAABB(const AABB& a, const AABB& b);
CollisionResult obbOBB(const OBB& a, const OBB& b);
CollisionResult capsuleCapsule(const Capsule& a, const Capsule& b);

// Ray ↔ Shape
RayHit raySphere(const Ray& r, const Sphere& s);
RayHit rayPlane(const Ray& r, const Plane& p);
RayHit rayAABB(const Ray& r, const AABB& b);
RayHit rayCapsule(const Ray& r, const Capsule& c);
RayHit rayOBB(const Ray& r, const OBB& o);
RayHit rayTriangle(const Ray& r, const Triangle& t);
```

**Touching convention:** When two shapes are exactly touching (distance equals combined radius), the result is `colliding = false`. This follows strict `<` separation to avoid generating contacts at zero penetration.

---

## Algorithms (`algorithms/`)

### SAT (Separating Axis Theorem)

Full 15-axis test for OBB ↔ OBB. Returns the minimum overlap and separating axis.

```cpp
f32 satOBBFull(const OBB& a, const OBB& b, Vector3& outNormal);
bool satOverlapOBB(const OBB& a, const Vector3& axis, const Vector3 axesA[3],
                    const OBB& b, const Vector3 axesB[3]);
```

### GJK (Gilbert-Johnson-Keerthi)

Iterative convex intersection test operating on Minkowski difference via support functions.

```cpp
template <typename SupportFunc>
bool gjkIntersect(SupportFunc support);
```

- `supportFunc`: `Vector3(const Vector3& dir) -> Vector3`
- Returns `true` if shapes intersect
- Converges in 10–30 iterations typically

### EPA (Expanding Polytope Algorithm)

Extends GJK to compute penetration depth and normal. Requires a 4-vertex simplex from GJK.

```cpp
template <typename SupportFunc>
EPAResult epa(const GJKSimplex& gjkSimplex, SupportFunc support);
```

- `EPAResult` contains: `converged`, `normal`, `depth`, `contactPoint`

---

## Support Mappings (`shapes/support.hpp`)

GJK and EPA operate through support functions that return the farthest point of a shape in a given direction.

```cpp
Vector3 supportSphere(const Sphere& s, const Vector3& d);
Vector3 supportAABB(const AABB& a, const Vector3& d);
Vector3 supportOBB(const OBB& o, const Vector3& d);
Vector3 supportCapsule(const Capsule& c, const Vector3& d);
Vector3 supportTriangle(const Triangle& t, const Vector3& d);
Vector3 supportConvexHull(const Vector3* verts, u32 count, const Vector3& d);

// Generic dispatch: calls the correct support based on shape type
template <typename T>
Vector3 supportPrimitive(const T& shape, const Vector3& d);
```

---

## Broadphase (`broadphase/`)

### Sweep and Prune (SAP)

Maintains sorted endpoint lists along each axis. Generates pairs by sweeping X and checking Y/Z overlap.

```cpp
struct SAPBroadphase {
    void insert(u32 bodyId, const AABB& aabb);
    void remove(u32 bodyId);
    void update(u32 bodyId, const AABB& aabb);
    void computePairs();
    u32 pairCount() const;
    u32 numBodies() const;
};
```

### Dynamic AABB Tree

Balanced binary tree of AABBs. O(log n) insert/remove/update, O(log n + k) queries.

```cpp
struct DynamicAABBTree {
    explicit DynamicAABBTree(u32 maxNodes = 256);
    i32 insert(u32 bodyId, const AABB& aabb);
    void remove(i32 idx);
    void update(i32 idx, const AABB& newAABB);
    void queryAABB(const AABB& aabb);
    void querySphere(const Vector3& center, f32 radius);
    void queryRay(const Vector3& origin, const Vector3& dir, f32 maxDist);
    u32 nodeCount() const;
    void clear();
};
```

- Uses surface-area heuristic for sibling selection
- AVL-style rebalancing after insert/remove
- Query results stored in `queryResults` vector

---

## CCD (`ccd/tof.hpp`)

Continuous collision detection computes Time of Impact between moving shapes.

```cpp
struct TOIResult {
    bool    hit;
    f32     toi;      // [0, 1], 0 = start, 1 = end
    Vector3 normal;
};

TOIResult toiSphereSphere(const Vector3& posA0, f32 rA,
                           const Vector3& posB0, f32 rB,
                           const Vector3& posA1, const Vector3& posB1);
TOIResult toiSpherePlane(const Vector3& pos0, f32 radius,
                          const Vector3& pos1,
                          const Vector3& planeNormal, f32 planeDist);
TOIResult toiSphereAABB(const Vector3& spherePos0, f32 radius,
                          const Vector3& spherePos1, const AABB& aabb);
TOIResult toiSphereCapsule(const Vector3& spherePos0, f32 radius,
                            const Vector3& spherePos1,
                            const Vector3& capStart0, const Vector3& capEnd0,
                            const Vector3& capStart1, const Vector3& capEnd1,
                            f32 capRadius);
```

---

## Unified Queries (`queries/collision_query.hpp`)

Stateless dispatch functions that wrap narrowphase and broadphase algorithms.

```cpp
// Boolean overlap
bool intersects(const Sphere& a, const Sphere& b);
bool intersects(const Sphere& s, const AABB& b);
bool intersects(const AABB& a, const AABB& b);
bool intersects(const OBB& a, const OBB& b);
bool intersects(const Capsule& a, const Capsule& b);
// ... and more combinations

// Full contact result
CollisionResult contact(const Sphere& a, const Sphere& b);
CollisionResult contact(const AABB& a, const AABB& b);
CollisionResult contact(const OBB& a, const OBB& b);
CollisionResult contact(const Capsule& a, const Capsule& b);

// GJK/EPA fallback for unsupported pairs
template <typename ShapeA, typename ShapeB>
CollisionResult contactGJK(const ShapeA& a, const ShapeB& b);

// Raycast
RayHit raycast(const Ray& r, const Sphere& s);
RayHit raycast(const Ray& r, const AABB& a);
RayHit raycast(const Ray& r, const OBB& o);
// ... and more combinations
```

---

## Design Principles

1. **Stateless algorithms** — Every function is pure; no hidden state or caches
2. **Value types** — All geometry and result types are plain structs
3. **Header-only** — Zero linking overhead, included via `primeon`
4. **No OOP hierarchy** — No `Shape` base class, no virtual dispatch
5. **Free functions** — All algorithms are standalone, composable
6. **Strict touching** — Zero-penetration contacts are non-colliding

---

## Contact Generation (`manifold/contact_generator.hpp`)

Multi-point manifold generators produce contact manifolds from overlapping shape pairs. Simple pairs (sphere-*) produce 1 contact point. Box-box pairs use reference face clipping to produce up to 4 points for stable stacking.

```cpp
CollisionResult generateSphereSphere(const Sphere& a, const Sphere& b, u32 idA = 0, u32 idB = 0);
CollisionResult generateSpherePlane(const Sphere& s, const Plane& p, u32 idA = 0, u32 idB = 0);
CollisionResult generateSphereAABB(const Sphere& s, const AABB& a, u32 idA = 0, u32 idB = 0);
CollisionResult generateSphereCapsule(const Sphere& s, const Capsule& c, u32 idA = 0, u32 idB = 0);
CollisionResult generateAABBAABB(const AABB& a, const AABB& b, u32 idA = 0, u32 idB = 0);
CollisionResult generateCapsuleCapsule(const Capsule& a, const Capsule& b, u32 idA = 0, u32 idB = 0);
CollisionResult generateOBBOBB(const OBB& a, const OBB& b, u32 idA = 0, u32 idB = 0);
```

All generators set `ContactFeature` on each contact point for persistence across frames, and compute tangent basis vectors for friction.

### Multi-point clipping (AABB-AABB, OBB-OBB)

For box-box contacts, the algorithm:
1. Identifies the reference face (least penetration axis) on shape A
2. Identifies the incident face on shape B (most anti-aligned with contact normal)
3. Generates 4 corners of the incident face
4. Clips against the 4 side planes of the reference face using Sutherland-Hodgman polygon clipping
5. Projects surviving points onto the reference face plane

---

## Contact Persistence (`manifold/contact_persistence.hpp`)

Maintains contact history across frames for impulse warm-starting and stable solver convergence.

```cpp
bool matchContact(ContactPoint& newContact, const ContactPoint* cached, u32 cachedCount);
void updateManifold(ContactManifold& output, const ContactManifold& cached, const ContactManifold& fresh);
void pruneDegenerateContacts(ContactManifold& manifold, f32 minPenetration = -0.001f);
void removeRedundantContacts(ContactManifold& manifold, f32 minSpacing = 0.01f);
void sortByPenetration(ContactManifold& manifold);
void finalizeManifold(ContactManifold& manifold);
```

### Algorithm

1. **Feature matching** — Each fresh contact is matched against cached contacts by `contactID`
2. **Impulse transfer** — Matched contacts inherit accumulated normal and friction impulses
3. **Degenerate removal** — Contacts with near-zero or negative penetration are pruned
4. **Redundancy removal** — Contacts closer than `minSpacing` are merged (keeping deepest)
5. **Sorting** — Contacts are sorted by penetration depth (deepest first) for solver ordering

---

## Constraints (`constraints/contact_constraint.hpp`)

Data structures for the impulse solver (Stage 8). No solving logic yet — only constraint definitions and accumulated impulse storage.

```cpp
struct InverseMassData {
    f32 inverseMass;
    Vector3 inverseInertiaDiag;
    Quaternion inverseInertiaOrientation;
};

struct ContactConstraint {
    u32 bodyIDA, bodyIDB;
    InverseMassData massA, massB;
    Vector3 pointA, pointB, normal;
    f32 penetration, restitution, friction;
    f32 slop, baumgarte;
    f32 accumulatedNormalImpulse;
    f32 normalMass;          // effective mass (computed per-frame)
    f32 velocityBias;        // position correction bias
    u32 contactID;
    f32 relativeVelocity(...) const;
};

struct FrictionConstraint {
    // Tangent direction constraint bounded by Coulomb cone
    f32 accumulatedFrictionImpulse;
    f32 tangentMass;
    f32 maxFrictionImpulse;  // = mu * normalImpulse
};

struct RestitutionConstraint {
    f32 restitution, velocityThreshold, targetVelocity;
    bool active;
    void computeTargetVelocity(f32 approachVelocity);
};
```

---

## Jacobians (`constraints/jacobian.hpp`)

Jacobian structures map impulses to velocity changes. For a contact with normal **n** and contact radii **r_A**, **r_B**:

```
J_normal = [n, r_A × n, -n, -(r_B × n)]
K = J · M⁻¹ · Jᵀ  (effective mass)
```

```cpp
struct Jacobian {
    JacobianLinear linear;
    JacobianAngular angular;
    static Jacobian normal(rA, rB, n);
    static Jacobian tangent(rA, rB, t);
    f32 effectiveMass(const InverseMassData& mA, const InverseMassData& mB) const;
};

struct ContactJacobianPair {
    Jacobian normal, tangentU, tangentV;
    f32 normalMass, tangentUMass, tangentVMass;
    static ContactJacobianPair fromConstraint(const ContactConstraint& cc, tangentU, tangentV);
    f32 computeBias(f32 penetration, f32 slop, f32 baumgarte, f32 dt) const;
};

struct JacobianMatrix {
    static constexpr u32 kMaxConstraints = 256;
    Jacobian rows[256];
    f32 effectiveMass[256];
    bool add(Jacobian j, f32 mass);
    f32 velocityError(u32 i, vA, wA, vB, wB) const;
};
```
