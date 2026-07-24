# Cambyses — Archetype-Based ECS

## Overview

Cambyses is a production-quality archetype-based Entity Component System (ECS) library. It provides cache-friendly, data-oriented game object management through composition over inheritance.

## Architecture

### Core Concepts

- **Entity**: A lightweight handle (`Entity{index, generation}`) that identifies a game object. The generation counter detects stale references.
- **Component**: Plain data structs (POD-like) attached to entities. No virtual functions or inheritance.
- **Archetype**: A container grouping entities that share the same set of component types. All entities in an archetype have identical memory layout.
- **World**: Top-level API that manages entities, archetypes, and component operations.

### Memory Layout

```
Archetype [Position, Velocity]
┌─────────────────────────────────────────┐
│ Entities: [e0, e1, e2, ...]            │  Entity handles
├─────────────────────────────────────────┤
│ Position Column: [p0, p1, p2, ...]     │  Contiguous Position data
├─────────────────────────────────────────┤
│ Velocity Column: [v0, v1, v2, ...]     │  Contiguous Velocity data
└─────────────────────────────────────────┘
```

Components of the same type are stored contiguously in columns, enabling cache-friendly iteration and SIMD optimization.

### Type System

- `ComponentTypeId`: `u32` assigned via static counter (`ComponentRegistry::type_id<T>()`)
- `Entity`: `{index: u32, generation: u32}` — compact 8-byte handle
- `kNullEntity`: `{0, 0}` — sentinel for invalid entities
- `ColumnOps`: Type-erased function pointers for construct/copy/move/destruct with trivial-type fast paths

## API

### Entity Management

```cpp
World world;
Entity e = world.create();       // Create entity in empty archetype
world.destroy(e);                // Remove entity and free slot
bool valid = world.is_valid(e);  // Check generation match
```

### Component Operations

```cpp
world.add_component(e, Position{1.0f, 2.0f});    // Migrates to new archetype
world.add_component(e, Velocity{0.1f, 0.0f});    // Migrates again
world.remove_component<Position>(e);              // Migrates to archetype without Position

Position& pos = world.get_component<Position>(e);
bool has = world.has_component<Velocity>(e);
```

Adding/removing components migrates the entity between archetypes. All existing component data is preserved during migration.

### Queries

```cpp
// Iterate all entities with Position and Velocity
world.query<Position, Velocity>().each([](Entity e, Position& pos, Velocity& vel) {
    pos.x += vel.dx;
    pos.y += vel.dy;
});

// Exclude entities with a Tag component
world.query<Position>().exclude<Tag>().each([](Entity e, Position& pos) {
    // Only entities with Position but without Tag
});

// Count matching entities
foundation::size_t count = world.query_count<Position, Velocity>();
```

### Systems

```cpp
world.add_system([](World& w) {
    w.query<Position, Velocity>().each([](Entity, Position& pos, const Velocity& vel) {
        pos.x += vel.dx;
        pos.y += vel.dy;
    });
});
world.run_systems();
```

### Events

```cpp
struct CollisionEvent { Entity a; Entity b; };

auto sub = world.subscribe<CollisionEvent>([](const CollisionEvent& e) {
    // Handle collision
});
world.publish(CollisionEvent{entity_a, entity_b});
world.unsubscribe<CollisionEvent>(sub);
```

## Implementation Details

### Archetype Management

- Archetypes are stored in a `DynamicArray<Archetype>` inside World
- `find_or_create_archetype()` searches for an existing archetype with matching component types, or creates a new one
- Component type lists are kept sorted for efficient matching

### Entity Migration

When a component is added/removed, the entity is migrated between archetypes:

1. New entity slot created in destination archetype
2. All shared component columns are copied (via `Column::copy_element`)
3. New component is set on the destination
4. Entity removed from source archetype (swap-remove for O(1) removal)

### Column Storage

`Column` provides type-erased storage with `ColumnOps` function pointers:
- **Trivial types** (`std::is_trivially_copyable`): `memcpy`/`memset` fast paths
- **Non-trivial types**: Dispatched through function pointers for construct/copy/move/destruct
- Growth strategy: 1.5x+1 capacity growth to avoid infinite loop at small sizes

### Entity Lifecycle

- `EntityManager` maintains a free-list for slot recycling
- Generation counter prevents use-after-free: stale handles are detected via generation mismatch
- `kNullEntity` sentinel marks invalid/empty slots

## Dependencies

- Foundation (core types, allocator, containers, assertions)
- C++20, GCC/Clang
- No runtime dependencies beyond standard library

## Test Coverage

55 tests across 7 suites:
- `EntityManagerTest` (9): creation, destruction, validity, recycling
- `ComponentRegistryTest` (7): type IDs, ops, trivial detection
- `ArchetypeTest` (8): add/remove, column access, iteration
- `QueryTest` (6): single/multi-component, exclusion, count
- `SystemTest` (3): registration, ordering
- `EventDispatcherTest` (7): subscribe, publish, unsubscribe
- `WorldTest` (15): integration tests covering full lifecycle

## File Structure

```
cambyses/
├── include/cambyses/
│   ├── core/types.hpp              Entity, ComponentTypeId
│   ├── component/component_registry.hpp   ColumnOps, ComponentRegistry
│   ├── archetype/column.hpp        Column (type-erased storage)
│   ├── archetype/archetype.hpp     Archetype
│   ├── entity/entity_manager.hpp   EntityManager
│   ├── query/query.hpp             QueryBuilder, With/Without
│   ├── events/event_dispatcher.hpp EventDispatcher
│   ├── system/system.hpp           SystemFunction
│   ├── serialization/serialization.hpp SerializationRegistry
│   └── world.hpp                   World (top-level API)
├── src/cambyses/
│   ├── component/component_registry.cpp
│   ├── archetype/column.cpp
│   ├── archetype/archetype.cpp
│   ├── entity/entity_manager.cpp
│   ├── serialization/serialization.cpp
│   └── world.cpp
└── CMakeLists.txt
```
