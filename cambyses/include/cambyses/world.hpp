#pragma once

#include "cambyses/archetype/archetype.hpp"
#include "cambyses/component/component_registry.hpp"
#include "cambyses/core/types.hpp"
#include "cambyses/entity/entity_manager.hpp"
#include "cambyses/events/event_dispatcher.hpp"
#include "cambyses/query/query.hpp"
#include "cambyses/system/system.hpp"
#include "foundation/containers/dynamic_array.hpp"
#include "foundation/core/assert.hpp"

#include <algorithm>
#include <functional>
#include <utility>

namespace cambyses {

class World {
public:
    World() = default;

    World(const World&) = delete;
    World& operator=(const World&) = delete;
    World(World&&) = default;
    World& operator=(World&&) = default;

    // ── Entity management ─────────────────────────────────────────────────

    [[nodiscard]] Entity create();
    void destroy(Entity entity);
    [[nodiscard]] bool is_valid(Entity entity) const noexcept;

    // ── Component operations ──────────────────────────────────────────────

    template <typename T>
    void add_component(Entity entity, const T& component) {
        PREONX_ASSERT(is_valid(entity));
        const auto& loc = entity_locations_[entity.index];
        const Archetype& src_arch = archetypes_[loc.archetype_index];

        foundation::DynamicArray<ComponentTypeId> new_types = src_arch.component_types();
        ComponentTypeId id = ComponentRegistry::type_id<T>();

        auto it = std::lower_bound(new_types.begin(), new_types.end(), id);
        if (it != new_types.end() && *it == id) {
            return;
        }
        new_types.insert(it, id);

        foundation::u32 dst_idx = find_or_create_archetype(new_types);
        migrate_entity(entity, loc.archetype_index, loc.row, dst_idx, [&](Archetype& dst, foundation::size_t dst_row) {
            dst.set_component(dst_row, component);
        });
    }

    template <typename T>
    void remove_component(Entity entity) {
        PREONX_ASSERT(is_valid(entity));
        const auto& loc = entity_locations_[entity.index];
        const Archetype& src_arch = archetypes_[loc.archetype_index];

        ComponentTypeId id = ComponentRegistry::type_id<T>();
        if (!src_arch.has_component_id(id)) {
            return;
        }

        foundation::DynamicArray<ComponentTypeId> new_types;
        new_types.reserve(src_arch.component_types().size() - 1);
        for (auto ct : src_arch.component_types()) {
            if (ct != id) {
                new_types.push_back(ct);
            }
        }

        foundation::u32 dst_idx = find_or_create_archetype(new_types);
        migrate_entity(entity, loc.archetype_index, loc.row, dst_idx, [](Archetype&, foundation::size_t) {});
    }

    template <typename T>
    [[nodiscard]] T& get_component(Entity entity) {
        PREONX_ASSERT(is_valid(entity));
        const auto& loc = entity_locations_[entity.index];
        return archetypes_[loc.archetype_index].get_component<T>(loc.row);
    }

    template <typename T>
    [[nodiscard]] const T& get_component(Entity entity) const {
        PREONX_ASSERT(is_valid(entity));
        const auto& loc = entity_locations_[entity.index];
        return archetypes_[loc.archetype_index].get_component<T>(loc.row);
    }

    template <typename T>
    [[nodiscard]] bool has_component(Entity entity) const {
        PREONX_ASSERT(is_valid(entity));
        const auto& loc = entity_locations_[entity.index];
        return archetypes_[loc.archetype_index].template has_component<T>();
    }

    // ── Queries ───────────────────────────────────────────────────────────

    template <typename... Components>
    [[nodiscard]] QueryBuilder<Components...> query() {
        return QueryBuilder<Components...>(*this);
    }

    template <typename... Components, typename Fn>
    void each(Fn&& fn) {
        execute_query<Components...>(std::forward<Fn>(fn),
                                    foundation::DynamicArray<ComponentTypeId>{});
    }

    // ── Systems ───────────────────────────────────────────────────────────

    void add_system(SystemFunction system) {
        systems_.push_back(std::move(system));
    }

    void run_systems();

    // ── Events ────────────────────────────────────────────────────────────

    template <typename Event>
    EventDispatcher::SubscriptionId subscribe(std::function<void(const Event&)> callback) {
        return events_.subscribe<Event>(std::move(callback));
    }

    template <typename Event>
    void unsubscribe(EventDispatcher::SubscriptionId id) {
        events_.unsubscribe<Event>(id);
    }

    template <typename Event>
    void publish(const Event& event) {
        events_.publish(event);
    }

    // ── Statistics ────────────────────────────────────────────────────────

    [[nodiscard]] foundation::u32 entity_count() const noexcept { return entities_.alive_count(); }
    [[nodiscard]] foundation::size_t archetype_count() const noexcept { return archetypes_.size(); }

    // ── Query execution (called by QueryBuilder) ──────────────────────────

    template <typename... Components, typename Fn>
    void execute_query(Fn&& fn, const foundation::DynamicArray<ComponentTypeId>& excluded) {
        foundation::DynamicArray<ComponentTypeId> required;
        required.reserve(sizeof...(Components));
        (required.push_back(ComponentRegistry::type_id<Components>()), ...);

        std::sort(required.begin(), required.end());

        for (foundation::size_t a = 0; a < archetypes_.size(); ++a) {
            Archetype& arch = archetypes_[a];
            if (arch.empty()) {
                continue;
            }

            const auto& arch_types = arch.component_types();
            if (!archetype_matches(arch_types, required, excluded)) {
                continue;
            }

            for (foundation::size_t row = 0; row < arch.size(); ++row) {
                fn(arch.entity(row), arch.get_component<Components>(row)...);
            }
        }
    }

    template <typename... Components>
    [[nodiscard]] foundation::size_t query_count() const {
        foundation::DynamicArray<ComponentTypeId> required;
        required.reserve(sizeof...(Components));
        (required.push_back(ComponentRegistry::type_id<Components>()), ...);
        std::sort(required.begin(), required.end());

        foundation::DynamicArray<ComponentTypeId> excluded{};
        foundation::size_t count = 0;

        for (const auto& arch : archetypes_) {
            if (arch.empty()) {
                continue;
            }
            const auto& arch_types = arch.component_types();
            if (archetype_matches(arch_types, required, excluded)) {
                count += arch.size();
            }
        }
        return count;
    }

private:
    struct EntityLocation {
        foundation::u32 archetype_index = 0;
        foundation::u32 row = 0;
    };

    EntityManager entities_;
    foundation::DynamicArray<Archetype> archetypes_;
    foundation::DynamicArray<EntityLocation> entity_locations_;
    foundation::DynamicArray<SystemFunction> systems_;
    EventDispatcher events_;

    foundation::u32 find_or_create_archetype(
        const foundation::DynamicArray<ComponentTypeId>& component_types);

    using MigrateCallback = std::function<void(Archetype&, foundation::size_t)>;
    void migrate_entity(Entity entity, foundation::u32 src_archetype_index,
                        foundation::u32 src_row, foundation::u32 dst_archetype_index,
                        const MigrateCallback& post_migrate_fn);

    static bool archetype_matches(
        const foundation::DynamicArray<ComponentTypeId>& arch_types,
        const foundation::DynamicArray<ComponentTypeId>& required,
        const foundation::DynamicArray<ComponentTypeId>& excluded);

    void init_archetype_columns(Archetype& arch,
                                const foundation::DynamicArray<ComponentTypeId>& types);
};

// ── QueryBuilder implementation ─────────────────────────────────────────────

template <typename... Components>
template <typename Fn>
void QueryBuilder<Components...>::each(Fn&& fn) const {
    world_.execute_query<Components...>(std::forward<Fn>(fn), excluded_ids_);
}

template <typename... Components>
foundation::size_t QueryBuilder<Components...>::count() const {
    return world_.query_count<Components...>();
}

} // namespace cambyses
