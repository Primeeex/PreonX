#include "cambyses/world.hpp"

#include <algorithm>
#include <utility>

namespace cambyses {

Entity World::create() {
    Entity entity = entities_.create();

    EntityLocation loc;
    loc.archetype_index = 0;
    loc.row = 0;

    if (entity.index >= static_cast<EntityIndex>(entity_locations_.size())) {
        entity_locations_.resize(entity.index + 1, loc);
    }

    foundation::DynamicArray<ComponentTypeId> empty_types;
    if (archetypes_.empty()) {
        foundation::DynamicArray<ColumnOps> empty_ops;
        archetypes_.emplace_back(std::move(empty_types), std::move(empty_ops));
    }

    loc.archetype_index = 0;
    loc.row = static_cast<foundation::u32>(archetypes_[0].add_entity(entity));
    entity_locations_[entity.index] = loc;

    return entity;
}

void World::destroy(Entity entity) {
    if (!is_valid(entity)) {
        return;
    }

    auto& loc = entity_locations_[entity.index];
    Archetype& arch = archetypes_[loc.archetype_index];

    Entity swapped = arch.remove_entity(loc.row);

    if (swapped != kNullEntity) {
        auto& swapped_loc = entity_locations_[swapped.index];
        swapped_loc.row = loc.row;
    }

    loc.archetype_index = 0;
    loc.row = 0;

    entities_.destroy(entity);
}

bool World::is_valid(Entity entity) const noexcept {
    return entities_.is_valid(entity);
}

foundation::u32 World::find_or_create_archetype(
    const foundation::DynamicArray<ComponentTypeId>& component_types) {
    for (foundation::size_t i = 0; i < archetypes_.size(); ++i) {
        const auto& types = archetypes_[i].component_types();
        if (types.size() != component_types.size()) {
            continue;
        }
        bool match = true;
        for (foundation::size_t j = 0; j < types.size(); ++j) {
            if (types[j] != component_types[j]) {
                match = false;
                break;
            }
        }
        if (match) {
            return static_cast<foundation::u32>(i);
        }
    }

    foundation::u32 new_idx = static_cast<foundation::u32>(archetypes_.size());
    foundation::DynamicArray<ColumnOps> ops_list;
    ops_list.reserve(component_types.size());
    for (auto id : component_types) {
        ops_list.push_back(ComponentRegistry::get_info(id).ops);
    }
    archetypes_.emplace_back(component_types, ops_list);
    return new_idx;
}

void World::migrate_entity(Entity entity, foundation::u32 src_archetype_index,
                           foundation::u32 src_row, foundation::u32 dst_archetype_index,
                           const MigrateCallback& post_migrate_fn) {
    Archetype& src_arch = archetypes_[src_archetype_index];
    Archetype& dst_arch = archetypes_[dst_archetype_index];

    foundation::size_t dst_row = dst_arch.add_entity(entity);

    const auto& dst_types = dst_arch.component_types();

    for (auto dst_ct : dst_types) {
        foundation::size_t src_col = src_arch.column_index_for(dst_ct);
        if (src_col != foundation::kNotFound) {
            foundation::size_t dst_col = dst_arch.column_index_for(dst_ct);
            dst_arch.columns()[dst_col].copy_element(dst_row, src_arch.columns()[src_col], src_row);
        }
    }

    post_migrate_fn(dst_arch, dst_row);

    Entity swapped = src_arch.remove_entity(src_row);

    if (swapped != kNullEntity) {
        entity_locations_[swapped.index].row = src_row;
    }

    entity_locations_[entity.index] = {dst_archetype_index, static_cast<foundation::u32>(dst_row)};
}

bool World::archetype_matches(
    const foundation::DynamicArray<ComponentTypeId>& arch_types,
    const foundation::DynamicArray<ComponentTypeId>& required,
    const foundation::DynamicArray<ComponentTypeId>& excluded) {
    for (auto req : required) {
        bool found = false;
        for (auto at : arch_types) {
            if (at == req) {
                found = true;
                break;
            }
        }
        if (!found) {
            return false;
        }
    }

    for (auto exc : excluded) {
        for (auto at : arch_types) {
            if (at == exc) {
                return false;
            }
        }
    }

    return true;
}

void World::run_systems() {
    for (auto& system : systems_) {
        system(*this);
    }
}

} // namespace cambyses
