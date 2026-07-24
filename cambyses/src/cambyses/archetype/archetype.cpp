#include "cambyses/archetype/archetype.hpp"

#include <algorithm>
#include <utility>

namespace cambyses {

Archetype::Archetype(foundation::DynamicArray<ComponentTypeId> component_types,
                     foundation::DynamicArray<ColumnOps> ops_list)
    : component_types_(std::move(component_types)) {
    columns_.reserve(component_types_.size());
    for (foundation::size_t i = 0; i < ops_list.size(); ++i) {
        columns_.emplace_back(std::move(ops_list[i]));
    }
}

foundation::size_t Archetype::add_entity(Entity entity) {
    foundation::size_t row = entities_.size();
    entities_.push_back(entity);
    for (auto& col : columns_) {
        col.emplace_default();
    }
    return row;
}

Entity Archetype::remove_entity(foundation::size_t row) {
    PREONX_ASSERT(row < entities_.size());
    foundation::size_t last = entities_.size() - 1;

    for (auto& col : columns_) {
        col.swap_remove(row);
    }

    Entity swapped = kNullEntity;
    if (row != last) {
        swapped = entities_[last];
        entities_[row] = entities_[last];
    }
    entities_.pop_back();

    return swapped;
}

foundation::size_t Archetype::column_index_for(ComponentTypeId id) const noexcept {
    for (foundation::size_t i = 0; i < component_types_.size(); ++i) {
        if (component_types_[i] == id) {
            return i;
        }
    }
    return foundation::kNotFound;
}

} // namespace cambyses
