#pragma once

#include "cambyses/archetype/column.hpp"
#include "cambyses/component/component_registry.hpp"
#include "cambyses/core/types.hpp"
#include "foundation/containers/dynamic_array.hpp"
#include "foundation/core/assert.hpp"

#include <algorithm>
#include <utility>

namespace cambyses {

class Archetype {
public:
    Archetype() = default;

    Archetype(foundation::DynamicArray<ComponentTypeId> component_types,
              foundation::DynamicArray<ColumnOps> ops_list);

    Archetype(const Archetype&) = delete;
    Archetype& operator=(const Archetype&) = delete;

    Archetype(Archetype&& other) noexcept = default;
    Archetype& operator=(Archetype&& other) noexcept = default;

    [[nodiscard]] foundation::size_t add_entity(Entity entity);
    [[nodiscard]] Entity remove_entity(foundation::size_t row);

    template <typename T>
    void set_component(foundation::size_t row, const T& value) {
        foundation::size_t col = column_index_for(ComponentRegistry::type_id<T>());
        PREONX_ASSERT(col != foundation::kNotFound);
        columns_[col].get_as<T>(row) = value;
    }

    template <typename T>
    void set_component_move(foundation::size_t row, T&& value) {
        foundation::size_t col = column_index_for(ComponentRegistry::type_id<T>());
        PREONX_ASSERT(col != foundation::kNotFound);
        columns_[col].get_as<T>(row) = std::move(value);
    }

    template <typename T>
    [[nodiscard]] T& get_component(foundation::size_t row) {
        foundation::size_t col = column_index_for(ComponentRegistry::type_id<T>());
        PREONX_ASSERT(col != foundation::kNotFound);
        return columns_[col].get_as<T>(row);
    }

    template <typename T>
    [[nodiscard]] const T& get_component(foundation::size_t row) const {
        foundation::size_t col = column_index_for(ComponentRegistry::type_id<T>());
        PREONX_ASSERT(col != foundation::kNotFound);
        return columns_[col].get_as<T>(row);
    }

    template <typename T>
    [[nodiscard]] bool has_component() const noexcept {
        return column_index_for(ComponentRegistry::type_id<T>()) != foundation::kNotFound;
    }

    [[nodiscard]] foundation::size_t column_index_for(ComponentTypeId id) const noexcept;

    [[nodiscard]] bool has_component_id(ComponentTypeId id) const noexcept {
        return column_index_for(id) != foundation::kNotFound;
    }

    [[nodiscard]] Entity entity(foundation::size_t row) const {
        PREONX_ASSERT(row < entities_.size());
        return entities_[row];
    }

    [[nodiscard]] Column& column(foundation::size_t index) { return columns_[index]; }
    [[nodiscard]] const Column& column(foundation::size_t index) const { return columns_[index]; }

    [[nodiscard]] const foundation::DynamicArray<ComponentTypeId>& component_types() const noexcept {
        return component_types_;
    }

    [[nodiscard]] const foundation::DynamicArray<Entity>& entities() const noexcept {
        return entities_;
    }

    [[nodiscard]] foundation::DynamicArray<Entity>& entities() noexcept { return entities_; }

    [[nodiscard]] foundation::DynamicArray<Column>& columns() noexcept { return columns_; }

    [[nodiscard]] const foundation::DynamicArray<Column>& columns() const noexcept {
        return columns_;
    }

    [[nodiscard]] foundation::size_t size() const noexcept { return entities_.size(); }
    [[nodiscard]] bool empty() const noexcept { return entities_.empty(); }

private:
    foundation::DynamicArray<Entity> entities_;
    foundation::DynamicArray<Column> columns_;
    foundation::DynamicArray<ComponentTypeId> component_types_;
};

} // namespace cambyses
