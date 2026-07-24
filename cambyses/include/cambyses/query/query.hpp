#pragma once

#include "cambyses/component/component_registry.hpp"
#include "cambyses/core/types.hpp"
#include "foundation/containers/dynamic_array.hpp"

namespace cambyses {

class World;

template <typename T>
struct With {};

template <typename T>
struct Without {};

template <typename... Components>
class QueryBuilder {
public:
    explicit QueryBuilder(World& world) : world_(world) {}

    template <typename T>
    QueryBuilder<Components...>& exclude() {
        excluded_ids_.push_back(ComponentRegistry::type_id<T>());
        return *this;
    }

    template <typename Fn>
    void each(Fn&& fn) const;

    [[nodiscard]] foundation::size_t count() const;

private:
    World& world_;
    foundation::DynamicArray<ComponentTypeId> excluded_ids_;
};

} // namespace cambyses
