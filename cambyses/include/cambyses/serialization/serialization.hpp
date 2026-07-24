#pragma once

#include "cambyses/component/component_registry.hpp"
#include "cambyses/core/types.hpp"
#include "foundation/containers/dynamic_array.hpp"

namespace cambyses {

struct ComponentMetadata {
    ComponentTypeId type_id;
    const char* name;
    foundation::size_t size;
    foundation::size_t alignment;
};

class SerializationRegistry {
public:
    template <typename T>
    static void register_component(const char* name = "Unknown") {
        ComponentRegistry::register_component<T>(name);
        ComponentTypeId id = ComponentRegistry::type_id<T>();
        if (id >= metadata_.size()) {
            metadata_.resize(id + 1);
        }
        metadata_[id] = ComponentMetadata{id, name, sizeof(T), alignof(T)};
    }

    [[nodiscard]] static const foundation::DynamicArray<ComponentMetadata>& metadata() {
        return metadata_;
    }

    [[nodiscard]] static bool has_metadata(ComponentTypeId id) noexcept {
        return id < metadata_.size() && metadata_[id].size > 0;
    }

    [[nodiscard]] static const ComponentMetadata& get_metadata(ComponentTypeId id) {
        return metadata_[id];
    }

    static void clear();

private:
    static foundation::DynamicArray<ComponentMetadata> metadata_;
};

} // namespace cambyses
