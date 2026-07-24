#pragma once

#include "cambyses/core/types.hpp"
#include "foundation/containers/dynamic_array.hpp"

#include <cstring>
#include <functional>
#include <new>
#include <type_traits>

namespace cambyses {

struct ColumnOps {
    using Fn0 = void (*)(void*);
    using Fn2 = void (*)(void*, void*);
    using Fn2C = void (*)(void*, const void*);

    Fn0 default_construct = nullptr;
    Fn2C copy_construct = nullptr;
    Fn2 move_construct = nullptr;
    Fn0 destruct = nullptr;
    Fn2 move_assign = nullptr;

    foundation::size_t element_size = 0;
    foundation::size_t element_alignment = 1;
    bool trivial_copy = false;
    bool trivial_destruct = false;
    bool trivial_construct = false;

    template <typename T>
    [[nodiscard]] static ColumnOps for_type() noexcept {
        return {
            [](void* p) { ::new (p) T(); },
            [](void* dst, const void* src) { ::new (dst) T(*static_cast<const T*>(src)); },
            [](void* dst, void* src) { ::new (dst) T(std::move(*static_cast<T*>(src))); },
            [](void* p) { static_cast<T*>(p)->~T(); },
            [](void* dst, void* src) { *static_cast<T*>(dst) = std::move(*static_cast<T*>(src)); },
            sizeof(T),
            alignof(T),
            std::is_trivially_copyable_v<T>,
            std::is_trivially_destructible_v<T>,
            std::is_trivially_constructible_v<T>,
        };
    }
};

class ComponentRegistry {
public:
    struct ComponentInfo {
        ComponentTypeId id = 0;
        foundation::size_t size = 0;
        foundation::size_t alignment = 0;
        const char* name = nullptr;
        ColumnOps ops = {};
    };

    template <typename T>
    [[nodiscard]] static ComponentTypeId type_id() noexcept {
        static const ComponentTypeId id = next_id();
        return id;
    }

    template <typename T>
    static void register_component(const char* name = nullptr) {
        ComponentTypeId id = type_id<T>();
        if (id >= infos_.size()) {
            infos_.resize(id + 1);
        }
        infos_[id] = ComponentInfo{id, sizeof(T), alignof(T), name, ColumnOps::for_type<T>()};
    }

    [[nodiscard]] static bool is_registered(ComponentTypeId id) noexcept {
        return id < infos_.size() && infos_[id].id == id && infos_[id].size > 0;
    }

    [[nodiscard]] static const ComponentInfo& get_info(ComponentTypeId id) {
        return infos_[id];
    }

    [[nodiscard]] static foundation::size_t registered_count() noexcept {
        return infos_.size();
    }

private:
    static foundation::u32 next_id_counter_;
    static foundation::DynamicArray<ComponentInfo> infos_;

    static ComponentTypeId next_id() {
        return next_id_counter_++;
    }
};

} // namespace cambyses
