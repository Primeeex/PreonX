#pragma once

#include "cambyses/core/types.hpp"
#include "foundation/containers/dynamic_array.hpp"

namespace cambyses {

class EntityManager {
public:
    [[nodiscard]] Entity create();
    void destroy(Entity entity);
    [[nodiscard]] bool is_valid(Entity entity) const noexcept;
    [[nodiscard]] EntityGeneration generation(EntityIndex index) const;
    [[nodiscard]] foundation::u32 alive_count() const noexcept { return alive_count_; }
    [[nodiscard]] foundation::u32 slot_count() const noexcept {
        return static_cast<foundation::u32>(generations_.size());
    }

private:
    foundation::DynamicArray<EntityGeneration> generations_;
    foundation::DynamicArray<EntityIndex> free_list_;
    foundation::u32 alive_count_ = 0;
};

} // namespace cambyses
