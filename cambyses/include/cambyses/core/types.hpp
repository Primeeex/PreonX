#pragma once

#include "foundation/core/types.hpp"

#include <cstdint>

namespace cambyses {

using EntityIndex = foundation::u32;
using EntityGeneration = foundation::u32;
using ComponentTypeId = foundation::u32;

struct Entity {
    EntityIndex index = 0;
    EntityGeneration generation = 0;

    bool operator==(const Entity& other) const noexcept = default;
    bool operator!=(const Entity& other) const noexcept = default;
};

inline constexpr Entity kNullEntity = {0, 0};

} // namespace cambyses
