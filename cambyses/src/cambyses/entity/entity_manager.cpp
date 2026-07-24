#include "cambyses/entity/entity_manager.hpp"

namespace cambyses {

Entity EntityManager::create() {
    EntityIndex index;

    if (!free_list_.empty()) {
        index = free_list_.back();
        free_list_.pop_back();
    } else {
        index = static_cast<EntityIndex>(generations_.size());
        generations_.push_back(0);
    }

    ++alive_count_;
    return Entity{index, generations_[index]};
}

void EntityManager::destroy(Entity entity) {
    if (!is_valid(entity)) {
        return;
    }
    ++generations_[entity.index];
    free_list_.push_back(entity.index);
    --alive_count_;
}

bool EntityManager::is_valid(Entity entity) const noexcept {
    if (entity.index >= static_cast<EntityIndex>(generations_.size())) {
        return false;
    }
    return generations_[entity.index] == entity.generation;
}

EntityGeneration EntityManager::generation(EntityIndex index) const {
    PREONX_ASSERT(index < static_cast<EntityIndex>(generations_.size()));
    return generations_[index];
}

} // namespace cambyses
