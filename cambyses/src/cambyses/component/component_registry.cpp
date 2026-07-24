#include "cambyses/component/component_registry.hpp"

namespace cambyses {

foundation::u32 ComponentRegistry::next_id_counter_ = 0;
foundation::DynamicArray<ComponentRegistry::ComponentInfo> ComponentRegistry::infos_;

} // namespace cambyses
