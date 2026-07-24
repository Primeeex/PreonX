#include "cambyses/serialization/serialization.hpp"

namespace cambyses {

foundation::DynamicArray<ComponentMetadata> SerializationRegistry::metadata_;

void SerializationRegistry::clear() {
    metadata_.clear();
}

} // namespace cambyses
