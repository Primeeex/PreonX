#pragma once

#include "cambyses/core/types.hpp"
#include <functional>

namespace cambyses {

class World;

using SystemFunction = std::function<void(World&)>;

} // namespace cambyses
