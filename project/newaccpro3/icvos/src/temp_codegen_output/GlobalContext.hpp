#pragma once

#include "FuncModule.hpp"

namespace control {

struct GlobalParams {
    Real MinDistance = 5.0;
    Real Kdist = 0.25;
    Real Kspeed = 0.4;
};

namespace global {
inline GlobalParams params{};
} // namespace global

} // namespace control
