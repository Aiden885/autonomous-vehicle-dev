#include "GlobalContext.hpp"

namespace control {

void GlobalParams::reset() noexcept {
  MinDistance = 5;
  Kdist = 0.9;
  Kspeed = 0.8;
}

void GlobalStates::reset() noexcept {
  (void)0;
}

namespace global {

GlobalParams params {
  .MinDistance = 5,
  .Kdist = 0.9,
  .Kspeed = 0.8,
};
GlobalStates states{};

} // namespace global

} // namespace control
