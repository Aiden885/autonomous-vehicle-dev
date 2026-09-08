#include "GlobalContext.hpp"

namespace control {

void GlobalParams::reset() noexcept {
  controlTs = 0.05;
}

void GlobalStates::reset() noexcept {
  feedback = {.lonVelFb = 0.0, .latPosFb = 0.0};
}

namespace global {

GlobalParams params {
  .controlTs = 0.05,
};
GlobalStates states {
  .feedback = {.lonVelFb = 0.0, .latPosFb = 0.0},
};

} // namespace global

} // namespace control
