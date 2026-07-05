#include "GlobalContext.hpp"

namespace control {

void GlobalParams::reset() noexcept {
  controlTs = 0.05;
  lks_l0 = 5.0;
  lks_rt = 0.5;
  lks_rAlpha = 0.6666667;
  lks_curvatureThreshold = 0.001;
  lks_nearPreviewDistance = 0.5;
  lks_w1 = 0.2;
  lks_w2 = 0.3;
  lks_w3 = 0.5;
  lks_Kp = 0.08;
  lks_steerScale = 0.6;
  lks_vMin = 1.0;
  lks_driverSteerThreshold = 0.1;
  lks_ayMax = 3.0;
  lks_wheelBase = 2.9;
  lks_frontWheelMaxRad = 0.5236;
}

void GlobalStates::reset() noexcept {
  feedback = {.lonVelFb = 0.0, .latPosFb = 0.0};
}

namespace global {

GlobalParams params {
  .controlTs = 0.05,
  .lks_l0 = 5.0,
  .lks_rt = 0.5,
  .lks_rAlpha = 0.6666667,
  .lks_curvatureThreshold = 0.001,
  .lks_nearPreviewDistance = 0.5,
  .lks_w1 = 0.2,
  .lks_w2 = 0.3,
  .lks_w3 = 0.5,
  .lks_Kp = 0.08,
  .lks_steerScale = 0.6,
  .lks_vMin = 1.0,
  .lks_driverSteerThreshold = 0.1,
  .lks_ayMax = 3.0,
  .lks_wheelBase = 2.9,
  .lks_frontWheelMaxRad = 0.5236,
};
GlobalStates states {
  .feedback = {.lonVelFb = 0.0, .latPosFb = 0.0},
};

} // namespace global

} // namespace control
