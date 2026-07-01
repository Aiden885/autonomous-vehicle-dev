#pragma once

#include "FuncModule.hpp"
#include "GlobalContextTypes.hpp"

namespace control {

/**
 * @brief GlobalParams 结构体
 */
struct GlobalParams {
  Real lks_l0 = 1;
  Real lks_rt = 0.5;
  Real lks_rAlpha = 0.6666667;
  Real lks_curvatureThreshold = 0.001;
  Real lks_nearPreviewDistance = 0.5;
  Real lks_w1 = 0.2;
  Real lks_w2 = 0.3;
  Real lks_w3 = 0.5;
  Real lks_Kp = 0.025;
  Real lks_steerScale = 0.6;
  Real lks_vMin = 5;
  Real lks_driverSteerThreshold = 0.1;
  Real lks_ayMax = 3;
  Real lks_wheelBase = 2.9;
  Real lks_frontWheelMaxRad = 0.5236;

  void reset() noexcept;
};

/**
 * @brief GlobalStates 结构体
 */
struct GlobalStates {

  void reset() noexcept;
};

namespace global {

extern GlobalParams params;
extern GlobalStates states;

} // namespace global

} // namespace control
