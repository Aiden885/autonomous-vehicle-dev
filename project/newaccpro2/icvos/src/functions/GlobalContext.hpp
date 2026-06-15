#pragma once

#include "FuncModule.hpp"
#include "GlobalContextTypes.hpp"

namespace control {

/**
 * @brief GlobalParams 结构体
 */
struct GlobalParams {
  Real MinDistance = 5;
  Real Kdist = 0.9;
  Real Kspeed = 0.8;

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
