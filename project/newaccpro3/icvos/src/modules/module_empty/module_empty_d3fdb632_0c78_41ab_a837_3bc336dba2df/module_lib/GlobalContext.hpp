#pragma once

#include "FuncModule.hpp"
#include "GlobalContextTypes.hpp"

namespace control {

/**
 * @brief GlobalParams 结构体
 */
struct GlobalParams {
  Real controlTs = 0.05;

  void reset() noexcept;
};

/**
 * @brief GlobalStates 结构体
 */
struct GlobalStates {
  FeedbackState feedback = {.lonVelFb = 0.0, .latPosFb = 0.0};

  void reset() noexcept;
};

namespace global {

extern GlobalParams params;
extern GlobalStates states;

} // namespace global

} // namespace control
