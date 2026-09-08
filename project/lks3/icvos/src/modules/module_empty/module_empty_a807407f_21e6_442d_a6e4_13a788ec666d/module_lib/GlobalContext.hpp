#pragma once

#include "FuncModule.hpp"
#include "GlobalContextTypes.hpp"

namespace control {

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
