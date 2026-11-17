#pragma once

#include "FuncModule.hpp"
#include "ApaPlanningCalibration.hpp"

namespace apa {

/** @brief 多个规划模块共同读取的工程标定，运行前配置。 */
struct GlobalParams {
  ApaPlanningCalibration planning{}; ///< 共享车辆参数与规划标定，由同一车辆配置生成默认值。
};

/** @brief 全局状态集合；恢复记忆属于观察模块，不在此共享。 */
struct GlobalStates {
  void reset() noexcept;
};

namespace global {
extern GlobalParams params;
extern GlobalStates states;
}

}
