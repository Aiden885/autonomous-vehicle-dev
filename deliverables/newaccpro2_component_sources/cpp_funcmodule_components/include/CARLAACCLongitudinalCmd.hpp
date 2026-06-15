#pragma once

#include "FuncModule.hpp"

namespace control {

/**
 * @brief CARLAACCLongitudinalCmd 模块接口结构集合。
 */
struct CARLAACCLongitudinalCmdTraits {
  /**
   * @brief 输入端口集合。
   */
  struct Input {
    Real speed = 0.0;  ///< 目标纵向速度 (m/s)
    int enable = 0;    ///< 控制使能，1 表示使能纵向控制
  };

  /**
   * @brief 输出端口集合。
   */
  struct Output {
    Real speed = 0.0; ///< 运行时边界层待发布的目标纵向速度 (m/s)
    int enable = 0;   ///< 运行时边界层待发布的控制使能
    int valid = 0;    ///< 纵向控制命令有效标志，1 表示有效
  };

  /**
   * @brief 参数集合。
   */
  struct Param {};

  /**
   * @brief 状态集合。
   */
  struct State {};

  /**
   * @brief 子模块集合。
   */
  struct Sub {};
};

/**
 * @brief 输出供运行时边界层发布的 CARLA 纵向速度控制命令。
 */
class CARLAACCLongitudinalCmd : public FuncModule<CARLAACCLongitudinalCmdTraits> {
public:
  using FuncModule::FuncModule;

  void run(const Input& input, Output& output) override;
};

} // namespace control
