#pragma once

#include "FuncModule.hpp"

namespace control {

// CARLA 适配器边界接口（项目自有适配库，非第三方开源依赖），C ABI，按值传参。
extern "C" int carla_adapter_publish_control_cmd(
    double targetSpeed,
    double targetAccel,
    double steerRad,
    int enable);

/**
 * @brief CARLAControlCmd 模块接口结构集合。
 */
struct CARLAControlCmdTraits {
  /**
   * @brief 输入端口集合。
   */
  struct Input {
    Real targetSpeed = 0.0;  ///< 目标速度 (m/s)
    Real targetAccel = 0.0;  ///< 目标加速度 (m/s^2)
    Real steerRad = 0.0;     ///< 目标转向角 (rad)
    int enable = 0;          ///< 控制使能，1 表示使能
  };

  /**
   * @brief 输出端口集合。
   */
  struct Output {
    int status = 0;  ///< 发布状态，0 表示成功
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
 * @brief 发布 CARLA 横纵向联合控制命令的边界适配函数模块。
 */
class CARLAControlCmd : public FuncModule<CARLAControlCmdTraits> {
public:
  using FuncModule::FuncModule;

  void run(const Input& input, Output& output) override;
};

} // namespace control
