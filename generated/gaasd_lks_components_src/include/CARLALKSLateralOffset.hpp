#pragma once

#include "FuncModule.hpp"

namespace control {

// CARLA 适配器边界接口（项目自有适配库，非第三方开源依赖），C ABI。
extern "C" int carla_adapter_read_lane_tracking(
    double* lateralOffset,
    double* headingError,
    int* laneId,
    int* roadId,
    int* valid);

/**
 * @brief CARLALKSLateralOffset 模块接口结构集合。
 */
struct CARLALKSLateralOffsetTraits {
  /**
   * @brief 输入端口集合。
   */
  struct Input {};

  /**
   * @brief 输出端口集合。
   */
  struct Output {
    Real lateralOffset = 0.0;  ///< 横向偏差 (m)，正值表示车道中心位于自车右侧
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
 * @brief 读取 CARLA 自车相对车道中心横向偏差的边界适配函数模块。
 */
class CARLALKSLateralOffset : public FuncModule<CARLALKSLateralOffsetTraits> {
public:
  using FuncModule::FuncModule;

  void run(const Input& input, Output& output) override;
};

} // namespace control
