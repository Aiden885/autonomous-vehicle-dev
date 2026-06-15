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
 * @brief CARLALKSHeadingError 模块接口结构集合。
 */
struct CARLALKSHeadingErrorTraits {
  /**
   * @brief 输入端口集合。
   */
  struct Input {};

  /**
   * @brief 输出端口集合。
   */
  struct Output {
    Real headingError = 0.0;  ///< 航向误差 (rad)，正值表示需向右修正
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
 * @brief 读取 CARLA 自车相对车道切线方向航向误差的边界适配函数模块。
 */
class CARLALKSHeadingError : public FuncModule<CARLALKSHeadingErrorTraits> {
public:
  using FuncModule::FuncModule;

  void run(const Input& input, Output& output) override;
};

} // namespace control
