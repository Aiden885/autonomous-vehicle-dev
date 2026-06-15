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
 * @brief CARLALKSValid 模块接口结构集合。
 */
struct CARLALKSValidTraits {
  /**
   * @brief 输入端口集合。
   */
  struct Input {};

  /**
   * @brief 输出端口集合。
   */
  struct Output {
    int valid = 0;  ///< 车道跟踪数据有效标志，1 表示有效
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
 * @brief 读取 CARLA 车道跟踪数据是否有效的边界适配函数模块。
 */
class CARLALKSValid : public FuncModule<CARLALKSValidTraits> {
public:
  using FuncModule::FuncModule;

  void run(const Input& input, Output& output) override;
};

} // namespace control
