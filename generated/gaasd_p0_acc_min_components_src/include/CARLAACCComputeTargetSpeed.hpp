#pragma once

#include "FuncModule.hpp"

#include <algorithm>

namespace control {

/**
 * @brief CARLAACCComputeTargetSpeed 模块接口结构集合。
 */
struct CARLAACCComputeTargetSpeedTraits {
  /**
   * @brief 输入端口集合。
   */
  struct Input {
    Real egoV = 0.0;      ///< 自车当前纵向速度 (m/s)
    Real leadV = 0.0;     ///< 前车当前纵向速度 (m/s)
    Real distance = 0.0;  ///< 自车到前车净距离 (m)
  };

  /**
   * @brief 输出端口集合。
   */
  struct Output {
    Real targetSpeed = 0.0;  ///< 限幅后的 ACC 目标速度 (m/s)
  };

  /**
   * @brief 参数集合。
   */
  struct Param {
    Real desiredDistance = 15.0; ///< 期望跟车净距离 (m)
    Real maxSpeed = 5.0;         ///< 目标速度上限 (m/s)
    Real kDist = 0.35;           ///< 距离误差比例增益
    Real kSpeed = 0.8;           ///< 速度差比例增益
  };

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
 * @brief 计算 ACC 目标跟车速度的函数模块。
 */
class CARLAACCComputeTargetSpeed
    : public FuncModule<CARLAACCComputeTargetSpeedTraits> {
public:
  using FuncModule::FuncModule;

  void run(const Input& input, Output& output) override;
};

} // namespace control
