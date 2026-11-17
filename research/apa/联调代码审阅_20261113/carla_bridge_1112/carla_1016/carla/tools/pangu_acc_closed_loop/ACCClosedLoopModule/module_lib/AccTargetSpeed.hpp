#pragma once

#include "FuncModule.hpp"

namespace control {

/**
 * @brief AccTargetSpeed Traits 是模块接口结构集合。
 */
struct AccTargetSpeedTraits {
  /**
   * @brief 输入端口集合，承载本周期输入信号。
   */
  struct Input {
    Real egoSpeed = 0.0; ///< 自车速度 (m/s)。
    Real leadSpeed = 0.0; ///< 前车速度 (m/s)。
    Real leadDistance = 0.0; ///< 前车距离 (m)。
    int commandType = 0; ///< 驾驶指令类型，0表示无指令。
  };

  /**
   * @brief 输出端口集合，承载本周期计算结果。
   */
  struct Output {
    Real targetSpeed = 0.0; ///< 目标速度 (m/s)。
    bool enable = false; ///< ACC使能标志。
    bool valid = false; ///< 计算结果有效标志。
    Real timeGap = 1.8; ///< 当前时距参数 (s)。
    Real maxSpeed = 5.0; ///< 当前最大速度参数 (m/s)。
    int decision = 0; ///< 当前决策编号。
    int systemState = 2; ///< 当前系统状态编号。
  };

  /**
   * @brief 参数集合，承载模块可配置常量。
   */
  struct Param {
    Real initialTimeGap = 1.8; ///< 初始时距 (s)。
    Real minDistance = 5.0; ///< 最小跟车距离 (m)。
    Real distanceGain = 0.25; ///< 距离增益系数。
    Real speedGain = 0.4; ///< 速度增益系数。
    Real initialMaxSpeed = 5.0; ///< 初始最大速度 (m/s)。
    Real vMin = 0.0; ///< 最低接管速度 (m/s)，mock阶段允许静止起步。
    Real timeGapStep = 0.2; ///< 时距调节步长 (s)。
    Real minTimeGap = 1.0; ///< 最小时距 (s)。
    Real maxTimeGap = 3.0; ///< 最大时距 (s)。
    Real speedStep = 1.3889; ///< 速度调节步长 (m/s)，约等于5 km/h。
    Real minSpeed = 0.0; ///< 最低设定速度 (m/s)。
    Real maxSpeedCap = 5.0; ///< 最高设定速度 (m/s)。
  };

  /**
   * @brief 状态集合，承载模块跨周期状态。
   */
  struct State {
    bool initialized = false; ///< 状态是否已经初始化。
    bool controlEnabled = false; ///< ACC内部控制使能状态。
    bool hasHistory = false; ///< 是否存在历史巡航参数。
    Real timeGap = 1.8; ///< 当前时距状态 (s)。
    Real maxSpeed = 5.0; ///< 当前设定最大速度状态 (m/s)。
    int lastDecision = 0; ///< 上一次非零决策编号。
  };

  /**
   * @brief 子模块集合，承载下游可复用算法模块实例。
   */
  struct Sub {
  };
};

class AccTargetSpeed : public FuncModule<AccTargetSpeedTraits> {
public:
  using FuncModule::FuncModule;

  void run(const Input& input, Output& output) override;
};

} // namespace control
