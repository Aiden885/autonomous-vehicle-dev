#pragma once

#include "FuncModule.hpp"
#include "ApaRecoveryDecisionTypes.hpp"

namespace apa {

/** @brief ApaRecoveryObservationTraits 是模块接口结构集合。 */
struct ApaRecoveryObservationTraits {
  /** @brief 输入端口集合。 */
  struct Input {
    ApaRecoveryRisk risk = ApaRecoveryRisk::Clear; ///< 当前起点或预测风险分类。
    Boolean stopped = false; ///< 同一恢复编号已确认停稳。
    Boolean blockerChanged = false; ///< 阻挡对象或其有效几何发生变化。
    Real64 blockerSpeedMps = 0.0; ///< 当前阻挡对象的平面速度大小，单位 m/s。
    Real64 simTimeSec = 0.0; ///< 当前有效反馈的仿真时间，单位 s。
  };
  /** @brief 输出端口集合。 */
  struct Output {
    Boolean transientRisk = false; ///< 存在动态或尚未确认静止的起点风险。
    Boolean stationaryConfirmed = false; ///< 同一阻挡对象已连续静止到阈值。
    Boolean waitingForClear = false; ///< 风险仍在或消失确认时间尚未满足。
  };
  /** @brief 模块局部参数集合。 */
  struct Param {
    Real64 stationaryDwellSec = 0.5; ///< 起点障碍连续静止确认时间，单位 s。
    Real64 clearDwellSec = 0.2; ///< 动态风险消失后的连续确认时间，单位 s。
    Real64 movingSpeedMps = 0.05; ///< 障碍运动判定速度，单位 m/s。
  };
  /** @brief 模块局部跨周期状态集合。 */
  struct State {
    Boolean observingStationary = false; ///< 是否正在连续确认当前阻挡对象静止。
    Boolean obstacleSeen = false; ///< 是否仍需完成本次障碍的消失确认。
    Real64 stationarySinceSec = 0.0; ///< 静止确认开始的仿真时间，单位 s。
    Real64 clearSinceSec = -1.0; ///< 风险消失确认开始的仿真时间，单位 s。
  };
  /** @brief 子模块集合。 */
  struct Sub {};
};

/** @brief 在连续停稳反馈上确认障碍静止及预测风险消失。 */
class ApaRecoveryObservation : public FuncModule<ApaRecoveryObservationTraits> {
 public:
  using FuncModule::FuncModule;
  /**
   * @brief 用仿真时间确认障碍静止和风险消失，不推进仿真或执行制动。
   * @cn_name APA恢复障碍观察
   * @type block
   * @tag_level0 自动泊车
   * @tag_level1 恢复
   * @tag_level2 障碍观察
   * @version 1.0
   * @date 2026-11-03
   * @author wang
   */
  void run(const Input& input, Output& output) override;
};

}
