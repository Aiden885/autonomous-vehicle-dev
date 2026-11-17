#pragma once

#include "FuncModule.hpp"
#include "ApaRecoveryDecisionTypes.hpp"
#include "ApaRecoveryObservation.hpp"

namespace apa {

/** @brief ApaRecoveryDecisionTraits 是模块接口结构集合。 */
struct ApaRecoveryDecisionTraits {
  /** @brief 输入端口集合。 */
  struct Input {
    ApaRecoveryRisk risk = ApaRecoveryRisk::Clear; ///< 当前起点或预测风险分类。
    Boolean stopped = false; ///< 同一恢复编号已确认停稳。
    Boolean blockerChanged = false; ///< 阻挡对象或其有效几何发生变化。
    Real64 blockerSpeedMps = 0.0; ///< 当前阻挡对象的平面速度大小，单位 m/s。
    Boolean plannerRunning = false; ///< 当前已有一个规划 worker。
    Boolean waitForSceneChange = false; ///< 上次候选失败后等待规划条件变化。
    Boolean problemChanged = true; ///< 起点或场景相对上次尝试已改变。
    Boolean retryDelayElapsed = true; ///< 真实时钟重试间隔已满足。
    Boolean attemptsAvailable = true; ///< 当前任务仍有规划次数。
    Boolean escapeAvailable = false; ///< 具备受约束净距恢复条件，路径由规划器验收。
    Real64 simTimeSec = 0.0; ///< 当前有效反馈的仿真时间，单位 s。
    Real64 remainingSec = 0.0; ///< 恢复总预算剩余真实时间，单位 s。
  };
  /** @brief 输出端口集合。 */
  struct Output {
    ApaRecoveryAction action = ApaRecoveryAction::Hold; ///< 节点本轮应执行的唯一动作。
    ApaRecoveryFailure failure = ApaRecoveryFailure::None; ///< 终止原因，无故障时为 None。
  };
  /** @brief 模块局部参数集合。 */
  struct Param {
    Real64 planningReserveSec = 5.0; ///< 障碍等待为下一次规划保留的真实时间，单位 s。
  };
  /** @brief 模块局部跨周期状态集合。 */
  struct State {};
  /** @brief 子模块集合。 */
  struct Sub {
    ApaRecoveryObservation observation; ///< 连续反馈的障碍观察和消失确认。
  };
};

/** @brief 根据停稳、风险和预算选择恢复动作，不执行通信或规划。 */
class ApaRecoveryDecision : public FuncModule<ApaRecoveryDecisionTraits> {
 public:
  ApaRecoveryDecision(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                ApaRecoveryObservation(
                    ApaRecoveryObservationTraits::Param{
                        .stationaryDwellSec = 0.5,
                        .clearDwellSec = 0.2,
                        .movingSpeedMps = 0.05},
                    ApaRecoveryObservationTraits::State{
                        .observingStationary = false,
                        .obstacleSeen = false,
                        .stationarySinceSec = 0.0,
                        .clearSinceSec = -1.0})},
            std::move(param),
            std::move(state)} {}
  /**
   * @brief 停稳后区分重规划、障碍观察和不可行起点，并限制重复计算。
   * @cn_name APA恢复动作决策
   * @type block
   * @tag_level0 自动泊车
   * @tag_level1 恢复
   * @tag_level2 动作决策
   * @version 1.0
   * @date 2026-11-03
   * @author wang
   */
  void run(const Input& input, Output& output) override;
};

}
