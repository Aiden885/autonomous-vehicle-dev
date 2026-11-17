#include "ApaRecoveryDecision.hpp"

namespace apa {

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
void ApaRecoveryDecision::run(const Input& input, Output& output) {
  /** @brief 观察模块输入：风险、停稳状态和仿真时间。 */
  ApaRecoveryObservationTraits::Input observation_in{};
  observation_in.risk = input.risk;
  observation_in.stopped = input.stopped;
  observation_in.blockerChanged = input.blockerChanged;
  observation_in.blockerSpeedMps = input.blockerSpeedMps;
  observation_in.simTimeSec = input.simTimeSec;
  ApaRecoveryObservationTraits::Output observation_out{};
  /** @brief 连续静止及风险消失确认。 */
  sub_.observation.run(observation_in, observation_out);

  /** @brief 恢复时间预算耗尽。 */
  const Boolean timedOut0 = input.remainingSec <= 0.0;
  /** @brief 停稳前制动保持条件。 */
  const Boolean notStopped0 = !input.stopped;
  /** @brief 虚拟约束起点无效。 */
  const Boolean virtualStart0 = input.risk == ApaRecoveryRisk::VirtualStartConstraint;
  /** @brief 同任务安全历史证书缺失。 */
  const Boolean noEscape0 = !input.escapeAvailable;
  /** @brief 缺少证书的虚拟约束脱困禁用条件。 */
  const Boolean virtualStartBlocked0 = virtualStart0 && noEscape0;
  /** @brief 固定约束起点无效。 */
  const Boolean fixedStart0 = input.risk == ApaRecoveryRisk::FixedStartConstraint;
  /** @brief 真实车身与实体重叠。 */
  const Boolean physicalOverlap0 = input.risk == ApaRecoveryRisk::PhysicalOverlap;
  /** @brief 安全结果无效。 */
  const Boolean invalidSafety0 = input.risk == ApaRecoveryRisk::Invalid;
  /** @brief 起点及有效场景未变。 */
  const Boolean noProblemChange0 = !input.problemChanged;
  /** @brief 候选失败且条件未变。 */
  const Boolean unchangedFailure0 = input.waitForSceneChange && noProblemChange0;
  /** @brief 剩余时间达到规划预留边界。 */
  const Boolean noPlanningReserve0 = input.remainingSec <= param_.planningReserveSec;
  /** @brief 动态风险等待预算耗尽。 */
  const Boolean obstacleWaitExpired0 = observation_out.transientRisk && noPlanningReserve0;
  /** @brief 规划次数耗尽。 */
  const Boolean noAttempts0 = !input.attemptsAvailable;
  /** @brief 无运行中的规划进程。 */
  const Boolean noWorker0 = !input.plannerRunning;
  /** @brief 无待消失确认的风险。 */
  const Boolean noObservation0 = !observation_out.waitingForClear;
  /** @brief 次数耗尽且无进行中的规划或观察。 */
  const Boolean attemptsExhausted0 = noAttempts0 && noWorker0 && noObservation0;
  /** @brief 重试间隔未到。 */
  const Boolean retryDelayed0 = !input.retryDelayElapsed;
  /** @brief 重试延迟或场景未变的等待条件。 */
  const Boolean waitForScene0 = retryDelayed0 || unchangedFailure0;
  /** @brief 按优先级选择的唯一失败原因。 */
  ApaRecoveryFailure failure0;
  if (timedOut0) {
    if (observation_out.waitingForClear) {
      failure0 = ApaRecoveryFailure::ObstacleWaitTimeout;
    } else if (unchangedFailure0) {
      failure0 = ApaRecoveryFailure::UnchangedProblem;
    } else {
      failure0 = ApaRecoveryFailure::RecoveryTimeout;
    }
  } else if (notStopped0) {
    failure0 = ApaRecoveryFailure::None;
  } else if (virtualStartBlocked0) {
    failure0 = ApaRecoveryFailure::VirtualStartRequiresEscape;
  } else if (fixedStart0) {
    failure0 = ApaRecoveryFailure::FixedStartBlocked;
  } else if (physicalOverlap0) {
    failure0 = ApaRecoveryFailure::PhysicalOverlap;
  } else if (invalidSafety0) {
    failure0 = ApaRecoveryFailure::InvalidSafety;
  } else if (observation_out.stationaryConfirmed) {
    failure0 = ApaRecoveryFailure::StationaryStartBlocked;
  } else if (obstacleWaitExpired0) {
    failure0 = ApaRecoveryFailure::ObstacleWaitTimeout;
  } else if (attemptsExhausted0) {
    failure0 = ApaRecoveryFailure::ReplanExhausted;
  } else {
    failure0 = ApaRecoveryFailure::None;
  }
  /** @brief 终止恢复条件。 */
  const Boolean failed0 = failure0 != ApaRecoveryFailure::None;
  /** @brief 由失败、停稳、观察和规划状态确定的唯一动作。 */
  ApaRecoveryAction action0;
  if (failed0) {
    action0 = ApaRecoveryAction::Fail;
  } else if (notStopped0) {
    action0 = ApaRecoveryAction::Hold;
  } else if (observation_out.waitingForClear) {
    action0 = ApaRecoveryAction::Observe;
  } else if (input.plannerRunning) {
    action0 = ApaRecoveryAction::PollPlanner;
  } else if (waitForScene0) {
    action0 = ApaRecoveryAction::WaitForScene;
  } else {
    action0 = ApaRecoveryAction::Plan;
  }
  /** @brief 本周期允许的恢复动作。 */
  output.action = action0;
  /** @brief 恢复终止原因；正常为 None。 */
  output.failure = failure0;
}

}
