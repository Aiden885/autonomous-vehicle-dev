#include "ApaRecoveryObservation.hpp"

namespace apa {

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
void ApaRecoveryObservation::run(const Input& input, Output& output) {
  /** @brief 车辆阻挡起点。 */
  const Boolean actorStart0 = input.risk == ApaRecoveryRisk::ActorStartConstraint;
  /** @brief 阻挡车辆低于静止速度阈值。 */
  const Boolean lowSpeed0 = input.blockerSpeedMps <= param_.movingSpeedMps;
  /** @brief 低速车辆起点阻挡。 */
  const Boolean stationary0 = actorStart0 && lowSpeed0;
  /** @brief 无接触的预测侵入。 */
  const Boolean predicted0 = input.risk == ApaRecoveryRisk::PredictedIntrusion;
  /** @brief 需要连续观察的动态或预测风险。 */
  const Boolean transient0 = actorStart0 || predicted0;
  /** @brief 自车停稳且阻挡车辆低速的观察条件。 */
  const Boolean observingStationary0 = input.stopped && stationary0;
  /** @brief 阻挡对象连续性。 */
  const Boolean sameBlocker0 = !input.blockerChanged;
  /** @brief 静止观察时钟的单调性。 */
  const Boolean stationaryClockValid0 = input.simTimeSec >= state_.stationarySinceSec;
  /** @brief 同一对象连续停稳且时钟有效。 */
  const Boolean preserveStationary0 = observingStationary0 && state_.observingStationary &&
      sameBlocker0 && stationaryClockValid0;
  /** @brief 连续静止观察起点。 */
  Real64 stationarySince0;
  if (preserveStationary0) {
    stationarySince0 = state_.stationarySinceSec;
  } else {
    stationarySince0 = input.simTimeSec;
  }
  /** @brief 连续静止时长，s。 */
  const Real64 stationaryElapsed0 = input.simTimeSec - stationarySince0;
  /** @brief 静止确认时长达标。 */
  const Boolean stationaryDwellReached0 = stationaryElapsed0 >= param_.stationaryDwellSec;
  /** @brief 停稳观察中的静止阻挡确认。 */
  const Boolean stationaryConfirmed0 = observingStationary0 && stationaryDwellReached0;
  /** @brief 消失确认时钟有效标记。 */
  const Boolean clearClockStarted0 = state_.clearSinceSec >= 0.0;
  /** @brief 消失确认时钟的单调性。 */
  const Boolean clearClockValid0 = input.simTimeSec >= state_.clearSinceSec;
  /** @brief 已启动且单调的消失确认时钟。 */
  const Boolean preserveClearClock0 = clearClockStarted0 && clearClockValid0;
  /** @brief 连续消失确认起点。 */
  Real64 clearStart0;
  if (preserveClearClock0) {
    clearStart0 = state_.clearSinceSec;
  } else {
    clearStart0 = input.simTimeSec;
  }
  /** @brief 风险连续消失时长，s。 */
  const Real64 clearElapsed0 = input.simTimeSec - clearStart0;
  /** @brief 消失确认时长未达标。 */
  const Boolean clearDwellPending0 = clearElapsed0 < param_.clearDwellSec;
  /** @brief 当前无动态或预测风险。 */
  const Boolean noTransient0 = !transient0;
  /** @brief 停稳后的历史风险消失确认条件。 */
  const Boolean confirmClear0 = noTransient0 && input.stopped && state_.obstacleSeen;
  /** @brief 仍待确认消失的风险状态。 */
  Boolean obstacleSeen0;
  /** @brief 消失确认起点；负值表示未计时。 */
  Real64 clearSince0;
  if (transient0) {
    obstacleSeen0 = true;
    clearSince0 = -1.0;
  } else if (confirmClear0) {
    obstacleSeen0 = clearDwellPending0;
    clearSince0 = clearStart0;
  } else {
    obstacleSeen0 = state_.obstacleSeen;
    clearSince0 = -1.0;
  }
  /** @brief 连续静止观察状态。 */
  state_.observingStationary = observingStationary0;
  /** @brief 连续静止观察起点，s。 */
  state_.stationarySinceSec = stationarySince0;
  /** @brief 待消失确认的风险状态。 */
  state_.obstacleSeen = obstacleSeen0;
  /** @brief 消失确认时钟；负值表示未计时。 */
  state_.clearSinceSec = clearSince0;
  /** @brief 当前动态或预测风险。 */
  output.transientRisk = transient0;
  /** @brief 持续静止确认结果。 */
  output.stationaryConfirmed = stationaryConfirmed0;
  /** @brief 风险消失观察要求。 */
  output.waitingForClear = obstacleSeen0;
}

}
