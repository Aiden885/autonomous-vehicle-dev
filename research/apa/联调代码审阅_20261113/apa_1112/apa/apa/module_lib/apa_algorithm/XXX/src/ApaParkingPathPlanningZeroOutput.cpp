#include "ApaParkingPathPlanningZeroOutput.hpp"

namespace apa {

/**
 * @brief 按 V5 apaParkingPathPlanningZeroOutput 语义初始化全部输出字段。
 * @cn_name 泊车规划输出初始化
 * @type block
 * @tag_level0 自动泊车
 * @tag_level1 规划
 * @tag_level2 输出初始化
 * @version 1.0
 * @date 2026-09-06
 * @author aide
 */
void ApaParkingPathPlanningZeroOutput::run(
    [[maybe_unused]] const Input& input,
    Output& output) {
  /** @brief 失败默认值和零几何量。 */
  const Integer selectedSlotKind0 = 0;
  const Integer planFlag0 = 0;
  const Boolean planOk0 = false;
  const Integer unknownErrorType0 = 31;
  const Integer plannerCase0 = 0;
  const Integer segmentCount0 = 0;
  const Integer directionChangeCount0 = 0;
  const Real pathLength0 = 0.0;
  const Integer pathPointCount0 = 0;
  const Integer maneuverStepCount0 = 0;
  const Integer secondStageEndPointOrdinal0 = 0;
  const Real firstForwardPointVehicleX0 = 0.0;
  const Real firstForwardPointVehicleY0 = 0.0;
  const Real firstForwardPointVehicleTheta0 = 0.0;

  /** @brief 默认失败状态及零路径统计。 */
  output.candidatePath.selectedSlotKind = selectedSlotKind0;
  output.candidatePath.planFlag = planFlag0;
  output.candidatePath.planOk = planOk0;
  output.candidatePath.errorType = unknownErrorType0;
  output.candidatePath.plannerCase = plannerCase0;
  output.candidatePath.segmentCount = segmentCount0;
  output.candidatePath.directionChangeCount = directionChangeCount0;
  output.candidatePath.pathLength = pathLength0;
  output.candidatePath.pathPointCount = pathPointCount0;
  output.candidatePath.maneuverStepCount = maneuverStepCount0;
  output.candidatePath.secondStageEndPointOrdinal = secondStageEndPointOrdinal0;
  output.candidatePath.firstForwardPointVehicleX = firstForwardPointVehicleX0;
  output.candidatePath.firstForwardPointVehicleY = firstForwardPointVehicleY0;
  output.candidatePath.firstForwardPointVehicleTheta = firstForwardPointVehicleTheta0;

  /** @brief 零路径位姿及有符号步长。 */
  Integer pathIndex0 = 0;
  while (pathIndex0 < 100) {
    output.candidatePath.pathPointX[pathIndex0] = 0.0;
    output.candidatePath.pathPointY[pathIndex0] = 0.0;
    output.candidatePath.pathPointTheta[pathIndex0] = 0.0;
    output.candidatePath.pathPointOdometer[pathIndex0] = 0.0;
    /** @brief 路径数组清零索引。 */
    const Integer pathIndexNext = pathIndex0 + 1;
    pathIndex0 = pathIndexNext;
  }
}

}
