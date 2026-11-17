#include "ApaParkingRouteSelection.hpp"

namespace apa {

/**
 * @brief 所有泊车业务经同一个 C 目标解析及候选验收入口。
 * @cn_name 泊车业务路由
 * @type block
 * @tag_level0 自动泊车
 * @tag_level1 规划
 * @tag_level2 泊车业务路由
 * @version 1.1
 * @date 2026-11-03
 * @author wang
 */
void ApaParkingRouteSelection::run(const Input& input, Output& output) {
  /** @brief 统一车位规划输入。 */
  ApaParkingSlotPathPlanningTraits::Input slotPathPlanning_in{};
  slotPathPlanning_in.plannerInput.sceneQueryId = input.plannerInput.sceneQueryId;
  slotPathPlanning_in.plannerInput.slotState = input.plannerInput.slotState;
  slotPathPlanning_in.plannerInput.parkingAction = input.plannerInput.parkingAction;
  slotPathPlanning_in.plannerInput.targetSlotKind = input.plannerInput.targetSlotKind;
  slotPathPlanning_in.plannerInput.vehicleX = input.plannerInput.vehicleX;
  slotPathPlanning_in.plannerInput.vehicleY = input.plannerInput.vehicleY;
  slotPathPlanning_in.plannerInput.vehicleYaw = input.plannerInput.vehicleYaw;
  slotPathPlanning_in.plannerInput.targetX = input.plannerInput.targetX;
  slotPathPlanning_in.plannerInput.targetY = input.plannerInput.targetY;
  slotPathPlanning_in.plannerInput.targetTheta = input.plannerInput.targetTheta;
  /** @brief 规划与控制共用的位置容差，m。 */
  slotPathPlanning_in.plannerInput.goalPositionTolerance = input.plannerInput.goalPositionTolerance;
  /** @brief 规划与控制共用的航向容差，rad。 */
  slotPathPlanning_in.plannerInput.goalYawTolerance = input.plannerInput.goalYawTolerance;
  /** @brief 车型半径和车位几何。 */
  slotPathPlanning_in.plannerInput.minimumTurningRadius = input.plannerInput.minimumTurningRadius;
  slotPathPlanning_in.plannerInput.entrancePointX = input.plannerInput.entrancePointX;
  slotPathPlanning_in.plannerInput.entrancePointY = input.plannerInput.entrancePointY;
  slotPathPlanning_in.plannerInput.slotPoint1X = input.plannerInput.slotPoint1X;
  slotPathPlanning_in.plannerInput.slotPoint1Y = input.plannerInput.slotPoint1Y;
  slotPathPlanning_in.plannerInput.slotPoint2X = input.plannerInput.slotPoint2X;
  slotPathPlanning_in.plannerInput.slotPoint2Y = input.plannerInput.slotPoint2Y;
  slotPathPlanning_in.plannerInput.slotPoint3X = input.plannerInput.slotPoint3X;
  slotPathPlanning_in.plannerInput.slotPoint3Y = input.plannerInput.slotPoint3Y;
  slotPathPlanning_in.plannerInput.slotPoint4X = input.plannerInput.slotPoint4X;
  slotPathPlanning_in.plannerInput.slotPoint4Y = input.plannerInput.slotPoint4Y;
  slotPathPlanning_in.plannerInput.slotEntrance = input.plannerInput.slotEntrance;
  slotPathPlanning_in.plannerInput.parkingType = input.plannerInput.parkingType;
  slotPathPlanning_in.plannerInput.surroundingType = input.plannerInput.surroundingType;
  slotPathPlanning_in.plannerInput.availableSpaceOffset = input.plannerInput.availableSpaceOffset;
  slotPathPlanning_in.plannerInput.boundaryLineCount = input.plannerInput.boundaryLineCount;
  slotPathPlanning_in.plannerInput.virtualBoundaryLineCount = input.plannerInput.virtualBoundaryLineCount;
  slotPathPlanning_in.plannerInput.slotTrackStopperCount = input.plannerInput.slotTrackStopperCount;
  slotPathPlanning_in.plannerInput.boundarySepPointCount = input.plannerInput.boundarySepPointCount;
  slotPathPlanning_in.plannerInput.stopperPointCount = input.plannerInput.stopperPointCount;
  slotPathPlanning_in.plannerInput.shortcutSwitch = input.plannerInput.shortcutSwitch;
  slotPathPlanning_in.plannerInput.shortcutThreshold = input.plannerInput.shortcutThreshold;
  slotPathPlanning_in.plannerInput.needHybridAlgorithmFusion = input.plannerInput.needHybridAlgorithmFusion;
  /** @brief 完整的原生边界、虚拟线和挡停线。 */
  for (Integer lineIndex0 = 0; lineIndex0 < 64; lineIndex0 = lineIndex0 + 1) {
    slotPathPlanning_in.plannerInput.boundaryLineX1[lineIndex0] = input.plannerInput.boundaryLineX1[lineIndex0];
    slotPathPlanning_in.plannerInput.boundaryLineY1[lineIndex0] = input.plannerInput.boundaryLineY1[lineIndex0];
    slotPathPlanning_in.plannerInput.boundaryLineX2[lineIndex0] = input.plannerInput.boundaryLineX2[lineIndex0];
    slotPathPlanning_in.plannerInput.boundaryLineY2[lineIndex0] = input.plannerInput.boundaryLineY2[lineIndex0];
    slotPathPlanning_in.plannerInput.virtualBoundaryPoint1X[lineIndex0] = input.plannerInput.virtualBoundaryPoint1X[lineIndex0];
    slotPathPlanning_in.plannerInput.virtualBoundaryPoint1Y[lineIndex0] = input.plannerInput.virtualBoundaryPoint1Y[lineIndex0];
    slotPathPlanning_in.plannerInput.virtualBoundaryPoint2X[lineIndex0] = input.plannerInput.virtualBoundaryPoint2X[lineIndex0];
    slotPathPlanning_in.plannerInput.virtualBoundaryPoint2Y[lineIndex0] = input.plannerInput.virtualBoundaryPoint2Y[lineIndex0];
    slotPathPlanning_in.plannerInput.slotTrackStopperX1[lineIndex0] = input.plannerInput.slotTrackStopperX1[lineIndex0];
    slotPathPlanning_in.plannerInput.slotTrackStopperY1[lineIndex0] = input.plannerInput.slotTrackStopperY1[lineIndex0];
    slotPathPlanning_in.plannerInput.slotTrackStopperX2[lineIndex0] = input.plannerInput.slotTrackStopperX2[lineIndex0];
    slotPathPlanning_in.plannerInput.slotTrackStopperY2[lineIndex0] = input.plannerInput.slotTrackStopperY2[lineIndex0];
  }
  /** @brief 离散障碍点和独立挡停点。 */
  for (Integer pointIndex0 = 0; pointIndex0 < 128; pointIndex0 = pointIndex0 + 1) {
    slotPathPlanning_in.plannerInput.boundarySepPointX[pointIndex0] = input.plannerInput.boundarySepPointX[pointIndex0];
    slotPathPlanning_in.plannerInput.boundarySepPointY[pointIndex0] = input.plannerInput.boundarySepPointY[pointIndex0];
    slotPathPlanning_in.plannerInput.stopperPointX[pointIndex0] = input.plannerInput.stopperPointX[pointIndex0];
    slotPathPlanning_in.plannerInput.stopperPointY[pointIndex0] = input.plannerInput.stopperPointY[pointIndex0];
  }
  /** @brief 统一 C 目标解析及候选验收。 */
  ApaParkingSlotPathPlanningTraits::Output slotPathPlanning_out{};
  sub_.slotPathPlanning.run(slotPathPlanning_in, slotPathPlanning_out);
  /** @brief 原生规划状态、诊断和解析场景。 */
  output.candidatePath.selectedSlotKind = slotPathPlanning_out.candidatePath.selectedSlotKind;
  output.candidatePath.planFlag = slotPathPlanning_out.candidatePath.planFlag;
  output.candidatePath.errorType = slotPathPlanning_out.candidatePath.errorType;
  output.candidatePath.plannerCase = slotPathPlanning_out.candidatePath.plannerCase;
  output.candidatePath.segmentCount = slotPathPlanning_out.candidatePath.segmentCount;
  output.candidatePath.directionChangeCount = slotPathPlanning_out.candidatePath.directionChangeCount;
  output.candidatePath.pathLength = slotPathPlanning_out.candidatePath.pathLength;
  output.candidatePath.pathPointCount = slotPathPlanning_out.candidatePath.pathPointCount;
  output.candidatePath.maneuverStepCount = slotPathPlanning_out.candidatePath.maneuverStepCount;
  output.candidatePath.secondStageEndPointOrdinal = slotPathPlanning_out.candidatePath.secondStageEndPointOrdinal;
  output.candidatePath.firstForwardPointVehicleX = slotPathPlanning_out.candidatePath.firstForwardPointVehicleX;
  output.candidatePath.firstForwardPointVehicleY = slotPathPlanning_out.candidatePath.firstForwardPointVehicleY;
  output.candidatePath.firstForwardPointVehicleTheta = slotPathPlanning_out.candidatePath.firstForwardPointVehicleTheta;
  output.candidatePath.planOk = slotPathPlanning_out.candidatePath.planOk;
  output.candidatePath.resolvedScene = slotPathPlanning_out.candidatePath.resolvedScene;
  /** @brief 100 点路径位姿及有符号步长。 */
  for (Integer pathIndex0 = 0; pathIndex0 < 100; pathIndex0 = pathIndex0 + 1) {
    output.candidatePath.pathPointX[pathIndex0] = slotPathPlanning_out.candidatePath.pathPointX[pathIndex0];
    output.candidatePath.pathPointY[pathIndex0] = slotPathPlanning_out.candidatePath.pathPointY[pathIndex0];
    output.candidatePath.pathPointTheta[pathIndex0] = slotPathPlanning_out.candidatePath.pathPointTheta[pathIndex0];
    output.candidatePath.pathPointOdometer[pathIndex0] = slotPathPlanning_out.candidatePath.pathPointOdometer[pathIndex0];
  }
  /** @brief 已解析业务请求。 */
  output.resolvedInput = slotPathPlanning_out.resolvedInput;
}

}
