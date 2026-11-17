#include "ApaParkingPathPlanning.hpp"

namespace apa {

/**
 * @brief 经统一 C 入口规划、连接及验收，再逐字段发布路径。
 * @cn_name APA泊车路径规划
 * @type block
 * @tag_level0 自动泊车
 * @tag_level1 规划
 * @tag_level2 APA泊车路径规划
 * @version 1.1
 * @date 2026-11-03
 * @author wang
 */
void ApaParkingPathPlanning::run(const Input& input, Output& output) {
  /** @brief 业务路由输入。 */
  ApaParkingRouteSelectionTraits::Input routeSelection_in{};
  routeSelection_in.plannerInput = input.plannerInput;
  ApaParkingRouteSelectionTraits::Output routeSelection_out{};
  sub_.routeSelection.run(routeSelection_in, routeSelection_out);
  /** @brief 原生规划状态、诊断和解析场景。 */
  output.candidatePath.selectedSlotKind = routeSelection_out.candidatePath.selectedSlotKind;
  output.candidatePath.planFlag = routeSelection_out.candidatePath.planFlag;
  output.candidatePath.errorType = routeSelection_out.candidatePath.errorType;
  output.candidatePath.plannerCase = routeSelection_out.candidatePath.plannerCase;
  output.candidatePath.segmentCount = routeSelection_out.candidatePath.segmentCount;
  output.candidatePath.directionChangeCount = routeSelection_out.candidatePath.directionChangeCount;
  output.candidatePath.pathLength = routeSelection_out.candidatePath.pathLength;
  output.candidatePath.pathPointCount = routeSelection_out.candidatePath.pathPointCount;
  output.candidatePath.maneuverStepCount = routeSelection_out.candidatePath.maneuverStepCount;
  output.candidatePath.secondStageEndPointOrdinal = routeSelection_out.candidatePath.secondStageEndPointOrdinal;
  output.candidatePath.firstForwardPointVehicleX = routeSelection_out.candidatePath.firstForwardPointVehicleX;
  output.candidatePath.firstForwardPointVehicleY = routeSelection_out.candidatePath.firstForwardPointVehicleY;
  output.candidatePath.firstForwardPointVehicleTheta = routeSelection_out.candidatePath.firstForwardPointVehicleTheta;
  output.candidatePath.planOk = routeSelection_out.candidatePath.planOk;
  output.candidatePath.resolvedScene = routeSelection_out.candidatePath.resolvedScene;
  /** @brief 100 点路径位姿及有符号步长。 */
  for (Integer pathIndex0 = 0; pathIndex0 < 100; pathIndex0 = pathIndex0 + 1) {
    output.candidatePath.pathPointX[pathIndex0] = routeSelection_out.candidatePath.pathPointX[pathIndex0];
    output.candidatePath.pathPointY[pathIndex0] = routeSelection_out.candidatePath.pathPointY[pathIndex0];
    output.candidatePath.pathPointTheta[pathIndex0] = routeSelection_out.candidatePath.pathPointTheta[pathIndex0];
    output.candidatePath.pathPointOdometer[pathIndex0] = routeSelection_out.candidatePath.pathPointOdometer[pathIndex0];
  }
}

}
