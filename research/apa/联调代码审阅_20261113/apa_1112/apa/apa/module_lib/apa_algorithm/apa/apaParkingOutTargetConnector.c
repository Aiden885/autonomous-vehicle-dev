#include "apaParkingOutTargetConnector.h"
#include "apaDebugTrace.h"

#include "apaCoordUtils.h"
#include "apaHybridAStar.h"
#include "apaParkingPathPlanningCommon.h"
#include "apaPlannerHybridParity.h"

#include <math.h>

#ifndef APA_PARKOUT_TARGET_CONNECTOR_POSITION_TOLERANCE
#define APA_PARKOUT_TARGET_CONNECTOR_POSITION_TOLERANCE 0.05f
#endif

#ifndef APA_PARKOUT_TARGET_CONNECTOR_YAW_TOLERANCE
#define APA_PARKOUT_TARGET_CONNECTOR_YAW_TOLERANCE 0.05f
#endif

#ifndef APA_PARKOUT_TARGET_CONNECTOR_DEFAULT_RADIUS
#define APA_PARKOUT_TARGET_CONNECTOR_DEFAULT_RADIUS 5.0f
#endif

#ifndef APA_PARKOUT_TARGET_CONNECTOR_ODO_EPSILON
#define APA_PARKOUT_TARGET_CONNECTOR_ODO_EPSILON 0.0001f
#endif

#ifndef APA_PARKOUT_TARGET_CONNECTOR_SAMPLE_STEP
#define APA_PARKOUT_TARGET_CONNECTOR_SAMPLE_STEP 0.4f
#endif

static Boolean
apaParkingOutTargetConnectorAtTarget(const ApaParkingPathPlanningInput_t *plannerInput,
                                     const ApaParkingPathPlanningOutput_t *path) {
  Integer lastIndex0;
  Real deltaX0;
  Real deltaY0;
  Real positionError0;
  Real yawError0;
  Boolean pathValid0;
  Boolean positionValid0;
  Boolean yawValid0;
  Boolean atTarget0;

  pathValid0 = (path->PlanFlg != 0);
  if (path->PtCount <= 0) {
    pathValid0 = false;
  }
  atTarget0 = false;
  if (pathValid0) {
    lastIndex0 = path->PtCount - 1;
    deltaX0 = path->PthPtX[lastIndex0] - plannerInput->TarX;
    deltaY0 = path->PthPtY[lastIndex0] - plannerInput->TarY;
    positionError0 = sqrtf((deltaX0 * deltaX0) + (deltaY0 * deltaY0));
    yawError0 = apaCoordWrapToPi(path->PthPtTht[lastIndex0] - plannerInput->TarTheta);
    positionValid0 = (positionError0 <= APA_PARKOUT_TARGET_CONNECTOR_POSITION_TOLERANCE);
    yawValid0 = (fabsf(yawError0) <= APA_PARKOUT_TARGET_CONNECTOR_YAW_TOLERANCE);
    atTarget0 = positionValid0 && yawValid0;
  }
  return atTarget0;
}

static void
apaParkingOutTargetConnectorFillSlotPoints(const ApaParkingPathPlanningInput_t *plannerInput,
                                           Real slotPoints[8]) {
  slotPoints[0] = plannerInput->SltPt1X;
  slotPoints[1] = plannerInput->SltPt1Y;
  slotPoints[2] = plannerInput->SltPt2X;
  slotPoints[3] = plannerInput->SltPt2Y;
  slotPoints[4] = plannerInput->SltPt3X;
  slotPoints[5] = plannerInput->SltPt3Y;
  slotPoints[6] = plannerInput->SltPt4X;
  slotPoints[7] = plannerInput->SltPt4Y;
}

static Boolean apaParkingOutTargetConnectorLoadCollisionEnvironment(
    const ApaParkingPathPlanningInput_t *plannerInput,
    ApaHybridAStarCollisionEnv_t *collisionEnvironment) {
  Integer lineIndex0;
  Integer lineCount0;
  Integer pointIndex0;
  Integer pointCount0;
  Boolean itemAdded0;
  Boolean loaded0;

  apaHybridAStarCollisionEnvInit(collisionEnvironment);
  loaded0 = true;
  lineCount0 = plannerInput->BoundaryLineNum;
  if (lineCount0 < 0) {
    lineCount0 = 0;
  }
  if (lineCount0 > APA_SCENARIO_BOUNDARY_LINE_MAX) {
    lineCount0 = APA_SCENARIO_BOUNDARY_LINE_MAX;
  }
  lineIndex0 = 0;
  while (lineIndex0 < lineCount0) {
    itemAdded0 =
        apaHybridAStarCollisionEnvAddBoundaryLine(collisionEnvironment,
                                                  plannerInput->BoundaryLineX1[lineIndex0],
                                                  plannerInput->BoundaryLineY1[lineIndex0],
                                                  plannerInput->BoundaryLineX2[lineIndex0],
                                                  plannerInput->BoundaryLineY2[lineIndex0]);
    if (!itemAdded0) {
      loaded0 = false;
    }
    lineIndex0 = lineIndex0 + 1;
  }

  lineCount0 = plannerInput->VirtualBoundaryLineNum;
  if (lineCount0 < 0) {
    lineCount0 = 0;
  }
  if (lineCount0 > APA_SCENARIO_BOUNDARY_LINE_MAX) {
    lineCount0 = APA_SCENARIO_BOUNDARY_LINE_MAX;
  }
  lineIndex0 = 0;
  while (lineIndex0 < lineCount0) {
    itemAdded0 =
        apaHybridAStarCollisionEnvAddVirtualLine(collisionEnvironment,
                                                 plannerInput->VirtualBoundaryP1X[lineIndex0],
                                                 plannerInput->VirtualBoundaryP1Y[lineIndex0],
                                                 plannerInput->VirtualBoundaryP2X[lineIndex0],
                                                 plannerInput->VirtualBoundaryP2Y[lineIndex0]);
    if (!itemAdded0) {
      loaded0 = false;
    }
    lineIndex0 = lineIndex0 + 1;
  }

  lineCount0 = plannerInput->SlotTrackStopperNum;
  if (lineCount0 < 0) {
    lineCount0 = 0;
  }
  if (lineCount0 > APA_SCENARIO_BOUNDARY_LINE_MAX) {
    lineCount0 = APA_SCENARIO_BOUNDARY_LINE_MAX;
  }
  lineIndex0 = 0;
  while (lineIndex0 < lineCount0) {
    itemAdded0 =
        apaHybridAStarCollisionEnvAddTrackStopper(collisionEnvironment,
                                                  plannerInput->SlotTrackStopperX1[lineIndex0],
                                                  plannerInput->SlotTrackStopperY1[lineIndex0],
                                                  plannerInput->SlotTrackStopperX2[lineIndex0],
                                                  plannerInput->SlotTrackStopperY2[lineIndex0]);
    if (!itemAdded0) {
      loaded0 = false;
    }
    lineIndex0 = lineIndex0 + 1;
  }

  pointCount0 = plannerInput->BoundarySepPtNum;
  if (pointCount0 < 0) {
    pointCount0 = 0;
  }
  if (pointCount0 > APA_SCENARIO_BOUNDARY_POINT_MAX) {
    pointCount0 = APA_SCENARIO_BOUNDARY_POINT_MAX;
  }
  pointIndex0 = 0;
  while (pointIndex0 < pointCount0) {
    itemAdded0 =
        apaHybridAStarCollisionEnvAddBoundaryPoint(collisionEnvironment,
                                                   plannerInput->BoundarySepPtX[pointIndex0],
                                                   plannerInput->BoundarySepPtY[pointIndex0]);
    if (!itemAdded0) {
      loaded0 = false;
    }
    pointIndex0 = pointIndex0 + 1;
  }

  pointCount0 = plannerInput->StopperPtNum;
  if (pointCount0 < 0) {
    pointCount0 = 0;
  }
  if (pointCount0 > APA_SCENARIO_BOUNDARY_POINT_MAX) {
    pointCount0 = APA_SCENARIO_BOUNDARY_POINT_MAX;
  }
  pointIndex0 = 0;
  while (pointIndex0 < pointCount0) {
    itemAdded0 =
        apaHybridAStarCollisionEnvAddSeparateStopper(collisionEnvironment,
                                                     plannerInput->StopperPtX[pointIndex0],
                                                     plannerInput->StopperPtY[pointIndex0]);
    if (!itemAdded0) {
      loaded0 = false;
    }
    pointIndex0 = pointIndex0 + 1;
  }
  return loaded0;
}

static Boolean
apaParkingOutTargetConnectorBuildAnalyticPath(const ApaParkingPathPlanningInput_t *plannerInput,
                                              const ApaParkingPathPlanningParam_t *plannerParam,
                                              const ApaParkingPathPlanningOutput_t *localPath,
                                              ApaPath_t *connectorPath) {
  ApaPlannerGlobalState_t plannerGlobal0;
  ApaHybridAStar_t planner0;
  ApaHybridAStarCollisionEnv_t collisionEnvironment0;
  ApaSlot_t slot0;
  Real slotPoints0[8];
  Real startPose0[3];
  Real goalPose0[3];
  Real radius0;
  Integer lastIndex0;
  Boolean connected0;
  Boolean collisionEnvironmentLoaded0;
  Boolean directAnalytic0;
  ApaHybridAStarState_e runState0;

  plannerGlobal0 = (ApaPlannerGlobalState_t)APA_PLANNER_GLOBAL_STATE_INIT;
  planner0 = (ApaHybridAStar_t){0};
  collisionEnvironment0 = (ApaHybridAStarCollisionEnv_t){0};
  slot0 = (ApaSlot_t){0};
  apaPathInit(connectorPath);
  lastIndex0 = localPath->PtCount - 1;
  startPose0[0] = localPath->PthPtX[lastIndex0];
  startPose0[1] = localPath->PthPtY[lastIndex0];
  startPose0[2] = localPath->PthPtTht[lastIndex0];
  goalPose0[0] = plannerInput->TarX;
  goalPose0[1] = plannerInput->TarY;
  goalPose0[2] = plannerInput->TarTheta;
  radius0 = plannerInput->Rmin;
  if (radius0 < 1.0f) {
    radius0 = APA_PARKOUT_TARGET_CONNECTOR_DEFAULT_RADIUS;
  }

  apaPlannerHybridInitPlannerGlobal(&plannerGlobal0, plannerParam->VehW, radius0, 1);
  plannerGlobal0.Planner.Map.MotionResolution = APA_PARKOUT_TARGET_CONNECTOR_SAMPLE_STEP;
  plannerGlobal0.Planner.Map.CollisionDetectStepLength = APA_PARKOUT_TARGET_CONNECTOR_SAMPLE_STEP;
  apaParkingOutTargetConnectorFillSlotPoints(plannerInput, slotPoints0);
  apaPlannerHybridInitSlot(&slot0,
                           plannerInput->TarX,
                           plannerInput->TarY,
                           plannerInput->TarTheta,
                           plannerInput->SltEntr,
                           slotPoints0,
                           plannerInput->PrkType,
                           plannerInput->SurdType,
                           APA_PARKING_DIREC_REAR);
  apaHybridAStarInit(&planner0);
  apaHybridAStarSetPlannerGlobal(&planner0, &plannerGlobal0);
  apaHybridAStarLoadSlot(&planner0, &slot0);
  planner0.ReversePlanFlag = true;

  planner0.GridSpec.OriginX =
      0.5f * (startPose0[0] + goalPose0[0]) -
      0.5f * (Real)(planner0.GridSpec.XCount - 1) * planner0.GridSpec.XResolution;
  planner0.GridSpec.OriginY =
      0.5f * (startPose0[1] + goalPose0[1]) -
      0.5f * (Real)(planner0.GridSpec.YCount - 1) * planner0.GridSpec.YResolution;
  if (!apaHybridAStarLoadStartGoalPoses(&planner0, startPose0, goalPose0)) {
    return false;
  }
  planner0.CurrentNode = planner0.StartNode;
  collisionEnvironmentLoaded0 =
      apaParkingOutTargetConnectorLoadCollisionEnvironment(plannerInput, &collisionEnvironment0);
  connected0 = false;
  if (collisionEnvironmentLoaded0) {
    apaHybridAStarSetDefaultCollisionEnv(&planner0, &collisionEnvironment0);
    apaHybridAStarSetParkingOutFlag(&planner0, true);
    apaHybridAStarSetCollisionDemand(&planner0, APA_COLLISION_DEMAND_ALL);
    directAnalytic0 = apaHybridAStarTryAnalyticConnect(&planner0);
    connected0 = directAnalytic0;
    if (directAnalytic0) {
      *connectorPath = planner0.AnalyticPath;
    } else {
      runState0 = apaHybridAStarRunToTerminal(&planner0);
      connected0 = (runState0 == APA_HYBRID_STATE_GOAL_REACHED);
      if (connected0) {
        connected0 = apaHybridAStarBuildPath(&planner0, connectorPath);
      }
    }
  }
  if (connected0) {
    connected0 = connectorPath->ValidFlag;
  }
  if (connected0) {
    connected0 = (connectorPath->PathN > 1);
  }
  return connected0;
}

static void apaParkingOutTargetConnectorRefreshMetadata(ApaParkingPathPlanningOutput_t *path) {
  Integer index0;
  Integer previousDirection0;
  Integer direction0;
  Integer segmentCount0;
  Integer directionChangeCount0;
  Real pathLength0;

  index0 = 0;
  previousDirection0 = 0;
  segmentCount0 = 0;
  directionChangeCount0 = 0;
  pathLength0 = 0.0f;
  while (index0 < (path->PtCount - 1)) {
    Real odo0;

    odo0 = path->PthPtOdo[index0];
    pathLength0 = pathLength0 + fabsf(odo0);
    direction0 = 0;
    if (odo0 > APA_PARKOUT_TARGET_CONNECTOR_ODO_EPSILON) {
      direction0 = 1;
    }
    if (odo0 < -APA_PARKOUT_TARGET_CONNECTOR_ODO_EPSILON) {
      direction0 = -1;
    }
    if (direction0 != 0) {
      if (previousDirection0 == 0) {
        segmentCount0 = 1;
      } else if (direction0 != previousDirection0) {
        segmentCount0 = segmentCount0 + 1;
        directionChangeCount0 = directionChangeCount0 + 1;
      }
      previousDirection0 = direction0;
    }
    index0 = index0 + 1;
  }
  if (path->PtCount > 0) {
    path->PthPtOdo[path->PtCount - 1] = 0.0f;
  }
  path->PathLen = pathLength0;
  path->SegNum = segmentCount0;
  path->StpN = directionChangeCount0;
}

static Boolean apaParkingOutTargetConnectorAppend(const ApaPath_t *connectorPath,
                                                  ApaParkingPathPlanningOutput_t *path) {
  Integer requiredPointCount0;
  Integer connectorIndex0;
  Integer targetIndex0;
  Boolean capacityValid0;

  requiredPointCount0 = path->PtCount + connectorPath->PathN - 1;
  capacityValid0 = (requiredPointCount0 <= APA_PARKING_PATH_PT_MAX);
  if (capacityValid0) {
    path->PthPtOdo[path->PtCount - 1] = connectorPath->OdoStep[0];
    connectorIndex0 = 1;
    while (connectorIndex0 < connectorPath->PathN) {
      targetIndex0 = path->PtCount + connectorIndex0 - 1;
      path->PthPtX[targetIndex0] = connectorPath->X[connectorIndex0];
      path->PthPtY[targetIndex0] = connectorPath->Y[connectorIndex0];
      path->PthPtTht[targetIndex0] = connectorPath->Tht[connectorIndex0];
      path->PthPtOdo[targetIndex0] = connectorPath->OdoStep[connectorIndex0];
      connectorIndex0 = connectorIndex0 + 1;
    }
    path->PtCount = requiredPointCount0;
    apaParkingOutTargetConnectorRefreshMetadata(path);
  }
  return capacityValid0;
}

/**
 * @brief 在底层泊出自然终点后追加曲率受限的 Reeds-Shepp 道路目标连接段。
 * @en_name apaParkingOutTargetConnector
 * @cn_name 泊出目标连接
 * @type module
 * @param[IN] ApaParkingPathPlanningInput_t PlannerInput 统一规划输入
 * @param[IN] ApaParkingPathPlanningParam_t PlannerParam 统一规划参数
 * @param[IN] ApaParkingPathPlanningOutput_t LocalPath 底层泊出局部路径
 * @param[OUT] Boolean Connected 是否成功连接道路目标
 * @param[OUT] ApaParkingPathPlanningOutput_t ConnectedPath 完整泊出路径
 * @granularity composite
 * @tag_level1 规划
 * @tag_level2 泊出目标连接
 * @source demo_new 泊出道路目标连接
 * @version 1.0
 * @date 2026-07-29
 * @author aide
 */
void apaParkingOutTargetConnector(const ApaParkingOutTargetConnectorInput_t *input,
                                  ApaParkingOutTargetConnectorOutput_t *output) {
  ApaPath_t connectorPath0;
  Boolean localPathValid0;
  Boolean alreadyAtTarget0;
  Boolean connectorBuilt0;
  Boolean connectorAppended0;
  Boolean connected0;

  output->Connected = false;
  output->ConnectedPath = input->LocalPath;
  connectorPath0 = (ApaPath_t){0};
  localPathValid0 = (input->LocalPath.PlanFlg != 0);
  if (input->LocalPath.PtCount <= 0) {
    localPathValid0 = false;
  }
  connected0 = false;
  if (localPathValid0) {
    alreadyAtTarget0 =
        apaParkingOutTargetConnectorAtTarget(&input->PlannerInput, &input->LocalPath);
    if (alreadyAtTarget0) {
      connected0 = true;
    } else {
      connectorBuilt0 = apaParkingOutTargetConnectorBuildAnalyticPath(
          &input->PlannerInput, &input->PlannerParam, &input->LocalPath, &connectorPath0);
      if (connectorBuilt0) {
        connectorAppended0 =
            apaParkingOutTargetConnectorAppend(&connectorPath0, &output->ConnectedPath);
        connected0 = connectorAppended0;
      }
    }
  }
  output->Connected = connected0;
  if (!connected0) {
    apaParkingPathPlanningZeroOutput(&output->ConnectedPath);
    output->ConnectedPath.SelectedSlotKind = input->LocalPath.SelectedSlotKind;
    output->ConnectedPath.PlannerCase = input->LocalPath.PlannerCase;
  }
  apaDebugTraceParkoutPath("PARKOUT_CONNECTOR",
                           apaDebugTraceCurrentFrame(),
                           &input->PlannerInput,
                           &output->ConnectedPath);
}
