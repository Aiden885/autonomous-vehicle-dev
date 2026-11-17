#include "apaParallelReplanInSlotTypes.h"

#include "apaCoordUtils.h"
#include "apaEnums.h"
#include "apaMacros.h"
#include "apaMathUtils.h"
#include "apaPathTypes.h"
#include "apaPathAlgo.h"
#include "apaPlannerConfig.h"
#include "apaVehicleUtils.h"
#include "geoInboxJudgeTypes.h"
#include "geoOffsetPlanningTypes.h"
#include "geoPthDiscretVtTypes.h"
#include "geoVtPlanningInSlotTypes.h"
#include "slotSurroundingAreaCheckTypes.h"
#include "vehicleCollisionCheckTypes.h"

#include <math.h>

#ifndef APA_PARALLEL_REPLAN_POINT_STEP
#define APA_PARALLEL_REPLAN_POINT_STEP 0.15f
#endif

#ifndef APA_PARALLEL_REPLAN_OFFSET_MIN_DIST
#define APA_PARALLEL_REPLAN_OFFSET_MIN_DIST 0.35f
#endif

#ifndef APA_PARALLEL_REPLAN_CUBIC_POINT_COUNT
#define APA_PARALLEL_REPLAN_CUBIC_POINT_COUNT 50
#endif

#ifndef APA_PARALLEL_REPLAN_CUBIC_MAX_PATH_LEN
#define APA_PARALLEL_REPLAN_CUBIC_MAX_PATH_LEN 15.0f
#endif

#ifndef APA_PARALLEL_REPLAN_CUBIC_MAX_ERR
#define APA_PARALLEL_REPLAN_CUBIC_MAX_ERR 0.1f
#endif

#ifndef APA_PARALLEL_REPLAN_CUBIC_ENTRY_LOCAL_X_MAX
#define APA_PARALLEL_REPLAN_CUBIC_ENTRY_LOCAL_X_MAX 1.5f
#endif

#ifndef APA_PARALLEL_REPLAN_CUBIC_ENTRY_LOCAL_Y_MAX
#define APA_PARALLEL_REPLAN_CUBIC_ENTRY_LOCAL_Y_MAX 1.0f
#endif

#ifndef APA_PARALLEL_REPLAN_CUBIC_ENTRY_LOCAL_THETA_MAX
#define APA_PARALLEL_REPLAN_CUBIC_ENTRY_LOCAL_THETA_MAX (APA_PI / 6.0f)
#endif

#ifndef APA_PARALLEL_REPLAN_CUBIC_MID_S_MAX
#define APA_PARALLEL_REPLAN_CUBIC_MID_S_MAX 5.0f
#endif

#ifndef APA_PARALLEL_REPLAN_CUBIC_MID_S_STEP
#define APA_PARALLEL_REPLAN_CUBIC_MID_S_STEP 0.05f
#endif

#ifndef APA_PARALLEL_REPLAN_NON_SLOT_LOCAL_X_MAX
#define APA_PARALLEL_REPLAN_NON_SLOT_LOCAL_X_MAX 1.0f
#endif

#ifndef APA_PARALLEL_REPLAN_FIRST_STEP
#define APA_PARALLEL_REPLAN_FIRST_STEP 0.05f
#endif

#ifndef APA_PARALLEL_REPLAN_SEGMENT_S_LIMIT
#define APA_PARALLEL_REPLAN_SEGMENT_S_LIMIT 4.0f
#endif

#ifndef APA_PARALLEL_REPLAN_COLLISION_EXPLORE_LIMIT
#define APA_PARALLEL_REPLAN_COLLISION_EXPLORE_LIMIT 2.0f
#endif

#ifndef APA_PARALLEL_REPLAN_HEADING_THRESH
#define APA_PARALLEL_REPLAN_HEADING_THRESH (2.0f * APA_PI / 180.0f)
#endif

#ifndef APA_PARALLEL_REPLAN_MANEUVER_ANGLE_THRESH
#define APA_PARALLEL_REPLAN_MANEUVER_ANGLE_THRESH 0.02f
#endif

#ifndef APA_PARALLEL_REPLAN_LAT_THRESH
#define APA_PARALLEL_REPLAN_LAT_THRESH 0.25f
#endif

#ifndef APA_PARALLEL_REPLAN_MOVE_SWITCH_LAT
#define APA_PARALLEL_REPLAN_MOVE_SWITCH_LAT 0.30f
#endif

#ifndef APA_PARALLEL_REPLAN_MOVE_STOP_LAT
#define APA_PARALLEL_REPLAN_MOVE_STOP_LAT 0.05f
#endif

#ifndef APA_PARALLEL_REPLAN_LAT_EXPAND_STOP
#define APA_PARALLEL_REPLAN_LAT_EXPAND_STOP 0.15f
#endif

#ifndef APA_PARALLEL_REPLAN_MANEUVER_LOOP_LIMIT
#define APA_PARALLEL_REPLAN_MANEUVER_LOOP_LIMIT 10
#endif

#ifndef APA_PARALLEL_REPLAN_MIN_FORWARD_COLLISION_SOFT_DIST
#define APA_PARALLEL_REPLAN_MIN_FORWARD_COLLISION_SOFT_DIST 1.0f
#endif

#ifndef APA_PARALLEL_REPLAN_POLYFIT_BIAS_STEP
#define APA_PARALLEL_REPLAN_POLYFIT_BIAS_STEP 0.01f
#endif

#ifndef APA_PARALLEL_REPLAN_SAFE_DELTA
#define APA_PARALLEL_REPLAN_SAFE_DELTA 0.001f
#endif

typedef struct {
  Real X[APA_PARALLEL_REPLAN_PATH_POINT_MAX];
  Real Y[APA_PARALLEL_REPLAN_PATH_POINT_MAX];
  Real Theta[APA_PARALLEL_REPLAN_PATH_POINT_MAX];
  Real Odo[APA_PARALLEL_REPLAN_PATH_POINT_MAX];
  Integer PtCount;
  Real PathLen;
} ApaParallelReplanPath_t;

static Boolean apaParallelReplanInSlotPathAppendPath(ApaParallelReplanPath_t *dst,
                                                     const ApaParallelReplanPath_t *src);
static Boolean apaParallelReplanInSlotCopyApaPathToCustomPath(const ApaPath_t *src,
                                                              ApaParallelReplanPath_t *dst);

static void apaParallelReplanInSlotLoadVehConfig(ApaPlannerVehConfig_t *vehCfg) {
  ApaPlannerConfig_t plannerCfg;

  plannerCfg = (ApaPlannerConfig_t){0};
  apaPlannerConfigInitDefault(&plannerCfg);
  *vehCfg = plannerCfg.Veh;
}

static Boolean apaParallelReplanInSlotCheckPointInSlot(const GeoInboxJudgeInput_t *templateInput,
                                                       Real pointX,
                                                       Real pointY) {
  GeoInboxJudgeInput_t judgeInput;
  GeoInboxJudgeOutput_t judgeOutput;

  judgeInput = *templateInput;
  judgeInput.PtX = pointX;
  judgeInput.PtY = pointY;
  judgeOutput = (GeoInboxJudgeOutput_t){0};
  geoInboxJudge(&judgeInput, &judgeOutput);
  return judgeOutput.Inbox;
}

static Boolean apaParallelReplanInSlotVehicleInSlot(const ApaParallelReplanInSlotInput_t *input) {
  ApaPlannerVehConfig_t vehCfg;
  GeoInboxJudgeInput_t judgeInput;
  Real frontAxleX;
  Real frontAxleY;
  Boolean vehicleInSlot0;

  apaParallelReplanInSlotLoadVehConfig(&vehCfg);
  judgeInput = (GeoInboxJudgeInput_t){0};
  geoInboxJudgeSetBox(&judgeInput,
                      input->SltPt1X,
                      input->SltPt1Y,
                      input->SltPt2X,
                      input->SltPt2Y,
                      input->SltPt3X,
                      input->SltPt3Y,
                      input->SltPt4X,
                      input->SltPt4Y);

  vehicleInSlot0 =
      apaParallelReplanInSlotCheckPointInSlot(&judgeInput, input->StartX, input->StartY);
  if (vehicleInSlot0 == false) {
    frontAxleX = input->StartX + vehCfg.WB * cosf(input->StartTheta);
    frontAxleY = input->StartY + vehCfg.WB * sinf(input->StartTheta);
    vehicleInSlot0 = apaParallelReplanInSlotCheckPointInSlot(&judgeInput, frontAxleX, frontAxleY);
  }
  return vehicleInSlot0;
}

static Real apaParallelReplanInSlotCalcPathLen(const ApaPath_t *path) {
  Integer pointIndex;
  Real pathLen;

  pathLen = 0.0f;
  pointIndex = 0;
  while (pointIndex + 1 < path->PathN && pointIndex < APA_PATH_MAX_POINTS) {
    pathLen = pathLen + fabsf(path->OdoStep[pointIndex]);
    pointIndex = pointIndex + 1;
  }
  return pathLen;
}

static void apaParallelReplanInSlotBuildCollisionInput(const ApaParallelReplanInSlotInput_t *input,
                                                       VehicleCollisionCheckInput_t *vccInput) {
  ApaPlannerVehConfig_t vehCfg;
  Integer boundaryIndex;
  Integer pointIndex;
  Integer stopperIndex;
  Integer virtualIndex;

  apaParallelReplanInSlotLoadVehConfig(&vehCfg);
  *vccInput = (VehicleCollisionCheckInput_t){0};
  vccInput->CheckType = VCC_CHECK_TYPE_PLANNING;
  vccInput->CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  vccInput->CollisionDemand = VCC_COLLISION_DEMAND_SIMPLIFY;
  vccInput->SlotClass = VCC_SLOT_CLASS_PARALLEL;
  vccInput->UseAutoMargin = true;
  vccInput->VehLF = vehCfg.LF;
  vccInput->VehLB = vehCfg.LB;
  vccInput->VehW = vehCfg.W;
  vccInput->VehWB = vehCfg.WB;
  vccInput->SlotPtsValid = true;
  vccInput->SlotPts[0] = input->SltPt1X;
  vccInput->SlotPts[1] = input->SltPt1Y;
  vccInput->SlotPts[2] = input->SltPt2X;
  vccInput->SlotPts[3] = input->SltPt2Y;
  vccInput->SlotPts[4] = input->SltPt3X;
  vccInput->SlotPts[5] = input->SltPt3Y;
  vccInput->SlotPts[6] = input->SltPt4X;
  vccInput->SlotPts[7] = input->SltPt4Y;

  boundaryIndex = 0;
  while ((boundaryIndex < input->BoundaryLineNum) && (boundaryIndex < VCC_MAX_BOUNDARY_LINES)) {
    vccInput->BoundaryLineX1[boundaryIndex] = input->BoundaryLineX1[boundaryIndex];
    vccInput->BoundaryLineY1[boundaryIndex] = input->BoundaryLineY1[boundaryIndex];
    vccInput->BoundaryLineX2[boundaryIndex] = input->BoundaryLineX2[boundaryIndex];
    vccInput->BoundaryLineY2[boundaryIndex] = input->BoundaryLineY2[boundaryIndex];
    boundaryIndex = boundaryIndex + 1;
  }
  vccInput->BoundaryLineNum = boundaryIndex;

  if (vccInput->BoundaryLineNum <= 0) {
    vccInput->BoundaryLineNum = 3;
    vccInput->BoundaryLineX1[0] = input->SltPt1X;
    vccInput->BoundaryLineY1[0] = input->SltPt1Y;
    vccInput->BoundaryLineX2[0] = input->SltPt4X;
    vccInput->BoundaryLineY2[0] = input->SltPt4Y;
    vccInput->BoundaryLineX1[1] = input->SltPt4X;
    vccInput->BoundaryLineY1[1] = input->SltPt4Y;
    vccInput->BoundaryLineX2[1] = input->SltPt3X;
    vccInput->BoundaryLineY2[1] = input->SltPt3Y;
    vccInput->BoundaryLineX1[2] = input->SltPt3X;
    vccInput->BoundaryLineY1[2] = input->SltPt3Y;
    vccInput->BoundaryLineX2[2] = input->SltPt2X;
    vccInput->BoundaryLineY2[2] = input->SltPt2Y;
  }

  pointIndex = 0;
  while ((pointIndex < input->BoundarySepPtNum) && (pointIndex < VCC_MAX_SEPARATE_PTS)) {
    vccInput->BoundarySepPtsX[pointIndex] = input->BoundarySepPtX[pointIndex];
    vccInput->BoundarySepPtsY[pointIndex] = input->BoundarySepPtY[pointIndex];
    pointIndex = pointIndex + 1;
  }
  vccInput->BoundarySepPtsNum = pointIndex;

  virtualIndex = 0;
  while ((virtualIndex < input->VirtualBoundaryLineNum) && (virtualIndex < VCC_MAX_VIRTUAL_LINES)) {
    vccInput->VirtualBoundaryLineX1[virtualIndex] = input->VirtualBoundaryLineX1[virtualIndex];
    vccInput->VirtualBoundaryLineY1[virtualIndex] = input->VirtualBoundaryLineY1[virtualIndex];
    vccInput->VirtualBoundaryLineX2[virtualIndex] = input->VirtualBoundaryLineX2[virtualIndex];
    vccInput->VirtualBoundaryLineY2[virtualIndex] = input->VirtualBoundaryLineY2[virtualIndex];
    virtualIndex = virtualIndex + 1;
  }
  vccInput->VirtualBoundaryLineNum = virtualIndex;

  stopperIndex = 0;
  while (stopperIndex < input->StopperPtNum && stopperIndex < VCC_MAX_SEPARATE_PTS) {
    vccInput->SeparateStopperX[stopperIndex] = input->StopperPtX[stopperIndex];
    vccInput->SeparateStopperY[stopperIndex] = input->StopperPtY[stopperIndex];
    stopperIndex = stopperIndex + 1;
  }
  vccInput->SeparateStopperNum = stopperIndex;
}

static Boolean
apaParallelReplanInSlotPathCollisionFree(const VehicleCollisionCheckInput_t *vccTemplate,
                                         const ApaPath_t *path) {
  VehicleCollisionCheckInput_t vccInput;
  VehicleCollisionCheckOutput_t vccOutput;
  Integer pointIndex;
  Boolean collisionFree0;

  collisionFree0 = true;
  pointIndex = 0;
  while ((pointIndex < path->PathN) && (pointIndex < APA_PATH_MAX_POINTS) && collisionFree0) {
    vccInput = *vccTemplate;
    vccInput.VehPosX = path->X[pointIndex];
    vccInput.VehPosY = path->Y[pointIndex];
    vccInput.VehPosTheta = path->Tht[pointIndex];
    vccOutput = (VehicleCollisionCheckOutput_t){0};
    vehicleCollisionCheck(&vccInput, &vccOutput);
    if (vccOutput.IsCollision) {
      collisionFree0 = false;
    }
    pointIndex = pointIndex + 1;
  }

  return collisionFree0;
}

static Boolean apaParallelReplanInSlotApplyProcessedPathEnd(ApaPath_t *path,
                                                            const Real endPose[3],
                                                            Real pathEndX,
                                                            Real pathEndY,
                                                            Real pathEndTheta,
                                                            Real pathPrevX,
                                                            Real pathPrevY,
                                                            Real pathPrevTheta,
                                                            Real errDis,
                                                            Real subDis,
                                                            Real subYaw) {
  Real errDisThresh;
  Real errYawThresh;
  Integer dirSign;
  Integer dirSignTemp;
  Boolean validPath0;

  validPath0 = true;
  errDisThresh = fminf(APA_PARALLEL_REPLAN_CUBIC_MAX_ERR, subDis);
  errYawThresh = fminf(APA_PARALLEL_REPLAN_CUBIC_MAX_ERR, subYaw);
  if (path->OdoStep[0] > 0.0f) {
    dirSign = 1;
  } else {
    dirSign = -1;
  }
  if (errDis > errDisThresh || fabsf(apaCoordWrapToPi(pathEndTheta - endPose[2])) > errYawThresh) {
    dirSignTemp = apaPathCubicResolveDirSign(
        (const Real[3]){pathEndX, pathEndY, pathEndTheta}, endPose[0], endPose[1]);
    if (dirSignTemp * dirSign < 0) {
      validPath0 = false;
    }
    if (path->PathN >= APA_PATH_MAX_POINTS) {
      validPath0 = false;
    }
    if (validPath0) {
      path->X[path->PathN] = endPose[0];
      path->Y[path->PathN] = endPose[1];
      path->Tht[path->PathN] = endPose[2];
      path->OdoStep[path->PathN - 1] = ((Real)dirSign) * errDis;
      path->OdoStep[path->PathN] = 0.0f;
      path->PathN = path->PathN + 1;
    }
  } else {
    dirSignTemp = apaPathCubicResolveDirSign(
        (const Real[3]){pathPrevX, pathPrevY, pathPrevTheta}, endPose[0], endPose[1]);
    if (dirSignTemp * dirSign < 0) {
      validPath0 = false;
    }
    if (validPath0) {
      path->X[path->PathN - 1] = endPose[0];
      path->Y[path->PathN - 1] = endPose[1];
      path->Tht[path->PathN - 1] = endPose[2];
      path->OdoStep[path->PathN - 2] = path->OdoStep[path->PathN - 2] + ((Real)dirSign) * errDis;
      path->OdoStep[path->PathN - 1] = 0.0f;
    }
  }
  return validPath0;
}

static Boolean apaParallelReplanInSlotProcessPathEnd(ApaPath_t *path, const Real endPose[3]) {
  Real pathEndX;
  Real pathEndY;
  Real pathEndTheta;
  Real pathPrevX;
  Real pathPrevY;
  Real pathPrevTheta;
  Real errDis;
  Real errYaw;
  Real subDis;
  Real subYaw;
  Real avgOdo;
  Real avgDeltaTheta;
  Real errDisThresh;
  Real errYawThresh;
  Integer pointIndex;
  Boolean validPath0;

  validPath0 = (path->PathN >= 2);
  if (validPath0) {
    pathEndX = path->X[path->PathN - 1];
    pathEndY = path->Y[path->PathN - 1];
    pathEndTheta = path->Tht[path->PathN - 1];
    pathPrevX = path->X[path->PathN - 2];
    pathPrevY = path->Y[path->PathN - 2];
    pathPrevTheta = path->Tht[path->PathN - 2];
    errDis = sqrtf((pathEndX - endPose[0]) * (pathEndX - endPose[0]) +
                   (pathEndY - endPose[1]) * (pathEndY - endPose[1]));
    errYaw = fabsf(apaCoordWrapToPi(pathEndTheta - endPose[2]));

    subDis = sqrtf((pathEndX - pathPrevX) * (pathEndX - pathPrevX) +
                   (pathEndY - pathPrevY) * (pathEndY - pathPrevY));
    {
      Integer divisor0;
      if (path->PathN > 1) {
        divisor0 = path->PathN - 1;
      } else {
        divisor0 = 1;
      }
      avgOdo = apaParallelReplanInSlotCalcPathLen(path) / (Real)divisor0;
    }
    if (avgOdo > subDis) {
      subDis = avgOdo;
    }

    subYaw = fabsf(apaCoordWrapToPi(pathEndTheta - pathPrevTheta));
    avgDeltaTheta = 0.0f;
    pointIndex = 1;
    while (pointIndex < path->PathN) {
      avgDeltaTheta = avgDeltaTheta +
                      fabsf(apaCoordWrapToPi(path->Tht[pointIndex] - path->Tht[pointIndex - 1]));
      pointIndex = pointIndex + 1;
    }
    {
      Integer divisor1;
      if (path->PathN > 1) {
        divisor1 = path->PathN - 1;
      } else {
        divisor1 = 1;
      }
      avgDeltaTheta = avgDeltaTheta / (Real)divisor1;
    }
    if (avgDeltaTheta > subYaw) {
      subYaw = avgDeltaTheta;
    }

    errDisThresh = fminf(APA_PARALLEL_REPLAN_CUBIC_MAX_ERR, 4.0f * subDis);
    errYawThresh = fminf(APA_PARALLEL_REPLAN_CUBIC_MAX_ERR, 4.0f * subYaw);
    if (errDis > errDisThresh || errYaw > errYawThresh) {
      validPath0 = false;
    }

    if (validPath0) {
      validPath0 = apaParallelReplanInSlotApplyProcessedPathEnd(path,
                                                                endPose,
                                                                pathEndX,
                                                                pathEndY,
                                                                pathEndTheta,
                                                                pathPrevX,
                                                                pathPrevY,
                                                                pathPrevTheta,
                                                                errDis,
                                                                subDis,
                                                                subYaw);
    }
  }
  return validPath0;
}

static Boolean apaParallelReplanInSlotDiscretizeDoubleCubic(const Real startPose[3],
                                                            const Real endPose[3],
                                                            ApaPath_t *path) {
  Integer polyfitType;
  Real coeff[4];
  Integer dirSign;
  Integer pointIndex;
  Real deltaCoor;
  Real coord;
  Real yawErr;
  Real deltaThetaThresh;
  Boolean validPath;

  yawErr = fabsf(apaCoordWrapToPi(startPose[2] - endPose[2]));
  validPath = (yawErr <= (APA_HALF_PI - 0.01f));

  polyfitType = APA_POLYFIT_INVALID;
  if (validPath) {
    validPath = apaPathCubicPolyfit(startPose, endPose, &polyfitType, coeff) &&
                (polyfitType != APA_POLYFIT_INVALID);
  }

  if (validPath) {
    dirSign = apaPathCubicResolveDirSign(startPose, endPose[0], endPose[1]);
    validPath = (dirSign != 0);
  }

  pointIndex = 0;
  if (validPath) {
    apaPathInit(path);
    path->PathSource = APA_PATH_SOURCE_SEARCH;
    path->PathType = APA_PATH_TYPE_ALL;
  }

  if (validPath && (polyfitType == APA_POLYFIT_DOMAIN_X)) {
    deltaCoor = (endPose[0] - startPose[0]) / (Real)APA_PARALLEL_REPLAN_CUBIC_POINT_COUNT;
    coord = startPose[0];
    while (validPath && (((coord < endPose[0]) && (endPose[0] > startPose[0])) ||
                         ((coord > endPose[0]) && (endPose[0] < startPose[0])))) {
      Real coordSq;
      Real coordCu;

      if (pointIndex >= APA_PATH_MAX_POINTS - 1) {
        validPath = false;
      } else {
        coordSq = coord * coord;
        coordCu = coordSq * coord;
        path->X[pointIndex] = coord;
        path->Y[pointIndex] = coeff[0] + coeff[1] * coord + coeff[2] * coordSq + coeff[3] * coordCu;
        pointIndex = pointIndex + 1;
        coord = coord + deltaCoor;
        if (!(((coord < endPose[0]) && (endPose[0] > startPose[0])) ||
              ((coord > endPose[0]) && (endPose[0] < startPose[0])))) {
          coord = coord - 0.25f * deltaCoor;
        }
      }
    }
  } else if (validPath) {
    deltaCoor = (endPose[1] - startPose[1]) / (Real)APA_PARALLEL_REPLAN_CUBIC_POINT_COUNT;
    coord = startPose[1];
    while (validPath && (((coord < endPose[1]) && (endPose[1] > startPose[1])) ||
                         ((coord > endPose[1]) && (endPose[1] < startPose[1])))) {
      Real coordSq;
      Real coordCu;

      if (pointIndex >= APA_PATH_MAX_POINTS - 1) {
        validPath = false;
      } else {
        coordSq = coord * coord;
        coordCu = coordSq * coord;
        path->X[pointIndex] = coeff[0] + coeff[1] * coord + coeff[2] * coordSq + coeff[3] * coordCu;
        path->Y[pointIndex] = coord;
        pointIndex = pointIndex + 1;
        coord = coord + deltaCoor;
        if (!(((coord < endPose[1]) && (endPose[1] > startPose[1])) ||
              ((coord > endPose[1]) && (endPose[1] < startPose[1])))) {
          coord = coord - 0.25f * deltaCoor;
        }
      }
    }
  }

  if (validPath) {
    validPath = (pointIndex > 3);
  }
  if (validPath) {
    path->PathN = pointIndex;
    path->Tht[0] = startPose[2];
    validPath = (apaPathCubicResolveDirSign(startPose, path->X[1], path->Y[1]) == dirSign);
  }
  if (validPath) {
    path->OdoStep[0] =
        ((Real)dirSign) * sqrtf((startPose[0] - path->X[1]) * (startPose[0] - path->X[1]) +
                                (startPose[1] - path->Y[1]) * (startPose[1] - path->Y[1]));

    deltaThetaThresh = 3.0f * yawErr / (Real)(path->PathN - 1);
    deltaThetaThresh = fmaxf(deltaThetaThresh, 30.0f * APA_PI / 180.0f);
  }
  pointIndex = 2;
  while (validPath && (pointIndex < path->PathN)) {
    Real lastX;
    Real lastY;
    Real curX;
    Real curY;
    Real dist;
    Real deltaTheta;

    lastX = path->X[pointIndex - 1];
    lastY = path->Y[pointIndex - 1];
    curX = path->X[pointIndex];
    curY = path->Y[pointIndex];
    dist = sqrtf((curX - lastX) * (curX - lastX) + (curY - lastY) * (curY - lastY));
    if (dirSign > 0) {
      path->Tht[pointIndex - 1] = atan2f(curY - lastY, curX - lastX);
      path->OdoStep[pointIndex - 1] = dist;
    } else {
      path->Tht[pointIndex - 1] = atan2f(lastY - curY, lastX - curX);
      path->OdoStep[pointIndex - 1] = -dist;
    }
    deltaTheta = fabsf(apaCoordWrapToPi(path->Tht[pointIndex - 1] - path->Tht[pointIndex - 2]));
    if (deltaTheta > deltaThetaThresh) {
      validPath = false;
    }
    pointIndex = pointIndex + 1;
  }
  if (validPath) {
    path->Tht[path->PathN - 1] = path->Tht[path->PathN - 2];
    validPath = apaParallelReplanInSlotProcessPathEnd(path, endPose);
  }

  if (validPath) {
    path->ValidFlag = true;
    path->ErrorType = APA_FAULT_NO_ERR;
  }
  return validPath;
}

static Boolean apaParallelReplanInSlotBuildCubicPathBetweenPoses(const Real startPose[3],
                                                                 const Real endPose[3],
                                                                 Real rminInput,
                                                                 ApaPath_t *path) {
  Real startPoseUse[3];
  Real endPoseUse[3];
  Real endPoseLocal[3];
  ApaPath_t localPath;
  Integer pointIndex;
  Real pathLenLimit;
  Real rminUse;
  Boolean success0;

  startPoseUse[0] = startPose[0];
  startPoseUse[1] = startPose[1];
  startPoseUse[2] = apaCoordWrapToPi(startPose[2]);
  endPoseUse[0] = endPose[0];
  endPoseUse[1] = endPose[1];
  endPoseUse[2] = apaCoordWrapToPi(endPose[2]);

  apaCoordGround2Veh(startPoseUse[0],
                     startPoseUse[1],
                     startPoseUse[2],
                     endPoseUse[0],
                     endPoseUse[1],
                     endPoseUse[2],
                     &endPoseLocal[0],
                     &endPoseLocal[1],
                     &endPoseLocal[2]);

  success0 = apaParallelReplanInSlotDiscretizeDoubleCubic(
      (const Real[3]){0.0f, 0.0f, 0.0f}, endPoseLocal, &localPath);

  rminUse = apaMathAbs(rminInput);
  if (success0) {
    pathLenLimit =
        2.0f * sqrtf((startPoseUse[0] - endPoseUse[0]) * (startPoseUse[0] - endPoseUse[0]) +
                     (startPoseUse[1] - endPoseUse[1]) * (startPoseUse[1] - endPoseUse[1]));
    pathLenLimit =
        pathLenLimit + 2.0f * rminUse * fabsf(apaCoordWrapToPi(startPoseUse[2] - endPoseUse[2]));
    if (pathLenLimit < APA_PARALLEL_REPLAN_CUBIC_MAX_PATH_LEN) {
      pathLenLimit = APA_PARALLEL_REPLAN_CUBIC_MAX_PATH_LEN;
    }
    success0 = (apaParallelReplanInSlotCalcPathLen(&localPath) <= pathLenLimit);
  }

  if (success0) {
    *path = localPath;
    pointIndex = 0;
    while (pointIndex < path->PathN && pointIndex < APA_PATH_MAX_POINTS) {
      Real globalX;
      Real globalY;
      Real globalTheta;

      apaCoordVeh2Ground(startPoseUse[0],
                         startPoseUse[1],
                         startPoseUse[2],
                         path->X[pointIndex],
                         path->Y[pointIndex],
                         path->Tht[pointIndex],
                         &globalX,
                         &globalY,
                         &globalTheta);
      path->X[pointIndex] = globalX;
      path->Y[pointIndex] = globalY;
      path->Tht[pointIndex] = globalTheta;
      pointIndex = pointIndex + 1;
    }
  }

  if (success0) {
    success0 =
        apaPathCurvatureCheck(path->X, path->Y, path->OdoStep, path->PathN, rminUse, path->Radius);
  }
  if (success0) {
    path->RadiusFlag = true;
  }
  return success0;
}

static Boolean apaParallelReplanInSlotBuildCubicPath(const ApaParallelReplanInSlotInput_t *input,
                                                     ApaPath_t *path) {
  Real startPose[3];
  Real goalPose[3];

  startPose[0] = input->StartX;
  startPose[1] = input->StartY;
  startPose[2] = apaCoordWrapToPi(input->StartTheta);
  goalPose[0] = input->GoalX;
  goalPose[1] = input->GoalY;
  goalPose[2] = apaCoordWrapToPi(input->GoalTheta);

  return apaParallelReplanInSlotBuildCubicPathBetweenPoses(startPose, goalPose, input->Rmin, path);
}

static void apaParallelReplanInSlotCopyApaPathOut(const ApaPath_t *path,
                                                  ApaParallelReplanInSlotOutput_t *output) {
  Integer pointIndex;

  if (path->ValidFlag && path->PathN > 1) {
    output->Success = 1;
  } else {
    output->Success = 0;
  }
  output->PtCount = path->PathN;
  output->PathLen = apaParallelReplanInSlotCalcPathLen(path);

  pointIndex = 0;
  while (pointIndex < path->PathN && pointIndex < APA_PARALLEL_REPLAN_PATH_POINT_MAX) {
    output->PthPtX[pointIndex] = path->X[pointIndex];
    output->PthPtY[pointIndex] = path->Y[pointIndex];
    output->PthPtTht[pointIndex] = path->Tht[pointIndex];
    output->PthPtOdo[pointIndex] = path->OdoStep[pointIndex];
    pointIndex = pointIndex + 1;
  }
}

static void apaParallelReplanInSlotPathInit(ApaParallelReplanPath_t *path,
                                            Real startX,
                                            Real startY,
                                            Real startTheta) {
  *path = (ApaParallelReplanPath_t){0};
  path->PtCount = 1;
  path->X[0] = startX;
  path->Y[0] = startY;
  path->Theta[0] = startTheta;
  path->Odo[0] = 0.0f;
}

static Boolean apaParallelReplanInSlotPathAppend(
    ApaParallelReplanPath_t *path, Real pointX, Real pointY, Real pointTheta, Real odoStep) {
  Integer pointIndex;
  Boolean appended0;

  pointIndex = path->PtCount;
  appended0 = (pointIndex < APA_PARALLEL_REPLAN_PATH_POINT_MAX);
  if (appended0) {
    path->X[pointIndex] = pointX;
    path->Y[pointIndex] = pointY;
    path->Theta[pointIndex] = apaCoordWrapToPi(pointTheta);
    path->Odo[pointIndex] = odoStep;
    if (pointIndex == 1) {
      path->Odo[0] = odoStep;
    }
    path->PathLen = path->PathLen + fabsf(odoStep);
    path->PtCount = pointIndex + 1;
  }
  return appended0;
}

static void apaParallelReplanInSlotPathFinalize(ApaParallelReplanPath_t *path) {
  if (path->PtCount > 0) {
    path->Odo[path->PtCount - 1] = 0.0f;
  }
}

static Boolean apaParallelReplanInSlotAppendStraight(ApaParallelReplanPath_t *path,
                                                     Real startX,
                                                     Real startY,
                                                     Real startTheta,
                                                     Real endX,
                                                     Real endY,
                                                     Real endTheta,
                                                     Integer goDir) {
  Real dx;
  Real dy;
  Real length;
  Real thetaDelta;
  Integer stepCount;
  Integer stepIndex;
  Real stepOdo;
  Boolean success0;

  dx = endX - startX;
  dy = endY - startY;
  length = sqrtf(dx * dx + dy * dy);
  success0 = true;
  if (length > 1.0e-6f) {
    thetaDelta = apaCoordWrapToPi(endTheta - startTheta);
    stepCount = (Integer)ceilf(length / APA_PARALLEL_REPLAN_POINT_STEP);
    if (stepCount < 1) {
      stepCount = 1;
    }
    stepOdo = ((Real)goDir) * length / (Real)stepCount;

    stepIndex = 1;
    while ((stepIndex <= stepCount) && success0) {
      Real ratio;

      ratio = (Real)stepIndex / (Real)stepCount;
      success0 = apaParallelReplanInSlotPathAppend(
          path, startX + dx * ratio, startY + dy * ratio, startTheta + thetaDelta * ratio, stepOdo);
      stepIndex = stepIndex + 1;
    }
  }
  return success0;
}

static Boolean apaParallelReplanInSlotAppendArc(ApaParallelReplanPath_t *path,
                                                Real startX,
                                                Real startY,
                                                Real startTheta,
                                                Real centerX,
                                                Real centerY,
                                                Real deltaTheta,
                                                Integer goDir,
                                                Real radius) {
  Real startAngle;
  Real arcLength;
  Integer stepCount;
  Integer stepIndex;
  Real stepOdo;
  Boolean success0;

  success0 = true;
  if ((radius > 1.0e-6f) && (fabsf(deltaTheta) > 1.0e-6f)) {
    startAngle = atan2f(startY - centerY, startX - centerX);
    arcLength = radius * fabsf(deltaTheta);
    stepCount = (Integer)ceilf(arcLength / APA_PARALLEL_REPLAN_POINT_STEP);
    if (stepCount < 1) {
      stepCount = 1;
    }
    stepOdo = ((Real)goDir) * arcLength / (Real)stepCount;

    stepIndex = 1;
    while ((stepIndex <= stepCount) && success0) {
      Real ratio;
      Real sweepAngle;

      ratio = (Real)stepIndex / (Real)stepCount;
      sweepAngle = startAngle + deltaTheta * ratio;
      success0 = apaParallelReplanInSlotPathAppend(path,
                                                   centerX + radius * cosf(sweepAngle),
                                                   centerY + radius * sinf(sweepAngle),
                                                   startTheta + deltaTheta * ratio,
                                                   stepOdo);
      stepIndex = stepIndex + 1;
    }
  }
  return success0;
}

static void apaParallelReplanInSlotCopyPathOut(const ApaParallelReplanPath_t *path,
                                               ApaParallelReplanInSlotOutput_t *output) {
  Integer index;

  if (path->PtCount > 1) {
    output->Success = 1;
  } else {
    output->Success = 0;
  }
  output->PtCount = path->PtCount;
  output->PathLen = path->PathLen;

  index = 0;
  while (index < path->PtCount && index < APA_PARALLEL_REPLAN_PATH_POINT_MAX) {
    output->PthPtX[index] = path->X[index];
    output->PthPtY[index] = path->Y[index];
    output->PthPtTht[index] = path->Theta[index];
    output->PthPtOdo[index] = path->Odo[index];
    index = index + 1;
  }
}

static Boolean apaParallelReplanInSlotAppendDiscretized(ApaParallelReplanPath_t *path,
                                                        const GeoPthDiscretVtOutput_t *discret,
                                                        Integer startIndex) {
  Integer pointIndex;
  Boolean success0;

  success0 = true;
  pointIndex = startIndex;
  while (pointIndex < discret->PtCount && pointIndex < APA_PARALLEL_REPLAN_PATH_POINT_MAX &&
         success0) {
    success0 = apaParallelReplanInSlotPathAppend(path,
                                                 discret->PthPtX[pointIndex],
                                                 discret->PthPtY[pointIndex],
                                                 discret->PthPtTht[pointIndex],
                                                 discret->PthPtOdo[pointIndex]);
    pointIndex = pointIndex + 1;
  }
  return success0;
}

static void apaParallelReplanInSlotBuildSlotPlanInput(const ApaParallelReplanInSlotInput_t *input,
                                                      Real startX,
                                                      Real startY,
                                                      Real startTheta,
                                                      GeoVtPlanningInSlotInput_t *slotPlanInput) {
  ApaPlannerVehConfig_t vehCfg;
  Integer stopperIndex;

  apaParallelReplanInSlotLoadVehConfig(&vehCfg);
  *slotPlanInput = (GeoVtPlanningInSlotInput_t){0};
  slotPlanInput->StartX = startX;
  slotPlanInput->StartY = startY;
  slotPlanInput->StartTheta = startTheta;
  slotPlanInput->GoalX = input->GoalX;
  slotPlanInput->GoalY = input->GoalY;
  slotPlanInput->GoalTheta = input->GoalTheta;
  slotPlanInput->R1 = apaMathAbs(input->Rmin);
  slotPlanInput->R2 = slotPlanInput->R1;
  slotPlanInput->VehLF = vehCfg.LF;
  slotPlanInput->VehLB = vehCfg.LB;
  slotPlanInput->VehW = vehCfg.W;
  slotPlanInput->SlotPointsX[0] = input->SltPt1X;
  slotPlanInput->SlotPointsY[0] = input->SltPt1Y;
  slotPlanInput->SlotPointsX[1] = input->SltPt2X;
  slotPlanInput->SlotPointsY[1] = input->SltPt2Y;
  slotPlanInput->SlotPointsX[2] = input->SltPt3X;
  slotPlanInput->SlotPointsY[2] = input->SltPt3Y;
  slotPlanInput->SlotPointsX[3] = input->SltPt4X;
  slotPlanInput->SlotPointsY[3] = input->SltPt4Y;

  stopperIndex = 0;
  while (stopperIndex < input->StopperPtNum && stopperIndex < VCC_MAX_SEPARATE_PTS) {
    slotPlanInput->SeparateStopperX[stopperIndex] = input->StopperPtX[stopperIndex];
    slotPlanInput->SeparateStopperY[stopperIndex] = input->StopperPtY[stopperIndex];
    stopperIndex = stopperIndex + 1;
  }
  slotPlanInput->SeparateStopperNum = stopperIndex;
}

static Boolean
apaParallelReplanInSlotRunSlotBottomFromPose(const ApaParallelReplanInSlotInput_t *input,
                                             const ApaParallelReplanInSlotParam_t *param,
                                             Real startX,
                                             Real startY,
                                             Real startTheta,
                                             GeoPthDiscretVtOutput_t *discretOutput) {
  GeoVtPlanningInSlotInput_t slotPlanInput;
  GeoVtPlanningInSlotOutput_t slotPlanOutput;
  GeoPthDiscretVtInput_t discretInput;
  Real pathLen;
  Integer segmentIndex;
  Boolean success0;

  apaParallelReplanInSlotBuildSlotPlanInput(input, startX, startY, startTheta, &slotPlanInput);
  success0 = (slotPlanInput.R1 > 1.0e-6f);
  if (success0) {
    slotPlanOutput = (GeoVtPlanningInSlotOutput_t){0};
    geoVtPlanningInSlot(&slotPlanInput, &slotPlanOutput);
    success0 = slotPlanOutput.Success;
  }

  if (success0) {
    discretInput = (GeoPthDiscretVtInput_t){0};
    discretInput.VehPosX = startX;
    discretInput.VehPosY = startY;
    discretInput.VehPosTheta = startTheta;
    discretInput.TarPtX = input->GoalX;
    discretInput.TarPtY = input->GoalY;
    discretInput.TarPtTheta = input->GoalTheta;
    discretInput.SegNum = 3;
    discretInput.ShortCutSw = param->ShortCutSw;
    discretInput.ShortCutThd = param->ShortCutThd;

    pathLen = 0.0f;
    segmentIndex = 0;
    while (segmentIndex < 3) {
      discretInput.SegGoDir[segmentIndex] = slotPlanOutput.GoDirs[segmentIndex];
      discretInput.SegTrnDir[segmentIndex] = slotPlanOutput.TurnDirs[segmentIndex];
      discretInput.SegCrv[segmentIndex] = slotPlanOutput.Curves[segmentIndex];
      discretInput.SegTht[segmentIndex] = slotPlanOutput.DThetas[segmentIndex];
      discretInput.SegLen[segmentIndex] = slotPlanOutput.Lengths[segmentIndex];
      discretInput.SegStX[segmentIndex] = slotPlanOutput.AnchorsX[segmentIndex];
      discretInput.SegStY[segmentIndex] = slotPlanOutput.AnchorsY[segmentIndex];
      discretInput.SegStTh[segmentIndex] = slotPlanOutput.AnchorsTheta[segmentIndex];
      discretInput.SegCcX[segmentIndex] = slotPlanOutput.CentersX[segmentIndex];
      discretInput.SegCcY[segmentIndex] = slotPlanOutput.CentersY[segmentIndex];
      pathLen = pathLen + fabsf(slotPlanOutput.Lengths[segmentIndex]);
      segmentIndex = segmentIndex + 1;
    }
    discretInput.PathLen = pathLen;

    *discretOutput = (GeoPthDiscretVtOutput_t){0};
    { geoPthDiscretVt(&discretInput, discretOutput); }
    success0 = (discretOutput->PtCount > 0);
  }
  return success0;
}

static void apaParallelReplanInSlotLoadPlannerConfig(ApaPlannerConfig_t *plannerCfg) {
  *plannerCfg = (ApaPlannerConfig_t){0};
  apaPlannerConfigInitDefault(plannerCfg);
}

static Boolean apaParallelReplanInSlotPathLengthValid(const ApaParallelReplanPath_t *path) {
  ApaPlannerConfig_t plannerCfg;
  Boolean valid0;

  apaParallelReplanInSlotLoadPlannerConfig(&plannerCfg);
  valid0 = (path->PtCount >= 2) && (path->PathLen >= plannerCfg.Conf.PathLenMin) &&
           (path->PathLen <= plannerCfg.Conf.PathLenMax);
  return valid0;
}

static void apaParallelReplanInSlotPathGetEndPose(const ApaParallelReplanPath_t *path,
                                                  const Real fallbackPose[3],
                                                  Real endPose[3]) {
  if (path->PtCount > 0) {
    endPose[0] = path->X[path->PtCount - 1];
    endPose[1] = path->Y[path->PtCount - 1];
    endPose[2] = path->Theta[path->PtCount - 1];
  } else {
    endPose[0] = fallbackPose[0];
    endPose[1] = fallbackPose[1];
    endPose[2] = fallbackPose[2];
  }
}

static Boolean apaParallelReplanInSlotCubicFitEntryAllowed(const Real startPose[3],
                                                           const Real endPose[3]) {
  Real startLocal[3];
  Boolean allowed0;

  apaCoordGroundToVehPose(endPose, startPose, startLocal);
  allowed0 = (fabsf(startLocal[0]) <= APA_PARALLEL_REPLAN_CUBIC_ENTRY_LOCAL_X_MAX) &&
             (fabsf(startLocal[1]) <= APA_PARALLEL_REPLAN_CUBIC_ENTRY_LOCAL_Y_MAX) &&
             (fabsf(startLocal[2]) <= APA_PARALLEL_REPLAN_CUBIC_ENTRY_LOCAL_THETA_MAX);
  return allowed0;
}

static Real
apaParallelReplanInSlotResolvePolyfitPathLenMin(const ApaParallelReplanInSlotInput_t *input) {
  ApaPlannerConfig_t plannerCfg;
  SlotSurroundingAreaCheckInput_t surroundInput;
  SlotSurroundingAreaCheckOutput_t surroundOutput;
  Integer boundaryIndex;
  Integer pointIndex;
  Integer stopperIndex;
  Integer virtualIndex;
  Real polyfitPathLenMin;

  apaParallelReplanInSlotLoadPlannerConfig(&plannerCfg);
  polyfitPathLenMin = plannerCfg.Conf.PathLenMin;

  surroundInput = (SlotSurroundingAreaCheckInput_t){0};
  surroundInput.TarPtX = input->GoalX;
  surroundInput.TarPtY = input->GoalY;
  surroundInput.TarPtTheta = input->GoalTheta;
  surroundInput.SltPt1X = input->SltPt1X;
  surroundInput.SltPt1Y = input->SltPt1Y;
  surroundInput.SltPt2X = input->SltPt2X;
  surroundInput.SltPt2Y = input->SltPt2Y;
  surroundInput.SltPt3X = input->SltPt3X;
  surroundInput.SltPt3Y = input->SltPt3Y;
  surroundInput.SltPt4X = input->SltPt4X;
  surroundInput.SltPt4Y = input->SltPt4Y;
  surroundInput.SltEntr = 1;

  boundaryIndex = 0;
  while ((boundaryIndex < input->BoundaryLineNum) && (boundaryIndex < SURD_LINE_MAX)) {
    surroundInput.BoundaryP1X[boundaryIndex] = input->BoundaryLineX1[boundaryIndex];
    surroundInput.BoundaryP1Y[boundaryIndex] = input->BoundaryLineY1[boundaryIndex];
    surroundInput.BoundaryP2X[boundaryIndex] = input->BoundaryLineX2[boundaryIndex];
    surroundInput.BoundaryP2Y[boundaryIndex] = input->BoundaryLineY2[boundaryIndex];
    boundaryIndex = boundaryIndex + 1;
  }
  surroundInput.BoundaryLineNum = boundaryIndex;

  pointIndex = 0;
  while ((pointIndex < input->BoundarySepPtNum) && (pointIndex < SURD_PT_MAX)) {
    surroundInput.BoundarySepPtX[pointIndex] = input->BoundarySepPtX[pointIndex];
    surroundInput.BoundarySepPtY[pointIndex] = input->BoundarySepPtY[pointIndex];
    pointIndex = pointIndex + 1;
  }
  surroundInput.BoundarySepPtNum = pointIndex;

  stopperIndex = 0;
  while (stopperIndex < input->StopperPtNum && stopperIndex < VCC_MAX_SEPARATE_PTS) {
    surroundInput.SeparateStopperX[stopperIndex] = input->StopperPtX[stopperIndex];
    surroundInput.SeparateStopperY[stopperIndex] = input->StopperPtY[stopperIndex];
    stopperIndex = stopperIndex + 1;
  }
  surroundInput.SeparateStopperNum = stopperIndex;

  virtualIndex = 0;
  while ((virtualIndex < input->VirtualBoundaryLineNum) && (virtualIndex < VCC_MAX_VIRTUAL_LINES)) {
    surroundInput.VirtualBoundaryLineX1[virtualIndex] = input->VirtualBoundaryLineX1[virtualIndex];
    surroundInput.VirtualBoundaryLineY1[virtualIndex] = input->VirtualBoundaryLineY1[virtualIndex];
    surroundInput.VirtualBoundaryLineX2[virtualIndex] = input->VirtualBoundaryLineX2[virtualIndex];
    surroundInput.VirtualBoundaryLineY2[virtualIndex] = input->VirtualBoundaryLineY2[virtualIndex];
    virtualIndex = virtualIndex + 1;
  }
  surroundInput.VirtualBoundaryLineNum = virtualIndex;

  surroundOutput = (SlotSurroundingAreaCheckOutput_t){0};
  slotSurroundingAreaCheck(&surroundInput, &surroundOutput);
  if ((surroundOutput.SurroundingType == APA_SURROUND_OBS_EXIST_RIGHT_REAR_ONLY) ||
      (surroundOutput.SurroundingType == APA_SURROUND_OBS_EXIST_NONE)) {
    polyfitPathLenMin = 2.5f;
  }
  return polyfitPathLenMin;
}

static Boolean apaParallelReplanInSlotTryDirectCubicFit(const ApaParallelReplanInSlotInput_t *input,
                                                        const Real startPose[3],
                                                        const Real endPose[3],
                                                        Real polyfitPathLenMin,
                                                        ApaParallelReplanPath_t *pathSeg) {
  VehicleCollisionCheckInput_t vccInput;
  ApaPath_t cubicPath;
  Real startLocal[3];
  Boolean success0;

  success0 = apaParallelReplanInSlotCubicFitEntryAllowed(startPose, endPose);
  if (success0) {
    apaCoordGroundToVehPose(endPose, startPose, startLocal);
    success0 = (fabsf(startLocal[0]) > polyfitPathLenMin);
  }
  if (success0) {
    success0 = apaParallelReplanInSlotBuildCubicPathBetweenPoses(
        startPose, endPose, input->Rmin, &cubicPath);
  }

  if (success0) {
    apaParallelReplanInSlotBuildCollisionInput(input, &vccInput);
    success0 = apaParallelReplanInSlotPathCollisionFree(&vccInput, &cubicPath);
  }

  if (success0) {
    success0 = apaParallelReplanInSlotCopyApaPathToCustomPath(&cubicPath, pathSeg);
  }
  return success0;
}

static Boolean apaParallelReplanInSlotTrySplitCubicFit(const ApaParallelReplanInSlotInput_t *input,
                                                       const Real startPose[3],
                                                       const Real endPose[3],
                                                       Real polyfitPathLenMin,
                                                       ApaParallelReplanPath_t *pathSeg) {
  Real startLocal[3];
  Real sBias;
  Boolean success0;
  Boolean eligible0;

  eligible0 = apaParallelReplanInSlotCubicFitEntryAllowed(startPose, endPose);
  success0 = false;
  sBias = APA_PARALLEL_REPLAN_CUBIC_MID_S_MAX;
  if (eligible0) {
    apaCoordGroundToVehPose(endPose, startPose, startLocal);
    sBias = polyfitPathLenMin;
    if (startLocal[0] > 0.0f) {
      sBias = startLocal[0] + polyfitPathLenMin;
    }
  }

  while (eligible0 && (sBias < APA_PARALLEL_REPLAN_CUBIC_MID_S_MAX) && (success0 == false)) {
    Real midLocal[3];
    Real midPose[3];
    ApaParallelReplanPath_t firstSeg;
    ApaParallelReplanPath_t secondSeg;
    ApaParallelReplanPath_t candidatePath;

    midLocal[0] = sBias;
    midLocal[1] = 0.5f * startLocal[1];
    midLocal[2] = 0.5f * startLocal[2];
    apaCoordVehToGroundPose(endPose, midLocal, midPose);

    if (apaParallelReplanInSlotTryDirectCubicFit(
            input, startPose, midPose, polyfitPathLenMin, &firstSeg) &&
        apaParallelReplanInSlotTryDirectCubicFit(
            input, midPose, endPose, polyfitPathLenMin, &secondSeg)) {
      candidatePath = firstSeg;
      if (apaParallelReplanInSlotPathAppendPath(&candidatePath, &secondSeg)) {
        apaParallelReplanInSlotPathFinalize(&candidatePath);
        if (apaParallelReplanInSlotPathLengthValid(&candidatePath)) {
          *pathSeg = candidatePath;
          success0 = true;
        }
      }
    }

    sBias = sBias + APA_PARALLEL_REPLAN_CUBIC_MID_S_STEP;
  }

  return success0;
}

static Boolean apaParallelReplanInSlotPathAppendPath(ApaParallelReplanPath_t *dst,
                                                     const ApaParallelReplanPath_t *src) {
  Integer pointIndex;
  Real endDx;
  Real endDy;
  Real endDistance;
  Boolean success0;

  success0 = true;

  if ((src->PtCount > 1) && (dst->PtCount > 0)) {
    endDx = dst->X[dst->PtCount - 1] - src->X[0];
    endDy = dst->Y[dst->PtCount - 1] - src->Y[0];
    endDistance = sqrtf(endDx * endDx + endDy * endDy);
    if (endDistance > 1.0e-3f) {
      success0 = false;
    }
  }

  pointIndex = 1;
  while ((pointIndex < src->PtCount) && success0) {
    Real dist;
    Real odoSignSource;
    Integer dirSign;

    dist = sqrtf((src->X[pointIndex] - src->X[pointIndex - 1]) *
                     (src->X[pointIndex] - src->X[pointIndex - 1]) +
                 (src->Y[pointIndex] - src->Y[pointIndex - 1]) *
                     (src->Y[pointIndex] - src->Y[pointIndex - 1]));
    odoSignSource = src->Odo[pointIndex];
    if ((fabsf(odoSignSource) <= 1.0e-6f) && (pointIndex > 0)) {
      odoSignSource = src->Odo[pointIndex - 1];
    }
    if (odoSignSource >= 0.0f) {
      dirSign = 1;
    } else {
      dirSign = -1;
    }
    success0 = apaParallelReplanInSlotPathAppend(dst,
                                                 src->X[pointIndex],
                                                 src->Y[pointIndex],
                                                 src->Theta[pointIndex],
                                                 ((Real)dirSign) * dist);
    pointIndex = pointIndex + 1;
  }
  return success0;
}

static Boolean
apaParallelReplanInSlotPathCollisionFreeCustom(const ApaParallelReplanInSlotInput_t *input,
                                               const ApaParallelReplanPath_t *path) {
  VehicleCollisionCheckInput_t vccInput;
  VehicleCollisionCheckOutput_t vccOutput;
  Integer pointIndex;
  Boolean collisionFree0;

  apaParallelReplanInSlotBuildCollisionInput(input, &vccInput);
  collisionFree0 = true;
  pointIndex = 0;
  while ((pointIndex < path->PtCount) && collisionFree0) {
    vccInput.VehPosX = path->X[pointIndex];
    vccInput.VehPosY = path->Y[pointIndex];
    vccInput.VehPosTheta = path->Theta[pointIndex];
    vccOutput = (VehicleCollisionCheckOutput_t){0};
    vehicleCollisionCheck(&vccInput, &vccOutput);
    if (vccOutput.IsCollision) {
      collisionFree0 = false;
    }
    pointIndex = pointIndex + 1;
  }
  return collisionFree0;
}

static Boolean apaParallelReplanInSlotCopyApaPathToCustomPath(const ApaPath_t *src,
                                                              ApaParallelReplanPath_t *dst) {
  Integer pointIndex;
  Boolean success0;

  success0 = src->ValidFlag && (src->PathN >= 2);
  if (success0) {
    apaParallelReplanInSlotPathInit(dst, src->X[0], src->Y[0], src->Tht[0]);
    pointIndex = 1;
    while ((pointIndex < src->PathN) && (pointIndex < APA_PATH_MAX_POINTS) && success0) {
      success0 = apaParallelReplanInSlotPathAppend(dst,
                                                   src->X[pointIndex],
                                                   src->Y[pointIndex],
                                                   src->Tht[pointIndex],
                                                   src->OdoStep[pointIndex - 1]);
      pointIndex = pointIndex + 1;
    }
    if (success0) {
      apaParallelReplanInSlotPathFinalize(dst);
      success0 = apaParallelReplanInSlotPathLengthValid(dst);
    }
  }
  return success0;
}

static void apaParallelReplanInSlotBuildSlotPlanInputEx(const ApaParallelReplanInSlotInput_t *input,
                                                        const Real startPose[3],
                                                        const Real goalPose[3],
                                                        GeoVtPlanningInSlotInput_t *slotPlanInput) {
  ApaPlannerVehConfig_t vehCfg;
  Integer stopperIndex;

  apaParallelReplanInSlotLoadVehConfig(&vehCfg);
  *slotPlanInput = (GeoVtPlanningInSlotInput_t){0};
  slotPlanInput->StartX = startPose[0];
  slotPlanInput->StartY = startPose[1];
  slotPlanInput->StartTheta = startPose[2];
  slotPlanInput->GoalX = goalPose[0];
  slotPlanInput->GoalY = goalPose[1];
  slotPlanInput->GoalTheta = goalPose[2];
  slotPlanInput->R1 = apaMathAbs(input->Rmin);
  slotPlanInput->R2 = slotPlanInput->R1;
  slotPlanInput->VehLF = vehCfg.LF;
  slotPlanInput->VehLB = vehCfg.LB;
  slotPlanInput->VehW = vehCfg.W;
  slotPlanInput->SlotPointsX[0] = input->SltPt1X;
  slotPlanInput->SlotPointsY[0] = input->SltPt1Y;
  slotPlanInput->SlotPointsX[1] = input->SltPt2X;
  slotPlanInput->SlotPointsY[1] = input->SltPt2Y;
  slotPlanInput->SlotPointsX[2] = input->SltPt3X;
  slotPlanInput->SlotPointsY[2] = input->SltPt3Y;
  slotPlanInput->SlotPointsX[3] = input->SltPt4X;
  slotPlanInput->SlotPointsY[3] = input->SltPt4Y;

  stopperIndex = 0;
  while (stopperIndex < input->StopperPtNum && stopperIndex < VCC_MAX_SEPARATE_PTS) {
    slotPlanInput->SeparateStopperX[stopperIndex] = input->StopperPtX[stopperIndex];
    slotPlanInput->SeparateStopperY[stopperIndex] = input->StopperPtY[stopperIndex];
    stopperIndex = stopperIndex + 1;
  }
  slotPlanInput->SeparateStopperNum = stopperIndex;
}

static Boolean
apaParallelReplanInSlotRunSlotBottomBetweenPoses(const ApaParallelReplanInSlotInput_t *input,
                                                 const ApaParallelReplanInSlotParam_t *param,
                                                 const Real startPose[3],
                                                 const Real goalPose[3],
                                                 GeoPthDiscretVtOutput_t *discretOutput) {
  GeoVtPlanningInSlotInput_t slotPlanInput;
  GeoVtPlanningInSlotOutput_t slotPlanOutput;
  GeoPthDiscretVtInput_t discretInput;
  Real pathLen;
  Integer segmentIndex;
  Boolean success0;

  apaParallelReplanInSlotBuildSlotPlanInputEx(input, startPose, goalPose, &slotPlanInput);
  success0 = (slotPlanInput.R1 > 1.0e-6f);
  if (success0) {
    slotPlanOutput = (GeoVtPlanningInSlotOutput_t){0};
    geoVtPlanningInSlot(&slotPlanInput, &slotPlanOutput);
    success0 = slotPlanOutput.Success;
  }

  if (success0) {
    discretInput = (GeoPthDiscretVtInput_t){0};
    discretInput.VehPosX = startPose[0];
    discretInput.VehPosY = startPose[1];
    discretInput.VehPosTheta = startPose[2];
    discretInput.TarPtX = goalPose[0];
    discretInput.TarPtY = goalPose[1];
    discretInput.TarPtTheta = goalPose[2];
    discretInput.SegNum = 3;
    discretInput.ShortCutSw = param->ShortCutSw;
    discretInput.ShortCutThd = param->ShortCutThd;

    pathLen = 0.0f;
    segmentIndex = 0;
    while (segmentIndex < 3) {
      discretInput.SegGoDir[segmentIndex] = slotPlanOutput.GoDirs[segmentIndex];
      discretInput.SegTrnDir[segmentIndex] = slotPlanOutput.TurnDirs[segmentIndex];
      discretInput.SegCrv[segmentIndex] = slotPlanOutput.Curves[segmentIndex];
      discretInput.SegTht[segmentIndex] = slotPlanOutput.DThetas[segmentIndex];
      discretInput.SegLen[segmentIndex] = slotPlanOutput.Lengths[segmentIndex];
      discretInput.SegStX[segmentIndex] = slotPlanOutput.AnchorsX[segmentIndex];
      discretInput.SegStY[segmentIndex] = slotPlanOutput.AnchorsY[segmentIndex];
      discretInput.SegStTh[segmentIndex] = slotPlanOutput.AnchorsTheta[segmentIndex];
      discretInput.SegCcX[segmentIndex] = slotPlanOutput.CentersX[segmentIndex];
      discretInput.SegCcY[segmentIndex] = slotPlanOutput.CentersY[segmentIndex];
      pathLen = pathLen + fabsf(slotPlanOutput.Lengths[segmentIndex]);
      segmentIndex = segmentIndex + 1;
    }
    discretInput.PathLen = pathLen;

    *discretOutput = (GeoPthDiscretVtOutput_t){0};
    { geoPthDiscretVt(&discretInput, discretOutput); }
    success0 = (discretOutput->PtCount > 0);
  }
  return success0;
}

static Boolean
apaParallelReplanInSlotResolveSlotOnRight(const ApaParallelReplanInSlotInput_t *input) {
  Real localPt3X;
  Real localPt3Y;
  Real localPt3Theta;
  Real localPt4X;
  Real localPt4Y;
  Real localPt4Theta;
  Boolean slotOnRight;

  slotOnRight = false;
  apaCoordGround2Veh(input->StartX,
                     input->StartY,
                     input->StartTheta,
                     input->SltPt3X,
                     input->SltPt3Y,
                     0.0f,
                     &localPt3X,
                     &localPt3Y,
                     &localPt3Theta);
  apaCoordGround2Veh(input->StartX,
                     input->StartY,
                     input->StartTheta,
                     input->SltPt4X,
                     input->SltPt4Y,
                     0.0f,
                     &localPt4X,
                     &localPt4Y,
                     &localPt4Theta);
  if (localPt3Y < 0.0f || localPt4Y < 0.0f) {
    slotOnRight = true;
  }
  return slotOnRight;
}

static void apaParallelReplanInSlotGetBackSlotPoints(const ApaParallelReplanInSlotInput_t *input,
                                                     Real backSlotPtNear[2],
                                                     Real backSlotPtAway[2]) {
  backSlotPtNear[0] = input->SltPt2X;
  backSlotPtNear[1] = input->SltPt2Y;
  backSlotPtAway[0] = input->SltPt3X;
  backSlotPtAway[1] = input->SltPt3Y;
}

static Boolean apaParallelReplanInSlotSatisfyHeadingErr(const ApaParallelReplanInSlotInput_t *input,
                                                        const Real pathEnd[3],
                                                        Real headingThresh,
                                                        Real *headingErr) {
  Real goalErrX;
  Real goalErrY;
  Real goalErrTheta;

  apaCoordGround2Veh(input->GoalX,
                     input->GoalY,
                     input->GoalTheta,
                     pathEnd[0],
                     pathEnd[1],
                     pathEnd[2],
                     &goalErrX,
                     &goalErrY,
                     &goalErrTheta);
  *headingErr = goalErrTheta;
  return fabsf(goalErrTheta) <= headingThresh;
}

static Boolean apaParallelReplanInSlotSatisfyLatErr(const ApaParallelReplanInSlotInput_t *input,
                                                    const Real pathEnd[3],
                                                    Real latThresh,
                                                    Boolean slotOnRight,
                                                    const Real backSlotPtNear[2],
                                                    const Real backSlotPtAway[2],
                                                    Real *latErr) {
  ApaPlannerVehConfig_t vehCfg;
  Real goalErrX;
  Real goalErrY;
  Real goalErrTheta;
  Real nearLocalX;
  Real nearLocalY;
  Real nearLocalTheta;
  Real awayLocalX;
  Real awayLocalY;
  Real awayLocalTheta;
  Real outsideMaxOffset;
  Real insideMaxOffset;
  Boolean satisfy0;

  apaParallelReplanInSlotLoadVehConfig(&vehCfg);
  apaCoordGround2Veh(input->GoalX,
                     input->GoalY,
                     input->GoalTheta,
                     pathEnd[0],
                     pathEnd[1],
                     pathEnd[2],
                     &goalErrX,
                     &goalErrY,
                     &goalErrTheta);
  *latErr = goalErrY;
  if ((input->IsHandleSlot != 0) || (input->SlotSource != 0)) {
    latThresh = 0.5f;
  }
  satisfy0 = (fabsf(goalErrY) <= latThresh);

  if (satisfy0 && (input->IsHandleSlot == 0) && (input->SlotSource == 0)) {
    apaCoordGround2Veh(input->GoalX,
                       input->GoalY,
                       input->GoalTheta,
                       backSlotPtNear[0],
                       backSlotPtNear[1],
                       0.0f,
                       &nearLocalX,
                       &nearLocalY,
                       &nearLocalTheta);
    apaCoordGround2Veh(input->GoalX,
                       input->GoalY,
                       input->GoalTheta,
                       backSlotPtAway[0],
                       backSlotPtAway[1],
                       0.0f,
                       &awayLocalX,
                       &awayLocalY,
                       &awayLocalTheta);

    outsideMaxOffset = fabsf(nearLocalY) - 0.5f * vehCfg.W;
    insideMaxOffset = fabsf(awayLocalY) - 0.5f * vehCfg.W;
    if (slotOnRight) {
      if (outsideMaxOffset > 0.0f && goalErrY > 0.0f && goalErrY > 0.8f * outsideMaxOffset) {
        satisfy0 = false;
      }
      if (insideMaxOffset > 0.0f && goalErrY < 0.0f && fabsf(goalErrY) > 0.5f * insideMaxOffset) {
        satisfy0 = false;
      }
    } else {
      if (outsideMaxOffset > 0.0f && goalErrY < 0.0f && fabsf(goalErrY) > 0.8f * outsideMaxOffset) {
        satisfy0 = false;
      }
      if (insideMaxOffset > 0.0f && goalErrY > 0.0f && goalErrY > 0.5f * insideMaxOffset) {
        satisfy0 = false;
      }
    }
  }
  return satisfy0;
}

static Boolean
apaParallelReplanInSlotSampleCollisionFree(const VehicleCollisionCheckInput_t *vccTemplate,
                                           Real posX,
                                           Real posY,
                                           Real posTheta,
                                           Integer allowSegNum,
                                           Integer direction,
                                           Real travelDistance,
                                           Integer *collisionType) {
  VehicleCollisionCheckInput_t vccInput;
  VehicleCollisionCheckOutput_t vccOutput;
  Boolean allowSoftStopper;
  Boolean collisionFree0;

  vccInput = *vccTemplate;
  vccInput.VehPosX = posX;
  vccInput.VehPosY = posY;
  vccInput.VehPosTheta = posTheta;
  vccOutput = (VehicleCollisionCheckOutput_t){0};
  vehicleCollisionCheck(&vccInput, &vccOutput);
  *collisionType = vccOutput.CollisionType;
  allowSoftStopper = false;
  if (vccOutput.IsCollision) {
    if (direction == 1 && allowSegNum == 0 && vccOutput.CollisionType == VCC_COLLISION_STOPPER &&
        travelDistance < APA_PARALLEL_REPLAN_MIN_FORWARD_COLLISION_SOFT_DIST) {
      allowSoftStopper = true;
    }
  }
  collisionFree0 = (!vccOutput.IsCollision) || allowSoftStopper;
  return collisionFree0;
}

static Boolean apaParallelReplanInSlotShouldStopStraightForLateralError(Integer direction,
                                                                        Boolean slotOnRight,
                                                                        Real goalErrY) {
  Boolean shouldStop0;

  shouldStop0 = false;
  if (direction == -1) {
    if (slotOnRight && goalErrY < -APA_PARALLEL_REPLAN_MOVE_STOP_LAT) {
      shouldStop0 = true;
    }
    if ((!slotOnRight) && goalErrY > APA_PARALLEL_REPLAN_MOVE_STOP_LAT) {
      shouldStop0 = true;
    }
  } else {
    if (slotOnRight && goalErrY > APA_PARALLEL_REPLAN_MOVE_STOP_LAT) {
      shouldStop0 = true;
    }
    if ((!slotOnRight) && goalErrY < -APA_PARALLEL_REPLAN_MOVE_STOP_LAT) {
      shouldStop0 = true;
    }
  }
  return shouldStop0;
}

static Boolean
apaParallelReplanInSlotGenerateStraightSegment(const ApaParallelReplanInSlotInput_t *input,
                                               const Real startPose[3],
                                               Integer direction,
                                               Boolean slotOnRight,
                                               ApaParallelReplanPath_t *pathSeg) {
  VehicleCollisionCheckInput_t vccInput;
  Boolean success;
  Boolean buildOk0;
  Boolean segmentOpen;
  Real moveDis;
  Real stepOdo;

  success = false;
  buildOk0 = true;
  apaParallelReplanInSlotBuildCollisionInput(input, &vccInput);
  apaParallelReplanInSlotPathInit(pathSeg, startPose[0], startPose[1], startPose[2]);
  moveDis = APA_PARALLEL_REPLAN_FIRST_STEP;
  segmentOpen = true;
  while ((moveDis < APA_PARALLEL_REPLAN_SEGMENT_S_LIMIT) && buildOk0 && segmentOpen) {
    Real tmpX;
    Real tmpY;
    Real tmpTheta;
    Integer collisionType;
    Real goalErrX;
    Real goalErrY;
    Real goalErrTheta;

    tmpX = startPose[0] + moveDis * cosf(startPose[2]) * (Real)direction;
    tmpY = startPose[1] + moveDis * sinf(startPose[2]) * (Real)direction;
    tmpTheta = startPose[2];
    if (!apaParallelReplanInSlotSampleCollisionFree(
            &vccInput, tmpX, tmpY, tmpTheta, 0, direction, moveDis, &collisionType)) {
      segmentOpen = false;
    } else {
      if (moveDis <= APA_PARALLEL_REPLAN_FIRST_STEP + 1.0e-6f) {
        stepOdo = (Real)direction * APA_PARALLEL_REPLAN_FIRST_STEP;
      } else {
        stepOdo = (Real)direction * APA_PARALLEL_REPLAN_POINT_STEP;
      }
      if (!apaParallelReplanInSlotPathAppend(pathSeg, tmpX, tmpY, tmpTheta, stepOdo)) {
        buildOk0 = false;
      }

      if (buildOk0) {
        apaCoordGround2Veh(input->GoalX,
                           input->GoalY,
                           input->GoalTheta,
                           tmpX,
                           tmpY,
                           tmpTheta,
                           &goalErrX,
                           &goalErrY,
                           &goalErrTheta);
        if (apaParallelReplanInSlotShouldStopStraightForLateralError(
                direction, slotOnRight, goalErrY)) {
          segmentOpen = false;
        } else {
          moveDis = moveDis + APA_PARALLEL_REPLAN_POINT_STEP;
        }
      }
    }
  }

  if (buildOk0) {
    apaParallelReplanInSlotPathFinalize(pathSeg);
    if (apaParallelReplanInSlotPathLengthValid(pathSeg)) {
      success = true;
    }
  }
  return success;
}

static Real apaParallelReplanInSlotExploreCollisionDistance(
    const ApaParallelReplanInSlotInput_t *input, const Real startPose[3], Integer direction) {
  VehicleCollisionCheckInput_t vccInput;
  Real moveDis;
  Integer loopIndex;
  Real collisionDistance0;
  Boolean collisionFound0;

  apaParallelReplanInSlotBuildCollisionInput(input, &vccInput);
  moveDis = APA_PARALLEL_REPLAN_FIRST_STEP;
  loopIndex = 0;
  collisionDistance0 = 0.0f;
  collisionFound0 = false;
  while ((moveDis < APA_PARALLEL_REPLAN_COLLISION_EXPLORE_LIMIT) && (collisionFound0 == false)) {
    Real tmpX;
    Real tmpY;
    Real tmpTheta;
    VehicleCollisionCheckOutput_t vccOutput;

    loopIndex = loopIndex + 1;
    tmpX = startPose[0] + moveDis * cosf(startPose[2]) * (Real)direction;
    tmpY = startPose[1] + moveDis * sinf(startPose[2]) * (Real)direction;
    tmpTheta = startPose[2];
    vccInput.VehPosX = tmpX;
    vccInput.VehPosY = tmpY;
    vccInput.VehPosTheta = tmpTheta;
    vccOutput = (VehicleCollisionCheckOutput_t){0};
    vehicleCollisionCheck(&vccInput, &vccOutput);
    if (vccOutput.IsCollision) {
      if (loopIndex == 1) {
        collisionDistance0 = moveDis - APA_PARALLEL_REPLAN_FIRST_STEP;
      } else {
        collisionDistance0 = moveDis - APA_PARALLEL_REPLAN_POINT_STEP;
      }
      collisionFound0 = true;
    }
    moveDis = moveDis + APA_PARALLEL_REPLAN_POINT_STEP;
  }
  return collisionDistance0;
}

static Boolean
apaParallelReplanInSlotGenerateManeuverSegment(const ApaParallelReplanInSlotInput_t *input,
                                               const Real startPose[3],
                                               Integer direction,
                                               Boolean turnLeft,
                                               Integer segNum,
                                               ApaParallelReplanPath_t *pathSeg,
                                               Boolean *headingMatch) {
  VehicleCollisionCheckInput_t vccInput;
  ApaPlannerConfig_t plannerCfg;
  Real goalHeading;
  Real rminUse;
  Real s;
  Boolean success;
  Boolean buildOk0;
  Boolean maneuverOpen;

  *headingMatch = false;
  success = false;
  buildOk0 = true;
  goalHeading = input->GoalTheta;
  rminUse = apaMathAbs(input->Rmin);
  apaParallelReplanInSlotBuildCollisionInput(input, &vccInput);
  apaParallelReplanInSlotLoadPlannerConfig(&plannerCfg);
  apaParallelReplanInSlotPathInit(pathSeg, startPose[0], startPose[1], startPose[2]);

  s = APA_PARALLEL_REPLAN_FIRST_STEP;
  maneuverOpen = true;
  while ((s < APA_PARALLEL_REPLAN_SEGMENT_S_LIMIT) && buildOk0 && maneuverOpen) {
    Real turnAngle;
    Real relX;
    Real relY;
    Real relTheta;
    Real tmpPose[3];
    Real startPoseLocal[3];
    Real tmpPoseLocal[3];
    Real latErrExpandDis;
    Integer collisionType;
    Real stepOdo;

    turnAngle = s / apaMathDivideZeroProcess(rminUse);
    if (direction == 1) {
      if (turnLeft) {
        relX = rminUse * sinf(turnAngle);
        relY = rminUse * (1.0f - cosf(turnAngle));
        relTheta = turnAngle;
      } else {
        relX = rminUse * sinf(turnAngle);
        relY = -rminUse * (1.0f - cosf(turnAngle));
        relTheta = -turnAngle;
      }
    } else {
      if (turnLeft) {
        relX = -rminUse * sinf(turnAngle);
        relY = rminUse * (1.0f - cosf(turnAngle));
        relTheta = -turnAngle;
      } else {
        relX = -rminUse * sinf(turnAngle);
        relY = -rminUse * (1.0f - cosf(turnAngle));
        relTheta = turnAngle;
      }
    }
    apaCoordVehToGroundPose(startPose, (const Real[3]){relX, relY, relTheta}, tmpPose);
    if (!apaParallelReplanInSlotSampleCollisionFree(
            &vccInput, tmpPose[0], tmpPose[1], tmpPose[2], segNum, direction, s, &collisionType)) {
      maneuverOpen = false;
    } else {
      if (s <= APA_PARALLEL_REPLAN_FIRST_STEP + 1.0e-6f) {
        stepOdo = (Real)direction * APA_PARALLEL_REPLAN_FIRST_STEP;
      } else {
        stepOdo = (Real)direction * APA_PARALLEL_REPLAN_POINT_STEP;
      }
      if (!apaParallelReplanInSlotPathAppend(
              pathSeg, tmpPose[0], tmpPose[1], tmpPose[2], stepOdo)) {
        buildOk0 = false;
      }
    }
    if (buildOk0 && maneuverOpen &&
        (fabsf(apaCoordWrapToPi(tmpPose[2] - goalHeading)) <=
         APA_PARALLEL_REPLAN_MANEUVER_ANGLE_THRESH)) {
      *headingMatch = true;
      maneuverOpen = false;
    }

    if (buildOk0 && maneuverOpen) {
      apaCoordGroundToVehPose(
          (const Real[3]){input->GoalX, input->GoalY, input->GoalTheta}, startPose, startPoseLocal);
      apaCoordGroundToVehPose(
          (const Real[3]){input->GoalX, input->GoalY, input->GoalTheta}, tmpPose, tmpPoseLocal);
      latErrExpandDis = fabsf(fabsf(startPoseLocal[1]) - fabsf(tmpPoseLocal[1]));
      if (s > plannerCfg.Conf.PathLenMin && latErrExpandDis > APA_PARALLEL_REPLAN_LAT_EXPAND_STOP) {
        maneuverOpen = false;
      } else {
        s = s + APA_PARALLEL_REPLAN_POINT_STEP;
      }
    }
  }

  if (buildOk0 && *headingMatch && s < plannerCfg.Conf.PathLenMin && pathSeg->PtCount >= 2) {
    Real delta;
    Real tmpLocal[3];
    Real tmpPose[3];
    Integer collisionType;

    delta = plannerCfg.Conf.PathLenMin - s + APA_PARALLEL_REPLAN_SAFE_DELTA;
    tmpLocal[0] = pathSeg->X[pathSeg->PtCount - 1];
    tmpLocal[1] = pathSeg->Y[pathSeg->PtCount - 1];
    tmpLocal[2] = pathSeg->Theta[pathSeg->PtCount - 1];
    apaCoordGroundToVehPose(
        (const Real[3]){input->GoalX, input->GoalY, input->GoalTheta}, tmpLocal, tmpLocal);
    tmpLocal[0] = tmpLocal[0] + (Real)direction * delta;
    apaCoordVehToGroundPose(
        (const Real[3]){input->GoalX, input->GoalY, input->GoalTheta}, tmpLocal, tmpPose);
    if (!apaParallelReplanInSlotSampleCollisionFree(&vccInput,
                                                    tmpPose[0],
                                                    tmpPose[1],
                                                    tmpPose[2],
                                                    segNum,
                                                    direction,
                                                    s + delta,
                                                    &collisionType)) {
      buildOk0 = false;
    }
    if (buildOk0 && !apaParallelReplanInSlotPathAppend(
                        pathSeg, tmpPose[0], tmpPose[1], tmpPose[2], (Real)direction * delta)) {
      buildOk0 = false;
    }
  }

  if (buildOk0) {
    apaParallelReplanInSlotPathFinalize(pathSeg);
    if (apaParallelReplanInSlotPathLengthValid(pathSeg)) {
      success = true;
    }
  }
  return success;
}

static Boolean apaParallelReplanInSlotGeneratePathType1(const ApaParallelReplanInSlotInput_t *input,
                                                        Boolean slotOnRight,
                                                        ApaParallelReplanPath_t *path) {
  Real startPose[3];
  Real goalPose[3];
  Real startInSlotCs[3];
  Boolean wantMoveBackwardFirst;
  Boolean wantMoveForwardFirst;
  Boolean startTurnForward;
  Integer segNum;
  Integer loopCount;
  Boolean isHeadingMatch;
  Boolean success0;

  startPose[0] = input->StartX;
  startPose[1] = input->StartY;
  startPose[2] = input->StartTheta;
  goalPose[0] = input->GoalX;
  goalPose[1] = input->GoalY;
  goalPose[2] = input->GoalTheta;
  apaCoordGroundToVehPose(goalPose, startPose, startInSlotCs);
  apaParallelReplanInSlotPathInit(path, startPose[0], startPose[1], startPose[2]);

  wantMoveBackwardFirst = false;
  wantMoveForwardFirst = false;
  if (((startInSlotCs[1] > APA_PARALLEL_REPLAN_MOVE_SWITCH_LAT) &&
       (startInSlotCs[2] > apaMathDegToRad(15.0f)) && (startInSlotCs[2] < apaMathDegToRad(90.0f)) &&
       slotOnRight) ||
      ((startInSlotCs[1] < -APA_PARALLEL_REPLAN_MOVE_SWITCH_LAT) &&
       (startInSlotCs[2] < -apaMathDegToRad(15.0f)) &&
       (startInSlotCs[2] > -apaMathDegToRad(90.0f)) && (!slotOnRight))) {
    wantMoveBackwardFirst = true;
  }
  if (((startInSlotCs[1] < -APA_PARALLEL_REPLAN_MOVE_SWITCH_LAT) &&
       (startInSlotCs[2] > apaMathDegToRad(15.0f)) && (startInSlotCs[2] < apaMathDegToRad(90.0f)) &&
       slotOnRight) ||
      ((startInSlotCs[1] > APA_PARALLEL_REPLAN_MOVE_SWITCH_LAT) &&
       (startInSlotCs[2] < -apaMathDegToRad(15.0f)) &&
       (startInSlotCs[2] > -apaMathDegToRad(90.0f)) && (!slotOnRight))) {
    wantMoveForwardFirst = true;
  }

  if (slotOnRight) {
    startTurnForward =
        (startInSlotCs[1] < 0.0f) && (input->ExpectDirFlag != APA_EXP_DIREC_FORCE_BACKWARD);
  } else {
    startTurnForward =
        (startInSlotCs[1] > 0.0f) && (input->ExpectDirFlag != APA_EXP_DIREC_FORCE_BACKWARD);
  }

  segNum = 0;
  success0 = true;
  if ((input->ExpectDirFlag != APA_EXP_DIREC_FORCE_FORWARD) && wantMoveBackwardFirst) {
    ApaParallelReplanPath_t initSeg;

    if (apaParallelReplanInSlotGenerateStraightSegment(
            input, startPose, -1, slotOnRight, &initSeg)) {
      if (!apaParallelReplanInSlotPathAppendPath(path, &initSeg)) {
        success0 = false;
      }
      if (success0) {
        segNum = segNum + 1;
        startTurnForward = true;
      }
    } else {
      startTurnForward = false;
    }
  }
  if (success0 && (input->ExpectDirFlag != APA_EXP_DIREC_FORCE_BACKWARD) && wantMoveForwardFirst) {
    ApaParallelReplanPath_t initSeg;

    if (apaParallelReplanInSlotGenerateStraightSegment(
            input, startPose, 1, slotOnRight, &initSeg)) {
      apaParallelReplanInSlotPathInit(path, startPose[0], startPose[1], startPose[2]);
      if (!apaParallelReplanInSlotPathAppendPath(path, &initSeg)) {
        success0 = false;
      }
      if (success0) {
        segNum = 1;
        startTurnForward = false;
      }
    } else {
      startTurnForward = true;
    }
  }

  isHeadingMatch = false;
  loopCount = 0;
  while ((!isHeadingMatch) && success0) {
    Real tmpOrigin[3];
    ApaParallelReplanPath_t segPath;
    Boolean headingMatch;
    Boolean successFirst;
    Boolean successSecond;

    loopCount = loopCount + 1;
    if (loopCount > APA_PARALLEL_REPLAN_MANEUVER_LOOP_LIMIT) {
      success0 = false;
    }

    successFirst = false;
    if (success0) {
      apaParallelReplanInSlotPathGetEndPose(path, startPose, tmpOrigin);
      if (startTurnForward) {
        successFirst = apaParallelReplanInSlotGenerateManeuverSegment(
            input, tmpOrigin, 1, !slotOnRight, segNum, &segPath, &headingMatch);
      } else {
        successFirst = apaParallelReplanInSlotGenerateManeuverSegment(
            input, tmpOrigin, -1, slotOnRight, segNum, &segPath, &headingMatch);
      }
      if (successFirst) {
        success0 = apaParallelReplanInSlotPathAppendPath(path, &segPath);
        if (success0) {
          segNum = segNum + 1;
          isHeadingMatch = headingMatch;
        }
      }
    }

    if ((!isHeadingMatch) && success0) {
      apaParallelReplanInSlotPathGetEndPose(path, startPose, tmpOrigin);
      if (startTurnForward) {
        successSecond = apaParallelReplanInSlotGenerateManeuverSegment(
            input, tmpOrigin, -1, slotOnRight, segNum, &segPath, &headingMatch);
      } else {
        successSecond = apaParallelReplanInSlotGenerateManeuverSegment(
            input, tmpOrigin, 1, !slotOnRight, segNum, &segPath, &headingMatch);
      }
      if (successSecond) {
        success0 = apaParallelReplanInSlotPathAppendPath(path, &segPath);
        if (success0) {
          segNum = segNum + 1;
          isHeadingMatch = headingMatch;
        }
      }
      if ((!successFirst) && (!successSecond)) {
        success0 = false;
      }
    }
  }
  if (success0) {
    apaParallelReplanInSlotPathFinalize(path);
    success0 = apaParallelReplanInSlotPathLengthValid(path);
  }
  return success0;
}

static Boolean apaParallelReplanInSlotGeneratePathType2(const ApaParallelReplanInSlotInput_t *input,
                                                        Boolean slotOnRight,
                                                        ApaParallelReplanPath_t *path) {
  Real startPose[3];
  Real goalPose[3];
  Real startInSlotCs[3];
  Boolean wantMoveBackwardFirst;
  Boolean wantMoveForwardFirst;
  Boolean startTurnForward;
  Integer segNum;
  Integer loopCount;
  Boolean isHeadingMatch;
  Boolean success0;

  startPose[0] = input->StartX;
  startPose[1] = input->StartY;
  startPose[2] = input->StartTheta;
  goalPose[0] = input->GoalX;
  goalPose[1] = input->GoalY;
  goalPose[2] = input->GoalTheta;
  apaCoordGroundToVehPose(goalPose, startPose, startInSlotCs);
  apaParallelReplanInSlotPathInit(path, startPose[0], startPose[1], startPose[2]);

  wantMoveBackwardFirst = false;
  wantMoveForwardFirst = false;
  if (((startInSlotCs[1] > APA_PARALLEL_REPLAN_MOVE_SWITCH_LAT) &&
       (startInSlotCs[2] < -apaMathDegToRad(15.0f)) &&
       (startInSlotCs[2] > -apaMathDegToRad(90.0f)) && slotOnRight) ||
      ((startInSlotCs[1] < -APA_PARALLEL_REPLAN_MOVE_SWITCH_LAT) &&
       (startInSlotCs[2] > apaMathDegToRad(15.0f)) && (startInSlotCs[2] < apaMathDegToRad(90.0f)) &&
       (!slotOnRight))) {
    wantMoveForwardFirst = true;
  }
  if (((startInSlotCs[1] < -APA_PARALLEL_REPLAN_MOVE_SWITCH_LAT) &&
       (startInSlotCs[2] < -apaMathDegToRad(15.0f)) &&
       (startInSlotCs[2] > -apaMathDegToRad(90.0f)) && slotOnRight) ||
      ((startInSlotCs[1] > APA_PARALLEL_REPLAN_MOVE_SWITCH_LAT) &&
       (startInSlotCs[2] > apaMathDegToRad(15.0f)) && (startInSlotCs[2] < apaMathDegToRad(90.0f)) &&
       (!slotOnRight))) {
    wantMoveBackwardFirst = true;
  }

  if (slotOnRight) {
    startTurnForward =
        !((startInSlotCs[1] < 0.0f) && (input->ExpectDirFlag != APA_EXP_DIREC_FORCE_FORWARD));
  } else {
    startTurnForward =
        !((startInSlotCs[1] > 0.0f) && (input->ExpectDirFlag != APA_EXP_DIREC_FORCE_FORWARD));
  }

  segNum = 0;
  success0 = true;
  if ((input->ExpectDirFlag != APA_EXP_DIREC_FORCE_BACKWARD) && wantMoveForwardFirst) {
    ApaParallelReplanPath_t initSeg;

    if (apaParallelReplanInSlotGenerateStraightSegment(
            input, startPose, 1, slotOnRight, &initSeg)) {
      if (!apaParallelReplanInSlotPathAppendPath(path, &initSeg)) {
        success0 = false;
      }
      if (success0) {
        segNum = segNum + 1;
        startTurnForward = false;
      }
    } else {
      startTurnForward = true;
    }
  }
  if (success0 && (input->ExpectDirFlag != APA_EXP_DIREC_FORCE_FORWARD) && wantMoveBackwardFirst) {
    ApaParallelReplanPath_t initSeg;

    if (apaParallelReplanInSlotGenerateStraightSegment(
            input, startPose, -1, slotOnRight, &initSeg)) {
      apaParallelReplanInSlotPathInit(path, startPose[0], startPose[1], startPose[2]);
      if (!apaParallelReplanInSlotPathAppendPath(path, &initSeg)) {
        success0 = false;
      }
      if (success0) {
        segNum = 1;
        startTurnForward = true;
      }
    } else {
      startTurnForward = false;
    }
  }

  isHeadingMatch = false;
  loopCount = 0;
  while ((!isHeadingMatch) && success0) {
    Real tmpOrigin[3];
    ApaParallelReplanPath_t segPath;
    Boolean headingMatch;
    Boolean successFirst;
    Boolean successSecond;

    loopCount = loopCount + 1;
    if (loopCount > APA_PARALLEL_REPLAN_MANEUVER_LOOP_LIMIT) {
      success0 = false;
    }

    successFirst = false;
    if (success0) {
      apaParallelReplanInSlotPathGetEndPose(path, startPose, tmpOrigin);
      if (startTurnForward) {
        successFirst = apaParallelReplanInSlotGenerateManeuverSegment(
            input, tmpOrigin, 1, slotOnRight, segNum, &segPath, &headingMatch);
      } else {
        successFirst = apaParallelReplanInSlotGenerateManeuverSegment(
            input, tmpOrigin, -1, !slotOnRight, segNum, &segPath, &headingMatch);
      }
      if (successFirst) {
        success0 = apaParallelReplanInSlotPathAppendPath(path, &segPath);
        if (success0) {
          segNum = segNum + 1;
          isHeadingMatch = headingMatch;
        }
      }
    }

    if ((!isHeadingMatch) && success0) {
      apaParallelReplanInSlotPathGetEndPose(path, startPose, tmpOrigin);
      if (startTurnForward) {
        successSecond = apaParallelReplanInSlotGenerateManeuverSegment(
            input, tmpOrigin, -1, !slotOnRight, segNum, &segPath, &headingMatch);
      } else {
        successSecond = apaParallelReplanInSlotGenerateManeuverSegment(
            input, tmpOrigin, 1, slotOnRight, segNum, &segPath, &headingMatch);
      }
      if (successSecond) {
        success0 = apaParallelReplanInSlotPathAppendPath(path, &segPath);
        if (success0) {
          segNum = segNum + 1;
          isHeadingMatch = headingMatch;
        }
      }
      if ((!successFirst) && (!successSecond)) {
        success0 = false;
      }
    }
  }
  if (success0) {
    apaParallelReplanInSlotPathFinalize(path);
    success0 = apaParallelReplanInSlotPathLengthValid(path);
  }
  return success0;
}

static Boolean
apaParallelReplanInSlotAddLongitudinalAdjustment(const ApaParallelReplanInSlotInput_t *input,
                                                 const Real startPose[3],
                                                 ApaParallelReplanPath_t *pathSeg) {
  ApaPlannerConfig_t plannerCfg;
  Real tempEndVehX;
  Real tempEndVehY;
  Real tempEndVehTheta;
  Real lonDelta;
  Real adjEndGlobal[3];
  Boolean success0;

  apaParallelReplanInSlotLoadPlannerConfig(&plannerCfg);
  apaCoordGround2Veh(input->GoalX,
                     input->GoalY,
                     input->GoalTheta,
                     startPose[0],
                     startPose[1],
                     startPose[2],
                     &tempEndVehX,
                     &tempEndVehY,
                     &tempEndVehTheta);
  lonDelta = tempEndVehX;
  if (fabsf(lonDelta) < plannerCfg.Conf.PathLenMin) {
    *pathSeg = (ApaParallelReplanPath_t){0};
    success0 = true;
  } else {
    apaCoordVeh2Ground(input->GoalX,
                       input->GoalY,
                       input->GoalTheta,
                       0.0f,
                       tempEndVehY,
                       0.0f,
                       &adjEndGlobal[0],
                       &adjEndGlobal[1],
                       &adjEndGlobal[2]);
    apaParallelReplanInSlotPathInit(pathSeg, startPose[0], startPose[1], startPose[2]);
    success0 = apaParallelReplanInSlotPathAppend(
        pathSeg, adjEndGlobal[0], adjEndGlobal[1], adjEndGlobal[2], -lonDelta);
    if (success0) {
      apaParallelReplanInSlotPathFinalize(pathSeg);
      success0 = apaParallelReplanInSlotPathLengthValid(pathSeg);
    }
  }
  return success0;
}

static Boolean apaParallelReplanInSlotSinglePolyFit(const ApaParallelReplanInSlotInput_t *input,
                                                    const ApaParallelReplanInSlotParam_t *param,
                                                    Real latErr,
                                                    Real latThresh,
                                                    const Real startPose[3],
                                                    const Real endPose[3],
                                                    Boolean slotOnRight,
                                                    const Real backSlotPtNear[2],
                                                    const Real backSlotPtAway[2],
                                                    ApaParallelReplanPath_t *pathSeg) {
  Real endLocal[3];
  Real latBias;
  GeoPthDiscretVtOutput_t discretOutput;
  Real polyfitPathLenMin;
  Boolean success0;
  Boolean searchOk0;

  apaCoordGroundToVehPose(
      (const Real[3]){input->GoalX, input->GoalY, input->GoalTheta}, endPose, endLocal);
  polyfitPathLenMin = apaParallelReplanInSlotResolvePolyfitPathLenMin(input);
  latBias = 0.0f;
  success0 = false;
  searchOk0 = true;
  while ((latBias <= fabsf(latErr) + 1.0e-6f) && (success0 == false) && searchOk0) {
    Real fitEndLocal[3];
    Real fitEndPose[3];
    Real pathEnd[3];
    Real runLatErr;
    Boolean fitOk;

    fitEndLocal[0] = endLocal[0];
    if (latErr >= 0.0f) {
      fitEndLocal[1] = latBias;
    } else {
      fitEndLocal[1] = -latBias;
    }
    fitEndLocal[2] = endLocal[2];
    apaCoordVehToGroundPose(
        (const Real[3]){input->GoalX, input->GoalY, input->GoalTheta}, fitEndLocal, fitEndPose);
    fitOk = false;
    if (apaParallelReplanInSlotTryDirectCubicFit(
            input, startPose, fitEndPose, polyfitPathLenMin, pathSeg)) {
      fitOk = true;
    } else if (apaParallelReplanInSlotTrySplitCubicFit(
                   input, startPose, fitEndPose, polyfitPathLenMin, pathSeg)) {
      fitOk = true;
    } else if (apaParallelReplanInSlotRunSlotBottomBetweenPoses(
                   input, param, startPose, fitEndPose, &discretOutput) &&
               discretOutput.PtCount >= 2) {
      apaParallelReplanInSlotPathInit(
          pathSeg, discretOutput.PthPtX[0], discretOutput.PthPtY[0], discretOutput.PthPtTht[0]);
      searchOk0 = apaParallelReplanInSlotAppendDiscretized(pathSeg, &discretOutput, 1);
      if (searchOk0) {
        apaParallelReplanInSlotPathFinalize(pathSeg);
        fitOk = true;
      }
    }
    if (fitOk && searchOk0) {
      apaParallelReplanInSlotPathGetEndPose(pathSeg, startPose, pathEnd);
      if (apaParallelReplanInSlotSatisfyLatErr(
              input, pathEnd, latThresh, slotOnRight, backSlotPtNear, backSlotPtAway, &runLatErr) &&
          apaParallelReplanInSlotPathCollisionFreeCustom(input, pathSeg)) {
        success0 = true;
      }
    }
    latBias = latBias + APA_PARALLEL_REPLAN_POLYFIT_BIAS_STEP;
  }
  return success0;
}

static Boolean apaParallelReplanInSlotTryGuidedRefine(const ApaParallelReplanInSlotInput_t *input,
                                                      const ApaParallelReplanInSlotParam_t *param,
                                                      ApaParallelReplanInSlotOutput_t *output) {
  Boolean slotOnRight;
  Real backSlotPtNear[2];
  Real backSlotPtAway[2];
  Real startPose[3];
  Real pathEnd[3];
  Real headingErr;
  Real latErr;
  ApaParallelReplanPath_t totalPath;
  Boolean success0;

  slotOnRight = apaParallelReplanInSlotResolveSlotOnRight(input);
  apaParallelReplanInSlotGetBackSlotPoints(input, backSlotPtNear, backSlotPtAway);
  startPose[0] = input->StartX;
  startPose[1] = input->StartY;
  startPose[2] = input->StartTheta;
  apaParallelReplanInSlotPathInit(&totalPath, startPose[0], startPose[1], startPose[2]);
  pathEnd[0] = startPose[0];
  pathEnd[1] = startPose[1];
  pathEnd[2] = startPose[2];
  success0 = true;

  if (!apaParallelReplanInSlotSatisfyHeadingErr(
          input, pathEnd, APA_PARALLEL_REPLAN_HEADING_THRESH, &headingErr)) {
    ApaParallelReplanPath_t headingPath;
    Boolean headingPlanOk;

    if ((slotOnRight && headingErr > 0.0f) || ((!slotOnRight) && headingErr < 0.0f)) {
      headingPlanOk = apaParallelReplanInSlotGeneratePathType1(input, slotOnRight, &headingPath);
    } else {
      headingPlanOk = apaParallelReplanInSlotGeneratePathType2(input, slotOnRight, &headingPath);
    }
    if (!headingPlanOk) {
      success0 = false;
    } else {
      success0 = apaParallelReplanInSlotPathAppendPath(&totalPath, &headingPath);
    }
    if (success0) {
      apaParallelReplanInSlotPathGetEndPose(&totalPath, startPose, pathEnd);
    }
  }

  if (success0 && !apaParallelReplanInSlotSatisfyLatErr(input,
                                                        pathEnd,
                                                        APA_PARALLEL_REPLAN_LAT_THRESH,
                                                        slotOnRight,
                                                        backSlotPtNear,
                                                        backSlotPtAway,
                                                        &latErr)) {
    Real backCollisionDisForStart;
    Real backCollisionDisForGoal;
    Real fitStartPose[3];
    Real fitEndPose[3];
    ApaParallelReplanPath_t fitPath;

    backCollisionDisForStart = apaParallelReplanInSlotExploreCollisionDistance(input, pathEnd, -1);
    backCollisionDisForGoal = apaParallelReplanInSlotExploreCollisionDistance(
        input, (const Real[3]){input->GoalX, input->GoalY, input->GoalTheta}, -1);

    fitStartPose[0] = pathEnd[0];
    fitStartPose[1] = pathEnd[1];
    fitStartPose[2] = pathEnd[2];
    fitEndPose[0] = input->GoalX;
    fitEndPose[1] = input->GoalY;
    fitEndPose[2] = input->GoalTheta;
    if (!apaParallelReplanInSlotSinglePolyFit(input,
                                              param,
                                              latErr,
                                              APA_PARALLEL_REPLAN_LAT_THRESH,
                                              fitStartPose,
                                              fitEndPose,
                                              slotOnRight,
                                              backSlotPtNear,
                                              backSlotPtAway,
                                              &fitPath)) {
      Real pathEndLocal[3];
      Boolean appendedAdjust;

      apaCoordGroundToVehPose(
          (const Real[3]){input->GoalX, input->GoalY, input->GoalTheta}, pathEnd, pathEndLocal);
      pathEndLocal[0] = pathEndLocal[0] - backCollisionDisForStart;
      apaCoordVehToGroundPose((const Real[3]){input->GoalX, input->GoalY, input->GoalTheta},
                              pathEndLocal,
                              fitStartPose);
      appendedAdjust = apaParallelReplanInSlotAppendStraight(&totalPath,
                                                             pathEnd[0],
                                                             pathEnd[1],
                                                             pathEnd[2],
                                                             fitStartPose[0],
                                                             fitStartPose[1],
                                                             fitStartPose[2],
                                                             -1);
      if (!appendedAdjust) {
        success0 = false;
      }
      if (success0 && !apaParallelReplanInSlotSinglePolyFit(input,
                                                            param,
                                                            latErr,
                                                            APA_PARALLEL_REPLAN_LAT_THRESH,
                                                            fitStartPose,
                                                            fitEndPose,
                                                            slotOnRight,
                                                            backSlotPtNear,
                                                            backSlotPtAway,
                                                            &fitPath)) {
        Real goalShiftLocal[3];

        goalShiftLocal[0] = -backCollisionDisForGoal;
        goalShiftLocal[1] = 0.0f;
        goalShiftLocal[2] = 0.0f;
        apaCoordVehToGroundPose((const Real[3]){input->GoalX, input->GoalY, input->GoalTheta},
                                goalShiftLocal,
                                fitEndPose);
        if (!apaParallelReplanInSlotSinglePolyFit(input,
                                                  param,
                                                  latErr,
                                                  APA_PARALLEL_REPLAN_LAT_THRESH,
                                                  fitStartPose,
                                                  fitEndPose,
                                                  slotOnRight,
                                                  backSlotPtNear,
                                                  backSlotPtAway,
                                                  &fitPath)) {
          success0 = false;
        }
      }
    }
    if (success0) {
      success0 = apaParallelReplanInSlotPathAppendPath(&totalPath, &fitPath);
    }
    if (success0) {
      apaParallelReplanInSlotPathGetEndPose(&totalPath, startPose, pathEnd);
    }
  }

  if (success0 &&
      apaParallelReplanInSlotSatisfyHeadingErr(
          input, pathEnd, APA_PARALLEL_REPLAN_HEADING_THRESH, &headingErr) &&
      apaParallelReplanInSlotSatisfyLatErr(input,
                                           pathEnd,
                                           APA_PARALLEL_REPLAN_LAT_THRESH,
                                           slotOnRight,
                                           backSlotPtNear,
                                           backSlotPtAway,
                                           &latErr)) {
    ApaParallelReplanPath_t finalSeg;

    if (!apaParallelReplanInSlotAddLongitudinalAdjustment(input, pathEnd, &finalSeg)) {
      success0 = false;
    }
    if (success0 && (finalSeg.PtCount > 1)) {
      success0 = apaParallelReplanInSlotPathAppendPath(&totalPath, &finalSeg);
    }
  }

  if (success0) {
    apaParallelReplanInSlotPathFinalize(&totalPath);
    success0 = apaParallelReplanInSlotPathLengthValid(&totalPath);
  }
  if (success0) {
    apaParallelReplanInSlotCopyPathOut(&totalPath, output);
    output->AppliedReqType = input->ReplanReqType;
    success0 = (output->Success != 0);
  }
  return success0;
}

static Boolean
apaParallelReplanInSlotTrySlotPlanFallback(const ApaParallelReplanInSlotInput_t *input,
                                           const ApaParallelReplanInSlotParam_t *param,
                                           ApaParallelReplanInSlotOutput_t *output) {
  GeoPthDiscretVtOutput_t discretOutput;
  ApaParallelReplanPath_t path;
  Boolean success0;

  success0 = apaParallelReplanInSlotRunSlotBottomFromPose(
      input, param, input->StartX, input->StartY, input->StartTheta, &discretOutput);
  if (success0) {
    success0 = (discretOutput.PtCount >= 2);
  }

  if (success0) {
    apaParallelReplanInSlotPathInit(
        &path, discretOutput.PthPtX[0], discretOutput.PthPtY[0], discretOutput.PthPtTht[0]);
    success0 = apaParallelReplanInSlotAppendDiscretized(&path, &discretOutput, 1);
  }

  if (success0) {
    apaParallelReplanInSlotPathFinalize(&path);
    apaParallelReplanInSlotCopyPathOut(&path, output);
    output->AppliedReqType = input->ReplanReqType;
    success0 = (output->Success != 0);
  }
  return success0;
}

static Boolean apaParallelReplanInSlotFindNearestStopper(
    const ApaParallelReplanInSlotInput_t *input, Real *stopperX, Real *stopperY) {
  Integer stopperIndex;
  Boolean foundStopper;
  Real bestDistSq;

  foundStopper = false;
  bestDistSq = 0.0f;
  stopperIndex = 0;
  while (stopperIndex < input->StopperPtNum && stopperIndex < VCC_MAX_SEPARATE_PTS) {
    Real dx;
    Real dy;
    Real distSq;

    dx = input->StopperPtX[stopperIndex] - input->StartX;
    dy = input->StopperPtY[stopperIndex] - input->StartY;
    distSq = dx * dx + dy * dy;
    if ((!foundStopper) || (distSq < bestDistSq)) {
      *stopperX = input->StopperPtX[stopperIndex];
      *stopperY = input->StopperPtY[stopperIndex];
      bestDistSq = distSq;
      foundStopper = true;
    }
    stopperIndex = stopperIndex + 1;
  }

  return foundStopper;
}

static Integer
apaParallelReplanInSlotChooseOffsetDirection(const ApaParallelReplanInSlotInput_t *input,
                                             Integer surroundingType) {
  Real stopperX;
  Real stopperY;
  Real localX;
  Real localY;
  Real localTheta;
  Integer offsetDirection0;

  offsetDirection0 = 0;
  if (apaParallelReplanInSlotFindNearestStopper(input, &stopperX, &stopperY)) {
    apaCoordGround2Veh(input->GoalX,
                       input->GoalY,
                       input->GoalTheta,
                       stopperX,
                       stopperY,
                       0.0f,
                       &localX,
                       &localY,
                       &localTheta);
    if (localY > 0.05f) {
      offsetDirection0 = -1;
    } else if (localY < -0.05f) {
      offsetDirection0 = 1;
    }
  }

  if (offsetDirection0 == 0) {
    if (surroundingType == APA_SURROUND_OBS_EXIST_LEFT_FRONT_ONLY) {
      offsetDirection0 = -1;
    } else {
      offsetDirection0 = 1;
    }
  }
  return offsetDirection0;
}

static void
apaParallelReplanInSlotBuildSurroundInput(const ApaParallelReplanInSlotInput_t *input,
                                          SlotSurroundingAreaCheckInput_t *surroundInput) {
  Integer boundaryIndex;
  Integer pointIndex;
  Integer stopperIndex;
  Integer virtualIndex;

  *surroundInput = (SlotSurroundingAreaCheckInput_t){0};
  surroundInput->TarPtX = input->GoalX;
  surroundInput->TarPtY = input->GoalY;
  surroundInput->TarPtTheta = input->GoalTheta;
  surroundInput->SltPt1X = input->SltPt1X;
  surroundInput->SltPt1Y = input->SltPt1Y;
  surroundInput->SltPt2X = input->SltPt2X;
  surroundInput->SltPt2Y = input->SltPt2Y;
  surroundInput->SltPt3X = input->SltPt3X;
  surroundInput->SltPt3Y = input->SltPt3Y;
  surroundInput->SltPt4X = input->SltPt4X;
  surroundInput->SltPt4Y = input->SltPt4Y;
  surroundInput->SltEntr = 1;

  boundaryIndex = 0;
  while ((boundaryIndex < input->BoundaryLineNum) && (boundaryIndex < SURD_LINE_MAX)) {
    surroundInput->BoundaryP1X[boundaryIndex] = input->BoundaryLineX1[boundaryIndex];
    surroundInput->BoundaryP1Y[boundaryIndex] = input->BoundaryLineY1[boundaryIndex];
    surroundInput->BoundaryP2X[boundaryIndex] = input->BoundaryLineX2[boundaryIndex];
    surroundInput->BoundaryP2Y[boundaryIndex] = input->BoundaryLineY2[boundaryIndex];
    boundaryIndex = boundaryIndex + 1;
  }
  surroundInput->BoundaryLineNum = boundaryIndex;

  pointIndex = 0;
  while ((pointIndex < input->BoundarySepPtNum) && (pointIndex < SURD_PT_MAX)) {
    surroundInput->BoundarySepPtX[pointIndex] = input->BoundarySepPtX[pointIndex];
    surroundInput->BoundarySepPtY[pointIndex] = input->BoundarySepPtY[pointIndex];
    pointIndex = pointIndex + 1;
  }
  surroundInput->BoundarySepPtNum = pointIndex;

  stopperIndex = 0;
  while (stopperIndex < input->StopperPtNum && stopperIndex < VCC_MAX_SEPARATE_PTS) {
    surroundInput->SeparateStopperX[stopperIndex] = input->StopperPtX[stopperIndex];
    surroundInput->SeparateStopperY[stopperIndex] = input->StopperPtY[stopperIndex];
    stopperIndex = stopperIndex + 1;
  }
  surroundInput->SeparateStopperNum = stopperIndex;

  virtualIndex = 0;
  while ((virtualIndex < input->VirtualBoundaryLineNum) && (virtualIndex < VCC_MAX_VIRTUAL_LINES)) {
    surroundInput->VirtualBoundaryLineX1[virtualIndex] = input->VirtualBoundaryLineX1[virtualIndex];
    surroundInput->VirtualBoundaryLineY1[virtualIndex] = input->VirtualBoundaryLineY1[virtualIndex];
    surroundInput->VirtualBoundaryLineX2[virtualIndex] = input->VirtualBoundaryLineX2[virtualIndex];
    surroundInput->VirtualBoundaryLineY2[virtualIndex] = input->VirtualBoundaryLineY2[virtualIndex];
    virtualIndex = virtualIndex + 1;
  }
  surroundInput->VirtualBoundaryLineNum = virtualIndex;
}

static void apaParallelReplanInSlotBuildOffsetInput(const ApaParallelReplanInSlotInput_t *input,
                                                    Integer offsetDirection,
                                                    GeoOffsetPlanningInput_t *offsetInput) {
  Integer stopperIndex;

  *offsetInput = (GeoOffsetPlanningInput_t){0};
  offsetInput->VehPosX = input->StartX;
  offsetInput->VehPosY = input->StartY;
  offsetInput->VehPosTheta = input->StartTheta;
  offsetInput->OffsetDirection = offsetDirection;
  offsetInput->AtLeastOffsetDistance = APA_PARALLEL_REPLAN_OFFSET_MIN_DIST;
  offsetInput->SlotPoseX = input->GoalX;
  offsetInput->SlotPoseY = input->GoalY;
  offsetInput->SlotPoseTheta = input->GoalTheta;
  offsetInput->SltPt1X = input->SltPt1X;
  offsetInput->SltPt1Y = input->SltPt1Y;
  offsetInput->SltPt2X = input->SltPt2X;
  offsetInput->SltPt2Y = input->SltPt2Y;
  offsetInput->SltPt3X = input->SltPt3X;
  offsetInput->SltPt3Y = input->SltPt3Y;
  offsetInput->SltPt4X = input->SltPt4X;
  offsetInput->SltPt4Y = input->SltPt4Y;
  offsetInput->Radius = apaMathAbs(input->Rmin);
  offsetInput->ExistWheelStop = 1;
  offsetInput->StopperPtNum = input->StopperPtNum;

  stopperIndex = 0;
  while (stopperIndex < input->StopperPtNum && stopperIndex < VCC_MAX_SEPARATE_PTS) {
    offsetInput->StopperPtX[stopperIndex] = input->StopperPtX[stopperIndex];
    offsetInput->StopperPtY[stopperIndex] = input->StopperPtY[stopperIndex];
    stopperIndex = stopperIndex + 1;
  }
}

static Boolean apaParallelReplanInSlotBuildOffsetPath(const GeoOffsetPlanningOutput_t *offsetOutput,
                                                      ApaParallelReplanPath_t *path) {
  Integer anchorIndex;
  Boolean success0;

  success0 = true;
  anchorIndex = 0;
  while ((anchorIndex + 1 < offsetOutput->AnchorsNum) && success0) {
    Real startX;
    Real startY;
    Real startTheta;
    Real endX;
    Real endY;
    Real endTheta;
    Real curve;
    Real radius;

    startX = offsetOutput->AnchorX[anchorIndex];
    startY = offsetOutput->AnchorY[anchorIndex];
    startTheta = offsetOutput->AnchorTheta[anchorIndex];
    endX = offsetOutput->AnchorX[anchorIndex + 1];
    endY = offsetOutput->AnchorY[anchorIndex + 1];
    endTheta = offsetOutput->AnchorTheta[anchorIndex + 1];
    curve = offsetOutput->AnchorCurv[anchorIndex];
    radius = 0.0f;
    if (fabsf(curve) > 1.0e-6f) {
      radius = 1.0f / fabsf(curve);
    }

    if (fabsf(curve) > 1.0e-6f && fabsf(offsetOutput->AnchorDTheta[anchorIndex]) > 1.0e-6f) {
      if (!apaParallelReplanInSlotAppendArc(path,
                                            startX,
                                            startY,
                                            startTheta,
                                            offsetOutput->AnchorCcX[anchorIndex],
                                            offsetOutput->AnchorCcY[anchorIndex],
                                            offsetOutput->AnchorDTheta[anchorIndex],
                                            offsetOutput->AnchorGoDir[anchorIndex],
                                            radius)) {
        success0 = false;
      }
    } else if (!apaParallelReplanInSlotAppendStraight(
                   path, startX, startY, startTheta, endX, endY, endTheta, 1)) {
      success0 = false;
    }

    anchorIndex = anchorIndex + 1;
  }

  return success0;
}

static Boolean apaParallelReplanInSlotTrySlotBottom(const ApaParallelReplanInSlotInput_t *input,
                                                    const ApaParallelReplanInSlotParam_t *param,
                                                    ApaParallelReplanInSlotOutput_t *output) {
  ApaParallelReplanPath_t path;
  GeoPthDiscretVtOutput_t discretOutput;
  Boolean success0;

  success0 = apaParallelReplanInSlotVehicleInSlot(input);
  if (success0) {
    success0 = apaParallelReplanInSlotRunSlotBottomFromPose(
        input, param, input->StartX, input->StartY, input->StartTheta, &discretOutput);
  }

  if (success0) {
    apaParallelReplanInSlotPathInit(
        &path, discretOutput.PthPtX[0], discretOutput.PthPtY[0], discretOutput.PthPtTht[0]);
    success0 = apaParallelReplanInSlotAppendDiscretized(&path, &discretOutput, 1);
  }
  if (success0) {
    apaParallelReplanInSlotPathFinalize(&path);
    apaParallelReplanInSlotCopyPathOut(&path, output);
    output->AppliedReqType = APA_REPLAN_SLOT_BOTTOM_REPLAN;
    success0 = (output->Success != 0);
  }
  return success0;
}

static Boolean apaParallelReplanInSlotTryPunchStopper(const ApaParallelReplanInSlotInput_t *input,
                                                      const ApaParallelReplanInSlotParam_t *param,
                                                      ApaParallelReplanInSlotOutput_t *output) {
  SlotSurroundingAreaCheckInput_t surroundInput;
  SlotSurroundingAreaCheckOutput_t surroundOutput;
  GeoOffsetPlanningInput_t offsetInput;
  GeoOffsetPlanningOutput_t offsetOutput;
  GeoPthDiscretVtOutput_t tailOutput;
  ApaParallelReplanPath_t path;
  Integer offsetDirection;
  Real tailStartX;
  Real tailStartY;
  Real tailStartTheta;
  Boolean success0;

  if (apaParallelReplanInSlotVehicleInSlot(input)) {
    output->VehicleInSlot = 1;
  } else {
    output->VehicleInSlot = 0;
  }
  success0 = (output->VehicleInSlot != 0);
  if (success0) {
    apaParallelReplanInSlotBuildSurroundInput(input, &surroundInput);
    surroundOutput = (SlotSurroundingAreaCheckOutput_t){0};
    slotSurroundingAreaCheck(&surroundInput, &surroundOutput);
    output->ExistStopperInSlot = surroundOutput.ExistStopperInSlot;
    output->SurroundingType = surroundOutput.SurroundingType;
    success0 = (surroundOutput.ExistStopperInSlot != 0);
  }

  if (success0) {
    offsetDirection =
        apaParallelReplanInSlotChooseOffsetDirection(input, surroundOutput.SurroundingType);
    apaParallelReplanInSlotBuildOffsetInput(input, offsetDirection, &offsetInput);
    offsetOutput = (GeoOffsetPlanningOutput_t){0};
    geoOffsetPlanning(&offsetInput, &offsetOutput);
    success0 = (offsetOutput.AnchorsNum >= 2);
  }

  if (success0) {
    apaParallelReplanInSlotPathInit(
        &path, offsetOutput.AnchorX[0], offsetOutput.AnchorY[0], offsetOutput.AnchorTheta[0]);
    success0 = apaParallelReplanInSlotBuildOffsetPath(&offsetOutput, &path);
  }

  if (success0) {
    tailStartX = path.X[path.PtCount - 1];
    tailStartY = path.Y[path.PtCount - 1];
    tailStartTheta = path.Theta[path.PtCount - 1];
    if (apaParallelReplanInSlotRunSlotBottomFromPose(
            input, param, tailStartX, tailStartY, tailStartTheta, &tailOutput)) {
      success0 = apaParallelReplanInSlotAppendDiscretized(&path, &tailOutput, 1);
    }
  }

  if (success0) {
    apaParallelReplanInSlotPathFinalize(&path);
    apaParallelReplanInSlotCopyPathOut(&path, output);
    output->AppliedReqType = APA_REPLAN_AVOID_PUNCH_STOPPER;
    success0 = (output->Success != 0);
  }
  return success0;
}

static Boolean apaParallelReplanInSlotTryCubicRefine(const ApaParallelReplanInSlotInput_t *input,
                                                     const ApaParallelReplanInSlotParam_t *param,
                                                     ApaParallelReplanInSlotOutput_t *output) {
  VehicleCollisionCheckInput_t vccInput;
  ApaPath_t path;
  Boolean allowDirectCubic;
  Boolean guidedEligible;
  Real startLocalX;
  Real startLocalY;
  Real startLocalTheta;
  Boolean success0;

  if (apaParallelReplanInSlotVehicleInSlot(input)) {
    output->VehicleInSlot = 1;
  } else {
    output->VehicleInSlot = 0;
  }
  allowDirectCubic = true;
  apaCoordGround2Veh(input->GoalX,
                     input->GoalY,
                     input->GoalTheta,
                     input->StartX,
                     input->StartY,
                     input->StartTheta,
                     &startLocalX,
                     &startLocalY,
                     &startLocalTheta);
  guidedEligible =
      (output->VehicleInSlot != 0) && ((fabsf(startLocalTheta) > apaMathDegToRad(15.0f)) ||
                                       (fabsf(startLocalY) > APA_PARALLEL_REPLAN_LAT_THRESH));

  if (output->VehicleInSlot == 0) {
    if (startLocalX > APA_PARALLEL_REPLAN_NON_SLOT_LOCAL_X_MAX) {
      allowDirectCubic = false;
    }
  }

  success0 = false;
  if (allowDirectCubic && apaParallelReplanInSlotBuildCubicPath(input, &path)) {
    apaParallelReplanInSlotBuildCollisionInput(input, &vccInput);
    if (apaParallelReplanInSlotPathCollisionFree(&vccInput, &path)) {
      apaParallelReplanInSlotCopyApaPathOut(&path, output);
      output->AppliedReqType = input->ReplanReqType;
      success0 = (output->Success != 0);
    }
  }

  if ((success0 == false) && allowDirectCubic && guidedEligible &&
      apaParallelReplanInSlotTryGuidedRefine(input, param, output)) {
    success0 = (output->Success != 0);
  }

  if ((success0 == false) && allowDirectCubic) {
    success0 = apaParallelReplanInSlotTrySlotPlanFallback(input, param, output);
  }
  return success0;
}

/**
 * @brief 槽内重规划运行时模块
 * @en_name apaParallelReplanInSlot
 * @cn_name 槽内重规划运行时模块
 * @type module
 * @param[IN] ApaParallelReplanInSlotInput_t Input 槽内重规划输入
 * @param[IN] ApaParallelReplanInSlotParam_t Param 槽内重规划参数
 * @param[OUT] ApaParallelReplanInSlotOutput_t Output 槽内重规划输出
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 槽内重规划
 * @source matlab_rewrite ClassParallelReplanInSlot.m (request-dispatch runtime projection)
 * @version 1.0
 * @date 2026-04-19
 * @author aide
 */
void apaParallelReplanInSlot(const ApaParallelReplanInSlotInput_t *input,
                             const ApaParallelReplanInSlotParam_t *param,
                             ApaParallelReplanInSlotOutput_t *output) {
  *output = (ApaParallelReplanInSlotOutput_t){0};
  output->AppliedReqType = APA_REPLAN_NONE;
  if (apaParallelReplanInSlotVehicleInSlot(input)) {
    output->VehicleInSlot = 1;
  } else {
    output->VehicleInSlot = 0;
  }

  if (input->ReplanReqType == APA_REPLAN_SLOT_BOTTOM_REPLAN) {
    (void)apaParallelReplanInSlotTrySlotBottom(input, param, output);
  } else if (input->ReplanReqType == APA_REPLAN_AVOID_PUNCH_STOPPER) {
    (void)apaParallelReplanInSlotTryPunchStopper(input, param, output);
  } else if (input->ReplanReqType == APA_REPLAN_GET_BETTER_GOAL_POS ||
             input->ReplanReqType == APA_REPLAN_REDUCE_CONTRL_ERR ||
             input->ReplanReqType == APA_REPLAN_REDUCE_PATH_SEG) {
    (void)apaParallelReplanInSlotTryCubicRefine(input, param, output);
  }
}
