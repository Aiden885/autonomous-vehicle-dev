/**
 * @file apaPlannerHybridParity.h
 * @brief Shared motion conversion, candidate policy and Hybrid quality selection.
 *
 * This header keeps the runtime-side Hybrid A* integration reusable across
 * VT/HT/DTH/VH entry points while retaining their geometric input adapters.
 *
 * @date 2026-04-19
 */
#ifndef APA_PLANNER_HYBRID_PARITY_H
#define APA_PLANNER_HYBRID_PARITY_H

#include "apaHybridAStar.h"
#include "apaHybridPlanningTypes.h"
#include "apaPathTypes.h"
#include "apaPlannerGlobal.h"
#include "apaVehicleCollisionCheck.h"
#include "baseTypes.h"
#include <math.h>

#ifndef APA_PATH_SEG_MAX
#define APA_PATH_SEG_MAX 30
#endif

#ifndef APA_PATH_CC_MAX
#define APA_PATH_CC_MAX 60
#endif

#ifndef APA_PLANNER_HYBRID_EPS
#define APA_PLANNER_HYBRID_EPS 1e-6f
#endif

#ifndef APA_PLANNER_HYBRID_COLLISION_STEP
#define APA_PLANNER_HYBRID_COLLISION_STEP 0.2f
#endif

#ifndef APA_PLANNER_HYBRID_MAX_ITER
#define APA_PLANNER_HYBRID_MAX_ITER 600
#endif

#ifndef APA_PLANNER_HYBRID_CURV_EPS
#define APA_PLANNER_HYBRID_CURV_EPS 1.0e-6f
#endif

#ifndef APA_PLANNER_HYBRID_FIT_POS_EPS
#define APA_PLANNER_HYBRID_FIT_POS_EPS 0.02f
#endif

#ifndef APA_PLANNER_HYBRID_FIT_YAW_EPS
#define APA_PLANNER_HYBRID_FIT_YAW_EPS 0.003f
#endif

#ifndef APA_PLANNER_HYBRID_GOAL_LONG_ERR
#define APA_PLANNER_HYBRID_GOAL_LONG_ERR 0.25f
#endif

#ifndef APA_PLANNER_HYBRID_GOAL_LAT_ERR
#define APA_PLANNER_HYBRID_GOAL_LAT_ERR 0.15f
#endif

#ifndef APA_PLANNER_HYBRID_GOAL_YAW_ERR
#define APA_PLANNER_HYBRID_GOAL_YAW_ERR 0.12f
#endif

typedef struct {
  ApaVehicleCollisionCheckContext_t CollisionContext;
  Integer CheckCount;
  ApaPlanningSceneQuery_t SceneQuery;
  VehicleCollisionCheckInput_t SceneTemplate;
  Boolean ReverseOutput; /* Adapter searched from the target to the vehicle. */
} ApaPlannerHybridCollisionContext_t;

static inline Integer apaPlannerHybridResolveFusion(Integer fusionFlag) {
  if (fusionFlag != 0) {
    return 1;
  }
  return 0;
}

static inline Integer apaPlannerHybridSign(Real value) {
  if (value > APA_PLANNER_HYBRID_EPS) {
    return 1;
  }
  if (value < -APA_PLANNER_HYBRID_EPS) {
    return -1;
  }
  return 0;
}

static inline void apaPlannerHybridInitPlannerGlobal(ApaPlannerGlobalState_t *plannerGlobal,
                                                     Real vehicleWidth,
                                                     Real minRadius,
                                                     Integer needHybridAlgFusion) {
  apaPlannerGlobalInit(plannerGlobal);

  plannerGlobal->Planner.Map.MotionResolution = APA_PLANNER_HYBRID_COLLISION_STEP;
  plannerGlobal->Planner.Map.CollisionDetectStepLength = APA_PLANNER_HYBRID_COLLISION_STEP;
  plannerGlobal->Planner.Veh.W = vehicleWidth;
  plannerGlobal->Planner.Veh.MinCircle = minRadius;
  if (minRadius > APA_PLANNER_HYBRID_EPS) {
    plannerGlobal->Planner.Veh.MaxSteer = atanf(plannerGlobal->Planner.Veh.WB / minRadius);
  }
  plannerGlobal->Planner.Conf.PlanningCalPowerMaxPerSlot = APA_PLANNER_HYBRID_MAX_ITER;
  plannerGlobal->Planner.Conf.LongAllowableErrMin = -APA_PLANNER_HYBRID_GOAL_LONG_ERR;
  plannerGlobal->Planner.Conf.LongAllowableErrMax = APA_PLANNER_HYBRID_GOAL_LONG_ERR;
  plannerGlobal->Planner.Conf.LatAllowableErrMin = -APA_PLANNER_HYBRID_GOAL_LAT_ERR;
  plannerGlobal->Planner.Conf.LatAllowableErrMax = APA_PLANNER_HYBRID_GOAL_LAT_ERR;
  plannerGlobal->Planner.Conf.YawAllowableErrMin = -APA_PLANNER_HYBRID_GOAL_YAW_ERR;
  plannerGlobal->Planner.Conf.YawAllowableErrMax = APA_PLANNER_HYBRID_GOAL_YAW_ERR;
  apaPlannerGlobalSetNeedHybridAlgFusion(plannerGlobal, needHybridAlgFusion);
}

static inline void apaPlannerHybridInitSlot(ApaSlot_t *slot,
                                            Real tarX,
                                            Real tarY,
                                            Real tarTheta,
                                            Integer enterEdge,
                                            const Real slotPts[8],
                                            Integer parkingType,
                                            Integer surroundingType,
                                            Integer parkingDirec) {
  Integer idx;

  apaSlotInit(slot);
  slot->EnterEdge = enterEdge;
  slot->TarPos[0] = tarX;
  slot->TarPos[1] = tarY;
  slot->TarPos[2] = tarTheta;
  slot->SlotStatus = APA_SLOT_STATUS_TARGET_SLOT;
  slot->SlotSubStatus = APA_SLOT_SUB_END_NONE;
  idx = 0;
  while (idx < 8) {
    slot->SlotPts[idx] = slotPts[idx];
    idx = idx + 1;
  }
  slot->SlotClass = apaSlotCalcSlotClass(slot->SlotPts, slot->EnterEdge);
  slot->ParkingType = parkingType;
  slot->SurroundingType = surroundingType;
  slot->ParkingDirec = parkingDirec;
}

static inline void apaPlannerHybridCollisionContextInit(ApaPlannerHybridCollisionContext_t *context,
                                                        ApaPlannerGlobalState_t *plannerGlobal,
                                                        const ApaSlot_t *slot) {
  apaVehicleCollisionCheckContextInit(&context->CollisionContext);
  apaVehicleCollisionCheckContextSetPlannerGlobal(&context->CollisionContext, plannerGlobal);
  apaVehicleCollisionCheckContextSetCollisionDemand(&context->CollisionContext,
                                                    APA_COLLISION_DEMAND_ALL);
  apaVehicleCollisionCheckContextSetSlot(&context->CollisionContext, slot);
  apaVehicleCollisionCheckContextSetTargetPose(&context->CollisionContext, slot->TarPos);
  context->CheckCount = 0;
  context->SceneQuery = (ApaPlanningSceneQuery_t){0};
  context->ReverseOutput = false;
  context->SceneTemplate = (VehicleCollisionCheckInput_t){0};
  VehicleCollisionCheckInput_t *native = &context->SceneTemplate;
  const ApaPlannerConfig_t *config = apaPlannerGlobalGetPlanner(plannerGlobal);
  native->CheckType = VCC_CHECK_TYPE_PLANNING;
  native->CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  native->CollisionDemand = VCC_COLLISION_DEMAND_SIMPLIFY;
  native->VehW = config->Veh.W;
  native->VehLF = config->Veh.LF;
  native->VehLB = config->Veh.LB;
  native->VehLL = config->Veh.LL;
  native->VehWB = config->Veh.WB;
  native->SlotClass = slot->SlotClass;
  native->SlotPtsValid = true;
  native->TarPosValid = true;
  native->TarPosX = slot->TarPos[0];
  native->TarPosY = slot->TarPos[1];
  native->TarPosTheta = slot->TarPos[2];
  for (Integer i = 0; i < 4; ++i)
    apaSlotGetEnterEdgePoint(
        slot->SlotPts, slot->EnterEdge, i, &native->SlotPts[2 * i], &native->SlotPts[2 * i + 1]);
  vehicleCollisionCheckLoadDefaultMargins(native);
}

static inline Boolean apaPlannerHybridCollisionAddBoundaryLine(
    ApaPlannerHybridCollisionContext_t *context, Real x1, Real y1, Real x2, Real y2) {
  return apaVehicleCollisionObsAddBoundaryLine(&context->CollisionContext.Obs, x1, y1, x2, y2);
}

static inline Boolean apaPlannerHybridCollisionAddVirtualBoundaryLine(
    ApaPlannerHybridCollisionContext_t *context, Real x1, Real y1, Real x2, Real y2) {
  return apaVehicleCollisionObsAddVirtualBoundaryLine(
      &context->CollisionContext.Obs, x1, y1, x2, y2);
}

static inline Boolean apaPlannerHybridCollisionAddSlotTrackStopper(
    ApaPlannerHybridCollisionContext_t *context, Real x1, Real y1, Real x2, Real y2) {
  return apaVehicleCollisionObsAddSlotTrackStopper(&context->CollisionContext.Obs, x1, y1, x2, y2);
}

static inline Boolean apaPlannerHybridCollisionAddBoundaryPoint(
    ApaPlannerHybridCollisionContext_t *context, Real x, Real y) {
  return apaVehicleCollisionObsAddBoundarySeparatePoint(&context->CollisionContext.Obs, x, y);
}

static inline Boolean
apaPlannerHybridCollisionAddStopper(ApaPlannerHybridCollisionContext_t *context, Real x, Real y) {
  return apaVehicleCollisionObsAddSeparateStopper(&context->CollisionContext.Obs, x, y);
}

static inline Boolean apaPlannerHybridCollisionCheck(const ApaHybridNode_t *fromNode,
                                                     const ApaHybridNode_t *toNode,
                                                     void *userData) {
  ApaPlannerHybridCollisionContext_t *context;
  const ApaPlannerConfig_t *plannerConfig;
  ApaVehicleCollisionCheckResult_t collisionResult;
  Real detectStep;
  Real travelDistance;
  Real collisionTravel;
  Integer sampleCount;
  Boolean collisionDetected;

  context = (ApaPlannerHybridCollisionContext_t *)userData;
  if (context == (ApaPlannerHybridCollisionContext_t *)0) {
    return false;
  }
  if (context->CollisionContext.PlannerGlobal == (ApaPlannerGlobalState_t *)0) {
    return false;
  }

  plannerConfig = apaPlannerGlobalGetPlanner(context->CollisionContext.PlannerGlobal);
  detectStep = plannerConfig->Map.CollisionDetectStepLength;
  if (detectStep <= APA_PLANNER_HYBRID_EPS) {
    detectStep = plannerConfig->Map.XyGridResolution;
  }
  travelDistance = fabsf(toNode->D);
  sampleCount = 1;
  if (detectStep > APA_PLANNER_HYBRID_EPS) {
    sampleCount = (Integer)ceilf(travelDistance / detectStep);
    if (sampleCount < 1) {
      sampleCount = 1;
    }
  }
  context->CheckCount = context->CheckCount + sampleCount;

  apaVehicleCollisionCheckResultInit(&collisionResult);
  collisionTravel = 0.0f;
  collisionDetected =
      apaVehicleCollisionCheckMotionDetailed(fromNode,
                                             toNode,
                                             &context->CollisionContext,
                                             context->CollisionContext.CollisionDemand,
                                             &collisionResult,
                                             &collisionTravel);
  if (collisionDetected)
    return true;
  if (context->SceneQuery.CheckPose || context->SceneQuery.TrackingReserve != 0.0f) {
    VehicleCollisionCheckInput_t native = context->SceneTemplate;
    VehicleCollisionCheckOutput_t checked;
    const Real dyaw = apaCoordWrapToPi(toNode->Theta - fromNode->Theta);
    const Real radius = hypotf(plannerConfig->Veh.LF, plannerConfig->Veh.W / 2.0f);
    const Integer samples =
        (Integer)ceilf(
            (hypotf(toNode->X - fromNode->X, toNode->Y - fromNode->Y) + radius * fabsf(dyaw)) /
            0.05f) +
        1;
    for (Integer i = 0; i <= samples; ++i) {
      const Real t = (Real)i / (Real)samples;
      native.VehPosX = fromNode->X + t * (toNode->X - fromNode->X);
      native.VehPosY = fromNode->Y + t * (toNode->Y - fromNode->Y);
      native.VehPosTheta = fromNode->Theta + t * dyaw;
      apaPlanningSceneCollisionCheck(&native, &context->SceneQuery, &checked);
      if (checked.IsCollision)
        return true;
    }
  }
  return false;
}

static inline void apaPlannerHybridSegmentsInit(ApaPlannerHybridSegments_t *segments) {
  Integer idx;

  segments->SegNum = 0;
  segments->StpN = 0;
  segments->PlnChkCnt = 0;
  segments->PlannerCase = 0;
  segments->PathLen = 0.0f;

  idx = 0;
  while (idx < APA_PATH_SEG_MAX) {
    segments->PathGoDir[idx] = 0;
    segments->PathTrnDir[idx] = 0;
    segments->PathCrv[idx] = 0.0f;
    segments->PathTht[idx] = 0.0f;
    segments->PathLenArr[idx] = 0.0f;
    segments->PathStx[idx] = 0.0f;
    segments->PathSty[idx] = 0.0f;
    segments->PathStTh[idx] = 0.0f;
    segments->PathCC[idx * 2] = 0.0f;
    segments->PathCC[idx * 2 + 1] = 0.0f;
    idx = idx + 1;
  }
}

static inline Boolean apaPlannerHybridAppendLineSegment(ApaPlannerHybridSegments_t *segments,
                                                        Integer goDir,
                                                        Real segLen,
                                                        Real stX,
                                                        Real stY,
                                                        Real stTheta) {
  Integer idx;

  idx = segments->SegNum;
  if ((segLen <= APA_PLANNER_HYBRID_EPS) || (idx >= APA_PATH_SEG_MAX)) {
    return false;
  }

  segments->PathGoDir[idx] = goDir;
  segments->PathTrnDir[idx] = 0;
  segments->PathCrv[idx] = 0.0f;
  segments->PathTht[idx] = 0.0f;
  segments->PathLenArr[idx] = segLen;
  segments->PathStx[idx] = stX;
  segments->PathSty[idx] = stY;
  segments->PathStTh[idx] = stTheta;
  segments->PathCC[idx * 2] = 0.0f;
  segments->PathCC[idx * 2 + 1] = 0.0f;
  segments->SegNum = idx + 1;
  return true;
}

static inline Boolean apaPlannerHybridAppendArcSegment(ApaPlannerHybridSegments_t *segments,
                                                       Integer goDir,
                                                       Integer trnDir,
                                                       Real radius,
                                                       Real segLen,
                                                       Real stX,
                                                       Real stY,
                                                       Real stTheta,
                                                       Real edTheta,
                                                       Real ccX,
                                                       Real ccY) {
  Integer idx;

  idx = segments->SegNum;
  if ((segLen <= APA_PLANNER_HYBRID_EPS) || (idx >= APA_PATH_SEG_MAX) ||
      (fabsf(radius) <= APA_PLANNER_HYBRID_EPS)) {
    return false;
  }

  segments->PathGoDir[idx] = goDir;
  segments->PathTrnDir[idx] = trnDir;
  segments->PathCrv[idx] = ((Real)trnDir) / radius;
  segments->PathTht[idx] = apaCoordWrapToPi(edTheta - stTheta) / (Real)goDir;
  segments->PathLenArr[idx] = segLen;
  segments->PathStx[idx] = stX;
  segments->PathSty[idx] = stY;
  segments->PathStTh[idx] = stTheta;
  segments->PathCC[idx * 2] = ccX;
  segments->PathCC[idx * 2 + 1] = ccY;
  segments->SegNum = idx + 1;
  return true;
}

static inline void apaPlannerHybridEvalPrimitivePose(Integer goDir,
                                                     Real curv,
                                                     Real travel,
                                                     Real stX,
                                                     Real stY,
                                                     Real stTheta,
                                                     Real *poseX,
                                                     Real *poseY,
                                                     Real *poseTheta) {
  if (fabsf(curv) <= APA_PLANNER_HYBRID_CURV_EPS) {
    *poseX = stX + (Real)goDir * travel * cosf(stTheta);
    *poseY = stY + (Real)goDir * travel * sinf(stTheta);
    *poseTheta = stTheta;
  } else {
    Real signedTravel;
    Real invCurv;
    Real centerX;
    Real centerY;
    Real theta;

    signedTravel = (Real)goDir * travel;
    invCurv = 1.0f / curv;
    centerX = stX - invCurv * sinf(stTheta);
    centerY = stY + invCurv * cosf(stTheta);
    theta = apaCoordWrapToPi(stTheta + curv * signedTravel);
    *poseX = centerX + invCurv * sinf(theta);
    *poseY = centerY - invCurv * cosf(theta);
    *poseTheta = theta;
  }
}

static inline Boolean apaPlannerHybridPoseMatches(
    Real refX, Real refY, Real refTheta, Real poseX, Real poseY, Real poseTheta) {
  Real dx;
  Real dy;
  Real posErr;
  Real thetaErr;

  dx = poseX - refX;
  dy = poseY - refY;
  posErr = sqrtf(dx * dx + dy * dy);
  thetaErr = fabsf(apaCoordWrapToPi(poseTheta - refTheta));
  return (posErr <= APA_PLANNER_HYBRID_FIT_POS_EPS) && (thetaErr <= APA_PLANNER_HYBRID_FIT_YAW_EPS);
}

static inline void apaPlannerHybridCalcSegEnd(Integer goDir,
                                              Integer trnDir,
                                              Real crv,
                                              Real tht,
                                              Real len,
                                              Real stX,
                                              Real stY,
                                              Real stTheta,
                                              Real ccX,
                                              Real ccY,
                                              Real *endX,
                                              Real *endY,
                                              Real *endTheta) {
  if ((trnDir != 0) && (fabsf(crv) > APA_PLANNER_HYBRID_EPS)) {
    Real invCrv;
    Real theta;

    invCrv = 1.0f / crv;
    theta = stTheta + ((Real)goDir) * tht;
    *endX = ccX + invCrv * sinf(theta);
    *endY = ccY - invCrv * cosf(theta);
    *endTheta = theta;
  } else {
    *endX = stX + ((Real)goDir) * len * cosf(stTheta);
    *endY = stY + ((Real)goDir) * len * sinf(stTheta);
    *endTheta = stTheta + tht;
  }
}

static inline Boolean
apaPlannerHybridSegmentsAreContinuous(const ApaPath_t *path,
                                      const ApaPlannerHybridSegments_t *segments) {
  Integer segIndex;

  if ((path->PathN <= 1) || (segments->SegNum <= 0)) {
    return false;
  }

  segIndex = 0;
  while (segIndex < segments->SegNum) {
    Real endX;
    Real endY;
    Real endTheta;

    if (segments->PathLenArr[segIndex] <= APA_PLANNER_HYBRID_EPS) {
      return false;
    }

    apaPlannerHybridCalcSegEnd(segments->PathGoDir[segIndex],
                               segments->PathTrnDir[segIndex],
                               segments->PathCrv[segIndex],
                               segments->PathTht[segIndex],
                               segments->PathLenArr[segIndex],
                               segments->PathStx[segIndex],
                               segments->PathSty[segIndex],
                               segments->PathStTh[segIndex],
                               segments->PathCC[segIndex * 2],
                               segments->PathCC[segIndex * 2 + 1],
                               &endX,
                               &endY,
                               &endTheta);
    if ((segIndex + 1) < segments->SegNum) {
      if (!apaPlannerHybridPoseMatches(segments->PathStx[segIndex + 1],
                                       segments->PathSty[segIndex + 1],
                                       segments->PathStTh[segIndex + 1],
                                       endX,
                                       endY,
                                       endTheta)) {
        return false;
      }
    } else {
      if (!apaPlannerHybridPoseMatches(path->X[path->PathN - 1],
                                       path->Y[path->PathN - 1],
                                       path->Tht[path->PathN - 1],
                                       endX,
                                       endY,
                                       endTheta)) {
        return false;
      }
    }
    segIndex = segIndex + 1;
  }

  return true;
}

/* Only same-gear, equal-curvature controls may merge. */
static inline Boolean
apaPlannerHybridBuildSegmentsFromMotion(const ApaHybridAStarMotionPath_t *motion,
                                        Integer maximum,
                                        ApaPlannerHybridSegments_t *segments,
                                        ApaPlanningDiagnostics_t *diagnostic) {
  const ApaPath_t *path = &motion->Path;
  apaPlannerHybridSegmentsInit(segments);
  if (path->PathN < 2 || path->PathN > APA_PATH_MAX_POINTS)
    return false;
  for (Integer i = 0; i + 1 < path->PathN; ++i) {
    const Real distance = path->OdoStep[i], curvature = motion->Curvature[i];
    const Integer direction = distance > 0.0f ? 1 : -1;
    if (!isfinite(distance) || !isfinite(curvature))
      return false;
    if (fabsf(distance) <= APA_PLANNER_HYBRID_EPS) {
      if (!apaPlannerHybridPoseMatches(path->X[i + 1],
                                       path->Y[i + 1],
                                       path->Tht[i + 1],
                                       path->X[i],
                                       path->Y[i],
                                       path->Tht[i]))
        return false;
      continue;
    }
    Integer j = segments->SegNum - 1;
    if (j >= 0 && segments->PathGoDir[j] == direction &&
        fabsf(segments->PathCrv[j] - curvature) <= 1.0e-6f) {
      segments->PathLenArr[j] += fabsf(distance);
    } else {
      if (++j >= maximum || j >= APA_PATH_SEG_MAX) {
        apaPlanningRecordFailure(
            diagnostic, APA_FAULT_GEO_SEG_NUM_OVERMANY, "hybrid_segment_capacity", 4);
        return false;
      }
      ++segments->SegNum;
      segments->PathGoDir[j] = direction;
      segments->PathCrv[j] = curvature;
      segments->PathLenArr[j] = fabsf(distance);
      segments->PathStx[j] = path->X[i];
      segments->PathSty[j] = path->Y[i];
      segments->PathStTh[j] = path->Tht[i];
      if (fabsf(curvature) > APA_PLANNER_HYBRID_CURV_EPS) {
        segments->PathTrnDir[j] = curvature > 0.0f ? 1 : -1;
        segments->PathCC[2 * j] = path->X[i] - sinf(path->Tht[i]) / curvature;
        segments->PathCC[2 * j + 1] = path->Y[i] + cosf(path->Tht[i]) / curvature;
      }
      if (j > 0 && segments->PathGoDir[j - 1] != direction)
        ++segments->StpN;
    }
    segments->PathTht[j] = segments->PathCrv[j] * segments->PathLenArr[j];
    segments->PathLen += fabsf(distance);
    Real x, y, yaw;
    apaPlannerHybridEvalPrimitivePose(direction,
                                      segments->PathCrv[j],
                                      segments->PathLenArr[j],
                                      segments->PathStx[j],
                                      segments->PathSty[j],
                                      segments->PathStTh[j],
                                      &x,
                                      &y,
                                      &yaw);
    if (!apaPlannerHybridPoseMatches(path->X[i + 1], path->Y[i + 1], path->Tht[i + 1], x, y, yaw)) {
      apaPlanningRecordFailure(
          diagnostic, APA_FAULT_PATH_NOT_MATCH_TARPOS_IN_LOCAL, "hybrid_motion_geometry", 4);
      return false;
    }
  }
  return segments->SegNum > 0 && apaPlannerHybridSegmentsAreContinuous(path, segments);
}

/* Reverse travel while retaining the steering curvature of every interval. */
static inline void apaPlannerHybridReverseMotion(ApaHybridAStarMotionPath_t *motion) {
  Real curvature[APA_PATH_MAX_POINTS];
  const Integer count = motion->Path.PathN;
  for (Integer i = 0; i + 1 < count; ++i)
    curvature[i] = motion->Curvature[count - 2 - i];
  apaPathReverse(&motion->Path);
  apaPathResetFlags(&motion->Path);
  for (Integer i = 0; i + 1 < count; ++i)
    motion->Curvature[i] = curvature[i];
  if (count > 0)
    motion->Curvature[count - 1] = 0.0f;
}

/* No interpolation or coordinate snapping at a connector/tail junction. */
static inline Boolean apaPlannerHybridAppendMotion(ApaHybridAStarMotionPath_t *motion,
                                                   const ApaHybridAStarMotionPath_t *tail) {
  ApaPath_t *path = &motion->Path;
  const Integer n = path->PathN, m = tail->Path.PathN;
  if (n < 1 || m < 2 || n > APA_PATH_MAX_POINTS || m > APA_PATH_MAX_POINTS ||
      n + m - 1 > APA_PATH_MAX_POINTS)
    return false;
  if (hypotf(path->X[n - 1] - tail->Path.X[0], path->Y[n - 1] - tail->Path.Y[0]) > 1.0e-4f ||
      fabsf(apaCoordWrapToPi(path->Tht[n - 1] - tail->Path.Tht[0])) > 1.0e-5f)
    return false;
  for (Integer i = 0; i + 1 < m; ++i) {
    const Integer j = n - 1 + i;
    path->OdoStep[j] = tail->Path.OdoStep[i];
    motion->Curvature[j] = tail->Curvature[i];
    path->X[j + 1] = tail->Path.X[i + 1];
    path->Y[j + 1] = tail->Path.Y[i + 1];
    path->Tht[j + 1] = tail->Path.Tht[i + 1];
  }
  path->PathN = n + m - 1;
  path->OdoStep[path->PathN - 1] = 0.0f;
  motion->Curvature[path->PathN - 1] = 0.0f;
  apaPathResetFlags(path);
  return true;
}

/* Complete endpoints only with aligned straight motion, never target snapping. */
static inline Boolean apaPlannerHybridCompleteMotion(const ApaHybridAStarMotionPath_t *source,
                                                     const Real start[3],
                                                     const Real goal[3],
                                                     ApaHybridAStarMotionPath_t *motion) {
  *motion = *source;
  ApaPath_t *path = &motion->Path;
  if (path->PathN < 2)
    return false;
  for (Integer end = 0; end < 2; ++end) {
    const Integer index = end ? path->PathN - 1 : 0;
    const Real *pose = end ? goal : start;
    const Real dx = end ? pose[0] - path->X[index] : path->X[0] - pose[0];
    const Real dy = end ? pose[1] - path->Y[index] : path->Y[0] - pose[1];
    const Real yaw = fabsf(apaCoordWrapToPi(path->Tht[index] - pose[2]));
    if (hypotf(dx, dy) <= APA_PLANNER_HYBRID_FIT_POS_EPS && yaw <= APA_PLANNER_HYBRID_FIT_YAW_EPS)
      continue;
    const Real c = cosf(pose[2]), s = sinf(pose[2]);
    if (fabsf(c * dy - s * dx) > 1.0e-4f || yaw > 1.0e-4f || path->PathN >= APA_PATH_MAX_POINTS)
      return false;
    if (end) {
      apaPathAppendPos(path, pose[0], pose[1], pose[2]);
      path->OdoStep[index] = c * dx + s * dy;
      motion->Curvature[index] = 0.0f;
    } else {
      for (Integer i = path->PathN - 2; i >= 0; --i)
        motion->Curvature[i + 1] = motion->Curvature[i];
      apaPathPrependPos(path, pose[0], pose[1], pose[2]);
      path->OdoStep[0] = c * dx + s * dy;
      motion->Curvature[0] = 0.0f;
    }
  }
  return true;
}

static inline Boolean apaPlannerHybridPathIsSafe(const ApaPath_t *path,
                                                 ApaPlannerHybridCollisionContext_t *context,
                                                 Integer *plnChkCnt) {
  Integer idx;
  Integer currentSegCnt;
  Integer currentDir;

  if (plnChkCnt != (Integer *)0) {
    *plnChkCnt = 0;
  }
  if (path->PathN <= 1) {
    return false;
  }

  currentSegCnt = 0;
  currentDir = 0;
  idx = 0;
  while (idx < (path->PathN - 1)) {
    ApaHybridNode_t fromNode;
    ApaHybridNode_t toNode;
    Integer beforeCount;
    Integer stepDir;
    Boolean collisionDetected;

    stepDir = apaPlannerHybridSign(path->OdoStep[idx]);
    if (stepDir != 0) {
      if ((currentDir != 0) && (stepDir != currentDir)) {
        currentSegCnt = currentSegCnt + 1;
      }
      currentDir = stepDir;

      apaNodeInit(&fromNode);
      apaNodeInit(&toNode);
      fromNode.X = path->X[idx];
      fromNode.Y = path->Y[idx];
      fromNode.Theta = path->Tht[idx];
      fromNode.SegCnt = (UInteger8)currentSegCnt;
      toNode.X = path->X[idx + 1];
      toNode.Y = path->Y[idx + 1];
      toNode.Theta = path->Tht[idx + 1];
      toNode.D = path->OdoStep[idx];
      toNode.SegCnt = (UInteger8)currentSegCnt;

      beforeCount = context->CheckCount;
      collisionDetected = apaPlannerHybridCollisionCheck(&fromNode, &toNode, context);
      if (plnChkCnt != (Integer *)0) {
        *plnChkCnt = *plnChkCnt + (context->CheckCount - beforeCount);
      }
      if (collisionDetected) {
        return false;
      }
    }

    idx = idx + 1;
  }

  return true;
}

/* Corridor/stopper inputs set quality exit thresholds without changing admission. */
static inline void apaPlannerHybridConfigureQuality(ApaHybridAStar_t *planner,
                                                    const ApaVehicleCollisionObs_t *obs) {
  RectCollisionCheckForBoundaryInput_t boundary = {0};
  boundary.BoundaryLineNum = obs->BoundaryLineNum;
  for (Integer i = 0; i < obs->BoundaryLineNum; ++i) {
    boundary.BoundaryLineX1[i] = obs->BoundaryLine[i].X1;
    boundary.BoundaryLineY1[i] = obs->BoundaryLine[i].Y1;
    boundary.BoundaryLineX2[i] = obs->BoundaryLine[i].X2;
    boundary.BoundaryLineY2[i] = obs->BoundaryLine[i].Y2;
  }
  boundary.SeparatePtsNum = obs->BoundarySeparatePointNum;
  for (Integer i = 0; i < boundary.SeparatePtsNum; ++i) {
    boundary.SeparatePtsX[i] = obs->BoundarySeparatePoint[i][0];
    boundary.SeparatePtsY[i] = obs->BoundarySeparatePoint[i][1];
  }
  Integer corridor;
  Real width;
  apaSlotCalcCorridorWidth(planner->ActiveSlot.SlotPts,
                           planner->ActiveSlot.EnterEdge,
                           planner->ActiveSlot.TarPos,
                           planner->SlotView.SlotClass,
                           &boundary,
                           &corridor,
                           &width);
  apaHybridAStarConfigureQuality(
      planner, obs->SeparateStopperNum > 0 || obs->SlotTrackStopperNum > 0, corridor);
}

typedef struct {
  ApaHybridAStar_t *Planner;
  ApaPlannerHybridCollisionContext_t *Collision;
  const ApaPlannerHybridPolicy_t *Policy;
  ApaPlanningDiagnostics_t *Diagnostic;
  const ApaHybridAStarMotionPath_t *Tail;
  Integer Maximum;
  ApaPlannerHybridSegments_t Last;
  ApaPlannerHybridSegments_t Best;
} ApaPlannerHybridTrial_t;

static inline Boolean apaPlannerHybridMotionIsSafe(const ApaHybridAStarMotionPath_t *motion,
                                                   ApaPlannerHybridCollisionContext_t *context) {
  const ApaPath_t *path = &motion->Path;
  const ApaPlannerConfig_t *config =
      apaPlannerGlobalGetPlanner(context->CollisionContext.PlannerGlobal);
  for (Integer i = 0; i + 1 < path->PathN; ++i) {
    const Real signedLength = path->OdoStep[i];
    const Real curvature = motion->Curvature[i];
    const Real step = fminf(config->Map.CollisionDetectStepLength, 0.05f);
    if (!isfinite(step) || step <= APA_PLANNER_HYBRID_EPS)
      return false;
    const Integer samples = (Integer)ceilf(fabsf(signedLength) / step);
    ApaHybridNode_t from, to;
    apaNodeInit(&from);
    apaNodeInit(&to);
    from.X = path->X[i];
    from.Y = path->Y[i];
    from.Theta = path->Tht[i];
    for (Integer j = 1; j <= samples; ++j) {
      apaPlannerHybridEvalPrimitivePose(signedLength > 0.0f ? 1 : -1,
                                        curvature,
                                        fabsf(signedLength) * (Real)j / (Real)samples,
                                        path->X[i],
                                        path->Y[i],
                                        path->Tht[i],
                                        &to.X,
                                        &to.Y,
                                        &to.Theta);
      to.D = signedLength / (Real)samples;
      to.Delta = atanf(config->Veh.WB * curvature);
      if (apaPlannerHybridCollisionCheck(&from, &to, context))
        return false;
      from = to;
    }
  }
  return true;
}

static inline Boolean apaPlannerHybridAcceptMotion(const ApaHybridAStarMotionPath_t *source,
                                                   ApaPath_t *qualityPath,
                                                   void *userData) {
  ApaPlannerHybridTrial_t *trial = userData;
  ApaHybridAStarMotionPath_t motion;
  if (!apaPlannerHybridCompleteMotion(
          source, trial->Planner->StartPose, trial->Planner->GoalPose, &motion)) {
    apaPlanningRecordFailure(
        trial->Diagnostic, APA_FAULT_PATH_NOT_MATCH_TARPOS, "hybrid_endpoint_alignment", 3);
    return false;
  }
  if (trial->Collision->ReverseOutput) {
    apaPlannerHybridReverseMotion(&motion);
  }
  if (trial->Tail && !apaPlannerHybridAppendMotion(&motion, trial->Tail)) {
    apaPlanningRecordFailure(trial->Diagnostic,
                             APA_FAULT_PATH_NOT_MATCH_TARPOS_IN_LOCAL,
                             "parallel_maneuver_junction_or_capacity",
                             3);
    return false;
  }

  if (!apaPlannerHybridBuildSegmentsFromMotion(
          &motion, trial->Maximum, &trial->Last, trial->Diagnostic))
    return false;
  if (!apaPlannerHybridMotionIsSafe(&motion, trial->Collision)) {
    apaPlanningRecordFailure(
        trial->Diagnostic, APA_FAULT_COLLISION_RISK_AT_NEW_OBS, "hybrid_motion_collision", 3);
    return false;
  }
  if (trial->Policy && trial->Policy->AcceptCandidate &&
      !trial->Policy->AcceptCandidate(&trial->Last, trial->Policy->UserData))
    return false;
  *qualityPath = motion.Path;

  qualityPath->CollisionFlag = APA_COLLISION_RISK_NOT_EXIST;
  return true;
}

static inline void apaPlannerHybridSelectMotion(void *userData) {
  ApaPlannerHybridTrial_t *trial = userData;
  trial->Best = trial->Last;
}

static inline void apaPlannerHybridBindScene(ApaPlannerHybridCollisionContext_t *collision,
                                             const ApaPlanningSceneQuery_t *scene) {
  if (scene)
    collision->SceneQuery = *scene;

  const ApaVehicleCollisionObs_t *obs = &collision->CollisionContext.Obs;
  VehicleCollisionCheckInput_t *native = &collision->SceneTemplate;
  native->BoundaryLineNum = obs->BoundaryLineNum;
  native->VirtualBoundaryLineNum = obs->VirtualBoundaryLineNum;
  native->SlotTrackStopperNum = obs->SlotTrackStopperNum;
  native->BoundarySepPtsNum = obs->BoundarySeparatePointNum;
  native->SeparateStopperNum = obs->SeparateStopperNum;
  for (Integer i = 0; i < obs->BoundaryLineNum; ++i) {
    native->BoundaryLineX1[i] = obs->BoundaryLine[i].X1;
    native->BoundaryLineY1[i] = obs->BoundaryLine[i].Y1;
    native->BoundaryLineX2[i] = obs->BoundaryLine[i].X2;
    native->BoundaryLineY2[i] = obs->BoundaryLine[i].Y2;
  }
  for (Integer i = 0; i < obs->VirtualBoundaryLineNum; ++i) {
    native->VirtualBoundaryLineX1[i] = obs->VirtualBoundaryLine[i].X1;
    native->VirtualBoundaryLineY1[i] = obs->VirtualBoundaryLine[i].Y1;
    native->VirtualBoundaryLineX2[i] = obs->VirtualBoundaryLine[i].X2;
    native->VirtualBoundaryLineY2[i] = obs->VirtualBoundaryLine[i].Y2;
  }
  for (Integer i = 0; i < obs->SlotTrackStopperNum; ++i) {
    native->SlotTrackStopperX1[i] = obs->SlotTrackStopper[i].X1;
    native->SlotTrackStopperY1[i] = obs->SlotTrackStopper[i].Y1;
    native->SlotTrackStopperX2[i] = obs->SlotTrackStopper[i].X2;
    native->SlotTrackStopperY2[i] = obs->SlotTrackStopper[i].Y2;
  }
  for (Integer i = 0; i < obs->BoundarySeparatePointNum; ++i) {
    native->BoundarySepPtsX[i] = obs->BoundarySeparatePoint[i][0];
    native->BoundarySepPtsY[i] = obs->BoundarySeparatePoint[i][1];
  }
  for (Integer i = 0; i < obs->SeparateStopperNum; ++i) {
    native->SeparateStopperX[i] = obs->SeparateStopper[i][0];
    native->SeparateStopperY[i] = obs->SeparateStopper[i][1];
  }
}

static inline Boolean apaPlannerHybridSearchWithTail(ApaHybridAStar_t *planner,
                                                     ApaPlannerHybridCollisionContext_t *collision,
                                                     const ApaPlanningSceneQuery_t *scene,
                                                     const ApaPlannerHybridPolicy_t *policy,
                                                     ApaPlanningDiagnostics_t *diagnostic,
                                                     Integer maximum,
                                                     const ApaHybridAStarMotionPath_t *tail,
                                                     ApaPlannerHybridSegments_t *segments,
                                                     ApaPath_t *path) {
  ApaPlannerHybridTrial_t trial = {0};
  trial.Planner = planner;
  trial.Collision = collision;
  trial.Policy = policy;
  trial.Diagnostic = diagnostic;
  trial.Maximum = maximum;
  trial.Tail = tail;
  apaPlannerHybridBindScene(collision, scene);
  planner->Diagnostics = diagnostic;
  planner->CandidateCheck = apaPlannerHybridAcceptMotion;
  planner->CandidateSelected = apaPlannerHybridSelectMotion;
  planner->CandidateUserData = &trial;
  apaPlannerHybridConfigureQuality(planner, &collision->CollisionContext.Obs);
  apaHybridAStarSetCollisionCheck(planner, apaPlannerHybridCollisionCheck, collision);
  const ApaPlannerConfig_t *config = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);
  fprintf(stderr,
          "[APA_HYBRID_BUDGET] slot_state=%ld max_nodes=%ld max_iterations=%ld "
          "xy_grid=%.3f yaw_grid=%.6f steer_samples=%ld quality_min_nodes=%ld\n",
          (long)planner->ActiveSlot.SlotStatus,
          (long)config->Conf.PlanningCalPowerMaxPerSlot,
          (long)planner->MaxExpandIterations,
          (double)config->Map.XyGridResolution,
          (double)config->Map.YawGridResolution,
          (long)apaHybridAStarGetSteerSampleCount(planner),
          (long)apaHybridAStarQualityMinimumNodes(planner));
  const ApaHybridAStarState_e state = apaHybridAStarRunToTerminal(planner);
  fprintf(stderr,
          "[APA_SEARCH_GRAPH] parking=%d motion_candidates=%ld open_dominated=%ld "
          "closed_dominated=%ld reopened=%ld min_step=%.3f\n",
          (int)config->Conf.ParkingSearch,
          (long)planner->MotionCandidates,
          (long)planner->OpenDominated,
          (long)planner->ClosedDominated,
          (long)planner->ReopenedStates,
          (double)config->Conf.PathLenMin);
  fprintf(stderr,
          "[APA_HYBRID_SEARCH] state=%d fault=%d expanded=%ld nodes=%ld "
          "accepted=%ld rejected=%ld truncated=%ld expected_segments=%ld "
          "quality_min_nodes=%ld max_nodes=%ld\n",
          (int)state,
          (int)planner->Fault,
          (long)planner->ExpandIterations,
          (long)planner->NodePool.ValidNodeCnt,
          (long)planner->AcceptedCandidates,
          (long)planner->RejectedGoalCandidates,
          (long)planner->TruncatedPrimitives,
          (long)planner->ExpectedSegmentCount,
          (long)apaHybridAStarQualityMinimumNodes(planner),
          (long)apaPlannerGlobalGetConf(planner->PlannerGlobal)->PlanningCalPowerMaxPerSlot);
  if (state != APA_HYBRID_STATE_GOAL_REACHED) {
    apaPlanningRecordFailure(
        diagnostic, apaPlanningSearchError(planner->Fault), "hybrid_search", 1);
    return false;
  }
  *segments = trial.Best;
  segments->PlnChkCnt = collision->CheckCount;
  if (path)
    *path = planner->BestQualityPath;
  return true;
}

static inline Boolean apaPlannerHybridSearch(ApaHybridAStar_t *planner,
                                             ApaPlannerHybridCollisionContext_t *collision,
                                             const ApaPlanningSceneQuery_t *scene,
                                             const ApaPlannerHybridPolicy_t *policy,
                                             ApaPlanningDiagnostics_t *diagnostic,
                                             Integer maximum,
                                             ApaPlannerHybridSegments_t *segments,
                                             ApaPath_t *path) {
  return apaPlannerHybridSearchWithTail(
      planner, collision, scene, policy, diagnostic, maximum, 0, segments, path);
}

static inline Boolean apaPlannerHybridReadGeometry(Integer count,
                                                   Integer maximum,
                                                   Integer plannerCase,
                                                   const Integer *go,
                                                   const Integer *turn,
                                                   const Real *curve,
                                                   const Real *angle,
                                                   const Real *length,
                                                   const Real *x,
                                                   const Real *y,
                                                   const Real *yaw,
                                                   const Real *cx,
                                                   const Real *cy,
                                                   ApaPlannerHybridSegments_t *result,
                                                   ApaPlanningDiagnostics_t *diagnostic) {
  if (count <= 0)
    return false;
  if (count > maximum || count > APA_PATH_SEG_MAX) {
    apaPlanningRecordFailure(
        diagnostic, APA_FAULT_GEO_SEG_NUM_OVERMANY, "geometry_segment_capacity", 4);
    return false;
  }
  ApaPlannerHybridSegments_t segments = {0};
  segments.SegNum = count;
  segments.PlannerCase = plannerCase;
  for (Integer i = 0; i < count; ++i) {
    segments.PathGoDir[i] = go[i];
    segments.PathTrnDir[i] = turn[i];
    segments.PathCrv[i] = curve[i];
    segments.PathTht[i] = angle[i];
    segments.PathLenArr[i] = length[i];
    segments.PathLen += length[i];
    segments.PathStx[i] = x[i];
    segments.PathSty[i] = y[i];
    segments.PathStTh[i] = yaw[i];
    segments.PathCC[2 * i] = cx[i];
    segments.PathCC[2 * i + 1] = cy[i];
    if (i > 0 && go[i] != go[i - 1])
      ++segments.StpN;
  }
  *result = segments;
  return true;
}

#define APA_HYBRID_GEOMETRY_FIELDS(g, maximum)                                                     \
  (g)->SegNum, (maximum), (g)->PlannerCase, (g)->SegGoDir, (g)->SegTrnDir, (g)->SegCrv,            \
      (g)->SegTht, (g)->SegLen, (g)->SegStX, (g)->SegStY, (g)->SegStTh, (g)->SegCcX, (g)->SegCcY

static inline Boolean apaPlannerHybridCheckGeometry(Integer count,
                                                    Integer maximum,
                                                    Integer plannerCase,
                                                    const Integer *go,
                                                    const Integer *turn,
                                                    const Real *curve,
                                                    const Real *angle,
                                                    const Real *length,
                                                    const Real *x,
                                                    const Real *y,
                                                    const Real *yaw,
                                                    const Real *cx,
                                                    const Real *cy,
                                                    const ApaPlannerHybridPolicy_t *policy,
                                                    ApaPlanningDiagnostics_t *diagnostic) {
  ApaPlannerHybridSegments_t segments;
  if (!apaPlannerHybridReadGeometry(count,
                                    maximum,
                                    plannerCase,
                                    go,
                                    turn,
                                    curve,
                                    angle,
                                    length,
                                    x,
                                    y,
                                    yaw,
                                    cx,
                                    cy,
                                    &segments,
                                    diagnostic))
    return false;
  return !policy || !policy->AcceptCandidate ||
         policy->AcceptCandidate(&segments, policy->UserData);
}

#endif
