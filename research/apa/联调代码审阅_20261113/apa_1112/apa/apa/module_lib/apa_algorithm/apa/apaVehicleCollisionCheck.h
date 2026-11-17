/* MATLAB sources: ClassSlot.m, VehicleCollisionCheck.m */
/** @brief 车辆位姿与运动碰撞查询。 */
#ifndef APA_VEHICLE_COLLISION_CHECK_H
#define APA_VEHICLE_COLLISION_CHECK_H

#include "apaCoordUtils.h"
#include "apaEnums.h"
#include "apaMathUtils.h"
#include "apaPlannerConfig.h"
#include "apaPlannerGlobal.h"
#include "apaSearchTypes.h"
#include "apaSlotTypes.h"
#include "baseTypes.h"
#include "vehicleCollisionCheckCore.h"
#include <math.h>

#define APA_VEHICLE_COLLISION_MAX_BOUNDARY_LINES VCC_MAX_BOUNDARY_LINES
#define APA_VEHICLE_COLLISION_MAX_BOUNDARY_SEP_PTS VCC_MAX_SEPARATE_PTS
#define APA_VEHICLE_COLLISION_MAX_SEPARATE_STOPPERS VCC_MAX_SEPARATE_PTS
#define APA_VEHICLE_COLLISION_MAX_SLOT_TRACK_STOPPERS VCC_MAX_TRACK_STOPPER
#define APA_VEHICLE_COLLISION_MAX_VIRTUAL_LINES VCC_MAX_VIRTUAL_LINES
#define APA_VEHICLE_COLLISION_AREA_COUNT VCC_COLLISION_AREA_COUNT

typedef enum {
  APA_VEHICLE_COLLISION_NONE = 0,
  APA_VEHICLE_COLLISION_BOUNDARY = 1,
  APA_VEHICLE_COLLISION_STOPPER = 2,
  APA_VEHICLE_COLLISION_VIRTUAL = 3
} ApaVehicleCollisionType_e;

typedef struct {
  Real X1;
  Real Y1;
  Real X2;
  Real Y2;
} ApaVehicleCollisionLineSeg_t;

typedef struct {
  Integer BoundaryLineNum;
  ApaVehicleCollisionLineSeg_t BoundaryLine[APA_VEHICLE_COLLISION_MAX_BOUNDARY_LINES];
  Integer BoundarySeparatePointNum;
  Real BoundarySeparatePoint[APA_VEHICLE_COLLISION_MAX_BOUNDARY_SEP_PTS][2];
  Integer SeparateStopperNum;
  Real SeparateStopper[APA_VEHICLE_COLLISION_MAX_SEPARATE_STOPPERS][2];
  Integer SlotTrackStopperNum;
  ApaVehicleCollisionLineSeg_t SlotTrackStopper[APA_VEHICLE_COLLISION_MAX_SLOT_TRACK_STOPPERS];
  Integer VirtualBoundaryLineNum;
  ApaVehicleCollisionLineSeg_t VirtualBoundaryLine[APA_VEHICLE_COLLISION_MAX_VIRTUAL_LINES];
} ApaVehicleCollisionObs_t;

typedef struct {
  ApaPlannerGlobalState_t *PlannerGlobal;
  Integer CheckType;
  Integer CollisionDemand;
  ApaSlot_t Slot;
  Boolean SlotValid;
  Boolean ParkingOutFlag;
  Boolean VerticalLastStep;
  Boolean TargetPoseValid;
  Real TargetPose[3];
  ApaVehicleCollisionObs_t Obs;
} ApaVehicleCollisionCheckContext_t;

typedef struct {
  Boolean IsCollision;
  Integer CollisionType;
  Integer CollisionIdx;
  UInteger8 CollisionArea[APA_VEHICLE_COLLISION_AREA_COUNT];
  Real CollisionDepth;
  UInteger8 CrossLine;
  Real SafeMargin1stBound[3];
  Real CollisionPoint[2];
  UInteger8 DeepPointArea;
} ApaVehicleCollisionCheckResult_t;

static inline void apaVehicleCollisionLineSegClear(ApaVehicleCollisionLineSeg_t *lineSeg) {
  lineSeg->X1 = 0.0f;
  lineSeg->Y1 = 0.0f;
  lineSeg->X2 = 0.0f;
  lineSeg->Y2 = 0.0f;
}

static inline void apaVehicleCollisionObsInit(ApaVehicleCollisionObs_t *obs) {
  Integer i;

  obs->BoundaryLineNum = 0;
  obs->BoundarySeparatePointNum = 0;
  obs->SeparateStopperNum = 0;
  obs->SlotTrackStopperNum = 0;
  obs->VirtualBoundaryLineNum = 0;

  for (i = 0; i < APA_VEHICLE_COLLISION_MAX_BOUNDARY_LINES; i = i + 1) {
    apaVehicleCollisionLineSegClear(&obs->BoundaryLine[i]);
  }
  for (i = 0; i < APA_VEHICLE_COLLISION_MAX_BOUNDARY_SEP_PTS; i = i + 1) {
    obs->BoundarySeparatePoint[i][0] = 0.0f;
    obs->BoundarySeparatePoint[i][1] = 0.0f;
  }
  for (i = 0; i < APA_VEHICLE_COLLISION_MAX_SEPARATE_STOPPERS; i = i + 1) {
    obs->SeparateStopper[i][0] = 0.0f;
    obs->SeparateStopper[i][1] = 0.0f;
  }
  for (i = 0; i < APA_VEHICLE_COLLISION_MAX_SLOT_TRACK_STOPPERS; i = i + 1) {
    apaVehicleCollisionLineSegClear(&obs->SlotTrackStopper[i]);
  }
  for (i = 0; i < APA_VEHICLE_COLLISION_MAX_VIRTUAL_LINES; i = i + 1) {
    apaVehicleCollisionLineSegClear(&obs->VirtualBoundaryLine[i]);
  }
}

static inline void apaVehicleCollisionCheckContextInit(ApaVehicleCollisionCheckContext_t *context) {
  context->PlannerGlobal = (ApaPlannerGlobalState_t *)0;
  context->CheckType = APA_COLLISION_CHECK_PLANNING;
  context->CollisionDemand = APA_COLLISION_DEMAND_SIMPLIFY;
  apaSlotInit(&context->Slot);
  context->SlotValid = false;
  context->ParkingOutFlag = false;
  context->VerticalLastStep = false;
  context->TargetPoseValid = false;
  context->TargetPose[0] = 0.0f;
  context->TargetPose[1] = 0.0f;
  context->TargetPose[2] = 0.0f;
  apaVehicleCollisionObsInit(&context->Obs);
}

static inline void
apaVehicleCollisionCheckContextSetPlannerGlobal(ApaVehicleCollisionCheckContext_t *context,
                                                ApaPlannerGlobalState_t *plannerGlobal) {
  context->PlannerGlobal = plannerGlobal;
}

static inline void
apaVehicleCollisionCheckContextSetCheckType(ApaVehicleCollisionCheckContext_t *context,
                                            Integer checkType) {
  context->CheckType = checkType;
}

static inline void
apaVehicleCollisionCheckContextSetCollisionDemand(ApaVehicleCollisionCheckContext_t *context,
                                                  Integer collisionDemand) {
  context->CollisionDemand = collisionDemand;
}

static inline void
apaVehicleCollisionCheckContextSetSlot(ApaVehicleCollisionCheckContext_t *context,
                                       const ApaSlot_t *slot) {
  apaSlotCopy(&context->Slot, slot);
  context->SlotValid = true;
}

static inline void
apaVehicleCollisionCheckContextClearSlot(ApaVehicleCollisionCheckContext_t *context) {
  apaSlotInit(&context->Slot);
  context->SlotValid = false;
}

static inline void
apaVehicleCollisionCheckContextSetParkingOutFlag(ApaVehicleCollisionCheckContext_t *context,
                                                 Boolean parkingOutFlag) {
  context->ParkingOutFlag = parkingOutFlag;
}

static inline void
apaVehicleCollisionCheckContextSetVerticalLastStep(ApaVehicleCollisionCheckContext_t *context,
                                                   Boolean verticalLastStep) {
  context->VerticalLastStep = verticalLastStep;
}

static inline void
apaVehicleCollisionCheckContextSetTargetPose(ApaVehicleCollisionCheckContext_t *context,
                                             const Real targetPose[3]) {
  context->TargetPose[0] = targetPose[0];
  context->TargetPose[1] = targetPose[1];
  context->TargetPose[2] = targetPose[2];
  context->TargetPoseValid = true;
}

static inline void
apaVehicleCollisionCheckContextClearTargetPose(ApaVehicleCollisionCheckContext_t *context) {
  context->TargetPose[0] = 0.0f;
  context->TargetPose[1] = 0.0f;
  context->TargetPose[2] = 0.0f;
  context->TargetPoseValid = false;
}

static inline void apaVehicleCollisionCheckResultInit(ApaVehicleCollisionCheckResult_t *result) {
  Integer i;

  result->IsCollision = false;
  result->CollisionType = APA_VEHICLE_COLLISION_NONE;
  result->CollisionIdx = 0;
  result->CollisionDepth = 99.0f;
  result->CrossLine = 0U;
  result->SafeMargin1stBound[0] = 0.0f;
  result->SafeMargin1stBound[1] = 0.0f;
  result->SafeMargin1stBound[2] = 0.0f;
  result->CollisionPoint[0] = 99.0f;
  result->CollisionPoint[1] = 99.0f;
  result->DeepPointArea = 0U;

  for (i = 0; i < APA_VEHICLE_COLLISION_AREA_COUNT; i = i + 1) {
    result->CollisionArea[i] = 0U;
  }
}

static inline Boolean apaVehicleCollisionObsAddBoundaryLine(
    ApaVehicleCollisionObs_t *obs, Real x1, Real y1, Real x2, Real y2) {
  Integer index;

  if (obs->BoundaryLineNum >= APA_VEHICLE_COLLISION_MAX_BOUNDARY_LINES) {
    return false;
  }

  index = obs->BoundaryLineNum;
  obs->BoundaryLine[index].X1 = x1;
  obs->BoundaryLine[index].Y1 = y1;
  obs->BoundaryLine[index].X2 = x2;
  obs->BoundaryLine[index].Y2 = y2;
  obs->BoundaryLineNum = obs->BoundaryLineNum + 1;
  return true;
}

static inline Boolean
apaVehicleCollisionObsAddBoundarySeparatePoint(ApaVehicleCollisionObs_t *obs, Real x, Real y) {
  Integer index;

  if (obs->BoundarySeparatePointNum >= APA_VEHICLE_COLLISION_MAX_BOUNDARY_SEP_PTS) {
    return false;
  }

  index = obs->BoundarySeparatePointNum;
  obs->BoundarySeparatePoint[index][0] = x;
  obs->BoundarySeparatePoint[index][1] = y;
  obs->BoundarySeparatePointNum = obs->BoundarySeparatePointNum + 1;
  return true;
}

static inline Boolean
apaVehicleCollisionObsAddSeparateStopper(ApaVehicleCollisionObs_t *obs, Real x, Real y) {
  Integer index;

  if (obs->SeparateStopperNum >= APA_VEHICLE_COLLISION_MAX_SEPARATE_STOPPERS) {
    return false;
  }

  index = obs->SeparateStopperNum;
  obs->SeparateStopper[index][0] = x;
  obs->SeparateStopper[index][1] = y;
  obs->SeparateStopperNum = obs->SeparateStopperNum + 1;
  return true;
}

static inline Boolean apaVehicleCollisionObsAddSlotTrackStopper(
    ApaVehicleCollisionObs_t *obs, Real x1, Real y1, Real x2, Real y2) {
  Integer index;

  if (obs->SlotTrackStopperNum >= APA_VEHICLE_COLLISION_MAX_SLOT_TRACK_STOPPERS) {
    return false;
  }

  index = obs->SlotTrackStopperNum;
  obs->SlotTrackStopper[index].X1 = x1;
  obs->SlotTrackStopper[index].Y1 = y1;
  obs->SlotTrackStopper[index].X2 = x2;
  obs->SlotTrackStopper[index].Y2 = y2;
  obs->SlotTrackStopperNum = obs->SlotTrackStopperNum + 1;
  return true;
}

static inline Boolean apaVehicleCollisionObsAddVirtualBoundaryLine(
    ApaVehicleCollisionObs_t *obs, Real x1, Real y1, Real x2, Real y2) {
  Integer index;

  if (obs->VirtualBoundaryLineNum >= APA_VEHICLE_COLLISION_MAX_VIRTUAL_LINES) {
    return false;
  }

  index = obs->VirtualBoundaryLineNum;
  obs->VirtualBoundaryLine[index].X1 = x1;
  obs->VirtualBoundaryLine[index].Y1 = y1;
  obs->VirtualBoundaryLine[index].X2 = x2;
  obs->VirtualBoundaryLine[index].Y2 = y2;
  obs->VirtualBoundaryLineNum = obs->VirtualBoundaryLineNum + 1;
  return true;
}

static inline Boolean apaVehicleCollisionRectContainsPoint(const Real rect[8], Real x, Real y) {
  return vehicleCollisionCheckRectContainsPoint(rect, x, y);
}

static inline Boolean apaVehicleCollisionLineHitsRect(const ApaVehicleCollisionLineSeg_t *line,
                                                      const Real rect[8]) {
  return vehicleCollisionCheckLineHitsRect(line->X1, line->Y1, line->X2, line->Y2, rect);
}

static inline Integer
apaVehicleCollisionResolveSlotClass(const ApaVehicleCollisionCheckContext_t *context) {
  Integer slotClass;

  slotClass = APA_SLOT_CLASS_UNKNOWN;
  if (context->SlotValid) {
    slotClass = context->Slot.SlotClass;
    if (slotClass == APA_SLOT_CLASS_UNKNOWN) {
      slotClass = apaSlotCalcSlotClass(context->Slot.SlotPts, context->Slot.EnterEdge);
    }
  }

  if (slotClass == APA_SLOT_CLASS_UNKNOWN) {
    slotClass = APA_SLOT_CLASS_VERTICAL;
  }

  return slotClass;
}

static inline Boolean
apaVehicleCollisionCalcVehInSlotFlag(const ApaPlannerVehConfig_t *vehicle,
                                     const ApaVehicleCollisionCheckContext_t *context,
                                     Real x,
                                     Real y,
                                     Real yaw) {
  if (!context->SlotValid)
    return false;
  const Real pose[3] = {x, y, yaw};
  Real bounds[4];
  apaSlotCalcSlotBounds(context->Slot.SlotPts, bounds);
  return apaSlotCalcVehInSlotFlag(pose, context->Slot.SlotPts, bounds, vehicle);
}

static inline Boolean
apaVehicleCollisionCalcVehNearSlotFlag(const ApaPlannerVehConfig_t *vehicle,
                                       const ApaVehicleCollisionCheckContext_t *context,
                                       Real x,
                                       Real y,
                                       Real yaw) {
  if (!context->SlotValid)
    return false;
  const Real pose[3] = {x, y, yaw};
  Real bounds[4];
  apaSlotCalcSlotBounds(context->Slot.SlotPts, bounds);
  return apaSlotCalcVehNearSlotFlag(pose, context->Slot.SlotPts, bounds, vehicle);
}

static inline Boolean apaVehicleCollisionCheckPose(const ApaVehicleCollisionCheckContext_t *context,
                                                   Real x,
                                                   Real y,
                                                   Real yaw,
                                                   ApaVehicleCollisionCheckResult_t *result) {
  if (!context || !context->PlannerGlobal)
    return false;
  const ApaPlannerConfig_t *config = apaPlannerGlobalGetPlanner(context->PlannerGlobal);
  const ApaPlannerVehConfig_t *veh = &config->Veh;
  VehicleCollisionCheckInput_t in;
  VehicleCollisionCheckDetailedOutput_t checked;
  in.CheckType = context->CheckType;
  in.CollisionModel = config->Conf.CollisionModel;
  in.ParkingOutFlag = context->ParkingOutFlag;
  in.VerticalLastStep = context->VerticalLastStep;
  in.TarPosValid = context->TargetPoseValid;
  in.TarPosX = context->TargetPose[0];
  in.TarPosY = context->TargetPose[1];
  in.TarPosTheta = context->TargetPose[2];
  in.CollisionDemand = context->CollisionDemand;
  in.VehPosX = x;
  in.VehPosY = y;
  in.VehPosTheta = yaw;
  in.VehW = veh->W;
  in.VehLF = veh->LF;
  in.VehLB = veh->LB;
  in.VehLL = veh->LL;
  in.VehWB = veh->WB;
  in.SlotClass = apaVehicleCollisionResolveSlotClass(context);
  in.VehNearSlotFlag = false;
  in.VehInSlotFlag = false;
  in.SlotPtsValid = context->SlotValid;
  for (Integer i = 0; i < 4; ++i)
    apaSlotGetEnterEdgePoint(context->Slot.SlotPts,
                             context->Slot.EnterEdge,
                             i,
                             &in.SlotPts[2 * i],
                             &in.SlotPts[2 * i + 1]);
  vehicleCollisionCheckLoadMargins(&in, veh);
  in.BoundaryLineNum = context->Obs.BoundaryLineNum;
  for (Integer i = 0; i < in.BoundaryLineNum; ++i) {
    in.BoundaryLineX1[i] = context->Obs.BoundaryLine[i].X1;
    in.BoundaryLineY1[i] = context->Obs.BoundaryLine[i].Y1;
    in.BoundaryLineX2[i] = context->Obs.BoundaryLine[i].X2;
    in.BoundaryLineY2[i] = context->Obs.BoundaryLine[i].Y2;
  }
  in.VirtualBoundaryLineNum = context->Obs.VirtualBoundaryLineNum;
  for (Integer i = 0; i < in.VirtualBoundaryLineNum; ++i) {
    in.VirtualBoundaryLineX1[i] = context->Obs.VirtualBoundaryLine[i].X1;
    in.VirtualBoundaryLineY1[i] = context->Obs.VirtualBoundaryLine[i].Y1;
    in.VirtualBoundaryLineX2[i] = context->Obs.VirtualBoundaryLine[i].X2;
    in.VirtualBoundaryLineY2[i] = context->Obs.VirtualBoundaryLine[i].Y2;
  }
  in.SlotTrackStopperNum = context->Obs.SlotTrackStopperNum;
  for (Integer i = 0; i < in.SlotTrackStopperNum; ++i) {
    in.SlotTrackStopperX1[i] = context->Obs.SlotTrackStopper[i].X1;
    in.SlotTrackStopperY1[i] = context->Obs.SlotTrackStopper[i].Y1;
    in.SlotTrackStopperX2[i] = context->Obs.SlotTrackStopper[i].X2;
    in.SlotTrackStopperY2[i] = context->Obs.SlotTrackStopper[i].Y2;
  }
  in.BoundarySepPtsNum = context->Obs.BoundarySeparatePointNum;
  for (Integer i = 0; i < in.BoundarySepPtsNum; ++i) {
    in.BoundarySepPtsX[i] = context->Obs.BoundarySeparatePoint[i][0];
    in.BoundarySepPtsY[i] = context->Obs.BoundarySeparatePoint[i][1];
  }
  in.SeparateStopperNum = context->Obs.SeparateStopperNum;
  for (Integer i = 0; i < in.SeparateStopperNum; ++i) {
    in.SeparateStopperX[i] = context->Obs.SeparateStopper[i][0];
    in.SeparateStopperY[i] = context->Obs.SeparateStopper[i][1];
  }
  vehicleCollisionCheckDetailed(&in, &checked);
  const VehicleCollisionCheckOutput_t out = checked.Summary;
  result->IsCollision = out.IsCollision;
  result->CollisionType = out.CollisionType;
  result->CollisionIdx = out.CollisionIndex;
  result->CollisionDepth = out.CollisionDepth;
  result->CrossLine = out.CrossLine;
  for (Integer i = 0; i < 9; ++i)
    result->CollisionArea[i] = out.CollisionArea[i];
  for (Integer i = 0; i < 3; ++i)
    result->SafeMargin1stBound[i] = out.SafeMargin1stBound[i];
  result->CollisionPoint[0] = checked.CollisionPoint[0];
  result->CollisionPoint[1] = checked.CollisionPoint[1];
  result->DeepPointArea = checked.DeepPointArea;
  return result->IsCollision;
}

static inline Boolean
apaVehicleCollisionCheckMotionDetailed(const ApaHybridNode_t *fromNode,
                                       const ApaHybridNode_t *toNode,
                                       void *userData,
                                       Integer collisionDemand,
                                       ApaVehicleCollisionCheckResult_t *result,
                                       Real *collisionTravel);

static inline Boolean apaVehicleCollisionCheckMotionDefault(const ApaHybridNode_t *fromNode,
                                                            const ApaHybridNode_t *toNode,
                                                            void *userData) {
  ApaVehicleCollisionCheckContext_t *context;
  ApaVehicleCollisionCheckResult_t result;
  Real collisionTravel;

  context = (ApaVehicleCollisionCheckContext_t *)userData;
  if (context == (ApaVehicleCollisionCheckContext_t *)0) {
    return false;
  }

  return apaVehicleCollisionCheckMotionDetailed(
      fromNode, toNode, userData, context->CollisionDemand, &result, &collisionTravel);
}

static inline Boolean
apaVehicleCollisionCheckMotionDetailed(const ApaHybridNode_t *fromNode,
                                       const ApaHybridNode_t *toNode,
                                       void *userData,
                                       Integer collisionDemand,
                                       ApaVehicleCollisionCheckResult_t *result,
                                       Real *collisionTravel) {
  ApaVehicleCollisionCheckContext_t *context;
  const ApaPlannerConfig_t *plannerConfig;
  ApaVehicleCollisionCheckResult_t localResult;
  Real detectStep;
  Real travelDistance;
  Integer sampleCount;
  Integer savedCollisionDemand;
  Boolean collisionDetected;
  Integer i;

  context = (ApaVehicleCollisionCheckContext_t *)userData;
  if (context == (ApaVehicleCollisionCheckContext_t *)0) {
    return false;
  }
  if (context->PlannerGlobal == (ApaPlannerGlobalState_t *)0) {
    return false;
  }
  if (result == (ApaVehicleCollisionCheckResult_t *)0) {
    result = &localResult;
  }
  apaVehicleCollisionCheckResultInit(result);
  if (collisionTravel != (Real *)0) {
    *collisionTravel = 0.0f;
  }

  plannerConfig = apaPlannerGlobalGetPlanner(context->PlannerGlobal);
  detectStep = plannerConfig->Map.CollisionDetectStepLength;
  if (detectStep <= APA_MATH_EPSILON) {
    detectStep = plannerConfig->Map.XyGridResolution;
  }

  travelDistance = fabsf(toNode->D);
  sampleCount = (Integer)ceilf(travelDistance / detectStep);
  if (sampleCount < 1) {
    sampleCount = 1;
  }

  savedCollisionDemand = context->CollisionDemand;
  context->CollisionDemand = collisionDemand;
  collisionDetected = false;
  for (i = 1; i <= sampleCount; i = i + 1) {
    Real ratio;
    Real posX;
    Real posY;
    Real yaw;
    Real sampleTravel;
    Boolean sampleCollision;

    ratio = (Real)i / (Real)sampleCount;
    posX = fromNode->X + ((toNode->X - fromNode->X) * ratio);
    posY = fromNode->Y + ((toNode->Y - fromNode->Y) * ratio);
    yaw = apaCoordWrapToPi(fromNode->Theta +
                           (apaCoordWrapToPi(toNode->Theta - fromNode->Theta) * ratio));
    sampleTravel = travelDistance * ratio;
    sampleCollision = false;
    if (!collisionDetected) {
      sampleCollision = apaVehicleCollisionCheckPose(context, posX, posY, yaw, result);
    }

    if (sampleCollision) {
      if (collisionTravel != (Real *)0) {
        *collisionTravel = sampleTravel;
      }
      collisionDetected = true;
    }
  }
  context->CollisionDemand = savedCollisionDemand;

  return collisionDetected;
}

#endif
