#include "geoVhPathPlanningTypes.h"
#include "apaCountUtils.h"
#include "apaGeoUtils.h"
#include "apaMathUtils.h"
#include "apaPlannerHybridParity.h"
#include "apaPlannerUtils.h"
#include "checkParkingTypeTypes.h"
#include "geoAxisTransTypes.h"
#include "geoCircleIntrvCheckTypes.h"
#include "geoPosTypClassPkInTypes.h"
#include "geoCircle1pt1lineTypes.h"
#include "geoCircle2lineRTypes.h"
#include "geo2CircleLinkHTTypes.h"
#include "geo2CircleLinkTypes.h"
#include "geoVhEntrPtDngPtTypes.h"
#include "geoLineIntrvCheckTypes.h"
#include "geoRev2CircleLinkTypes.h"
#include "geoStPtSpcVtTypes.h"
#include "apaPlannerConfig.h"
#include "apaSlotTypes.h"
#include "geoDivideTypes.h"
#include "vehicleCollisionCheckTypes.h"
#include <math.h>

#ifndef GEO_VH_PLANNER_CASE_NONE
#define GEO_VH_PLANNER_CASE_NONE 0
#endif

#ifndef GEO_VH_PLANNER_CASE_MAIN
#define GEO_VH_PLANNER_CASE_MAIN 1
#endif

#ifndef GEO_VH_PLANNER_CASE_HYBRID_FALLBACK
#define GEO_VH_PLANNER_CASE_HYBRID_FALLBACK 4
#endif

#ifndef GEO_VH_FALLBACK_WINNER_NONE
#define GEO_VH_FALLBACK_WINNER_NONE 0
#endif

#ifndef GEO_VH_FALLBACK_WINNER_GEOM
#define GEO_VH_FALLBACK_WINNER_GEOM 1
#endif

#ifndef GEO_VH_FALLBACK_WINNER_HYBRID
#define GEO_VH_FALLBACK_WINNER_HYBRID 2
#endif

#ifndef GEO_VH_FALLBACK_DECISION_NONE
#define GEO_VH_FALLBACK_DECISION_NONE 0
#endif

#ifndef GEO_VH_FALLBACK_DECISION_VALID_ONLY
#define GEO_VH_FALLBACK_DECISION_VALID_ONLY 1
#endif

#ifndef GEO_VH_FALLBACK_DECISION_STEP_COUNT
#define GEO_VH_FALLBACK_DECISION_STEP_COUNT 2
#endif

#ifndef GEO_VH_FALLBACK_DECISION_PATH_LEN
#define GEO_VH_FALLBACK_DECISION_PATH_LEN 3
#endif

#ifndef GEO_VH_FALLBACK_DECISION_SEG_NUM
#define GEO_VH_FALLBACK_DECISION_SEG_NUM 4
#endif

#ifndef GEO_VH_FALLBACK_DECISION_PLN_CHK_CNT
#define GEO_VH_FALLBACK_DECISION_PLN_CHK_CNT 5
#endif

#ifndef GEO_VH_CANDIDATE_PATH_EPS
#define GEO_VH_CANDIDATE_PATH_EPS 0.05f
#endif

#ifndef GEO_VH_TARGET_FRAME_SAMPLE_STEP
#define GEO_VH_TARGET_FRAME_SAMPLE_STEP 0.2f
#endif

#ifndef GEO_VH_HYBRID_OVERRIDE_STEP_DELTA_MAX
#define GEO_VH_HYBRID_OVERRIDE_STEP_DELTA_MAX 2
#endif

#ifndef GEO_VH_HYBRID_OVERRIDE_PATH_GAIN_MIN
#define GEO_VH_HYBRID_OVERRIDE_PATH_GAIN_MIN 2.0f
#endif

#ifndef GEO_VH_HYBRID_OVERRIDE_PATH_RATIO_MIN
#define GEO_VH_HYBRID_OVERRIDE_PATH_RATIO_MIN 0.15f
#endif

#ifndef GEO_VH_CIR_LEN_MAX
#define GEO_VH_CIR_LEN_MAX 10.0f
#endif

#ifndef GEO_VH_CIR_EDX_MAX
#define GEO_VH_CIR_EDX_MAX 5.0f
#endif

#ifndef GEO_VH_CIR_EDY_MAX
#define GEO_VH_CIR_EDY_MAX 10.0f
#endif

#ifndef GEO_VH_CIR_THETA_MAX
#define GEO_VH_CIR_THETA_MAX (APA_PI / 6.0f)
#endif

#ifndef GEO_VH_SECOND_GROUP_SUBTYPE_NONE
#define GEO_VH_SECOND_GROUP_SUBTYPE_NONE 0
#endif

#ifndef GEO_VH_SECOND_GROUP_SUBTYPE_1
#define GEO_VH_SECOND_GROUP_SUBTYPE_1 1
#endif

#ifndef GEO_VH_SECOND_GROUP_SUBTYPE_2
#define GEO_VH_SECOND_GROUP_SUBTYPE_2 2
#endif

#ifndef GEO_VH_SECOND_GROUP_SUBTYPE_3
#define GEO_VH_SECOND_GROUP_SUBTYPE_3 3
#endif

#ifndef GEO_VH_START_SPACE_FALLBACK
#define GEO_VH_START_SPACE_FALLBACK 4.1f
#endif

#ifndef GEO_VH_INITIAL_TOTAL_SPACE_MIN
#define GEO_VH_INITIAL_TOTAL_SPACE_MIN 2.5f
#endif

#ifndef GEO_VH_INITIAL_SIDE_SPACE_MIN
#define GEO_VH_INITIAL_SIDE_SPACE_MIN 1.3f
#endif

#ifndef GEO_VH_INITIAL_SIDE_SPACE_TARGET
#define GEO_VH_INITIAL_SIDE_SPACE_TARGET 1.2f
#endif

#ifndef GEO_VH_INITIAL_FORWARD_LEN
#define GEO_VH_INITIAL_FORWARD_LEN 2.0f
#endif

typedef struct {
  Integer GoDir[VH_SEG_MAX];
  Integer TrnDir[VH_SEG_MAX];
  Real Crv[VH_SEG_MAX];
  Real Tht[VH_SEG_MAX];
  Real Len[VH_SEG_MAX];
  Real Stx[VH_SEG_MAX];
  Real Sty[VH_SEG_MAX];
  Real StTh[VH_SEG_MAX];
  Real CcX[VH_SEG_MAX];
  Real CcY[VH_SEG_MAX];
  Integer SegNum;
  Integer IniSegNum;
  Integer PlanCmd;
  Integer FailRs;
  Integer PlnChkCnt;
  Real PlanEndX;
  Real PlanEndY;
  Real PlanEndTheta;
} VhSegWorkspace_t;

static void geoVhCopyObstacleFamiliesToVcc(const GeoVhPathPlanningInput_t *input,
                                           VehicleCollisionCheckInput_t *collisionInput) {
  Integer idx;
  Integer boundaryLineCount0;
  Integer virtualLineCount0;
  Integer boundarySepCount0;
  Integer stopperCount0;
  Integer trackStopperCount0;

  boundaryLineCount0 =
      apaClampCountToLimits(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_BOUNDARY_LINES);
  virtualLineCount0 = apaClampCountToLimits(
      input->VirtualBoundaryLineNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_VIRTUAL_LINES);
  boundarySepCount0 =
      apaClampCountToLimits(input->BoundarySepPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  stopperCount0 =
      apaClampCountToLimits(input->StopperPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  trackStopperCount0 = apaClampCountToLimits(
      input->SlotTrackStopperNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_TRACK_STOPPER);

  collisionInput->BoundaryLineNum = boundaryLineCount0;
  collisionInput->VirtualBoundaryLineNum = virtualLineCount0;
  collisionInput->BoundarySepPtsNum = boundarySepCount0;
  collisionInput->SeparateStopperNum = stopperCount0;
  collisionInput->SlotTrackStopperNum = trackStopperCount0;

  idx = 0;
  while (idx < boundaryLineCount0) {
    collisionInput->BoundaryLineX1[idx] = input->BoundaryLineX1[idx];
    collisionInput->BoundaryLineY1[idx] = input->BoundaryLineY1[idx];
    collisionInput->BoundaryLineX2[idx] = input->BoundaryLineX2[idx];
    collisionInput->BoundaryLineY2[idx] = input->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < virtualLineCount0) {
    collisionInput->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    collisionInput->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    collisionInput->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    collisionInput->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < boundarySepCount0) {
    collisionInput->BoundarySepPtsX[idx] = input->BoundarySepPtX[idx];
    collisionInput->BoundarySepPtsY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < stopperCount0) {
    collisionInput->SeparateStopperX[idx] = input->StopperPtX[idx];
    collisionInput->SeparateStopperY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < trackStopperCount0) {
    collisionInput->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    collisionInput->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    collisionInput->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    collisionInput->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    idx = idx + 1;
  }
}

static void geoVhCopyObstacleFamiliesToCircleCheck(const GeoVhPathPlanningInput_t *input,
                                                   GeoCircleIntrvCheckInput_t *checkInput) {
  Integer idx;
  Integer boundaryLineCount0;
  Integer virtualLineCount0;
  Integer boundarySepCount0;
  Integer stopperCount0;
  Integer trackStopperCount0;

  boundaryLineCount0 =
      apaClampCountToLimits(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX, GEO_BOUNDARY_LINE_MAX);
  virtualLineCount0 = apaClampCountToLimits(
      input->VirtualBoundaryLineNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_VIRTUAL_LINES);
  boundarySepCount0 =
      apaClampCountToLimits(input->BoundarySepPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  stopperCount0 =
      apaClampCountToLimits(input->StopperPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  trackStopperCount0 = apaClampCountToLimits(
      input->SlotTrackStopperNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_TRACK_STOPPER);

  checkInput->BoundaryLineNum = boundaryLineCount0;
  checkInput->VirtualBoundaryLineNum = virtualLineCount0;
  checkInput->BoundarySepPtNum = boundarySepCount0;
  checkInput->StopperPtNum = stopperCount0;
  checkInput->SlotTrackStopperNum = trackStopperCount0;

  idx = 0;
  while (idx < boundaryLineCount0) {
    checkInput->BoundaryLineX1[idx] = input->BoundaryLineX1[idx];
    checkInput->BoundaryLineY1[idx] = input->BoundaryLineY1[idx];
    checkInput->BoundaryLineX2[idx] = input->BoundaryLineX2[idx];
    checkInput->BoundaryLineY2[idx] = input->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < virtualLineCount0) {
    checkInput->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    checkInput->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    checkInput->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    checkInput->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < boundarySepCount0) {
    checkInput->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    checkInput->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < stopperCount0) {
    checkInput->StopperPtX[idx] = input->StopperPtX[idx];
    checkInput->StopperPtY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < trackStopperCount0) {
    checkInput->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    checkInput->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    checkInput->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    checkInput->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    idx = idx + 1;
  }
}

static void geoVhCopyObstacleFamiliesToLineCheck(const GeoVhPathPlanningInput_t *input,
                                                 GeoLineIntrvCheckInput_t *checkInput) {
  Integer idx;
  Integer boundaryLineCount0;
  Integer virtualLineCount0;
  Integer boundarySepCount0;
  Integer stopperCount0;
  Integer trackStopperCount0;

  boundaryLineCount0 =
      apaClampCountToLimits(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX, GEO_BOUNDARY_LINE_MAX);
  virtualLineCount0 = apaClampCountToLimits(
      input->VirtualBoundaryLineNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_VIRTUAL_LINES);
  boundarySepCount0 =
      apaClampCountToLimits(input->BoundarySepPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  stopperCount0 =
      apaClampCountToLimits(input->StopperPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  trackStopperCount0 = apaClampCountToLimits(
      input->SlotTrackStopperNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_TRACK_STOPPER);

  checkInput->BoundaryLineNum = boundaryLineCount0;
  checkInput->VirtualBoundaryLineNum = virtualLineCount0;
  checkInput->BoundarySepPtNum = boundarySepCount0;
  checkInput->SeparateStopperNum = stopperCount0;
  checkInput->SlotTrackStopperNum = trackStopperCount0;

  idx = 0;
  while (idx < boundaryLineCount0) {
    checkInput->BoundaryLineX1[idx] = input->BoundaryLineX1[idx];
    checkInput->BoundaryLineY1[idx] = input->BoundaryLineY1[idx];
    checkInput->BoundaryLineX2[idx] = input->BoundaryLineX2[idx];
    checkInput->BoundaryLineY2[idx] = input->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < virtualLineCount0) {
    checkInput->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    checkInput->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    checkInput->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    checkInput->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < boundarySepCount0) {
    checkInput->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    checkInput->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < stopperCount0) {
    checkInput->SeparateStopperX[idx] = input->StopperPtX[idx];
    checkInput->SeparateStopperY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < trackStopperCount0) {
    checkInput->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    checkInput->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    checkInput->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    checkInput->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    idx = idx + 1;
  }
}

static void geoVhCopyObstacleFamiliesToEntryPoint(const GeoVhPathPlanningInput_t *input,
                                                  GeoVhEntrPtDngPtInput_t *entryInput) {
  Integer idx;
  Integer boundaryLineCount0;
  Integer virtualLineCount0;
  Integer boundarySepCount0;
  Integer stopperCount0;

  boundaryLineCount0 =
      apaClampCountToLimits(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX, DGR_LINE_MAX);
  virtualLineCount0 =
      apaClampCountToLimits(input->VirtualBoundaryLineNum, GEO_BOUNDARY_LINE_MAX, DGR_LINE_MAX);
  boundarySepCount0 =
      apaClampCountToLimits(input->BoundarySepPtNum, GEO_BOUNDARY_POINT_MAX, DGR_PT_MAX);
  stopperCount0 = apaClampCountToLimits(input->StopperPtNum, GEO_BOUNDARY_POINT_MAX, DGR_PT_MAX);

  entryInput->BoundaryLineNum = boundaryLineCount0;
  entryInput->VirtualBoundaryLineNum = virtualLineCount0;
  entryInput->BoundarySepPtNum = boundarySepCount0;
  entryInput->StopperPtNum = stopperCount0;

  idx = 0;
  while (idx < boundaryLineCount0) {
    entryInput->BoundaryP1X[idx] = input->BoundaryLineX1[idx];
    entryInput->BoundaryP1Y[idx] = input->BoundaryLineY1[idx];
    entryInput->BoundaryP2X[idx] = input->BoundaryLineX2[idx];
    entryInput->BoundaryP2Y[idx] = input->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < virtualLineCount0) {
    entryInput->VirtualBoundaryP1X[idx] = input->VirtualBoundaryP1X[idx];
    entryInput->VirtualBoundaryP1Y[idx] = input->VirtualBoundaryP1Y[idx];
    entryInput->VirtualBoundaryP2X[idx] = input->VirtualBoundaryP2X[idx];
    entryInput->VirtualBoundaryP2Y[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < boundarySepCount0) {
    entryInput->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    entryInput->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < stopperCount0) {
    entryInput->StopperPtX[idx] = input->StopperPtX[idx];
    entryInput->StopperPtY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }
}

static Real geoVhResolveVehWidth(const GeoVhPathPlanningParam_t *param) {
  ApaPlannerConfig_t config0;
  Real vehWidth0;

  apaPlannerConfigInitDefault(&config0);
  vehWidth0 = param->VehW;
  if (vehWidth0 <= GEO_EPSILON) {
    vehWidth0 = config0.Veh.W;
  }
  return vehWidth0;
}

static Real geoVhResolveVehFrontOffset(const GeoVhPathPlanningParam_t *param) {
  ApaPlannerConfig_t config0;
  Real frontOffset0;

  apaPlannerConfigInitDefault(&config0);
  frontOffset0 = param->VehLlF;
  if (frontOffset0 <= GEO_EPSILON) {
    frontOffset0 = config0.Veh.LF;
  }
  return frontOffset0;
}

static Real geoVhResolveVehRearOffset(void) {
  ApaPlannerConfig_t config0;

  apaPlannerConfigInitDefault(&config0);
  return config0.Veh.LB;
}

static Real geoVhResolveWheelBase(const GeoVhPathPlanningParam_t *param) {
  ApaPlannerConfig_t config0;

  (void)param;
  apaPlannerConfigInitDefault(&config0);
  return config0.Veh.WB;
}

static void geoVhBuildTargetPrecheckCollisionInput(const GeoVhPathPlanningInput_t *input,
                                                   const GeoVhPathPlanningParam_t *param,
                                                   Real checkX,
                                                   Real checkY,
                                                   Real checkTheta,
                                                   VehicleCollisionCheckInput_t *collisionInput) {
  Real frontOffset0;
  Real rearOffset0;

  *collisionInput = (VehicleCollisionCheckInput_t){0};
  collisionInput->CheckType = VCC_CHECK_TYPE_PLANNING;
  collisionInput->CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  collisionInput->CollisionDemand = VCC_COLLISION_DEMAND_SIMPLIFY;
  collisionInput->SlotClass = VCC_SLOT_CLASS_VERTICAL;
  collisionInput->SlotPtsValid = true;
  collisionInput->SlotPts[0] = input->SltPt1X;
  collisionInput->SlotPts[1] = input->SltPt1Y;
  collisionInput->SlotPts[2] = input->SltPt2X;
  collisionInput->SlotPts[3] = input->SltPt2Y;
  collisionInput->SlotPts[4] = input->SltPt3X;
  collisionInput->SlotPts[5] = input->SltPt3Y;
  collisionInput->SlotPts[6] = input->SltPt4X;
  collisionInput->SlotPts[7] = input->SltPt4Y;
  apaSlotOrderFromEntrance(input->SltEntr,
                           &collisionInput->SlotPts[0],
                           &collisionInput->SlotPts[1],
                           &collisionInput->SlotPts[2],
                           &collisionInput->SlotPts[3],
                           &collisionInput->SlotPts[4],
                           &collisionInput->SlotPts[5],
                           &collisionInput->SlotPts[6],
                           &collisionInput->SlotPts[7]);
  collisionInput->TarPosValid = true;
  collisionInput->TarPosX = input->TarX;
  collisionInput->TarPosY = input->TarY;
  collisionInput->TarPosTheta = input->TarTheta;
  collisionInput->VehPosX = checkX;
  collisionInput->VehPosY = checkY;
  collisionInput->VehPosTheta = checkTheta;
  frontOffset0 = geoVhResolveVehFrontOffset(param);
  rearOffset0 = geoVhResolveVehRearOffset();
  collisionInput->VehLF = frontOffset0;
  collisionInput->VehLB = rearOffset0;
  collisionInput->VehLL = frontOffset0 + rearOffset0;
  collisionInput->VehW = geoVhResolveVehWidth(param);
  collisionInput->VehWB = geoVhResolveWheelBase(param);
  vehicleCollisionCheckLoadDefaultMargins(collisionInput);
  geoVhCopyObstacleFamiliesToVcc(input, collisionInput);
}

static Boolean geoVhRunTargetPrecheck(const GeoVhPathPlanningInput_t *input,
                                      const GeoVhPathPlanningParam_t *param,
                                      GeoVhPathPlanningOutput_t *output) {
  Real checkPose0[3];
  Integer checkIdx0;
  Integer failRs0;
  Boolean targetSafe0;

  checkIdx0 = 0;
  failRs0 = 0;
  targetSafe0 = true;
  while (checkIdx0 < 2) {
    if (targetSafe0) {
      VehicleCollisionCheckInput_t collisionInput;
      VehicleCollisionCheckOutput_t collisionOutput;

      if (checkIdx0 == 0) {
        checkPose0[0] = input->TarX;
        checkPose0[1] = input->TarY;
        checkPose0[2] = input->TarTheta;
      } else {
        checkPose0[0] = input->TarX - (cosf(input->TarTheta) * 1.5f);
        checkPose0[1] = input->TarY - (sinf(input->TarTheta) * 1.5f);
        checkPose0[2] = input->TarTheta;
      }
      geoVhBuildTargetPrecheckCollisionInput(
          input, param, checkPose0[0], checkPose0[1], checkPose0[2], &collisionInput);
      collisionOutput = (VehicleCollisionCheckOutput_t){0};
      apaPlanningSceneCollisionCheck(&collisionInput, &param->SceneQuery, &collisionOutput);
      if (collisionOutput.IsCollision) {
        targetSafe0 = false;
        failRs0 = apaPlannerCollisionFailReason(collisionOutput.CollisionType, 0);
      }
    }
    checkIdx0 = checkIdx0 + 1;
  }

  if (!targetSafe0) {
    output->PlanFlg = 0;
    output->SegNum = 0;
    output->PathLen = 0.0f;
    output->PlannerCase = GEO_VH_PLANNER_CASE_NONE;
    output->FailRs0 = failRs0;
  }
  return targetSafe0;
}

static Boolean geoVhPoseTouchesTargetSlotFrame(const GeoVhPathPlanningInput_t *input,
                                               const GeoVhPathPlanningParam_t *param,
                                               Real poseX,
                                               Real poseY,
                                               Real poseTheta) {
  Real frontOffset0;
  Real rearOffset0;
  Real polyX0[8];
  Real polyY0[8];
  Real slotPts0[8];
  Integer edgeIdx;
  Boolean touchesFrame0;

  frontOffset0 = geoVhResolveVehFrontOffset(param);
  rearOffset0 = geoVhResolveVehRearOffset();
  apaGeoBuildVehicleOctagon(poseX,
                            poseY,
                            poseTheta,
                            frontOffset0,
                            rearOffset0,
                            geoVhResolveVehWidth(param),
                            0.0f,
                            polyX0,
                            polyY0);
  apaSlotSetPoints(slotPts0,
                   input->SltPt1X,
                   input->SltPt1Y,
                   input->SltPt2X,
                   input->SltPt2Y,
                   input->SltPt3X,
                   input->SltPt3Y,
                   input->SltPt4X,
                   input->SltPt4Y);

  touchesFrame0 = false;
  edgeIdx = 1;
  while ((edgeIdx <= 4) && (touchesFrame0 == false)) {
    if (edgeIdx != input->SltEntr) {
      Real edgeX1;
      Real edgeY1;
      Real edgeX2;
      Real edgeY2;

      apaSlotGetEdge(slotPts0, edgeIdx, &edgeX1, &edgeY1, &edgeX2, &edgeY2);
      if (apaGeoSegmentTouchesOctagon(edgeX1, edgeY1, edgeX2, edgeY2, polyX0, polyY0)) {
        touchesFrame0 = true;
      }
    }
    edgeIdx = edgeIdx + 1;
  }

  return touchesFrame0;
}

static void geoVhFillCircleInput(const GeoVhPathPlanningInput_t *input,
                                 const GeoVhPathPlanningParam_t *param,
                                 Real ctX,
                                 Real ctY,
                                 Real rd,
                                 Real stX,
                                 Real stY,
                                 Real stTh,
                                 Real edX,
                                 Real edY,
                                 Real edTh,
                                 Integer dirS,
                                 Integer trnD,
                                 Real kStp,
                                 GeoCircleIntrvCheckInput_t *chk) {
  *chk = (GeoCircleIntrvCheckInput_t){0};
  chk->SlotClass = VCC_SLOT_CLASS_VERTICAL;
  chk->CptX = ctX;
  chk->CptY = ctY;
  chk->Radius = rd;
  chk->Pt1X = stX;
  chk->Pt1Y = stY;
  chk->Pt1Theta = stTh;
  chk->Pt2X = edX;
  chk->Pt2Y = edY;
  chk->Pt2Theta = edTh;
  chk->GoDir = dirS;
  chk->TrnDir = trnD;
  chk->CheckStep = kStp;
  chk->SlotP1X = input->SltPt1X;
  chk->SlotP1Y = input->SltPt1Y;
  chk->SlotP2X = input->SltPt2X;
  chk->SlotP2Y = input->SltPt2Y;
  chk->SlotP3X = input->SltPt3X;
  chk->SlotP3Y = input->SltPt3Y;
  chk->SlotP4X = input->SltPt4X;
  chk->SlotP4Y = input->SltPt4Y;
  apaSlotOrderFromEntrance(input->SltEntr,
                           &chk->SlotP1X,
                           &chk->SlotP1Y,
                           &chk->SlotP2X,
                           &chk->SlotP2Y,
                           &chk->SlotP3X,
                           &chk->SlotP3Y,
                           &chk->SlotP4X,
                           &chk->SlotP4Y);
  chk->VehWidth = geoVhResolveVehWidth(param);
  chk->VehFrontOffset = geoVhResolveVehFrontOffset(param);
  chk->VehRearOffset = geoVhResolveVehRearOffset();
  chk->UsePlanningCheck = true;
  chk->DisableSlotBoundaryFallback = true;
  geoVhCopyObstacleFamiliesToCircleCheck(input, chk);
}

static void geoVhWriteArcSeg(VhSegWorkspace_t *ws,
                             Integer idx,
                             Real stX,
                             Real stY,
                             Real stTh,
                             Integer goDir,
                             Integer trnD,
                             Real rd,
                             Real tht,
                             Real len,
                             Real ccX,
                             Real ccY) {
  ws->Stx[idx] = stX;
  ws->Sty[idx] = stY;
  ws->StTh[idx] = stTh;
  ws->GoDir[idx] = goDir;
  ws->TrnDir[idx] = trnD;
  ws->Crv[idx] = geoDivideScalar((Real)trnD, rd);
  ws->Tht[idx] = tht;
  ws->Len[idx] = len;
  ws->CcX[idx] = ccX;
  ws->CcY[idx] = ccY;
}

static void geoVhWriteLineSeg(
    VhSegWorkspace_t *ws, Integer idx, Real stX, Real stY, Real stTh, Integer goDir, Real len) {
  ws->Stx[idx] = stX;
  ws->Sty[idx] = stY;
  ws->StTh[idx] = stTh;
  ws->GoDir[idx] = goDir;
  ws->TrnDir[idx] = 0;
  ws->Crv[idx] = 0.0f;
  ws->Tht[idx] = 0.0f;
  ws->Len[idx] = len;
  ws->CcX[idx] = 0.0f;
  ws->CcY[idx] = 0.0f;
}

static void geoVhCopyWorkspaceSegmentToOutput(const VhSegWorkspace_t *ws,
                                              Integer srcIdx,
                                              Integer dstIdx,
                                              GeoVhPathPlanningOutput_t *output) {
  output->SegGoDir[dstIdx] = ws->GoDir[srcIdx];
  output->SegTrnDir[dstIdx] = ws->TrnDir[srcIdx];
  output->SegCrv[dstIdx] = ws->Crv[srcIdx];
  output->SegTht[dstIdx] = ws->Tht[srcIdx];
  output->SegLen[dstIdx] = ws->Len[srcIdx];
  output->SegStX[dstIdx] = ws->Stx[srcIdx];
  output->SegStY[dstIdx] = ws->Sty[srcIdx];
  output->SegStTh[dstIdx] = ws->StTh[srcIdx];
  output->SegCcX[dstIdx] = ws->CcX[srcIdx];
  output->SegCcY[dstIdx] = ws->CcY[srcIdx];
}

static void geoVhAssembleOutput(const VhSegWorkspace_t *ws, GeoVhPathPlanningOutput_t *output) {
  Integer finalSegCount0;
  Integer srcStart;
  Integer si;
  Real totalLen;
  Integer stpN0;
  Integer prevDir;
  Boolean assembleOk0;

  finalSegCount0 = ws->IniSegNum + ws->SegNum;
  srcStart = VH_SEG_MAX - ws->SegNum;
  totalLen = 0.0f;
  assembleOk0 = false;
  if ((finalSegCount0 > 0) && (finalSegCount0 <= VH_SEG_OUT_MAX)) {
    assembleOk0 = true;
  }

  if (assembleOk0) {
    si = 0;
    while (si < ws->IniSegNum) {
      geoVhCopyWorkspaceSegmentToOutput(ws, si, si, output);
      si = si + 1;
    }
  }

  if (assembleOk0) {
    si = 0;
    while (si < ws->SegNum) {
      Integer srcIdx;
      Integer dstIdx;
      srcIdx = srcStart + si;
      dstIdx = ws->IniSegNum + si;
      geoVhCopyWorkspaceSegmentToOutput(ws, srcIdx, dstIdx, output);
      si = si + 1;
    }
  }

  if (assembleOk0) {
    output->SegNum = finalSegCount0;
    si = 0;
    while (si < finalSegCount0) {
      totalLen = totalLen + fabsf(output->SegLen[si]);
      si = si + 1;
    }
    output->PathLen = totalLen;
    output->PlanFlg = 1;
  }

  stpN0 = 0;
  if (output->SegNum > 0) {
    prevDir = output->SegGoDir[0];
    if (prevDir < 0) {
      stpN0 = 1;
    }
    si = 1;
    while (si < output->SegNum) {
      if (output->SegGoDir[si] * prevDir < 0) {
        stpN0 = stpN0 + 1;
        prevDir = output->SegGoDir[si];
      }
      si = si + 1;
    }
  }
  output->StpN = stpN0;
}

static void geoVhBuildParkingTypeInput(const GeoVhPathPlanningInput_t *input,
                                       CheckParkingTypeInput_t *typeInput) {
  *typeInput = (CheckParkingTypeInput_t){0};
  typeInput->VehX = input->VehX;
  typeInput->VehY = input->VehY;
  typeInput->VehTheta = input->VehTheta;
  typeInput->TarX = input->TarX;
  typeInput->TarY = input->TarY;
  typeInput->TarTheta = input->TarTheta;
  typeInput->SltPt1X = input->SltPt1X;
  typeInput->SltPt1Y = input->SltPt1Y;
  typeInput->SltPt2X = input->SltPt2X;
  typeInput->SltPt2Y = input->SltPt2Y;
  typeInput->SltPt3X = input->SltPt3X;
  typeInput->SltPt3Y = input->SltPt3Y;
  typeInput->SltPt4X = input->SltPt4X;
  typeInput->SltPt4Y = input->SltPt4Y;
  typeInput->EnterEdge = input->SltEntr;
}

static Integer geoVhResolveParkingType(const GeoVhPathPlanningInput_t *input) {
  CheckParkingTypeInput_t typeInput;
  CheckParkingTypeOutput_t typeOutput;

  geoVhBuildParkingTypeInput(input, &typeInput);
  typeOutput = (CheckParkingTypeOutput_t){0};
  checkParkingType(&typeInput, &typeOutput);
  return typeOutput.ParkingType;
}

static void geoVhResolveEntryPointFromPose(const GeoVhPathPlanningInput_t *input,
                                           Real planEndX,
                                           Real planEndY,
                                           Real planEndTheta,
                                           Real *entryX,
                                           Real *entryY) {
  GeoVhEntrPtDngPtInput_t entryInput;
  GeoVhEntrPtDngPtOutput_t entryOutput;

  entryInput = (GeoVhEntrPtDngPtInput_t){0};
  entryInput.SltPt1X = input->SltPt1X;
  entryInput.SltPt1Y = input->SltPt1Y;
  entryInput.SltPt2X = input->SltPt2X;
  entryInput.SltPt2Y = input->SltPt2Y;
  entryInput.SltPt3X = input->SltPt3X;
  entryInput.SltPt3Y = input->SltPt3Y;
  entryInput.SltPt4X = input->SltPt4X;
  entryInput.SltPt4Y = input->SltPt4Y;
  entryInput.SltEntr = input->SltEntr;
  entryInput.VehX = planEndX;
  entryInput.VehY = planEndY;
  entryInput.VehTheta = planEndTheta;
  entryInput.TgtX = input->TarX;
  entryInput.TgtY = input->TarY;
  entryInput.TgtTheta = input->TarTheta;
  entryInput.Radius = input->Rmin;
  entryInput.PrkType = geoVhResolveParkingType(input);
  entryInput.SurdType = 0;
  geoVhCopyObstacleFamiliesToEntryPoint(input, &entryInput);

  entryOutput = (GeoVhEntrPtDngPtOutput_t){0};
  geoVhEntrPtDngPt(&entryInput, &entryOutput);
  *entryX = entryOutput.X1;
  *entryY = entryOutput.Y1;
  if ((*entryX != *entryX) || (*entryY != *entryY)) {
    *entryX = input->EntrPtX;
    *entryY = input->EntrPtY;
  } else {
    if ((fabsf(*entryX) > 1000000.0f) || (fabsf(*entryY) > 1000000.0f)) {
      *entryX = input->EntrPtX;
      *entryY = input->EntrPtY;
    }
  }
}

static Boolean geoVhPathTypeInTwoCircleFamily(Integer pathType) {
  Boolean inFamily0;

  inFamily0 = false;
  if ((pathType == 4) || (pathType == 34) || (pathType == 8) || (pathType == 38) ||
      (pathType == 9) || (pathType == 39) || (pathType == 2) || (pathType == 32) ||
      (pathType == 6) || (pathType == 36) || (pathType == 13) || (pathType == 43) ||
      (pathType == 15) || (pathType == 45)) {
    inFamily0 = true;
  }
  return inFamily0;
}

static Boolean geoVhPathTypeInCircleLineFamily(Integer pathType) {
  Boolean inFamily0;

  inFamily0 = false;
  if ((pathType == 1) || (pathType == 31) || (pathType == 3) || (pathType == 33) ||
      (pathType == 5) || (pathType == 35) || (pathType == 7) || (pathType == 37) ||
      (pathType == 10) || (pathType == 40) || (pathType == 11) || (pathType == 41) ||
      (pathType == 12) || (pathType == 42) || (pathType == 14) || (pathType == 44) ||
      (pathType == 16) || (pathType == 46)) {
    inFamily0 = true;
  }
  return inFamily0;
}

static void geoVhClassifyLoopPose(Real stX,
                                  Real stY,
                                  Real stTheta,
                                  Real endX,
                                  Real endY,
                                  Real endTheta,
                                  Integer dirS,
                                  GeoPosTypClassPkInOutput_t *classOutput) {
  GeoPosTypClassPkInInput_t classInput;

  classInput = (GeoPosTypClassPkInInput_t){0};
  classInput.StPtX = stX;
  classInput.StPtY = stY;
  classInput.StPtTheta = stTheta;
  classInput.EdPtX = endX;
  classInput.EdPtY = endY;
  classInput.EdPtTheta = endTheta;
  classInput.Dir = dirS;
  *classOutput = (GeoPosTypClassPkInOutput_t){0};
  geoPosTypClassPkIn(&classInput, classOutput);
}

static GeoCircleIntrvCheckOutput_t geoVhRunCircleIntrvCheck(const GeoVhPathPlanningInput_t *input,
                                                            const GeoVhPathPlanningParam_t *param,
                                                            Real centerX,
                                                            Real centerY,
                                                            Real radius,
                                                            Real startX,
                                                            Real startY,
                                                            Real startTheta,
                                                            Real endX,
                                                            Real endY,
                                                            Real endTheta,
                                                            Integer goDir,
                                                            Integer trnDir,
                                                            Real checkStep) {
  GeoCircleIntrvCheckInput_t checkInput;
  GeoCircleIntrvCheckOutput_t checkOutput;

  geoVhFillCircleInput(input,
                       param,
                       centerX,
                       centerY,
                       radius,
                       startX,
                       startY,
                       startTheta,
                       endX,
                       endY,
                       endTheta,
                       goDir,
                       trnDir,
                       checkStep,
                       &checkInput);
  checkOutput = (GeoCircleIntrvCheckOutput_t){0};
  geoCircleIntrvCheckWithScene(&checkInput, &param->SceneQuery, &checkOutput);
  return checkOutput;
}

static GeoLineIntrvCheckOutput_t geoVhRunLineIntrvCheck(const GeoVhPathPlanningInput_t *input,
                                                        const GeoVhPathPlanningParam_t *param,
                                                        Real startX,
                                                        Real startY,
                                                        Real startTheta,
                                                        Real endX,
                                                        Real endY,
                                                        Real endTheta,
                                                        Real checkStep) {
  GeoLineIntrvCheckInput_t checkInput;
  GeoLineIntrvCheckOutput_t checkOutput;

  checkInput = (GeoLineIntrvCheckInput_t){0};
  checkInput.Pt1X = startX;
  checkInput.Pt1Y = startY;
  checkInput.Pt1Theta = startTheta;
  checkInput.Pt2X = endX;
  checkInput.Pt2Y = endY;
  checkInput.Pt2Theta = endTheta;
  checkInput.CheckStep = checkStep;
  checkInput.GoalX = input->TarX;
  checkInput.GoalY = input->TarY;
  checkInput.GoalYaw = input->TarTheta;
  checkInput.SlotP1X = input->SltPt1X;
  checkInput.SlotP1Y = input->SltPt1Y;
  checkInput.SlotP2X = input->SltPt2X;
  checkInput.SlotP2Y = input->SltPt2Y;
  checkInput.SlotP3X = input->SltPt3X;
  checkInput.SlotP3Y = input->SltPt3Y;
  checkInput.SlotP4X = input->SltPt4X;
  checkInput.SlotP4Y = input->SltPt4Y;
  apaSlotOrderFromEntrance(input->SltEntr,
                           &checkInput.SlotP1X,
                           &checkInput.SlotP1Y,
                           &checkInput.SlotP2X,
                           &checkInput.SlotP2Y,
                           &checkInput.SlotP3X,
                           &checkInput.SlotP3Y,
                           &checkInput.SlotP4X,
                           &checkInput.SlotP4Y);
  checkInput.VehWidth = geoVhResolveVehWidth(param);
  checkInput.VehFrontOffset = geoVhResolveVehFrontOffset(param);
  checkInput.VehRearOffset = geoVhResolveVehRearOffset();
  checkInput.UsePlanningCheck = true;
  checkInput.DisableSlotBoundaryFallback = true;
  checkInput.SlotClass = VCC_SLOT_CLASS_VERTICAL;
  geoVhCopyObstacleFamiliesToLineCheck(input, &checkInput);

  checkOutput = (GeoLineIntrvCheckOutput_t){0};
  geoLineIntrvCheckWithScene(&checkInput, &param->SceneQuery, &checkOutput);
  return checkOutput;
}

static void geoVhBuildStartSpaceInput(const GeoVhPathPlanningInput_t *input,
                                      const GeoVhPathPlanningParam_t *param,
                                      Real planEndX,
                                      Real planEndY,
                                      Real planEndTheta,
                                      Real checkLen,
                                      GeoStPtSpcVtInput_t *spaceInput) {
  *spaceInput = (GeoStPtSpcVtInput_t){0};
  spaceInput->VehPosX = planEndX;
  spaceInput->VehPosY = planEndY;
  spaceInput->VehPosTheta = planEndTheta;
  spaceInput->TarPtX = input->TarX;
  spaceInput->TarPtY = input->TarY;
  spaceInput->TarPtTheta = input->TarTheta;
  spaceInput->SltPt1X = input->SltPt1X;
  spaceInput->SltPt1Y = input->SltPt1Y;
  spaceInput->SltPt2X = input->SltPt2X;
  spaceInput->SltPt2Y = input->SltPt2Y;
  spaceInput->SltPt3X = input->SltPt3X;
  spaceInput->SltPt3Y = input->SltPt3Y;
  spaceInput->SltPt4X = input->SltPt4X;
  spaceInput->SltPt4Y = input->SltPt4Y;
  spaceInput->SltEntr = input->SltEntr;
  spaceInput->CheckLen = checkLen;
  spaceInput->SltType = VCC_SLOT_CLASS_VERTICAL;
  spaceInput->FallbackSpace = GEO_VH_START_SPACE_FALLBACK;
  spaceInput->UseCollisionCheck = true;
  geoVhBuildTargetPrecheckCollisionInput(
      input, param, planEndX, planEndY, planEndTheta, &spaceInput->VccTemplate);
}

static Boolean
geoVhResolveInitialOffset(Real relativeY, Real leftSpace, Real rightSpace, Real *offset) {
  Real totalSpace0;
  Boolean desired0;

  totalSpace0 = leftSpace + rightSpace;
  desired0 = false;
  *offset = 0.0f;
  if (relativeY < 0.0f) {
    if ((totalSpace0 > GEO_VH_INITIAL_TOTAL_SPACE_MIN) &&
        (rightSpace < GEO_VH_INITIAL_SIDE_SPACE_MIN)) {
      *offset = GEO_VH_INITIAL_SIDE_SPACE_TARGET - rightSpace;
      desired0 = true;
    }
  } else {
    if ((totalSpace0 > GEO_VH_INITIAL_TOTAL_SPACE_MIN) &&
        (leftSpace < GEO_VH_INITIAL_SIDE_SPACE_MIN)) {
      *offset = -(GEO_VH_INITIAL_SIDE_SPACE_TARGET - leftSpace);
      desired0 = true;
    }
  }
  return desired0;
}

static Boolean geoVhTryInitialOffset(const GeoVhPathPlanningInput_t *input,
                                     const GeoVhPathPlanningParam_t *param,
                                     VhSegWorkspace_t *ws) {
  GeoStPtSpcVtInput_t spaceInput;
  GeoStPtSpcVtOutput_t spaceOutput;
  Geo2CircleLinkHTInput_t linkInput;
  Geo2CircleLinkHTOutput_t linkOutput;
  GeoCircleIntrvCheckOutput_t firstOutput;
  GeoCircleIntrvCheckOutput_t secondOutput;
  Real targetRelX0;
  Real targetRelY0;
  Real targetRelTheta0;
  Real checkLen0;
  Real offset0;
  Integer offsetDir0;
  Real firstLen0;
  Real secondLen0;
  Boolean desired0;
  Boolean linkOk0;
  Boolean firstArcOk0;
  Boolean secondArcOk0;
  Boolean ok0;

  geoAxisTransToTarget(input->TarX,
                       input->TarY,
                       input->TarTheta,
                       ws->PlanEndX,
                       ws->PlanEndY,
                       ws->PlanEndTheta,
                       &targetRelX0,
                       &targetRelY0,
                       &targetRelTheta0);
  checkLen0 = 3.0f + targetRelX0;
  geoVhBuildStartSpaceInput(
      input, param, ws->PlanEndX, ws->PlanEndY, ws->PlanEndTheta, checkLen0, &spaceInput);
  spaceOutput = (GeoStPtSpcVtOutput_t){0};
  geoStPtSpcVtWithScene(&spaceInput, &param->SceneQuery, &spaceOutput);
  ws->PlnChkCnt = ws->PlnChkCnt + spaceOutput.CheckCount;

  desired0 = geoVhResolveInitialOffset(
      targetRelY0, spaceOutput.LeftSpace, spaceOutput.RightSpace, &offset0);
  offsetDir0 = apaMathSign(offset0);
  linkOk0 = false;
  firstArcOk0 = false;
  secondArcOk0 = false;
  ok0 = false;
  if (desired0 && (offsetDir0 != 0) && (input->Rmin > GEO_EPSILON)) {
    linkInput = (Geo2CircleLinkHTInput_t){0};
    linkInput.Pt1X = ws->PlanEndX;
    linkInput.Pt1Y = ws->PlanEndY;
    linkInput.Pt1Theta = ws->PlanEndTheta;
    linkInput.Pt2X = ws->PlanEndX - (offset0 * sinf(ws->PlanEndTheta)) +
                     (GEO_VH_INITIAL_FORWARD_LEN * cosf(ws->PlanEndTheta));
    linkInput.Pt2Y = ws->PlanEndY + (offset0 * cosf(ws->PlanEndTheta)) +
                     (GEO_VH_INITIAL_FORWARD_LEN * sinf(ws->PlanEndTheta));
    linkInput.Pt2Theta = ws->PlanEndTheta;
    linkInput.Dir = offsetDir0;
    linkInput.R1 = input->Rmin;
    linkInput.R2 = input->Rmin;
    linkOutput = (Geo2CircleLinkHTOutput_t){0};
    geo2CircleLinkHT(&linkInput, &linkOutput);
    linkOk0 = (linkOutput.FailFlag == 0);
  }

  if (linkOk0) {
    firstOutput = geoVhRunCircleIntrvCheck(input,
                                           param,
                                           linkOutput.Ct1X,
                                           linkOutput.Ct1Y,
                                           input->Rmin,
                                           linkInput.Pt1X,
                                           linkInput.Pt1Y,
                                           linkInput.Pt1Theta,
                                           linkOutput.St2X,
                                           linkOutput.St2Y,
                                           linkOutput.St2Theta,
                                           1,
                                           linkOutput.TrnD1,
                                           0.1f);
    ws->PlnChkCnt = ws->PlnChkCnt + firstOutput.CheckCount;
    firstArcOk0 = (firstOutput.IntrvFlg == 0);
  }

  if (firstArcOk0) {
    secondOutput = geoVhRunCircleIntrvCheck(input,
                                            param,
                                            linkOutput.Ct2X,
                                            linkOutput.Ct2Y,
                                            input->Rmin,
                                            linkOutput.St2X,
                                            linkOutput.St2Y,
                                            linkOutput.St2Theta,
                                            linkOutput.St3X,
                                            linkOutput.St3Y,
                                            linkOutput.St3Theta,
                                            1,
                                            linkOutput.TrnD2,
                                            0.1f);
    ws->PlnChkCnt = ws->PlnChkCnt + secondOutput.CheckCount;
    secondArcOk0 = (secondOutput.IntrvFlg == 0);
  }

  if (secondArcOk0) {
    firstLen0 = apaMathResolvePositiveOrFallback(firstOutput.SafeLen, linkOutput.Len1);
    secondLen0 = apaMathResolvePositiveOrFallback(secondOutput.SafeLen, linkOutput.Len2);
    geoVhWriteArcSeg(ws,
                     0,
                     linkInput.Pt1X,
                     linkInput.Pt1Y,
                     linkInput.Pt1Theta,
                     1,
                     linkOutput.TrnD1,
                     input->Rmin,
                     linkOutput.St2Theta - linkInput.Pt1Theta,
                     firstLen0,
                     linkOutput.Ct1X,
                     linkOutput.Ct1Y);
    geoVhWriteArcSeg(ws,
                     1,
                     linkOutput.St2X,
                     linkOutput.St2Y,
                     linkOutput.St2Theta,
                     1,
                     linkOutput.TrnD2,
                     input->Rmin,
                     linkOutput.St3Theta - linkOutput.St2Theta,
                     secondLen0,
                     linkOutput.Ct2X,
                     linkOutput.Ct2Y);
    ws->IniSegNum = 2;
    ws->PlanEndX = linkOutput.St3X;
    ws->PlanEndY = linkOutput.St3Y;
    ws->PlanEndTheta = linkOutput.St3Theta;
    ok0 = true;
  }

  (void)targetRelTheta0;
  return ok0;
}

static Boolean geoVhAppendArcToTail(VhSegWorkspace_t *ws,
                                    Real endX,
                                    Real endY,
                                    Real endTheta,
                                    Integer goDir,
                                    Integer trnDir,
                                    Real radius,
                                    Real safeLen,
                                    Real centerX,
                                    Real centerY) {
  Integer tailIdx0;
  Integer writeIdx0;
  Real theta0;
  Boolean ok0;

  ok0 = false;
  tailIdx0 = VH_SEG_MAX - ws->SegNum;
  writeIdx0 = tailIdx0 - 1;
  if ((ws->SegNum >= VH_SEG_MAX) || (tailIdx0 < 0) || (tailIdx0 >= VH_SEG_MAX) || (writeIdx0 < 0)) {
    ws->PlanCmd = 5;
    if (ws->FailRs == 0) {
      ws->FailRs = 9;
    }
  } else if (fabsf(radius) <= GEO_EPSILON) {
    ws->PlanCmd = 5;
    if (ws->FailRs == 0) {
      ws->FailRs = 7;
    }
  } else {
    theta0 = ws->StTh[tailIdx0] - endTheta;
    geoVhWriteArcSeg(ws,
                     writeIdx0,
                     endX,
                     endY,
                     endTheta,
                     goDir,
                     trnDir,
                     radius,
                     theta0,
                     safeLen,
                     centerX,
                     centerY);
    ws->SegNum = ws->SegNum + 1;
    ok0 = true;
  }
  return ok0;
}

static Boolean geoVhAppendLineToTail(
    VhSegWorkspace_t *ws, Real lineStartX, Real lineStartY, Real lineStartTheta, Integer goDir) {
  Integer tailIdx0;
  Integer writeIdx0;
  Real dx0;
  Real dy0;
  Real len0;
  Boolean ok0;

  ok0 = false;
  tailIdx0 = VH_SEG_MAX - ws->SegNum;
  writeIdx0 = tailIdx0 - 1;
  if ((ws->SegNum >= VH_SEG_MAX) || (tailIdx0 < 0) || (tailIdx0 >= VH_SEG_MAX) || (writeIdx0 < 0)) {
    ws->PlanCmd = 5;
    if (ws->FailRs == 0) {
      ws->FailRs = 9;
    }
  } else {
    dx0 = lineStartX - ws->Stx[tailIdx0];
    dy0 = lineStartY - ws->Sty[tailIdx0];
    len0 = sqrtf((dx0 * dx0) + (dy0 * dy0));
    geoVhWriteLineSeg(ws, writeIdx0, lineStartX, lineStartY, lineStartTheta, goDir, len0);
    ws->SegNum = ws->SegNum + 1;
    ok0 = true;
  }
  return ok0;
}

static Boolean geoVhAppendFinalStraightIfNeeded(
    VhSegWorkspace_t *ws, Real endX, Real endY, Real endTheta, Real relX) {
  Integer goDir0;
  Boolean ok0;

  ok0 = true;
  if (fabsf(relX) > 0.2f) {
    goDir0 = 1;
    if (relX < 0.0f) {
      goDir0 = -1;
    }
    ok0 = geoVhAppendLineToTail(ws, endX, endY, endTheta, goDir0);
  }
  return ok0;
}

static Boolean geoVhApplyCircleLineIntrvResults(VhSegWorkspace_t *ws,
                                                const GeoCircle1pt1lineOutput_t *circleOutput,
                                                Real endX,
                                                Real endY,
                                                Real endTheta,
                                                Integer dirS,
                                                Real radius,
                                                const GeoCircleIntrvCheckOutput_t *circleIntrv) {
  Boolean appendOk0;
  Boolean ok0;

  appendOk0 = false;
  ok0 = false;
  if ((circleOutput == (const GeoCircle1pt1lineOutput_t *)0) ||
      (circleIntrv == (const GeoCircleIntrvCheckOutput_t *)0) || (radius <= GEO_EPSILON)) {
    ws->PlanCmd = 5;
    if (ws->FailRs == 0) {
      ws->FailRs = 7;
    }
  } else if (circleIntrv->IntrvFlg == 2) {
    ws->PlanCmd = 5;
    ws->FailRs = 8;
  } else if (circleIntrv->IntrvFlg == 0) {
    appendOk0 = geoVhAppendArcToTail(ws,
                                     circleOutput->LnPtX,
                                     circleOutput->LnPtY,
                                     circleOutput->LnPtTheta,
                                     -dirS,
                                     circleOutput->TrnDir,
                                     radius,
                                     circleIntrv->SafeLen,
                                     circleOutput->CptX,
                                     circleOutput->CptY);
    if (appendOk0) {
      appendOk0 = geoVhAppendFinalStraightIfNeeded(ws, endX, endY, endTheta, circleOutput->LnPtRX);
    }
    if (appendOk0) {
      ws->PlanCmd = 1;
      ok0 = true;
    }
  } else {
    appendOk0 = geoVhAppendArcToTail(ws,
                                     circleIntrv->AvlPosX,
                                     circleIntrv->AvlPosY,
                                     circleIntrv->AvlPosTheta,
                                     -dirS,
                                     circleOutput->TrnDir,
                                     radius,
                                     circleIntrv->SafeLen,
                                     circleOutput->CptX,
                                     circleOutput->CptY);
    if (appendOk0) {
      ws->PlanCmd = 2;
      ok0 = true;
    }
  }

  if ((!appendOk0) && (ws->PlanCmd != 5)) {
    ws->PlanCmd = 5;
    if (ws->FailRs == 0) {
      ws->FailRs = 7;
    }
  }
  return ok0;
}

static Boolean geoVhTryCircleLineSubType1(VhSegWorkspace_t *ws,
                                          const GeoVhPathPlanningInput_t *input,
                                          const GeoVhPathPlanningParam_t *param,
                                          Real startX,
                                          Real startY,
                                          Real startTheta,
                                          Real endX,
                                          Real endY,
                                          Real endTheta,
                                          Integer dirS,
                                          const GeoCircle1pt1lineOutput_t *circleOutput) {
  GeoCircleIntrvCheckOutput_t circleIntrv;
  Real radius0;
  Boolean ok0;

  radius0 = fabsf(circleOutput->Radius);
  circleIntrv = geoVhRunCircleIntrvCheck(input,
                                         param,
                                         circleOutput->CptX,
                                         circleOutput->CptY,
                                         radius0,
                                         startX,
                                         startY,
                                         startTheta,
                                         circleOutput->LnPtX,
                                         circleOutput->LnPtY,
                                         circleOutput->LnPtTheta,
                                         dirS,
                                         circleOutput->TrnDir,
                                         0.1f);
  ws->PlnChkCnt = ws->PlnChkCnt + circleIntrv.CheckCount;
  ok0 = geoVhApplyCircleLineIntrvResults(
      ws, circleOutput, endX, endY, endTheta, dirS, radius0, &circleIntrv);
  return ok0;
}

static Boolean geoVhApplyTwoCircleIntrvResults(VhSegWorkspace_t *ws,
                                               const Geo2CircleLinkOutput_t *linkOutput,
                                               Real endX,
                                               Real endY,
                                               Real endTheta,
                                               Integer dirS,
                                               Real radius,
                                               const GeoCircleIntrvCheckOutput_t *firstIntrv,
                                               const GeoCircleIntrvCheckOutput_t *secondIntrv) {
  Boolean appendOk0;
  Boolean ok0;

  appendOk0 = false;
  ok0 = false;
  if ((linkOutput == (const Geo2CircleLinkOutput_t *)0) || (linkOutput->PlnFlg != 1)) {
    ws->PlanCmd = 5;
    if (ws->FailRs == 0) {
      ws->FailRs = 7;
    }
  } else if (firstIntrv->IntrvFlg == 2) {
    ws->PlanCmd = 5;
    ws->FailRs = 8;
  } else if (firstIntrv->IntrvFlg == 0) {
    appendOk0 = geoVhAppendArcToTail(ws,
                                     linkOutput->St2X,
                                     linkOutput->St2Y,
                                     linkOutput->St2Theta,
                                     -dirS,
                                     linkOutput->TrnD1,
                                     radius,
                                     firstIntrv->SafeLen,
                                     linkOutput->Ct1X,
                                     linkOutput->Ct1Y);
    if (appendOk0) {
      if (secondIntrv->IntrvFlg == 2) {
        ws->PlanCmd = 5;
        ws->FailRs = 8;
        appendOk0 = false;
      } else if (secondIntrv->IntrvFlg == 0) {
        appendOk0 = geoVhAppendArcToTail(ws,
                                         linkOutput->St3X,
                                         linkOutput->St3Y,
                                         linkOutput->St3Theta,
                                         -dirS,
                                         linkOutput->TrnD2,
                                         radius,
                                         secondIntrv->SafeLen,
                                         linkOutput->Ct2X,
                                         linkOutput->Ct2Y);
        if (appendOk0) {
          appendOk0 = geoVhAppendFinalStraightIfNeeded(ws, endX, endY, endTheta, linkOutput->St3rX);
        }
        if (appendOk0) {
          ws->PlanCmd = 1;
          ok0 = true;
        }
      } else {
        appendOk0 = geoVhAppendArcToTail(ws,
                                         secondIntrv->AvlPosX,
                                         secondIntrv->AvlPosY,
                                         secondIntrv->AvlPosTheta,
                                         -dirS,
                                         linkOutput->TrnD2,
                                         radius,
                                         secondIntrv->SafeLen,
                                         linkOutput->Ct2X,
                                         linkOutput->Ct2Y);
        if (appendOk0) {
          ws->PlanCmd = 2;
          ok0 = true;
        }
      }
    }
  } else {
    appendOk0 = geoVhAppendArcToTail(ws,
                                     firstIntrv->AvlPosX,
                                     firstIntrv->AvlPosY,
                                     firstIntrv->AvlPosTheta,
                                     -dirS,
                                     linkOutput->TrnD1,
                                     radius,
                                     firstIntrv->SafeLen,
                                     linkOutput->Ct1X,
                                     linkOutput->Ct1Y);
    if (appendOk0) {
      ws->PlanCmd = 2;
      ok0 = true;
    }
  }

  if ((!appendOk0) && (ws->PlanCmd != 5)) {
    ws->PlanCmd = 5;
    if (ws->FailRs == 0) {
      ws->FailRs = 7;
    }
  }
  return ok0;
}

static Boolean geoVhTryTwoCircleBranch(VhSegWorkspace_t *ws,
                                       const GeoVhPathPlanningInput_t *input,
                                       const GeoVhPathPlanningParam_t *param,
                                       Real startX,
                                       Real startY,
                                       Real startTheta,
                                       Real endX,
                                       Real endY,
                                       Real endTheta,
                                       Integer dirS,
                                       Real radius) {
  Geo2CircleLinkInput_t linkInput;
  Geo2CircleLinkOutput_t linkOutput;
  GeoCircleIntrvCheckOutput_t firstIntrv;
  GeoCircleIntrvCheckOutput_t secondIntrv;
  Boolean ok0;

  linkInput = (Geo2CircleLinkInput_t){0};
  linkInput.Pt1X = startX;
  linkInput.Pt1Y = startY;
  linkInput.Pt1Theta = startTheta;
  linkInput.Pt2X = endX;
  linkInput.Pt2Y = endY;
  linkInput.Pt2Theta = endTheta;
  linkInput.Dir = dirS;
  linkInput.R1 = radius;
  linkInput.R2 = radius;
  linkOutput = (Geo2CircleLinkOutput_t){0};
  geo2CircleLink(&linkInput, &linkOutput);

  firstIntrv = (GeoCircleIntrvCheckOutput_t){0};
  secondIntrv = (GeoCircleIntrvCheckOutput_t){0};
  if (linkOutput.PlnFlg == 1) {
    firstIntrv = geoVhRunCircleIntrvCheck(input,
                                          param,
                                          linkOutput.Ct1X,
                                          linkOutput.Ct1Y,
                                          radius,
                                          startX,
                                          startY,
                                          startTheta,
                                          linkOutput.St2X,
                                          linkOutput.St2Y,
                                          linkOutput.St2Theta,
                                          dirS,
                                          linkOutput.TrnD1,
                                          0.1f);
    ws->PlnChkCnt = ws->PlnChkCnt + firstIntrv.CheckCount;
    if (firstIntrv.IntrvFlg == 0) {
      secondIntrv = geoVhRunCircleIntrvCheck(input,
                                             param,
                                             linkOutput.Ct2X,
                                             linkOutput.Ct2Y,
                                             radius,
                                             linkOutput.St2X,
                                             linkOutput.St2Y,
                                             linkOutput.St2Theta,
                                             linkOutput.St3X,
                                             linkOutput.St3Y,
                                             linkOutput.St3Theta,
                                             dirS,
                                             linkOutput.TrnD2,
                                             0.1f);
      ws->PlnChkCnt = ws->PlnChkCnt + secondIntrv.CheckCount;
    }
  }

  ok0 = geoVhApplyTwoCircleIntrvResults(
      ws, &linkOutput, endX, endY, endTheta, dirS, radius, &firstIntrv, &secondIntrv);
  return ok0;
}

static Boolean geoVhApplyRev2CircleIntrvResults(VhSegWorkspace_t *ws,
                                                const GeoRev2CircleLinkOutput_t *linkOutput,
                                                Real endX,
                                                Real endY,
                                                Real endTheta,
                                                Integer dirS,
                                                Real radius,
                                                const GeoCircleIntrvCheckOutput_t *firstIntrv,
                                                const GeoCircleIntrvCheckOutput_t *secondIntrv) {
  Boolean appendOk0;
  Boolean ok0;

  appendOk0 = false;
  ok0 = false;
  if ((linkOutput == (const GeoRev2CircleLinkOutput_t *)0) || (linkOutput->GeoOk != 1)) {
    ws->PlanCmd = 5;
    if (ws->FailRs == 0) {
      ws->FailRs = 7;
    }
  } else if (firstIntrv->IntrvFlg == 2) {
    ws->PlanCmd = 5;
    ws->FailRs = 8;
  } else if (firstIntrv->IntrvFlg == 0) {
    appendOk0 = geoVhAppendArcToTail(ws,
                                     linkOutput->St2X,
                                     linkOutput->St2Y,
                                     linkOutput->St2Theta,
                                     -dirS,
                                     linkOutput->TrnD1,
                                     radius,
                                     firstIntrv->SafeLen,
                                     linkOutput->Ct1X,
                                     linkOutput->Ct1Y);
    if (appendOk0) {
      if (secondIntrv->IntrvFlg == 2) {
        ws->PlanCmd = 5;
        ws->FailRs = 8;
        appendOk0 = false;
      } else if (secondIntrv->IntrvFlg == 0) {
        appendOk0 = geoVhAppendArcToTail(ws,
                                         linkOutput->St3X,
                                         linkOutput->St3Y,
                                         linkOutput->St3Theta,
                                         dirS,
                                         linkOutput->TrnD2,
                                         radius,
                                         secondIntrv->SafeLen,
                                         linkOutput->Ct2X,
                                         linkOutput->Ct2Y);
        if (appendOk0) {
          appendOk0 = geoVhAppendFinalStraightIfNeeded(ws, endX, endY, endTheta, linkOutput->St3rX);
        }
        if (appendOk0) {
          ws->PlanCmd = 1;
          ok0 = true;
        }
      } else {
        appendOk0 = geoVhAppendArcToTail(ws,
                                         secondIntrv->AvlPosX,
                                         secondIntrv->AvlPosY,
                                         secondIntrv->AvlPosTheta,
                                         dirS,
                                         linkOutput->TrnD2,
                                         radius,
                                         secondIntrv->SafeLen,
                                         linkOutput->Ct2X,
                                         linkOutput->Ct2Y);
        if (appendOk0) {
          ws->PlanCmd = 2;
          ok0 = true;
        }
      }
    }
  } else {
    appendOk0 = geoVhAppendArcToTail(ws,
                                     firstIntrv->AvlPosX,
                                     firstIntrv->AvlPosY,
                                     firstIntrv->AvlPosTheta,
                                     -dirS,
                                     linkOutput->TrnD1,
                                     radius,
                                     firstIntrv->SafeLen,
                                     linkOutput->Ct1X,
                                     linkOutput->Ct1Y);
    if (appendOk0) {
      ws->PlanCmd = 2;
      ok0 = true;
    }
  }

  if ((!appendOk0) && (ws->PlanCmd != 5)) {
    ws->PlanCmd = 5;
    if (ws->FailRs == 0) {
      ws->FailRs = 7;
    }
  }
  return ok0;
}

static Boolean geoVhTryRev2CircleBranch(VhSegWorkspace_t *ws,
                                        const GeoVhPathPlanningInput_t *input,
                                        const GeoVhPathPlanningParam_t *param,
                                        Real startX,
                                        Real startY,
                                        Real startTheta,
                                        Real endX,
                                        Real endY,
                                        Real endTheta,
                                        Integer dirS,
                                        Real radius) {
  GeoRev2CircleLinkInput_t linkInput;
  GeoRev2CircleLinkOutput_t linkOutput;
  GeoCircleIntrvCheckOutput_t firstIntrv;
  GeoCircleIntrvCheckOutput_t secondIntrv;
  Boolean ok0;

  linkInput = (GeoRev2CircleLinkInput_t){0};
  linkInput.Pt1X = startX;
  linkInput.Pt1Y = startY;
  linkInput.Pt1Theta = startTheta;
  linkInput.Pt2X = endX;
  linkInput.Pt2Y = endY;
  linkInput.Pt2Theta = endTheta;
  linkInput.Dir = dirS;
  linkInput.R1 = radius;
  linkInput.R2 = radius;
  linkOutput = (GeoRev2CircleLinkOutput_t){0};
  geoRev2CircleLink(&linkInput, &linkOutput);

  firstIntrv = (GeoCircleIntrvCheckOutput_t){0};
  secondIntrv = (GeoCircleIntrvCheckOutput_t){0};
  if (linkOutput.GeoOk == 1) {
    firstIntrv = geoVhRunCircleIntrvCheck(input,
                                          param,
                                          linkOutput.Ct1X,
                                          linkOutput.Ct1Y,
                                          radius,
                                          startX,
                                          startY,
                                          startTheta,
                                          linkOutput.St2X,
                                          linkOutput.St2Y,
                                          linkOutput.St2Theta,
                                          dirS,
                                          linkOutput.TrnD1,
                                          0.1f);
    ws->PlnChkCnt = ws->PlnChkCnt + firstIntrv.CheckCount;
    if (firstIntrv.IntrvFlg == 0) {
      secondIntrv = geoVhRunCircleIntrvCheck(input,
                                             param,
                                             linkOutput.Ct2X,
                                             linkOutput.Ct2Y,
                                             radius,
                                             linkOutput.St2X,
                                             linkOutput.St2Y,
                                             linkOutput.St2Theta,
                                             linkOutput.St3X,
                                             linkOutput.St3Y,
                                             linkOutput.St3Theta,
                                             -dirS,
                                             linkOutput.TrnD2,
                                             0.1f);
      ws->PlnChkCnt = ws->PlnChkCnt + secondIntrv.CheckCount;
    }
  }

  ok0 = geoVhApplyRev2CircleIntrvResults(
      ws, &linkOutput, endX, endY, endTheta, dirS, radius, &firstIntrv, &secondIntrv);
  return ok0;
}

static Integer geoVhResolveSecondGroupSubType(const GeoCircle1pt1lineOutput_t *circleOutput,
                                              Real startRelY,
                                              Real startRelTheta) {
  Boolean longArc0;
  Boolean postureSmall0;
  Integer subType0;

  longArc0 = false;
  postureSmall0 = false;
  subType0 = GEO_VH_SECOND_GROUP_SUBTYPE_NONE;
  if (circleOutput != (const GeoCircle1pt1lineOutput_t *)0) {
    if (circleOutput->ArcLen > GEO_VH_CIR_LEN_MAX) {
      longArc0 = true;
    }
    if (fabsf(circleOutput->LnPtRX) > GEO_VH_CIR_EDX_MAX) {
      longArc0 = true;
    }
    if ((fabsf(startRelY) < GEO_VH_CIR_EDY_MAX) && (fabsf(startRelTheta) < GEO_VH_CIR_THETA_MAX)) {
      postureSmall0 = true;
    }
    if (longArc0) {
      if (postureSmall0) {
        subType0 = GEO_VH_SECOND_GROUP_SUBTYPE_2;
      } else {
        subType0 = GEO_VH_SECOND_GROUP_SUBTYPE_3;
      }
    } else {
      subType0 = GEO_VH_SECOND_GROUP_SUBTYPE_1;
    }
  }
  return subType0;
}

static Boolean geoVhTryCircle2LineSubType3(VhSegWorkspace_t *ws,
                                           const GeoVhPathPlanningInput_t *input,
                                           const GeoVhPathPlanningParam_t *param,
                                           Real startX,
                                           Real startY,
                                           Real startTheta,
                                           Real endX,
                                           Real endY,
                                           Real endTheta,
                                           Integer pathType,
                                           Integer dirS,
                                           Real radius) {
  GeoCircle2lineRInput_t circleInput;
  GeoCircle2lineROutput_t circleOutput;
  GeoLineIntrvCheckOutput_t lineIntrv;
  GeoCircleIntrvCheckOutput_t circleIntrv;
  Real lineRelX0;
  Real lineRelY0;
  Real lineRelTheta0;
  Integer lineGoDir0;
  Boolean appendOk0;
  Boolean ok0;

  circleInput = (GeoCircle2lineRInput_t){0};
  circleInput.VstX = startX;
  circleInput.VstY = startY;
  circleInput.VstTheta = startTheta;
  circleInput.VtgtX = endX;
  circleInput.VtgtY = endY;
  circleInput.VtgtTheta = endTheta;
  circleInput.Ptyp = pathType;
  circleInput.Dir = dirS;
  circleInput.Radius = radius;
  circleOutput = (GeoCircle2lineROutput_t){0};
  geoCircle2lineR(&circleInput, &circleOutput);

  lineIntrv = geoVhRunLineIntrvCheck(input,
                                     param,
                                     startX,
                                     startY,
                                     startTheta,
                                     circleOutput.StPtX,
                                     circleOutput.StPtY,
                                     circleOutput.StPtTheta,
                                     2.0f);
  ws->PlnChkCnt = ws->PlnChkCnt + lineIntrv.CheckCount;
  ok0 = false;
  appendOk0 = false;
  if (lineIntrv.IntrvFlg == 2) {
    ws->PlanCmd = 5;
    ws->FailRs = 8;
  } else if (lineIntrv.IntrvFlg == 1) {
    ws->PlanCmd = 5;
    if (ws->FailRs == 0) {
      ws->FailRs = 14;
    }
  } else {
    geoAxisTransToTarget(lineIntrv.AvlPosX,
                         lineIntrv.AvlPosY,
                         lineIntrv.AvlPosTheta,
                         startX,
                         startY,
                         startTheta,
                         &lineRelX0,
                         &lineRelY0,
                         &lineRelTheta0);
    (void)lineRelY0;
    (void)lineRelTheta0;
    lineGoDir0 = 1;
    if (lineRelX0 > 0.0f) {
      lineGoDir0 = -1;
    }
    appendOk0 = geoVhAppendLineToTail(
        ws, circleOutput.StPtX, circleOutput.StPtY, circleOutput.StPtTheta, lineGoDir0);
    if (appendOk0) {
      circleIntrv = geoVhRunCircleIntrvCheck(input,
                                             param,
                                             circleOutput.CptX,
                                             circleOutput.CptY,
                                             radius,
                                             circleOutput.StPtX,
                                             circleOutput.StPtY,
                                             circleOutput.StPtTheta,
                                             circleOutput.EdPtX,
                                             circleOutput.EdPtY,
                                             circleOutput.EdPtTheta,
                                             dirS,
                                             circleOutput.TrnDir,
                                             0.1f);
      ws->PlnChkCnt = ws->PlnChkCnt + circleIntrv.CheckCount;
      if (circleIntrv.IntrvFlg == 2) {
        ws->PlanCmd = 5;
        ws->FailRs = 8;
        appendOk0 = false;
      } else if (circleIntrv.IntrvFlg == 0) {
        appendOk0 = geoVhAppendArcToTail(ws,
                                         circleOutput.EdPtX,
                                         circleOutput.EdPtY,
                                         circleOutput.EdPtTheta,
                                         -circleOutput.GoDir,
                                         circleOutput.TrnDir,
                                         radius,
                                         circleIntrv.SafeLen,
                                         circleOutput.CptX,
                                         circleOutput.CptY);
        if (appendOk0) {
          appendOk0 =
              geoVhAppendFinalStraightIfNeeded(ws, endX, endY, endTheta, circleOutput.EdPtrX);
        }
        if (appendOk0) {
          ws->PlanCmd = 1;
          ok0 = true;
        }
      } else {
        appendOk0 = geoVhAppendArcToTail(ws,
                                         circleIntrv.AvlPosX,
                                         circleIntrv.AvlPosY,
                                         circleIntrv.AvlPosTheta,
                                         -dirS,
                                         circleOutput.TrnDir,
                                         radius,
                                         circleIntrv.SafeLen,
                                         circleOutput.CptX,
                                         circleOutput.CptY);
        if (appendOk0) {
          ws->PlanCmd = 2;
          ok0 = true;
        }
      }
    }
  }

  if ((!appendOk0) && (ws->PlanCmd != 5)) {
    ws->PlanCmd = 5;
    if (ws->FailRs == 0) {
      ws->FailRs = 7;
    }
  }
  return ok0;
}

static Boolean geoVhTryFirstGroupBranch(VhSegWorkspace_t *ws,
                                        const GeoVhPathPlanningInput_t *input,
                                        const GeoVhPathPlanningParam_t *param,
                                        Real startX,
                                        Real startY,
                                        Real startTheta,
                                        Real endX,
                                        Real endY,
                                        Real endTheta,
                                        Integer pathType,
                                        Integer dirS,
                                        Real radiusMin) {
  GeoCircle1pt1lineInput_t circleInput;
  GeoCircle1pt1lineOutput_t circleOutput;
  Real radius0;
  Real branchRadius0;
  Boolean ok0;

  (void)pathType;
  circleInput = (GeoCircle1pt1lineInput_t){0};
  circleInput.VstX = startX;
  circleInput.VstY = startY;
  circleInput.VstTheta = startTheta;
  circleInput.VtgtX = endX;
  circleInput.VtgtY = endY;
  circleInput.VtgtTheta = endTheta;
  circleInput.Dir = -dirS;
  circleOutput = (GeoCircle1pt1lineOutput_t){0};
  geoCircle1pt1line(&circleInput, &circleOutput);

  radius0 = fabsf(circleOutput.Radius);
  branchRadius0 = radiusMin;
  if (radius0 >= radiusMin) {
    ok0 = geoVhTryTwoCircleBranch(
        ws, input, param, startX, startY, startTheta, endX, endY, endTheta, dirS, branchRadius0);
  } else {
    branchRadius0 = radiusMin + 0.1f;
    ok0 = geoVhTryRev2CircleBranch(
        ws, input, param, startX, startY, startTheta, endX, endY, endTheta, dirS, branchRadius0);
  }
  return ok0;
}

static Boolean geoVhTrySecondGroupBranch(VhSegWorkspace_t *ws,
                                         const GeoVhPathPlanningInput_t *input,
                                         const GeoVhPathPlanningParam_t *param,
                                         Real startX,
                                         Real startY,
                                         Real startTheta,
                                         Real endX,
                                         Real endY,
                                         Real endTheta,
                                         Integer pathType,
                                         Integer dirS,
                                         Real radiusMin,
                                         const GeoPosTypClassPkInOutput_t *classOutput) {
  GeoCircle1pt1lineInput_t circleInput;
  GeoCircle1pt1lineOutput_t circleOutput;
  Real radius0;
  Integer subType0;
  Boolean ok0;

  circleInput = (GeoCircle1pt1lineInput_t){0};
  circleInput.VstX = startX;
  circleInput.VstY = startY;
  circleInput.VstTheta = startTheta;
  circleInput.VtgtX = endX;
  circleInput.VtgtY = endY;
  circleInput.VtgtTheta = endTheta;
  circleInput.Dir = dirS;
  circleOutput = (GeoCircle1pt1lineOutput_t){0};
  geoCircle1pt1line(&circleInput, &circleOutput);

  radius0 = fabsf(circleOutput.Radius);
  if (radius0 > radiusMin) {
    subType0 = geoVhResolveSecondGroupSubType(
        &circleOutput, classOutput->PtRelaY, classOutput->PtRelaTheta);
    if (subType0 == GEO_VH_SECOND_GROUP_SUBTYPE_1) {
      ok0 = geoVhTryCircleLineSubType1(
          ws, input, param, startX, startY, startTheta, endX, endY, endTheta, dirS, &circleOutput);
    } else if (subType0 == GEO_VH_SECOND_GROUP_SUBTYPE_2) {
      ok0 = geoVhTryTwoCircleBranch(
          ws, input, param, startX, startY, startTheta, endX, endY, endTheta, dirS, radiusMin);
    } else if (subType0 == GEO_VH_SECOND_GROUP_SUBTYPE_3) {
      ok0 = geoVhTryCircle2LineSubType3(ws,
                                        input,
                                        param,
                                        startX,
                                        startY,
                                        startTheta,
                                        endX,
                                        endY,
                                        endTheta,
                                        pathType,
                                        dirS,
                                        radiusMin);
    } else {
      ws->PlanCmd = 5;
      if (ws->FailRs == 0) {
        ws->FailRs = 7;
      }
      ok0 = false;
    }
  } else {
    ok0 = geoVhTryRev2CircleBranch(
        ws, input, param, startX, startY, startTheta, endX, endY, endTheta, dirS, radiusMin + 0.2f);
  }
  return ok0;
}

static void geoVhHybridBuildSlot(const GeoVhPathPlanningInput_t *input, ApaSlot_t *slot) {
  Real slotPts[8];
  Integer parkingType;

  slotPts[0] = input->SltPt1X;
  slotPts[1] = input->SltPt1Y;
  slotPts[2] = input->SltPt2X;
  slotPts[3] = input->SltPt2Y;
  slotPts[4] = input->SltPt3X;
  slotPts[5] = input->SltPt3Y;
  slotPts[6] = input->SltPt4X;
  slotPts[7] = input->SltPt4Y;
  parkingType = geoVhResolveParkingType(input);
  apaPlannerHybridInitSlot(slot,
                           input->TarX,
                           input->TarY,
                           input->TarTheta,
                           input->SltEntr,
                           slotPts,
                           parkingType,
                           APA_SURROUND_UNKNOWN,
                           APA_PARKING_DIREC_INVALID);
}

static void geoVhHybridInitCollisionContext(const GeoVhPathPlanningInput_t *input,
                                            const ApaSlot_t *slot,
                                            ApaPlannerGlobalState_t *plannerGlobal,
                                            ApaPlannerHybridCollisionContext_t *context) {
  Integer idx;

  apaPlannerHybridCollisionContextInit(context, plannerGlobal, slot);
  idx = 0;
  while ((idx < input->BoundaryLineNum) && (idx < GEO_BOUNDARY_LINE_MAX)) {
    (void)apaPlannerHybridCollisionAddBoundaryLine(context,
                                                   input->BoundaryLineX1[idx],
                                                   input->BoundaryLineY1[idx],
                                                   input->BoundaryLineX2[idx],
                                                   input->BoundaryLineY2[idx]);
    idx = idx + 1;
  }
  idx = 0;
  while ((idx < input->VirtualBoundaryLineNum) && (idx < GEO_BOUNDARY_LINE_MAX)) {
    (void)apaPlannerHybridCollisionAddVirtualBoundaryLine(context,
                                                          input->VirtualBoundaryP1X[idx],
                                                          input->VirtualBoundaryP1Y[idx],
                                                          input->VirtualBoundaryP2X[idx],
                                                          input->VirtualBoundaryP2Y[idx]);
    idx = idx + 1;
  }
  idx = 0;
  while ((idx < input->BoundarySepPtNum) && (idx < GEO_BOUNDARY_POINT_MAX)) {
    (void)apaPlannerHybridCollisionAddBoundaryPoint(
        context, input->BoundarySepPtX[idx], input->BoundarySepPtY[idx]);
    idx = idx + 1;
  }
  idx = 0;
  while ((idx < input->StopperPtNum) && (idx < GEO_BOUNDARY_POINT_MAX)) {
    (void)apaPlannerHybridCollisionAddStopper(
        context, input->StopperPtX[idx], input->StopperPtY[idx]);
    idx = idx + 1;
  }
  idx = 0;
  while ((idx < input->SlotTrackStopperNum) && (idx < VCC_MAX_TRACK_STOPPER)) {
    (void)apaPlannerHybridCollisionAddSlotTrackStopper(context,
                                                       input->SlotTrackStopperX1[idx],
                                                       input->SlotTrackStopperY1[idx],
                                                       input->SlotTrackStopperX2[idx],
                                                       input->SlotTrackStopperY2[idx]);
    idx = idx + 1;
  }
}

static Boolean geoVhHasValidCandidate(const GeoVhPathPlanningOutput_t *candidate) {
  return (candidate->PlanFlg == 1) && (candidate->SegNum > 0) &&
         (candidate->PathLen > APA_PLANNER_HYBRID_EPS);
}

static Integer geoVhResolveStepCount(const GeoVhPathPlanningOutput_t *candidate) {
  Integer idx;
  Integer stpN0;
  Integer prevDir;

  stpN0 = 0;
  if (candidate->SegNum > 0) {
    prevDir = candidate->SegGoDir[0];
    if (prevDir < 0) {
      stpN0 = 1;
    }
    idx = 1;
    while (idx < candidate->SegNum) {
      if (candidate->SegGoDir[idx] * prevDir < 0) {
        stpN0 = stpN0 + 1;
        prevDir = candidate->SegGoDir[idx];
      }
      idx = idx + 1;
    }
  }
  return stpN0;
}

static Boolean geoVhCandidateIsHybridFallback(const GeoVhPathPlanningOutput_t *candidate) {
  return candidate->PlannerCase == GEO_VH_PLANNER_CASE_HYBRID_FALLBACK;
}

static Boolean
geoVhHybridHasSignificantPathAdvantage(const GeoVhPathPlanningOutput_t *hybridCandidate,
                                       const GeoVhPathPlanningOutput_t *otherCandidate) {
  Integer stepDelta;
  Real pathGain;
  Real pathGainRatio;

  stepDelta = geoVhResolveStepCount(hybridCandidate) - geoVhResolveStepCount(otherCandidate);
  pathGain = otherCandidate->PathLen - hybridCandidate->PathLen;
  pathGainRatio = 0.0f;
  if (otherCandidate->PathLen > APA_PLANNER_HYBRID_EPS) {
    pathGainRatio = pathGain / otherCandidate->PathLen;
  }
  return (stepDelta > 0) && (stepDelta <= GEO_VH_HYBRID_OVERRIDE_STEP_DELTA_MAX) &&
         (pathGain > GEO_VH_HYBRID_OVERRIDE_PATH_GAIN_MIN) &&
         (pathGainRatio > GEO_VH_HYBRID_OVERRIDE_PATH_RATIO_MIN);
}

static Integer
geoVhResolveCandidateDecisionReason(const GeoVhPathPlanningOutput_t *currentCandidate,
                                    const GeoVhPathPlanningOutput_t *nextCandidate,
                                    Boolean *replaceCandidate) {
  Boolean currentValid;
  Boolean nextValid;
  Boolean currentHybrid;
  Boolean nextHybrid;
  Boolean shorterStepCount;
  Boolean longerStepCount;
  Boolean shorterPath;
  Boolean longerPath;
  Boolean fewerSegments;
  Boolean moreSegments;
  Boolean fewerChecks;
  Boolean moreChecks;
  Boolean preferHybridPathQuality;
  Integer currentStpN;
  Integer nextStpN;
  Integer decisionReason0;

  currentValid = geoVhHasValidCandidate(currentCandidate);
  nextValid = geoVhHasValidCandidate(nextCandidate);
  *replaceCandidate = false;
  decisionReason0 = GEO_VH_FALLBACK_DECISION_NONE;

  if (!nextValid) {
    decisionReason0 = GEO_VH_FALLBACK_DECISION_NONE;
  } else if (!currentValid) {
    *replaceCandidate = true;
    decisionReason0 = GEO_VH_FALLBACK_DECISION_VALID_ONLY;
  } else {
    currentHybrid = geoVhCandidateIsHybridFallback(currentCandidate);
    nextHybrid = geoVhCandidateIsHybridFallback(nextCandidate);
    currentStpN = geoVhResolveStepCount(currentCandidate);
    nextStpN = geoVhResolveStepCount(nextCandidate);
    shorterStepCount = (nextStpN < currentStpN);
    longerStepCount = (nextStpN > currentStpN);
    shorterPath =
        ((nextCandidate->PathLen + GEO_VH_CANDIDATE_PATH_EPS) < currentCandidate->PathLen);
    longerPath = ((currentCandidate->PathLen + GEO_VH_CANDIDATE_PATH_EPS) < nextCandidate->PathLen);
    fewerSegments = (nextCandidate->SegNum < currentCandidate->SegNum);
    moreSegments = (nextCandidate->SegNum > currentCandidate->SegNum);
    fewerChecks = (nextCandidate->PlnChkCnt < currentCandidate->PlnChkCnt);
    moreChecks = (nextCandidate->PlnChkCnt > currentCandidate->PlnChkCnt);
    preferHybridPathQuality = false;

    if ((!currentHybrid) && nextHybrid) {
      preferHybridPathQuality =
          geoVhHybridHasSignificantPathAdvantage(nextCandidate, currentCandidate);
    } else if (currentHybrid && (!nextHybrid)) {
      if (geoVhHybridHasSignificantPathAdvantage(currentCandidate, nextCandidate)) {
        decisionReason0 = GEO_VH_FALLBACK_DECISION_PATH_LEN;
      }
    }

    if (preferHybridPathQuality) {
      *replaceCandidate = true;
      decisionReason0 = GEO_VH_FALLBACK_DECISION_PATH_LEN;
    } else if (decisionReason0 == GEO_VH_FALLBACK_DECISION_PATH_LEN) {
      decisionReason0 = GEO_VH_FALLBACK_DECISION_PATH_LEN;
    } else if (shorterStepCount) {
      *replaceCandidate = true;
      decisionReason0 = GEO_VH_FALLBACK_DECISION_STEP_COUNT;
    } else if (longerStepCount) {
      decisionReason0 = GEO_VH_FALLBACK_DECISION_STEP_COUNT;
    } else if (shorterPath) {
      *replaceCandidate = true;
      decisionReason0 = GEO_VH_FALLBACK_DECISION_PATH_LEN;
    } else if (longerPath) {
      decisionReason0 = GEO_VH_FALLBACK_DECISION_PATH_LEN;
    } else if (fewerSegments) {
      *replaceCandidate = true;
      decisionReason0 = GEO_VH_FALLBACK_DECISION_SEG_NUM;
    } else if (moreSegments) {
      decisionReason0 = GEO_VH_FALLBACK_DECISION_SEG_NUM;
    } else if (fewerChecks) {
      *replaceCandidate = true;
      decisionReason0 = GEO_VH_FALLBACK_DECISION_PLN_CHK_CNT;
    } else if (moreChecks) {
      decisionReason0 = GEO_VH_FALLBACK_DECISION_PLN_CHK_CNT;
    }
  }
  return decisionReason0;
}

static Boolean geoVhShouldReplaceCandidate(const GeoVhPathPlanningOutput_t *currentCandidate,
                                           const GeoVhPathPlanningOutput_t *nextCandidate,
                                           const GeoVhPathPlanningParam_t *param) {
  if (param->HybridPolicy.CompareCandidates && currentCandidate->PlanFlg &&
      nextCandidate->PlanFlg) {
    ApaPlannerHybridSegments_t first, second;
    if (apaPlannerHybridReadGeometry(
            APA_HYBRID_GEOMETRY_FIELDS(currentCandidate, VH_SEG_OUT_MAX), &first, 0) &&
        apaPlannerHybridReadGeometry(
            APA_HYBRID_GEOMETRY_FIELDS(nextCandidate, VH_SEG_OUT_MAX), &second, 0))
      return param->HybridPolicy.CompareCandidates(&first, &second, param->HybridPolicy.UserData) ==
             APA_PATH_COMP_NEW_PATH_BETTER;
  }

  Boolean replaceCandidate;

  replaceCandidate = false;
  (void)geoVhResolveCandidateDecisionReason(currentCandidate, nextCandidate, &replaceCandidate);
  return replaceCandidate;
}

static void geoVhStoreFallbackDiagFromCandidate(Boolean candidateOk,
                                                const GeoVhPathPlanningOutput_t *candidate,
                                                Integer *stpN,
                                                Real *pathLen,
                                                Integer *segNum,
                                                Integer *plnChkCnt) {
  if (candidateOk) {
    *stpN = geoVhResolveStepCount(candidate);
    *pathLen = candidate->PathLen;
    *segNum = candidate->SegNum;
    *plnChkCnt = candidate->PlnChkCnt;
  } else {
    *stpN = 0;
    *pathLen = 0.0f;
    *segNum = 0;
    *plnChkCnt = 0;
  }
}

static Integer geoVhResolveFallbackWinnerKind(Boolean geometricOk,
                                              Boolean hybridOk,
                                              const GeoVhPathPlanningOutput_t *bestCandidate) {
  Integer winnerKind0;

  if (!geoVhHasValidCandidate(bestCandidate)) {
    winnerKind0 = GEO_VH_FALLBACK_WINNER_NONE;
  } else if (hybridOk && (bestCandidate->PlannerCase == GEO_VH_PLANNER_CASE_HYBRID_FALLBACK)) {
    winnerKind0 = GEO_VH_FALLBACK_WINNER_HYBRID;
  } else if (geometricOk) {
    winnerKind0 = GEO_VH_FALLBACK_WINNER_GEOM;
  } else {
    winnerKind0 = GEO_VH_FALLBACK_WINNER_NONE;
  }
  return winnerKind0;
}

static Integer
geoVhResolveFallbackDecisionReason(Boolean geometricOk,
                                   Boolean hybridOk,
                                   const GeoVhPathPlanningOutput_t *geometricCandidate,
                                   const GeoVhPathPlanningOutput_t *hybridCandidate,
                                   const GeoVhPathPlanningParam_t *param) {
  if (geometricOk && hybridOk && param->HybridPolicy.CompareCandidates)
    return 6;

  Boolean replaceCandidate;
  Integer decisionReason0;

  if (!geometricOk && !hybridOk) {
    decisionReason0 = GEO_VH_FALLBACK_DECISION_NONE;
  } else {
    replaceCandidate = false;
    decisionReason0 =
        geoVhResolveCandidateDecisionReason(geometricCandidate, hybridCandidate, &replaceCandidate);
  }
  return decisionReason0;
}

static void geoVhHybridCopySegmentsToOutput(const ApaPlannerHybridSegments_t *segments,
                                            GeoVhPathPlanningOutput_t *output) {
  Integer idx;

  *output = (GeoVhPathPlanningOutput_t){0};
  output->PlanFlg = 1;
  output->SegNum = segments->SegNum;
  output->PathLen = segments->PathLen;
  output->PlnChkCnt = segments->PlnChkCnt;
  output->PlannerCase = GEO_VH_PLANNER_CASE_HYBRID_FALLBACK;
  idx = 0;
  while (idx < segments->SegNum) {
    output->SegGoDir[idx] = segments->PathGoDir[idx];
    output->SegTrnDir[idx] = segments->PathTrnDir[idx];
    output->SegCrv[idx] = segments->PathCrv[idx];
    output->SegTht[idx] = segments->PathTht[idx];
    output->SegLen[idx] = segments->PathLenArr[idx];
    output->SegStX[idx] = segments->PathStx[idx];
    output->SegStY[idx] = segments->PathSty[idx];
    output->SegStTh[idx] = segments->PathStTh[idx];
    output->SegCcX[idx] = segments->PathCC[idx * 2];
    output->SegCcY[idx] = segments->PathCC[idx * 2 + 1];
    idx = idx + 1;
  }
  output->StpN = geoVhResolveStepCount(output);
}

static Boolean
geoVhHybridCandidateTouchesTargetSlotFrame(const GeoVhPathPlanningInput_t *input,
                                           const GeoVhPathPlanningParam_t *param,
                                           const GeoVhPathPlanningOutput_t *candidate) {
  Integer segIdx;
  Boolean touchesFrame0;

  touchesFrame0 = false;
  segIdx = 0;
  while ((segIdx < candidate->SegNum) && (touchesFrame0 == false)) {
    Real segLen0;
    Integer sampleCount;
    Integer sampleIdx;

    segLen0 = fabsf(candidate->SegLen[segIdx]);
    sampleCount = (Integer)ceilf(segLen0 / GEO_VH_TARGET_FRAME_SAMPLE_STEP);
    if (sampleCount < 1) {
      sampleCount = 1;
    }
    sampleIdx = 0;
    while ((sampleIdx <= sampleCount) && (touchesFrame0 == false)) {
      Real travel0;
      Real poseX;
      Real poseY;
      Real poseTheta;

      travel0 = segLen0 * ((Real)sampleIdx) / ((Real)sampleCount);
      apaPlannerHybridEvalPrimitivePose(candidate->SegGoDir[segIdx],
                                        candidate->SegCrv[segIdx],
                                        travel0,
                                        candidate->SegStX[segIdx],
                                        candidate->SegStY[segIdx],
                                        candidate->SegStTh[segIdx],
                                        &poseX,
                                        &poseY,
                                        &poseTheta);
      if (geoVhPoseTouchesTargetSlotFrame(input, param, poseX, poseY, poseTheta)) {
        touchesFrame0 = true;
      }
      sampleIdx = sampleIdx + 1;
    }
    segIdx = segIdx + 1;
  }

  return touchesFrame0;
}

typedef struct {
  const GeoVhPathPlanningInput_t *Input;
  const GeoVhPathPlanningParam_t *Param;
} GeoVhHybridAcceptance_t;

static Boolean geoVhAcceptHybridSegments(const ApaPlannerHybridSegments_t *segments,
                                         void *userData) {
  const GeoVhHybridAcceptance_t *trial = userData;
  GeoVhPathPlanningOutput_t candidate;
  geoVhHybridCopySegmentsToOutput(segments, &candidate);
  if (geoVhHybridCandidateTouchesTargetSlotFrame(trial->Input, trial->Param, &candidate)) {
    apaPlanningRecordFailure(
        trial->Param->Diagnostics, APA_FAULT_COLLISION_RISK_AT_NEW_OBS, "vh_target_slot_frame", 4);
    return false;
  }
  const ApaPlannerHybridPolicy_t *policy = &trial->Param->HybridPolicy;
  return !policy->AcceptCandidate || policy->AcceptCandidate(segments, policy->UserData);
}

static Boolean geoVhHybridSearch(const GeoVhPathPlanningInput_t *input,
                                 const GeoVhPathPlanningParam_t *param,
                                 ApaPlannerHybridSegments_t *segments,
                                 ApaPath_t *path) {
  if (!apaPlannerHybridResolveFusion(param->NeedHybridAlgFusion))
    return false;
  ApaPlannerGlobalState_t global = APA_PLANNER_GLOBAL_STATE_INIT;
  ApaPlannerHybridCollisionContext_t collision = {0};
  ApaHybridAStar_t planner = {0};
  ApaSlot_t slot = {0};
  const Real start[3] = {input->VehX, input->VehY, input->VehTheta};
  const Real goal[3] = {input->TarX, input->TarY, input->TarTheta};
  apaPlannerHybridInitPlannerGlobal(&global, param->VehW, input->Rmin, param->NeedHybridAlgFusion);
  global.Planner.Veh.LF = param->VehLlF;
  if (param->VehLr > 0.0f)
    global.Planner.Veh.LB = param->VehLr;
  global.Planner.Veh.LL = global.Planner.Veh.LF + global.Planner.Veh.LB;
  global.Planner.Veh.Lf = global.Planner.Veh.LF - global.Planner.Veh.WB;
  geoVhHybridBuildSlot(input, &slot);
  geoVhHybridInitCollisionContext(input, &slot, &global, &collision);
  apaHybridAStarConfigure(&planner, &global, start, goal, &slot, true);
  GeoVhHybridAcceptance_t acceptance = {input, param};
  ApaPlannerHybridPolicy_t policy = {0};
  policy.AcceptCandidate = geoVhAcceptHybridSegments;
  policy.UserData = &acceptance;
  return apaPlannerHybridSearch(&planner,
                                &collision,
                                &param->SceneQuery,
                                &policy,
                                param->Diagnostics,
                                VH_SEG_OUT_MAX,
                                segments,
                                path);
}

Boolean geoVhPathPlanningHybridRawPath(const GeoVhPathPlanningInput_t *input,
                                       const GeoVhPathPlanningParam_t *param,
                                       ApaPath_t *path,
                                       Integer *plnChkCnt) {
  ApaPlannerHybridSegments_t segments = {0};
  if (plnChkCnt)
    *plnChkCnt = 0;
  if (!path)
    return false;
  apaPathInit(path);
  const Boolean accepted = geoVhHybridSearch(input, param, &segments, path);
  if (plnChkCnt)
    *plnChkCnt = segments.PlnChkCnt;
  return accepted;
}

static Boolean geoVhTryHybridFallback(const GeoVhPathPlanningInput_t *input,
                                      const GeoVhPathPlanningParam_t *param,
                                      GeoVhPathPlanningOutput_t *output) {
  ApaPlannerHybridSegments_t segments;
  if (!geoVhHybridSearch(input, param, &segments, 0))
    return false;
  geoVhHybridCopySegmentsToOutput(&segments, output);
  return true;
}

/**
 * @brief VH(垂直头入)路径规划器 — 编排层
 * @en_name geoVhPathPlanning
 * @cn_name VH路径规划器
 * @type widget_module
 * @config cn_name=车位状态, en_name=SltState, desc=APA车位状态输入, unit=1, type=Integer, range=[0,6], default=1
 * @config cn_name=车宽, en_name=VehW, desc=车辆宽度, unit=m, type=Real, range=[1.0,3.0], default=1.9
 * @config cn_name=前悬到参考点, en_name=VehLlF, desc=前悬到规划参考点长度, unit=m, type=Real, range=[0.1,3.0], default=0.9
 * @param[IN] Real VehX 车辆X
 * @param[IN] Real VehY 车辆Y
 * @param[IN] Real VehTheta 车辆航向
 * @param[IN] Real TarX 目标X
 * @param[IN] Real TarY 目标Y
 * @param[IN] Real TarTheta 目标航向
 * @param[IN] Real Rmin 最小转弯半径
 * @param[IN] Real EntrPtX 入口点X
 * @param[IN] Real EntrPtY 入口点Y
 * @param[IN] Real SltPt1X 车位角点1X
 * @param[IN] Real SltPt1Y 车位角点1Y
 * @param[IN] Real SltPt2X 车位角点2X
 * @param[IN] Real SltPt2Y 车位角点2Y
 * @param[IN] Real SltPt3X 车位角点3X
 * @param[IN] Real SltPt3Y 车位角点3Y
 * @param[IN] Real SltPt4X 车位角点4X
 * @param[IN] Real SltPt4Y 车位角点4Y
 * @param[IN] Integer SltEntr 车位入口边编号
 * @param[IN] Integer BoundaryLineNum 障碍边界线段数
 * @param[IN] Real[64] BoundaryLineX1 障碍边界线段起点X
 * @param[IN] Real[64] BoundaryLineY1 障碍边界线段起点Y
 * @param[IN] Real[64] BoundaryLineX2 障碍边界线段终点X
 * @param[IN] Real[64] BoundaryLineY2 障碍边界线段终点Y
 * @param[OUT] Integer PlanFlg 规划成功标志
 * @param[OUT] Integer SegNum 路径段数
 * @param[OUT] Real PathLen 路径总长
 * @param[OUT] Integer[9] SegGoDir 各段行驶方向
 * @param[OUT] Integer[9] SegTrnDir 各段转弯方向
 * @param[OUT] Real[9] SegCrv 各段曲率
 * @param[OUT] Real[9] SegTht 各段转角
 * @param[OUT] Real[9] SegLen 各段长度
 * @param[OUT] Real[9] SegStX 各段起点X
 * @param[OUT] Real[9] SegStY 各段起点Y
 * @param[OUT] Real[9] SegStTh 各段起点航向
 * @param[OUT] Real[9] SegCcX 各段圆心X
 * @param[OUT] Real[9] SegCcY 各段圆心Y
 * @param[OUT] Integer StpN 换向次数
 * @param[OUT] Integer PlnChkCnt 规划检查次数
 * @param[OUT] Integer FailRs0 失败原因首项
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 VH垂直头入
 * @source matlab_rewrite geo_VH_PathPlanning.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
static Boolean geoVhCandidateAllowed(const GeoVhPathPlanningOutput_t *candidate,
                                     const GeoVhPathPlanningParam_t *param) {
  return candidate->PlanFlg != 0 && apaPlannerHybridCheckGeometry(candidate->SegNum,
                                                                  VH_SEG_OUT_MAX,
                                                                  candidate->PlannerCase,
                                                                  candidate->SegGoDir,
                                                                  candidate->SegTrnDir,
                                                                  candidate->SegCrv,
                                                                  candidate->SegTht,
                                                                  candidate->SegLen,
                                                                  candidate->SegStX,
                                                                  candidate->SegStY,
                                                                  candidate->SegStTh,
                                                                  candidate->SegCcX,
                                                                  candidate->SegCcY,
                                                                  &param->HybridPolicy,
                                                                  param->Diagnostics);
}

void geoVhPathPlanning(const GeoVhPathPlanningInput_t *input,
                       const GeoVhPathPlanningParam_t *param,
                       GeoVhPathPlanningOutput_t *output) {
  VhSegWorkspace_t ws;
  Integer si;
  Real rd0;
  Integer dirSTmp0;
  Integer calcnt0;
  Real tarX0;
  Real tarY0;
  Real tarTh0;
  Real entryX0;
  Real entryY0;
  Boolean targetPrecheckSafe;

  *output = (GeoVhPathPlanningOutput_t){0};
  for (si = 0; si < VH_SEG_MAX; si = si + 1) {
    ws.GoDir[si] = 0;
    ws.TrnDir[si] = 0;
    ws.Crv[si] = 0.0f;
    ws.Tht[si] = 0.0f;
    ws.Len[si] = 0.0f;
    ws.Stx[si] = 0.0f;
    ws.Sty[si] = 0.0f;
    ws.StTh[si] = 0.0f;
    ws.CcX[si] = 0.0f;
    ws.CcY[si] = 0.0f;
  }
  ws.SegNum = 0;
  ws.IniSegNum = 0;
  ws.PlanCmd = 0;
  ws.FailRs = 0;
  ws.PlnChkCnt = 0;
  ws.PlanEndX = input->VehX;
  ws.PlanEndY = input->VehY;
  ws.PlanEndTheta = input->VehTheta;

  targetPrecheckSafe = geoVhRunTargetPrecheck(input, param, output);
  if (!targetPrecheckSafe)
    apaPlanningRecordFailure(
        param->Diagnostics, APA_FAULT_COLLISION_AT_TARGET_SLOT, "vh_target_precheck", 5);
  if (targetPrecheckSafe) {
    rd0 = input->Rmin;
    tarX0 = input->TarX;
    tarY0 = input->TarY;
    tarTh0 = input->TarTheta;
    (void)geoVhTryInitialOffset(input, param, &ws);
    geoVhResolveEntryPointFromPose(
        input, ws.PlanEndX, ws.PlanEndY, ws.PlanEndTheta, &entryX0, &entryY0);

    {
      Integer mn0;
      Real firstLen;
      Real dx0;
      Real dy0;

      dx0 = entryX0 - tarX0;
      dy0 = entryY0 - tarY0;
      firstLen = sqrtf(dx0 * dx0 + dy0 * dy0);

      mn0 = VH_SEG_MAX - ws.SegNum - 1;
      geoVhWriteLineSeg(&ws, mn0, entryX0, entryY0, tarTh0, 1, firstLen);
      ws.SegNum = ws.SegNum + 1;
    }

    {
      Real entryRelX;
      Real entryRelY;
      Real entryRelTh;

      geoAxisTransToTarget(entryX0,
                           entryY0,
                           tarTh0,
                           ws.PlanEndX,
                           ws.PlanEndY,
                           ws.PlanEndTheta,
                           &entryRelX,
                           &entryRelY,
                           &entryRelTh);
      if ((fabsf(entryRelY) < 0.2f) && (fabsf(entryRelTh) < 0.1f)) {
        if (fabsf(entryRelX) > 0.05f) {
          Integer mn0;
          Integer linDir;

          mn0 = VH_SEG_MAX - ws.SegNum - 1;
          linDir = 1;
          if (entryRelX < 0.0f) {
            linDir = -1;
          }
          geoVhWriteLineSeg(
              &ws, mn0, ws.PlanEndX, ws.PlanEndY, ws.PlanEndTheta, linDir, fabsf(entryRelX));
          ws.SegNum = ws.SegNum + 1;
        }
        ws.PlanCmd = 1;
      }
    }

    dirSTmp0 = -1;
    calcnt0 = 0;

    while ((ws.SegNum < (9 - ws.IniSegNum)) && ws.PlanCmd != 5 && ws.PlanCmd != 1) {
      Real stpX0;
      Real stpY0;
      Real stpTh0;
      GeoPosTypClassPkInOutput_t classOutput;
      Integer pathType0;
      Boolean firstGroup0;
      Boolean secondGroup0;
      Boolean branchOk0;

      {
        Integer prevIdx;
        prevIdx = VH_SEG_MAX - ws.SegNum;
        stpX0 = ws.Stx[prevIdx];
        stpY0 = ws.Sty[prevIdx];
        stpTh0 = ws.StTh[prevIdx];
      }

      geoVhClassifyLoopPose(
          stpX0, stpY0, stpTh0, ws.PlanEndX, ws.PlanEndY, ws.PlanEndTheta, dirSTmp0, &classOutput);
      pathType0 = classOutput.TarPTyp;
      firstGroup0 = geoVhPathTypeInTwoCircleFamily(pathType0);
      secondGroup0 = geoVhPathTypeInCircleLineFamily(pathType0);
      branchOk0 = false;

      if (firstGroup0) {
        branchOk0 = geoVhTryFirstGroupBranch(&ws,
                                             input,
                                             param,
                                             stpX0,
                                             stpY0,
                                             stpTh0,
                                             ws.PlanEndX,
                                             ws.PlanEndY,
                                             ws.PlanEndTheta,
                                             pathType0,
                                             dirSTmp0,
                                             rd0);
      } else if (secondGroup0) {
        branchOk0 = geoVhTrySecondGroupBranch(&ws,
                                              input,
                                              param,
                                              stpX0,
                                              stpY0,
                                              stpTh0,
                                              ws.PlanEndX,
                                              ws.PlanEndY,
                                              ws.PlanEndTheta,
                                              pathType0,
                                              dirSTmp0,
                                              rd0,
                                              &classOutput);
      } else {
        ws.PlanCmd = 5;
        ws.FailRs = 7;
      }

      if ((!branchOk0) && (ws.PlanCmd != 1) && (ws.PlanCmd != 2) && (ws.PlanCmd != 5)) {
        ws.PlanCmd = 5;
        if (ws.FailRs == 0) {
          ws.FailRs = 7;
        }
      }

      if (ws.PlanCmd == 2) {
        Integer dirIdx0;
        dirIdx0 = VH_SEG_MAX - ws.SegNum;
        if ((dirIdx0 >= 0) && (dirIdx0 < VH_SEG_MAX)) {
          dirSTmp0 = ws.GoDir[dirIdx0];
        } else {
          ws.PlanCmd = 5;
          if (ws.FailRs == 0) {
            ws.FailRs = 9;
          }
        }
      }

      calcnt0 = calcnt0 + 1;
      if (calcnt0 > 10) {
        ws.FailRs = 9;
        ws.PlanCmd = 5;
      }
    }

    if (ws.PlanCmd == 1) {
      geoVhAssembleOutput(&ws, output);
      output->PlannerCase = GEO_VH_PLANNER_CASE_MAIN;
    } else {
      output->PlanFlg = 0;
    }
    output->PlnChkCnt = ws.PlnChkCnt;
    output->FailRs0 = ws.FailRs;
  }

  {
    Boolean geometricOk;
    Boolean hybridOk;
    Boolean allowHybridFallback;
    Integer fallbackGeomCandidateFlg;
    Integer fallbackHybridCandidateFlg;
    Integer fallbackWinnerKind;
    Integer fallbackDecisionReason;
    Integer fallbackGeomStpN;
    Real fallbackGeomPathLen;
    Integer fallbackGeomSegNum;
    Integer fallbackGeomPlnChkCnt;
    Integer fallbackHybridStpN;
    Real fallbackHybridPathLen;
    Integer fallbackHybridSegNum;
    Integer fallbackHybridPlnChkCnt;
    GeoVhPathPlanningOutput_t geometricCandidate;
    GeoVhPathPlanningOutput_t hybridCandidate;
    GeoVhPathPlanningOutput_t bestCandidate;

    geometricCandidate = *output;
    hybridCandidate = (GeoVhPathPlanningOutput_t){0};
    bestCandidate = geometricCandidate;
    geometricOk = geoVhHasValidCandidate(&geometricCandidate) &&
                  geoVhCandidateAllowed(&geometricCandidate, param);
    if (!geometricOk) {
      geometricCandidate = (GeoVhPathPlanningOutput_t){0};
      bestCandidate = geometricCandidate;
      output->PlanFlg = 0;
    }
    hybridOk = false;
    allowHybridFallback = false;
    if (targetPrecheckSafe) {
      allowHybridFallback = (apaPlannerHybridResolveFusion(param->NeedHybridAlgFusion) != 0);
    }
    if (geometricOk) {
      fallbackGeomCandidateFlg = 1;
    } else {
      fallbackGeomCandidateFlg = 0;
    }
    fallbackHybridCandidateFlg = 0;
    fallbackGeomStpN = 0;
    fallbackGeomPathLen = 0.0f;
    fallbackGeomSegNum = 0;
    fallbackGeomPlnChkCnt = 0;
    fallbackHybridStpN = 0;
    fallbackHybridPathLen = 0.0f;
    fallbackHybridSegNum = 0;
    fallbackHybridPlnChkCnt = 0;

    geoVhStoreFallbackDiagFromCandidate(geometricOk,
                                        &geometricCandidate,
                                        &fallbackGeomStpN,
                                        &fallbackGeomPathLen,
                                        &fallbackGeomSegNum,
                                        &fallbackGeomPlnChkCnt);

    if (allowHybridFallback) {
      hybridOk = geoVhTryHybridFallback(input, param, &hybridCandidate);
      if (hybridOk) {
        fallbackHybridCandidateFlg = 1;
      } else {
        fallbackHybridCandidateFlg = 0;
      }
      geoVhStoreFallbackDiagFromCandidate(hybridOk,
                                          &hybridCandidate,
                                          &fallbackHybridStpN,
                                          &fallbackHybridPathLen,
                                          &fallbackHybridSegNum,
                                          &fallbackHybridPlnChkCnt);
      if (geoVhShouldReplaceCandidate(&bestCandidate, &hybridCandidate, param)) {
        bestCandidate = hybridCandidate;
      }
    }

    fallbackWinnerKind = geoVhResolveFallbackWinnerKind(geometricOk, hybridOk, &bestCandidate);
    fallbackDecisionReason = geoVhResolveFallbackDecisionReason(
        geometricOk, hybridOk, &geometricCandidate, &hybridCandidate, param);
    if (geoVhHasValidCandidate(&bestCandidate)) {
      *output = bestCandidate;
    }
    output->FallbackGeomCandidateFlg = fallbackGeomCandidateFlg;
    output->FallbackHybridCandidateFlg = fallbackHybridCandidateFlg;
    output->FallbackWinnerKind = fallbackWinnerKind;
    output->FallbackDecisionReason = fallbackDecisionReason;
    output->FallbackGeomStpN = fallbackGeomStpN;
    output->FallbackGeomPathLen = fallbackGeomPathLen;
    output->FallbackGeomSegNum = fallbackGeomSegNum;
    output->FallbackGeomPlnChkCnt = fallbackGeomPlnChkCnt;
    output->FallbackHybridStpN = fallbackHybridStpN;
    output->FallbackHybridPathLen = fallbackHybridPathLen;
    output->FallbackHybridSegNum = fallbackHybridSegNum;
    output->FallbackHybridPlnChkCnt = fallbackHybridPlnChkCnt;
  }
}
