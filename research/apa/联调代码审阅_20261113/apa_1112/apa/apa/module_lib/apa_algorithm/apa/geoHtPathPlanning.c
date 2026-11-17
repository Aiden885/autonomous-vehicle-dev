/* MATLAB sources: APA_Cal.m, geo_HT_PathPlanning.m */

#include "apaSlotTypes.h"
#include "apaPlanningProgress.h"

#include "geoHtPathPlanningTypes.h"
#include "apaVehicleDefaults.h"
#include "apaCoordUtils.h"
#include "apaCountUtils.h"
#include "apaMathUtils.h"
#include "apaPlannerHybridParity.h"
#include "apaParallelParkinManeuver.h"
#include "apaPlannerUtils.h"
#include "checkParkingTypeTypes.h"
#include "geoAxisTransTypes.h"
#include "geoCircleIntrvCheckTypes.h"
#include "geo2CircleLinkHTTypes.h"
#include "geoDivideTypes.h"
#include "geoHtDangerPtTypes.h"
#include "geoHt1RFixTypes.h"
#include "geoLineIntrvCheckTypes.h"
#include "geoRev2CircleLinkTypes.h"
#include "geoStPtSpcVtTypes.h"
#include <math.h>

#ifndef GEO_HT_DEFAULT_WHEELBASE
#define GEO_HT_DEFAULT_WHEELBASE APA_VEHICLE_WHEELBASE_M
#endif

#ifndef GEO_HT_PLANNER_CASE_NONE
#define GEO_HT_PLANNER_CASE_NONE 0
#endif

#ifndef GEO_HT_PLANNER_CASE_MAIN
#define GEO_HT_PLANNER_CASE_MAIN 1
#endif

#ifndef GEO_HT_PLANNER_CASE_HYBRID_FALLBACK
#define GEO_HT_PLANNER_CASE_HYBRID_FALLBACK 4
#endif

#ifndef GEO_HT_FALLBACK_WINNER_NONE
#define GEO_HT_FALLBACK_WINNER_NONE 0
#endif

#ifndef GEO_HT_FALLBACK_WINNER_GEOM
#define GEO_HT_FALLBACK_WINNER_GEOM 1
#endif

#ifndef GEO_HT_FALLBACK_WINNER_HYBRID
#define GEO_HT_FALLBACK_WINNER_HYBRID 2
#endif

#ifndef GEO_HT_FALLBACK_DECISION_NONE
#define GEO_HT_FALLBACK_DECISION_NONE 0
#endif

#ifndef GEO_HT_FALLBACK_DECISION_VALID_ONLY
#define GEO_HT_FALLBACK_DECISION_VALID_ONLY 1
#endif

#ifndef GEO_HT_FALLBACK_DECISION_STEP_COUNT
#define GEO_HT_FALLBACK_DECISION_STEP_COUNT 2
#endif

#ifndef GEO_HT_FALLBACK_DECISION_PATH_LEN
#define GEO_HT_FALLBACK_DECISION_PATH_LEN 3
#endif

#ifndef GEO_HT_FALLBACK_DECISION_SEG_NUM
#define GEO_HT_FALLBACK_DECISION_SEG_NUM 4
#endif

#ifndef GEO_HT_FALLBACK_DECISION_PLN_CHK_CNT
#define GEO_HT_FALLBACK_DECISION_PLN_CHK_CNT 5
#endif

#ifndef GEO_HT_CANDIDATE_PATH_EPS
#define GEO_HT_CANDIDATE_PATH_EPS 0.05f
#endif

#ifndef GEO_HT_HYBRID_OVERRIDE_STEP_DELTA_MAX
#define GEO_HT_HYBRID_OVERRIDE_STEP_DELTA_MAX 2
#endif

#ifndef GEO_HT_HYBRID_OVERRIDE_PATH_GAIN_MIN
#define GEO_HT_HYBRID_OVERRIDE_PATH_GAIN_MIN 2.0f
#endif

#ifndef GEO_HT_HYBRID_OVERRIDE_PATH_RATIO_MIN
#define GEO_HT_HYBRID_OVERRIDE_PATH_RATIO_MIN 0.15f
#endif

typedef struct {
  Integer GoDir[HT_SEG_MAX];
  Integer TrnDir[HT_SEG_MAX];
  Real Crv[HT_SEG_MAX];
  Real Tht[HT_SEG_MAX];
  Real Len[HT_SEG_MAX];
  Real Stx[HT_SEG_MAX];
  Real Sty[HT_SEG_MAX];
  Real StTh[HT_SEG_MAX];
  Real CcX[HT_SEG_MAX];
  Real CcY[HT_SEG_MAX];
  Integer SegNum;
  Integer PlanCmd;
  Integer FailRs;
  Integer PlnChkCnt;
} HtSegWorkspace_t;

typedef struct {
  Integer DirS;
  Integer PrkDir;
  Integer TrnD;
  Integer UsesVirtualDanger;
  Real RelY;
  Real DangerX;
  Real DangerY;
} GeoHtMatlabMainNormalEntryDecision_t;

typedef struct {
  Integer DrFixAngFlg;
  Integer InitialGoDir;
  Integer InitialTrnD;
  Real RelX;
  Real RelTheta;
} GeoHtMatlabMainDrFixDecision_t;

typedef struct {
  Integer PlanCmd;
  Integer FailRs;
  Integer DrFixAngFlg;
  Integer AddSegment;
  Integer SegmentGoDir;
  Integer SegmentTrnD;
  Integer PlnChkCnt;
  Real SegmentCrv;
  Real SegmentTht;
  Real SegmentLen;
  Real SegmentStartX;
  Real SegmentStartY;
  Real SegmentStartTheta;
  Real SegmentCenterX;
  Real SegmentCenterY;
  Real PlnEndX;
  Real PlnEndY;
  Real PlnEndTheta;
} GeoHtMatlabMainDrFixResult_t;

typedef struct {
  Integer PlanCmd;
  Integer FailRs;
  Integer IniOfstDesir;
  Integer IniOfstFlg;
  Integer AddSegmentCount;
  Integer FirstArcIndex;
  Integer SecondArcIndex;
  Integer PlnChkCnt;
  Real CheckLen;
  Real IniOfst;
  Real StartX;
  Real StartY;
  Real StartTheta;
  Real TargetX;
  Real TargetY;
  Real TargetTheta;
  Real PlnEndX;
  Real PlnEndY;
  Real PlnEndTheta;
  Real FirstSafeLen;
  Real SecondSafeLen;
} GeoHtMatlabMainInitialOffsetDecision_t;

typedef struct {
  Real X;
  Real Y;
  Real Theta;
} GeoHtMatlabMainLoopPose_t;

typedef struct {
  Integer PlanCmd;
  Integer FailRs;
  Integer Stp1Done;
  Integer HasStopperBarrier;
  Integer NeedsRearProbe;
  Integer AddSegment;
  Integer SegmentGoDir;
  Integer DangerFlag;
  Integer PlnChkCnt;
  Real Pt1X;
  Real Pt1Y;
  Real Pt1Theta;
  Real DangerX;
  Real DangerY;
  Real SegmentStartX;
  Real SegmentStartY;
  Real SegmentStartTheta;
  Real SegmentLen;
  Real FrontSafeLen;
  Real RearSafeLen;
} GeoHtMatlabMainPt1Decision_t;

static Boolean geoHtMatlabMainShouldUseInSlotReplan(const GeoHtPathPlanningInput_t *input,
                                                    const GeoHtPathPlanningParam_t *param);
static Real geoHtMatlabMainResolveInSlotEndLatThreshold(const GeoHtPathPlanningInput_t *input);
static Boolean geoHtTryMatlabInSlotReplan(const GeoHtPathPlanningInput_t *input,
                                          const GeoHtPathPlanningParam_t *param,
                                          GeoHtPathPlanningOutput_t *output);
static Boolean geoHtIsLongEdgeEntrySlot(const GeoHtPathPlanningInput_t *input);
static Integer geoHtMatlabMainCountOutputStpN(const GeoHtPathPlanningOutput_t *output);

static inline Integer geoHtMatlabMainResolveEntryDirection(const GeoHtPathPlanningInput_t *input) {
  Real probeX0;
  Real probeY0;
  Real relX0;
  Real relY0;
  Real relTh0;
  Integer entryDir0;
  Boolean useSecondCorner0;

  useSecondCorner0 = false;
  if (input->SltEntr == 1) {
    useSecondCorner0 = true;
  } else {
    if (input->SltEntr == 2) {
      useSecondCorner0 = true;
    }
  }

  if (useSecondCorner0) {
    probeX0 = input->SltPt2X;
    probeY0 = input->SltPt2Y;
  } else {
    probeX0 = input->SltPt4X;
    probeY0 = input->SltPt4Y;
  }

  geoAxisTransToTarget(
      probeX0, probeY0, 0.0f, input->TarX, input->TarY, input->TarTheta, &relX0, &relY0, &relTh0);
  (void)relX0;
  (void)relTh0;
  entryDir0 = -1;
  if (relY0 > 0.0f) {
    entryDir0 = 1;
  }

  return entryDir0;
}

static Boolean geoHtMatlabMainShouldUseInSlotReplan(const GeoHtPathPlanningInput_t *input,
                                                    const GeoHtPathPlanningParam_t *param) {
  Real slotRect0[8];
  Boolean pointInsideSlot0;
  Boolean stateRequestsReplan0;
  Boolean useInSlotReplan0;

  slotRect0[0] = input->SltPt1X;
  slotRect0[1] = input->SltPt1Y;
  slotRect0[2] = input->SltPt2X;
  slotRect0[3] = input->SltPt2Y;
  slotRect0[4] = input->SltPt3X;
  slotRect0[5] = input->SltPt3Y;
  slotRect0[6] = input->SltPt4X;
  slotRect0[7] = input->SltPt4Y;

  pointInsideSlot0 = apaMathIsPtInRect(input->VehX, input->VehY, slotRect0);
  stateRequestsReplan0 = (param->SltState == 6);
  useInSlotReplan0 = false;
  if (pointInsideSlot0) {
    if (stateRequestsReplan0) {
      useInSlotReplan0 = true;
    }
  }

  return useInSlotReplan0;
}

static Real geoHtMatlabMainResolveInSlotEndLatThreshold(const GeoHtPathPlanningInput_t *input) {
  Real threshold0;

  threshold0 = 0.25f;
  if (input->SlotType == 1) {
    threshold0 = 0.4f;
  }

  return threshold0;
}

static inline void geoHtMatlabMainBuildEntryCheckPose(const GeoHtPathPlanningInput_t *input,
                                                      Real checkPose[3]) {
  Integer entryDir0;
  Real checkTheta0;

  entryDir0 = geoHtMatlabMainResolveEntryDirection(input);
  checkTheta0 = input->TarTheta + ((Real)entryDir0 * APA_HALF_PI);
  checkPose[0] = input->TarX + cosf(checkTheta0);
  checkPose[1] = input->TarY + sinf(checkTheta0);
  checkPose[2] = input->TarTheta;
}

static void
geoHtMatlabMainBuildTargetPrecheckCollisionInput(const GeoHtPathPlanningInput_t *input,
                                                 const GeoHtPathPlanningParam_t *param,
                                                 Real checkX,
                                                 Real checkY,
                                                 Real checkTheta,
                                                 VehicleCollisionCheckInput_t *collisionInput) {
  Integer idx;
  Integer boundaryLineCount0;
  Integer boundarySepCount0;
  Integer stopperCount0;
  Integer virtualLineCount0;
  Integer trackStopperCount0;

  *collisionInput = (VehicleCollisionCheckInput_t){0};
  collisionInput->CheckType = VCC_CHECK_TYPE_PLANNING;
  collisionInput->CollisionModel = VCC_COLLISION_MODEL_FOUR_EDGES;
  collisionInput->CollisionDemand = VCC_COLLISION_DEMAND_SIMPLIFY;
  collisionInput->SlotClass = VCC_SLOT_CLASS_PARALLEL;
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
  collisionInput->VehLF = param->VehLlF;
  collisionInput->VehLB = param->VehLr;
  collisionInput->VehLL = param->VehLlF + param->VehLr;
  collisionInput->VehW = param->VehW;
  collisionInput->VehWB = GEO_HT_DEFAULT_WHEELBASE;
  vehicleCollisionCheckLoadDefaultMargins(collisionInput);

  boundaryLineCount0 =
      apaClampCountToLimits(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_BOUNDARY_LINES);
  boundarySepCount0 =
      apaClampCountToLimits(input->BoundarySepPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  stopperCount0 =
      apaClampCountToLimits(input->StopperPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  virtualLineCount0 = apaClampCountToLimits(
      input->VirtualBoundaryLineNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_VIRTUAL_LINES);
  trackStopperCount0 = apaClampCountToLimits(
      input->SlotTrackStopperNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_TRACK_STOPPER);

  collisionInput->BoundaryLineNum = boundaryLineCount0;
  collisionInput->BoundarySepPtsNum = boundarySepCount0;
  collisionInput->SeparateStopperNum = stopperCount0;
  collisionInput->VirtualBoundaryLineNum = virtualLineCount0;
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
  while (idx < virtualLineCount0) {
    collisionInput->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    collisionInput->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    collisionInput->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    collisionInput->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
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

static Integer geoHtResolveUnknownSurroundingType(const GeoHtPathPlanningInput_t *input,
                                                  const GeoHtPathPlanningParam_t *param) {
  Real pts[8] = {input->SltPt1X,
                 input->SltPt1Y,
                 input->SltPt2X,
                 input->SltPt2Y,
                 input->SltPt3X,
                 input->SltPt3Y,
                 input->SltPt4X,
                 input->SltPt4Y};
  Real target[3] = {input->TarX, input->TarY, input->TarTheta};
  Real front[4], rear[4];
  apaSlotCalcSideLine(pts, input->SltEntr, target, APA_SLOT_CLASS_PARALLEL, front, rear);
  Real lonX = front[0] - rear[0], lonY = front[1] - rear[1];
  Real latX = front[0] - front[2], latY = front[1] - front[3];
  Real lon = hypotf(lonX, lonY), lat = hypotf(latX, latY);
  if (lon <= APA_MATH_EPSILON || lat <= APA_MATH_EPSILON)
    return APA_SURROUND_UNKNOWN;
  Boolean blocked[2];
  for (Integer i = 0; i < 2; ++i) {
    Real direction = i == 0 ? 1.0f : -1.0f;
    VehicleCollisionCheckInput_t check;
    VehicleCollisionCheckOutput_t result = {0};
    geoHtMatlabMainBuildTargetPrecheckCollisionInput(
        input,
        param,
        target[0] + direction * 1.5f * lonX / lon + 0.5f * latX / lat,
        target[1] + direction * 1.5f * lonY / lon + 0.5f * latY / lat,
        target[2],
        &check);
    apaPlanningSceneCollisionCheck(&check, &param->SceneQuery, &result);
    blocked[i] = result.IsCollision;
  }
  if (blocked[0] && blocked[1])
    return APA_SURROUND_OBS_EXIST_BOTH;
  if (blocked[0])
    return APA_SURROUND_OBS_EXIST_LEFT_FRONT_ONLY;
  if (blocked[1])
    return APA_SURROUND_OBS_EXIST_RIGHT_REAR_ONLY;
  return APA_SURROUND_OBS_EXIST_NONE;
}

static Boolean geoHtMatlabMainRunTargetPrecheck(const GeoHtPathPlanningInput_t *input,
                                                const GeoHtPathPlanningParam_t *param,
                                                GeoHtPathPlanningOutput_t *output) {
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
        geoHtMatlabMainBuildEntryCheckPose(input, checkPose0);
      }
      geoHtMatlabMainBuildTargetPrecheckCollisionInput(
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
    output->PlannerCase = GEO_HT_PLANNER_CASE_NONE;
    output->FailRs0 = failRs0;
  }
  return targetSafe0;
}

static Real geoHtResolveVehFrontOffset(const GeoHtPathPlanningParam_t *param) {
  Real frontOffset;

  frontOffset = param->VehLlF;
  if (frontOffset <= GEO_EPSILON) {
    frontOffset = 1.2f;
  }
  return frontOffset;
}

static Real geoHtResolveVehRearOffset(const GeoHtPathPlanningParam_t *param) {
  Real rearOffset;

  rearOffset = param->VehLr;
  if (rearOffset <= GEO_EPSILON) {
    rearOffset = APA_VEHICLE_REAR_TO_BACK_M;
  }
  return rearOffset;
}

static void geoHtFillCircleInput(const GeoHtPathPlanningInput_t *input,
                                 const GeoHtPathPlanningParam_t *param,
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
  Integer bi;
  Integer lineCount;
  Integer virtualLineCount;
  Integer sepPointCount;
  Integer stopperPointCount;
  Integer trackStopperCount;
  *chk = (GeoCircleIntrvCheckInput_t){0};
  chk->UsePlanningCheck = true;
  chk->SlotClass = VCC_SLOT_CLASS_PARALLEL;

  chk->DisableSlotBoundaryFallback = true;
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
  chk->VehWidth = param->VehW;
  chk->VehFrontOffset = geoHtResolveVehFrontOffset(param);
  chk->VehRearOffset = geoHtResolveVehRearOffset(param);

  lineCount = input->BoundaryLineNum;
  if (lineCount > GEO_BOUNDARY_LINE_MAX) {
    lineCount = GEO_BOUNDARY_LINE_MAX;
  }
  if (lineCount < 0) {
    lineCount = 0;
  }
  chk->BoundaryLineNum = lineCount;
  for (bi = 0; bi < lineCount; bi = bi + 1) {
    chk->BoundaryLineX1[bi] = input->BoundaryLineX1[bi];
    chk->BoundaryLineY1[bi] = input->BoundaryLineY1[bi];
    chk->BoundaryLineX2[bi] = input->BoundaryLineX2[bi];
    chk->BoundaryLineY2[bi] = input->BoundaryLineY2[bi];
  }

  virtualLineCount = input->VirtualBoundaryLineNum;
  if (virtualLineCount > GEO_BOUNDARY_LINE_MAX) {
    virtualLineCount = GEO_BOUNDARY_LINE_MAX;
  }
  if (virtualLineCount > VCC_MAX_VIRTUAL_LINES) {
    virtualLineCount = VCC_MAX_VIRTUAL_LINES;
  }
  if (virtualLineCount < 0) {
    virtualLineCount = 0;
  }
  chk->VirtualBoundaryLineNum = virtualLineCount;
  for (bi = 0; bi < virtualLineCount; bi = bi + 1) {
    chk->VirtualBoundaryLineX1[bi] = input->VirtualBoundaryP1X[bi];
    chk->VirtualBoundaryLineY1[bi] = input->VirtualBoundaryP1Y[bi];
    chk->VirtualBoundaryLineX2[bi] = input->VirtualBoundaryP2X[bi];
    chk->VirtualBoundaryLineY2[bi] = input->VirtualBoundaryP2Y[bi];
  }

  sepPointCount = input->BoundarySepPtNum;
  if (sepPointCount > VCC_MAX_SEPARATE_PTS) {
    sepPointCount = VCC_MAX_SEPARATE_PTS;
  }
  if (sepPointCount < 0) {
    sepPointCount = 0;
  }
  chk->BoundarySepPtNum = sepPointCount;
  for (bi = 0; bi < sepPointCount; bi = bi + 1) {
    chk->BoundarySepPtX[bi] = input->BoundarySepPtX[bi];
    chk->BoundarySepPtY[bi] = input->BoundarySepPtY[bi];
  }

  stopperPointCount = input->StopperPtNum;
  if (stopperPointCount > VCC_MAX_SEPARATE_PTS) {
    stopperPointCount = VCC_MAX_SEPARATE_PTS;
  }
  if (stopperPointCount < 0) {
    stopperPointCount = 0;
  }
  chk->StopperPtNum = stopperPointCount;
  for (bi = 0; bi < stopperPointCount; bi = bi + 1) {
    chk->StopperPtX[bi] = input->StopperPtX[bi];
    chk->StopperPtY[bi] = input->StopperPtY[bi];
  }

  trackStopperCount = apaClampCountToLimits(
      input->SlotTrackStopperNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_TRACK_STOPPER);
  chk->SlotTrackStopperNum = trackStopperCount;
  bi = 0;
  while (bi < trackStopperCount) {
    chk->SlotTrackStopperX1[bi] = input->SlotTrackStopperX1[bi];
    chk->SlotTrackStopperY1[bi] = input->SlotTrackStopperY1[bi];
    chk->SlotTrackStopperX2[bi] = input->SlotTrackStopperX2[bi];
    chk->SlotTrackStopperY2[bi] = input->SlotTrackStopperY2[bi];
    bi = bi + 1;
  }
}

static void geoHtWriteArcSeg(HtSegWorkspace_t *ws,
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

static void geoHtWriteLineSeg(
    HtSegWorkspace_t *ws, Integer idx, Real stX, Real stY, Real stTh, Integer goDir, Real len) {
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

static void geoHtMatlabMainInitWorkspace(HtSegWorkspace_t *ws) {
  Integer idx;

  idx = 0;
  while (idx < HT_SEG_MAX) {
    ws->GoDir[idx] = 0;
    ws->TrnDir[idx] = 0;
    ws->Crv[idx] = 0.0f;
    ws->Tht[idx] = 0.0f;
    ws->Len[idx] = 0.0f;
    ws->Stx[idx] = 0.0f;
    ws->Sty[idx] = 0.0f;
    ws->StTh[idx] = 0.0f;
    ws->CcX[idx] = 0.0f;
    ws->CcY[idx] = 0.0f;
    idx = idx + 1;
  }
  ws->SegNum = 0;
  ws->PlanCmd = 0;
  ws->FailRs = 0;
  ws->PlnChkCnt = 0;
}

static Integer geoHtMatlabMainResolveIniSegN(Integer drFixAngFlg, Integer iniOfstFlg) {
  Integer result0;

  result0 = 0;
  if (iniOfstFlg == 1) {
    result0 = 2;
  }
  if (drFixAngFlg == 3) {
    result0 = result0 + 1;
  }
  return result0;
}

static Integer geoHtMatlabMainTailWriteIndex(const HtSegWorkspace_t *ws) {
  Integer index0;

  index0 = (HT_SEG_MAX - ws->SegNum) - 1;
  index0 = apaClampCount(index0, HT_SEG_MAX - 1);
  return index0;
}

static Integer geoHtMatlabMainTailHeadIndex(const HtSegWorkspace_t *ws) {
  Integer index0;

  index0 = 0;
  if (ws->SegNum > 0) {
    index0 = HT_SEG_MAX - ws->SegNum;
  }
  index0 = apaClampCount(index0, HT_SEG_MAX - 1);
  return index0;
}

static void geoHtReadTailStateForPlanCmd2(const HtSegWorkspace_t *ws,
                                          Integer *dirS,
                                          Integer *trnD,
                                          Integer *planCmd) {
  Integer index0;

  if (ws->SegNum > 0) {
    index0 = geoHtMatlabMainTailHeadIndex(ws);
    *dirS = ws->GoDir[index0];
    *trnD = -ws->TrnDir[index0];
    *planCmd = 0;
  }
}

static void geoHtCommitDirectLineSeg(GeoHtPathPlanningOutput_t *output,
                                     Integer segIdx,
                                     Real stX,
                                     Real stY,
                                     Real stTh,
                                     Integer goDir,
                                     Real len) {
  output->SegGoDir[segIdx] = goDir;
  output->SegTrnDir[segIdx] = 0;
  output->SegCrv[segIdx] = 0.0f;
  output->SegTht[segIdx] = 0.0f;
  output->SegLen[segIdx] = len;
  output->SegStX[segIdx] = stX;
  output->SegStY[segIdx] = stY;
  output->SegStTh[segIdx] = stTh;
  output->SegCcX[segIdx] = 0.0f;
  output->SegCcY[segIdx] = 0.0f;
}

static void geoHtCommitInSlotArcSeg(GeoHtPathPlanningOutput_t *output,
                                    Integer segIdx,
                                    Real stX,
                                    Real stY,
                                    Real stTh,
                                    Integer goDir,
                                    Integer trnDir,
                                    Real radius,
                                    Real endTheta,
                                    Real arcLen,
                                    Real ccX,
                                    Real ccY) {
  output->SegGoDir[segIdx] = goDir;
  output->SegTrnDir[segIdx] = trnDir;
  output->SegCrv[segIdx] = geoDivideScalar((Real)trnDir, radius);
  output->SegTht[segIdx] = endTheta - stTh;
  output->SegLen[segIdx] = arcLen;
  output->SegStX[segIdx] = stX;
  output->SegStY[segIdx] = stY;
  output->SegStTh[segIdx] = stTh;
  output->SegCcX[segIdx] = ccX;
  output->SegCcY[segIdx] = ccY;
}

static void
geoHtTrimInSlotShortTail(GeoHtPathPlanningOutput_t *output, Integer *segIdx, Real *totalLen) {
  Integer lastIdx;
  Boolean trimShortTail;

  trimShortTail = false;
  if (*segIdx > 1) {
    lastIdx = *segIdx - 1;
    if (output->SegLen[lastIdx] < 0.1f) {
      trimShortTail = true;
    }
  }

  if (trimShortTail) {
    lastIdx = *segIdx - 1;
    *totalLen = *totalLen - output->SegLen[lastIdx];
    if (*totalLen < 0.0f) {
      *totalLen = 0.0f;
    }
    output->SegGoDir[lastIdx] = 0;
    output->SegTrnDir[lastIdx] = 0;
    output->SegCrv[lastIdx] = 0.0f;
    output->SegTht[lastIdx] = 0.0f;
    output->SegLen[lastIdx] = 0.0f;
    output->SegStX[lastIdx] = 0.0f;
    output->SegStY[lastIdx] = 0.0f;
    output->SegStTh[lastIdx] = 0.0f;
    output->SegCcX[lastIdx] = 0.0f;
    output->SegCcY[lastIdx] = 0.0f;
    *segIdx = lastIdx;
  }
}

static Boolean geoHtTryMatlabInSlotReplan(const GeoHtPathPlanningInput_t *input,
                                          const GeoHtPathPlanningParam_t *param,
                                          GeoHtPathPlanningOutput_t *output) {
  GeoCircleIntrvCheckInput_t arcInput;
  GeoCircleIntrvCheckOutput_t arcOutput;
  Real tarRelX;
  Real tarRelY;
  Real tarRelTheta;
  Real centerX;
  Real centerY;
  Real endX;
  Real endY;
  Real endTheta;
  Real avlRelX;
  Real avlRelY;
  Real avlRelTheta;
  Real endLatThreshold;
  Real forwardCenterX;
  Real forwardCenterY;
  Real forwardAvlPosX;
  Real forwardAvlPosY;
  Real forwardAvlPosTheta;
  Real forwardSafeLen;
  Real loopStartX;
  Real loopStartY;
  Real loopStartTheta;
  Real totalLen;
  Integer checkCount;
  Integer trnDir;
  Integer forwardTrnDir;
  Integer loopGoDir;
  Integer loopTrnDir;
  Integer lineDir;
  Integer segIdx;
  Integer failRs;
  Boolean hasForwardCandidate;
  Boolean hasLoopSeed;
  Boolean loopCond;
  Boolean planOk;
  Boolean needsReverse;

  *output = (GeoHtPathPlanningOutput_t){0};
  planOk = false;
  needsReverse = false;
  hasForwardCandidate = false;
  hasLoopSeed = false;
  failRs = 0;
  checkCount = 0;
  endLatThreshold = geoHtMatlabMainResolveInSlotEndLatThreshold(input);
  totalLen = 0.0f;
  segIdx = 0;
  forwardCenterX = 0.0f;
  forwardCenterY = 0.0f;
  forwardAvlPosX = 0.0f;
  forwardAvlPosY = 0.0f;
  forwardAvlPosTheta = 0.0f;
  forwardSafeLen = 0.0f;
  forwardTrnDir = 0;
  loopStartX = 0.0f;
  loopStartY = 0.0f;
  loopStartTheta = 0.0f;
  loopGoDir = 0;
  loopTrnDir = 0;

  geoAxisTransToTarget(input->TarX,
                       input->TarY,
                       input->TarTheta,
                       input->VehX,
                       input->VehY,
                       input->VehTheta,
                       &tarRelX,
                       &tarRelY,
                       &tarRelTheta);
  (void)tarRelX;
  (void)tarRelY;
  trnDir = apaMathSign(tarRelTheta);
  if (trnDir == 0) {
    trnDir = 1;
  }

  centerX = input->VehX + (input->Rmin * cosf(input->VehTheta + ((Real)trnDir * APA_HALF_PI)));
  centerY = input->VehY + (input->Rmin * sinf(input->VehTheta + ((Real)trnDir * APA_HALF_PI)));
  endX = centerX + (input->Rmin * cosf(input->TarTheta - ((Real)trnDir * APA_HALF_PI)));
  endY = centerY + (input->Rmin * sinf(input->TarTheta - ((Real)trnDir * APA_HALF_PI)));
  endTheta = input->TarTheta;

  geoHtFillCircleInput(input,
                       param,
                       centerX,
                       centerY,
                       input->Rmin,
                       input->VehX,
                       input->VehY,
                       input->VehTheta,
                       endX,
                       endY,
                       endTheta,
                       1,
                       trnDir,
                       0.1f,
                       &arcInput);
  arcOutput = (GeoCircleIntrvCheckOutput_t){0};
  geoCircleIntrvCheckWithScene(&arcInput, &param->SceneQuery, &arcOutput);
  checkCount = checkCount + arcOutput.CheckCount;

  if (arcOutput.IntrvFlg == 2) {
    failRs = 8;
  } else {
    hasForwardCandidate = true;
    forwardCenterX = centerX;
    forwardCenterY = centerY;
    forwardTrnDir = trnDir;
    forwardSafeLen = arcOutput.SafeLen;
    forwardAvlPosX = arcOutput.AvlPosX;
    forwardAvlPosY = arcOutput.AvlPosY;
    forwardAvlPosTheta = arcOutput.AvlPosTheta;
    if (arcOutput.IntrvFlg == 0) {
      geoAxisTransToTarget(arcOutput.AvlPosX,
                           arcOutput.AvlPosY,
                           arcOutput.AvlPosTheta,
                           input->TarX,
                           input->TarY,
                           input->TarTheta,
                           &avlRelX,
                           &avlRelY,
                           &avlRelTheta);
      (void)avlRelTheta;
      if (fabsf(avlRelY) < 0.5f) {
        if (fabsf(avlRelY) >= endLatThreshold) {
          failRs = 7;
        } else {
          geoHtCommitInSlotArcSeg(output,
                                  segIdx,
                                  input->VehX,
                                  input->VehY,
                                  input->VehTheta,
                                  1,
                                  trnDir,
                                  input->Rmin,
                                  arcOutput.AvlPosTheta,
                                  arcOutput.SafeLen,
                                  centerX,
                                  centerY);
          totalLen = totalLen + output->SegLen[segIdx];
          segIdx = segIdx + 1;
          if (fabsf(avlRelX) >= 0.2f) {
            lineDir = -apaMathSign(avlRelX);
            if (lineDir == 0) {
              lineDir = 1;
            }
            geoHtCommitDirectLineSeg(output,
                                     segIdx,
                                     arcOutput.AvlPosX,
                                     arcOutput.AvlPosY,
                                     arcOutput.AvlPosTheta,
                                     lineDir,
                                     fabsf(avlRelX));
            totalLen = totalLen + output->SegLen[segIdx];
            segIdx = segIdx + 1;
          }
          planOk = true;
        }
      } else {
        needsReverse = true;
      }
    } else {
      needsReverse = true;
    }
  }

  if ((!planOk) && needsReverse && (failRs == 0)) {
    trnDir = -trnDir;
    centerX = input->VehX + (input->Rmin * cosf(input->VehTheta + ((Real)trnDir * APA_HALF_PI)));
    centerY = input->VehY + (input->Rmin * sinf(input->VehTheta + ((Real)trnDir * APA_HALF_PI)));
    endX = centerX + (input->Rmin * cosf(input->TarTheta - ((Real)trnDir * APA_HALF_PI)));
    endY = centerY + (input->Rmin * sinf(input->TarTheta - ((Real)trnDir * APA_HALF_PI)));
    geoHtFillCircleInput(input,
                         param,
                         centerX,
                         centerY,
                         input->Rmin,
                         input->VehX,
                         input->VehY,
                         input->VehTheta,
                         endX,
                         endY,
                         endTheta,
                         -1,
                         trnDir,
                         0.1f,
                         &arcInput);
    arcOutput = (GeoCircleIntrvCheckOutput_t){0};
    geoCircleIntrvCheckWithScene(&arcInput, &param->SceneQuery, &arcOutput);
    checkCount = checkCount + arcOutput.CheckCount;

    if (arcOutput.IntrvFlg == 2) {
      failRs = 8;
    } else {
      if (arcOutput.IntrvFlg == 0) {
        geoAxisTransToTarget(arcOutput.AvlPosX,
                             arcOutput.AvlPosY,
                             arcOutput.AvlPosTheta,
                             input->TarX,
                             input->TarY,
                             input->TarTheta,
                             &avlRelX,
                             &avlRelY,
                             &avlRelTheta);
        (void)avlRelTheta;
        if (fabsf(avlRelY) < 0.5f) {
          if (fabsf(avlRelY) >= endLatThreshold) {
            failRs = 7;
          } else {
            geoHtCommitInSlotArcSeg(output,
                                    segIdx,
                                    input->VehX,
                                    input->VehY,
                                    input->VehTheta,
                                    -1,
                                    trnDir,
                                    input->Rmin,
                                    arcOutput.AvlPosTheta,
                                    arcOutput.SafeLen,
                                    centerX,
                                    centerY);
            totalLen = totalLen + output->SegLen[segIdx];
            segIdx = segIdx + 1;
            if (fabsf(avlRelX) >= 0.2f) {
              lineDir = -apaMathSign(avlRelX);
              if (lineDir == 0) {
                lineDir = 1;
              }
              geoHtCommitDirectLineSeg(output,
                                       segIdx,
                                       arcOutput.AvlPosX,
                                       arcOutput.AvlPosY,
                                       arcOutput.AvlPosTheta,
                                       lineDir,
                                       fabsf(avlRelX));
              totalLen = totalLen + output->SegLen[segIdx];
              segIdx = segIdx + 1;
            }
            planOk = true;
          }
        } else {
          failRs = 7;
        }
      } else {
        if (hasForwardCandidate && (forwardSafeLen >= arcOutput.SafeLen)) {
          if (segIdx < HT_SEG_OUT_MAX) {
            geoHtCommitInSlotArcSeg(output,
                                    segIdx,
                                    input->VehX,
                                    input->VehY,
                                    input->VehTheta,
                                    1,
                                    forwardTrnDir,
                                    input->Rmin,
                                    forwardAvlPosTheta,
                                    forwardSafeLen,
                                    forwardCenterX,
                                    forwardCenterY);
            totalLen = totalLen + output->SegLen[segIdx];
            loopStartX = forwardAvlPosX;
            loopStartY = forwardAvlPosY;
            loopStartTheta = forwardAvlPosTheta;
            loopGoDir = -1;
            loopTrnDir = -forwardTrnDir;
            segIdx = segIdx + 1;
            hasLoopSeed = true;
          } else {
            failRs = 7;
          }
        } else {
          if (segIdx < HT_SEG_OUT_MAX) {
            geoHtCommitInSlotArcSeg(output,
                                    segIdx,
                                    input->VehX,
                                    input->VehY,
                                    input->VehTheta,
                                    -1,
                                    trnDir,
                                    input->Rmin,
                                    arcOutput.AvlPosTheta,
                                    arcOutput.SafeLen,
                                    centerX,
                                    centerY);
            totalLen = totalLen + output->SegLen[segIdx];
            loopStartX = arcOutput.AvlPosX;
            loopStartY = arcOutput.AvlPosY;
            loopStartTheta = arcOutput.AvlPosTheta;
            loopGoDir = 1;
            loopTrnDir = -trnDir;
            segIdx = segIdx + 1;
            hasLoopSeed = true;
          } else {
            failRs = 7;
          }
        }
      }
    }
  }

  loopCond = hasLoopSeed;
  while (loopCond) {
    if ((!planOk) && (failRs == 0) && (segIdx < 7) && (segIdx < HT_SEG_OUT_MAX)) {
      centerX =
          loopStartX + (input->Rmin * cosf(loopStartTheta + ((Real)loopTrnDir * APA_HALF_PI)));
      centerY =
          loopStartY + (input->Rmin * sinf(loopStartTheta + ((Real)loopTrnDir * APA_HALF_PI)));
      endX = centerX + (input->Rmin * cosf(input->TarTheta - ((Real)loopTrnDir * APA_HALF_PI)));
      endY = centerY + (input->Rmin * sinf(input->TarTheta - ((Real)loopTrnDir * APA_HALF_PI)));
      geoHtFillCircleInput(input,
                           param,
                           centerX,
                           centerY,
                           input->Rmin,
                           loopStartX,
                           loopStartY,
                           loopStartTheta,
                           endX,
                           endY,
                           endTheta,
                           loopGoDir,
                           loopTrnDir,
                           0.1f,
                           &arcInput);
      arcOutput = (GeoCircleIntrvCheckOutput_t){0};
      geoCircleIntrvCheckWithScene(&arcInput, &param->SceneQuery, &arcOutput);
      checkCount = checkCount + arcOutput.CheckCount;

      if (arcOutput.IntrvFlg == 2) {
        failRs = 7;
        loopCond = false;
      } else {
        if (arcOutput.IntrvFlg == 0) {
          geoAxisTransToTarget(arcOutput.AvlPosX,
                               arcOutput.AvlPosY,
                               arcOutput.AvlPosTheta,
                               input->TarX,
                               input->TarY,
                               input->TarTheta,
                               &avlRelX,
                               &avlRelY,
                               &avlRelTheta);
          (void)avlRelTheta;
          if (fabsf(avlRelY) >= endLatThreshold) {
            failRs = 7;
            loopCond = false;
          } else {
            geoHtCommitInSlotArcSeg(output,
                                    segIdx,
                                    loopStartX,
                                    loopStartY,
                                    loopStartTheta,
                                    loopGoDir,
                                    loopTrnDir,
                                    input->Rmin,
                                    arcOutput.AvlPosTheta,
                                    arcOutput.SafeLen,
                                    centerX,
                                    centerY);
            totalLen = totalLen + output->SegLen[segIdx];
            segIdx = segIdx + 1;
            if (fabsf(avlRelX) >= 0.2f) {
              if (segIdx < HT_SEG_OUT_MAX) {
                lineDir = -apaMathSign(avlRelX);
                if (lineDir == 0) {
                  lineDir = 1;
                }
                geoHtCommitDirectLineSeg(output,
                                         segIdx,
                                         arcOutput.AvlPosX,
                                         arcOutput.AvlPosY,
                                         arcOutput.AvlPosTheta,
                                         lineDir,
                                         fabsf(avlRelX));
                totalLen = totalLen + output->SegLen[segIdx];
                segIdx = segIdx + 1;
                planOk = true;
              } else {
                failRs = 7;
              }
            } else {
              planOk = true;
            }
            loopCond = false;
          }
        } else {
          geoHtCommitInSlotArcSeg(output,
                                  segIdx,
                                  loopStartX,
                                  loopStartY,
                                  loopStartTheta,
                                  loopGoDir,
                                  loopTrnDir,
                                  input->Rmin,
                                  arcOutput.AvlPosTheta,
                                  arcOutput.SafeLen,
                                  centerX,
                                  centerY);
          totalLen = totalLen + output->SegLen[segIdx];
          loopStartX = arcOutput.AvlPosX;
          loopStartY = arcOutput.AvlPosY;
          loopStartTheta = arcOutput.AvlPosTheta;
          loopGoDir = -loopGoDir;
          loopTrnDir = -loopTrnDir;
          segIdx = segIdx + 1;
        }
      }
    } else {
      loopCond = false;
    }
  }

  if ((!planOk) && hasLoopSeed && (failRs == 0)) {
    failRs = 7;
  }

  output->PlnChkCnt = checkCount;
  output->FailRs0 = failRs;
  if (planOk) {
    geoHtTrimInSlotShortTail(output, &segIdx, &totalLen);
    output->PlanFlg = 1;
    output->SegNum = segIdx;
    output->PathLen = totalLen;
    output->StpN = geoHtMatlabMainCountOutputStpN(output);
    output->PlannerCase = GEO_HT_PLANNER_CASE_MAIN;
    output->FallbackGeomCandidateFlg = 1;
    output->FallbackWinnerKind = GEO_HT_FALLBACK_WINNER_GEOM;
    output->FallbackGeomStpN = output->StpN;
    output->FallbackGeomPathLen = output->PathLen;
    output->FallbackGeomSegNum = output->SegNum;
    output->FallbackGeomPlnChkCnt = output->PlnChkCnt;
  }

  return planOk;
}

static void geoHtMatlabMainCopyWorkspaceSegmentToOutput(const HtSegWorkspace_t *ws,
                                                        Integer sourceIndex,
                                                        Integer outputIndex,
                                                        GeoHtPathPlanningOutput_t *output) {
  Integer sourceIndex0;

  sourceIndex0 = apaClampCount(sourceIndex, HT_SEG_MAX - 1);
  output->SegGoDir[outputIndex] = ws->GoDir[sourceIndex0];
  output->SegTrnDir[outputIndex] = ws->TrnDir[sourceIndex0];
  output->SegCrv[outputIndex] = ws->Crv[sourceIndex0];
  output->SegTht[outputIndex] = ws->Tht[sourceIndex0];
  output->SegLen[outputIndex] = ws->Len[sourceIndex0];
  output->SegStX[outputIndex] = ws->Stx[sourceIndex0];
  output->SegStY[outputIndex] = ws->Sty[sourceIndex0];
  output->SegStTh[outputIndex] = ws->StTh[sourceIndex0];
  output->SegCcX[outputIndex] = ws->CcX[sourceIndex0];
  output->SegCcY[outputIndex] = ws->CcY[sourceIndex0];
}

static Integer geoHtMatlabMainCountOutputStpN(const GeoHtPathPlanningOutput_t *output) {
  Integer idx;
  Integer dirTmp0;
  Integer stpN0;
  Integer dirProduct0;

  idx = 1;
  stpN0 = 0;
  if (output->SegNum > 0) {
    dirTmp0 = output->SegGoDir[0];
    if (dirTmp0 < 0) {
      stpN0 = 1;
    }
    while (idx < output->SegNum) {
      dirProduct0 = output->SegGoDir[idx] * dirTmp0;
      if (dirProduct0 < 0) {
        stpN0 = stpN0 + 1;
        dirTmp0 = output->SegGoDir[idx];
      }
      idx = idx + 1;
    }
  }
  return stpN0;
}

static Boolean geoHtAssembleOutputWithPrefix(const HtSegWorkspace_t *ws,
                                             Integer iniSegN,
                                             GeoHtPathPlanningOutput_t *output) {
  Integer finalSegNum0;
  Integer idx;
  Integer outputIndex0;
  Integer tailStart0;
  Boolean planDone0;
  Boolean hasAnySegment0;
  Boolean stageCountOk0;
  Boolean outputCountOk0;
  Boolean assembleOk0;

  *output = (GeoHtPathPlanningOutput_t){0};
  finalSegNum0 = ws->SegNum + iniSegN;
  planDone0 = (ws->PlanCmd == 1);
  hasAnySegment0 = (finalSegNum0 > 0);
  stageCountOk0 = (finalSegNum0 <= HT_SEG_MAX);
  outputCountOk0 = (finalSegNum0 <= HT_SEG_OUT_MAX);
  assembleOk0 = false;
  if (planDone0) {
    if (hasAnySegment0) {
      if (stageCountOk0) {
        if (outputCountOk0) {
          assembleOk0 = true;
        }
      }
    }
  }

  if (assembleOk0) {
    output->PlanFlg = 1;
    output->SegNum = finalSegNum0;
    idx = 0;
    while (idx < iniSegN) {
      geoHtMatlabMainCopyWorkspaceSegmentToOutput(ws, idx, idx, output);
      idx = idx + 1;
    }
    tailStart0 = HT_SEG_MAX - ws->SegNum;
    idx = 0;
    while (idx < ws->SegNum) {
      outputIndex0 = iniSegN + idx;
      geoHtMatlabMainCopyWorkspaceSegmentToOutput(ws, tailStart0 + idx, outputIndex0, output);
      idx = idx + 1;
    }
    output->PathLen = 0.0f;
    idx = 0;
    while (idx < output->SegNum) {
      output->PathLen = output->PathLen + fabsf(output->SegLen[idx]);
      idx = idx + 1;
    }
    output->StpN = geoHtMatlabMainCountOutputStpN(output);
    output->PlnChkCnt = ws->PlnChkCnt;
    output->FailRs0 = ws->FailRs;
    output->PlannerCase = GEO_HT_PLANNER_CASE_MAIN;
  } else {
    output->PlanFlg = 0;
    output->SegNum = 0;
    output->PathLen = 0.0f;
    output->StpN = 0;
    output->PlnChkCnt = ws->PlnChkCnt;
    output->FailRs0 = ws->FailRs;
  }

  return assembleOk0;
}

static void geoHtSelectEntryEdge(const GeoHtPathPlanningInput_t *input,
                                 Real *entryP1X,
                                 Real *entryP1Y,
                                 Real *entryP2X,
                                 Real *entryP2Y) {
  if (input->SltEntr == 2) {
    *entryP1X = input->SltPt2X;
    *entryP1Y = input->SltPt2Y;
    *entryP2X = input->SltPt3X;
    *entryP2Y = input->SltPt3Y;
  } else if (input->SltEntr == 3) {
    *entryP1X = input->SltPt3X;
    *entryP1Y = input->SltPt3Y;
    *entryP2X = input->SltPt4X;
    *entryP2Y = input->SltPt4Y;
  } else if (input->SltEntr == 4) {
    *entryP1X = input->SltPt4X;
    *entryP1Y = input->SltPt4Y;
    *entryP2X = input->SltPt1X;
    *entryP2Y = input->SltPt1Y;
  } else {
    *entryP1X = input->SltPt1X;
    *entryP1Y = input->SltPt1Y;
    *entryP2X = input->SltPt2X;
    *entryP2Y = input->SltPt2Y;
  }
}

static void geoHtFillDangerInput(const GeoHtPathPlanningInput_t *input,
                                 const GeoHtPathPlanningParam_t *param,
                                 Integer prkType,
                                 Real pt1X,
                                 Real pt1Y,
                                 Real pt1Theta,
                                 GeoHtDangerPtInput_t *dangerInput) {
  Integer idx;
  Integer lineCount;

  *dangerInput = (GeoHtDangerPtInput_t){0};
  geoHtSelectEntryEdge(input,
                       &dangerInput->SltEntrPt1X,
                       &dangerInput->SltEntrPt1Y,
                       &dangerInput->SltEntrPt2X,
                       &dangerInput->SltEntrPt2Y);
  dangerInput->TgtX = input->TarX;
  dangerInput->TgtY = input->TarY;
  dangerInput->TgtTheta = input->TarTheta;
  dangerInput->VehX = input->VehX;
  dangerInput->VehY = input->VehY;
  dangerInput->VehTheta = input->VehTheta;
  dangerInput->Pt1X = pt1X;
  dangerInput->Pt1Y = pt1Y;
  dangerInput->Pt1Theta = pt1Theta;
  dangerInput->Radius = input->Rmin;
  dangerInput->PrkType = prkType;
  dangerInput->LLf = param->VehLlF;
  dangerInput->VehW = param->VehW;

  lineCount = apaClampCountToLimits(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX, DGR_LINE_MAX);
  dangerInput->BoundaryLineNum = lineCount;
  idx = 0;
  while (idx < lineCount) {
    dangerInput->BoundaryP1X[idx] = input->BoundaryLineX1[idx];
    dangerInput->BoundaryP1Y[idx] = input->BoundaryLineY1[idx];
    dangerInput->BoundaryP2X[idx] = input->BoundaryLineX2[idx];
    dangerInput->BoundaryP2Y[idx] = input->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  lineCount =
      apaClampCountToLimits(input->VirtualBoundaryLineNum, GEO_BOUNDARY_LINE_MAX, DGR_LINE_MAX);
  dangerInput->VirtualBoundaryLineNum = lineCount;
  idx = 0;
  while (idx < lineCount) {
    dangerInput->VirtualBoundaryP1X[idx] = input->VirtualBoundaryP1X[idx];
    dangerInput->VirtualBoundaryP1Y[idx] = input->VirtualBoundaryP1Y[idx];
    dangerInput->VirtualBoundaryP2X[idx] = input->VirtualBoundaryP2X[idx];
    dangerInput->VirtualBoundaryP2Y[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }

  lineCount = apaClampCountToLimits(input->BoundarySepPtNum, DGR_PT_MAX, DGR_PT_MAX);
  dangerInput->BoundarySepPtNum = lineCount;
  idx = 0;
  while (idx < lineCount) {
    dangerInput->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    dangerInput->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  lineCount = apaClampCountToLimits(input->StopperPtNum, DGR_PT_MAX, DGR_PT_MAX);
  dangerInput->StopperPtNum = lineCount;
  idx = 0;
  while (idx < lineCount) {
    dangerInput->StopperPtX[idx] = input->StopperPtX[idx];
    dangerInput->StopperPtY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }
}

static void geoHtSynthesizeDangerPoint(const GeoHtPathPlanningInput_t *input,
                                       const GeoHtPathPlanningParam_t *param,
                                       Real relY,
                                       Real pt1X,
                                       Real pt1Y,
                                       Real pt1Theta,
                                       Real *dangerX,
                                       Real *dangerY) {
  Real widthHalf;
  Real frontOffset;
  Real radius;
  Real longGuard;
  Real latGuard;
  Real safeRootA;
  Real safeRootB;
  Real safeRoot;

  (void)input;
  widthHalf = param->VehW * 0.5f;
  frontOffset = geoHtResolveVehFrontOffset(param);
  radius = input->Rmin;
  safeRootA = radius + widthHalf;
  safeRootA = safeRootA * safeRootA;
  safeRootB = radius - widthHalf - 0.2f;
  safeRootB = safeRootB * safeRootB;
  safeRoot = safeRootA + (frontOffset * frontOffset) - safeRootB;
  if (safeRoot < 0.0f) {
    safeRoot = 0.0f;
  }
  longGuard = sqrtf(safeRoot) + 0.5f;
  latGuard = widthHalf + 0.3f;

  if (relY > 0.0f) {
    *dangerX = pt1X - (latGuard * sinf(pt1Theta)) + (longGuard * cosf(pt1Theta));
    *dangerY = pt1Y + (latGuard * cosf(pt1Theta)) + (longGuard * sinf(pt1Theta));
  } else {
    *dangerX = pt1X + (latGuard * sinf(pt1Theta)) + (longGuard * cosf(pt1Theta));
    *dangerY = pt1Y - (latGuard * cosf(pt1Theta)) + (longGuard * sinf(pt1Theta));
  }
}

static inline void
geoHtMatlabMainResolveNormalEntryDecision(const GeoHtPathPlanningInput_t *input,
                                          const GeoHtPathPlanningParam_t *param,
                                          Real pt1X,
                                          Real pt1Y,
                                          Real pt1Theta,
                                          Integer dangerFlag,
                                          Real dangerX,
                                          Real dangerY,
                                          GeoHtMatlabMainNormalEntryDecision_t *decision) {
  Real relX0;
  Real relY0;
  Real relTh0;
  Boolean hasRealDanger0;

  geoAxisTransToTarget(input->VehX,
                       input->VehY,
                       input->VehTheta,
                       input->TarX,
                       input->TarY,
                       input->TarTheta,
                       &relX0,
                       &relY0,
                       &relTh0);
  (void)relX0;
  (void)relTh0;

  decision->DirS = 1;
  decision->RelY = relY0;
  if (relY0 > 0.0f) {
    decision->PrkDir = -1;
    decision->TrnD = 1;
  } else {
    decision->PrkDir = 1;
    decision->TrnD = -1;
  }

  hasRealDanger0 = (dangerFlag > 0);
  if (hasRealDanger0) {
    decision->DangerX = dangerX;
    decision->DangerY = dangerY;
    decision->UsesVirtualDanger = 0;
  } else {
    geoHtSynthesizeDangerPoint(
        input, param, relY0, pt1X, pt1Y, pt1Theta, &decision->DangerX, &decision->DangerY);
    decision->UsesVirtualDanger = 1;
  }
}

static inline void geoHtMatlabMainResolveDrFixDecision(const GeoHtPathPlanningInput_t *input,
                                                       Boolean isInSlotRect,
                                                       GeoHtMatlabMainDrFixDecision_t *decision) {
  Real relX0;
  Real relY0;
  Real relTheta0;
  Real absRelTheta0;
  Integer signTheta0;
  Boolean noFixNeeded0;
  Boolean allowForwardFix0;

  geoAxisTransToTarget(input->TarX,
                       input->TarY,
                       input->TarTheta,
                       input->VehX,
                       input->VehY,
                       input->VehTheta,
                       &relX0,
                       &relY0,
                       &relTheta0);
  (void)relY0;
  absRelTheta0 = fabsf(relTheta0);
  decision->RelX = relX0;
  decision->RelTheta = relTheta0;
  decision->DrFixAngFlg = 0;
  decision->InitialGoDir = 0;
  decision->InitialTrnD = 0;

  noFixNeeded0 = (absRelTheta0 < (10.0f * APA_PI / 180.0f));
  if (isInSlotRect) {
    noFixNeeded0 = true;
  }

  if (!noFixNeeded0) {
    signTheta0 = 1;
    if (relTheta0 < 0.0f) {
      signTheta0 = -1;
    }
    allowForwardFix0 = ((relX0 >= -5.0f) && (absRelTheta0 <= (20.0f * APA_PI / 180.0f)));
    decision->DrFixAngFlg = 1;
    if (allowForwardFix0) {
      decision->InitialGoDir = 1;
      decision->InitialTrnD = signTheta0;
    } else {
      decision->InitialGoDir = -1;
      decision->InitialTrnD = -signTheta0;
    }
  }
}

static inline void geoHtMatlabMainInitDrFixResult(const GeoHtPathPlanningInput_t *input,
                                                  const GeoHtMatlabMainDrFixDecision_t *decision,
                                                  GeoHtMatlabMainDrFixResult_t *result) {
  result->PlanCmd = 0;
  result->FailRs = 0;
  result->DrFixAngFlg = decision->DrFixAngFlg;
  result->AddSegment = 0;
  result->SegmentGoDir = 0;
  result->SegmentTrnD = 0;
  result->PlnChkCnt = 0;
  result->SegmentCrv = 0.0f;
  result->SegmentTht = 0.0f;
  result->SegmentLen = 0.0f;
  result->SegmentStartX = input->VehX;
  result->SegmentStartY = input->VehY;
  result->SegmentStartTheta = input->VehTheta;
  result->SegmentCenterX = 0.0f;
  result->SegmentCenterY = 0.0f;
  result->PlnEndX = input->VehX;
  result->PlnEndY = input->VehY;
  result->PlnEndTheta = input->VehTheta;
}

static inline void geoHtMatlabMainWriteDrFixSuccess(const GeoHtPathPlanningInput_t *input,
                                                    const GeoHtMatlabMainDrFixDecision_t *decision,
                                                    Integer goDir,
                                                    Integer trnD,
                                                    Real safeLen,
                                                    GeoHtMatlabMainDrFixResult_t *result) {
  Real rd0;
  Real vehSin0;
  Real vehCos0;
  Real tarSin0;
  Real tarCos0;
  Real trnDReal0;
  Real centerX0;
  Real centerY0;
  Real endX0;
  Real endY0;

  rd0 = input->Rmin;
  if (rd0 <= GEO_EPSILON) {
    rd0 = 5.0f;
  }
  vehSin0 = sinf(input->VehTheta);
  vehCos0 = cosf(input->VehTheta);
  tarSin0 = sinf(input->TarTheta);
  tarCos0 = cosf(input->TarTheta);
  trnDReal0 = (Real)trnD;
  centerX0 = input->VehX - (trnDReal0 * rd0 * vehSin0);
  centerY0 = input->VehY + (trnDReal0 * rd0 * vehCos0);
  endX0 = centerX0 + (trnDReal0 * rd0 * tarSin0);
  endY0 = centerY0 - (trnDReal0 * rd0 * tarCos0);

  result->PlanCmd = 0;
  result->FailRs = 0;
  result->DrFixAngFlg = 3;
  result->AddSegment = 1;
  result->SegmentGoDir = goDir;
  result->SegmentTrnD = trnD;
  result->SegmentCrv = geoDivideScalar(trnDReal0, rd0);
  result->SegmentTht = decision->RelTheta;
  result->SegmentLen = safeLen;
  result->SegmentStartX = input->VehX;
  result->SegmentStartY = input->VehY;
  result->SegmentStartTheta = input->VehTheta;
  result->SegmentCenterX = centerX0;
  result->SegmentCenterY = centerY0;
  result->PlnEndX = endX0;
  result->PlnEndY = endY0;
  result->PlnEndTheta = input->TarTheta;
}

static inline void
geoHtMatlabMainApplyDrFixProbeDecision(const GeoHtPathPlanningInput_t *input,
                                       const GeoHtMatlabMainDrFixDecision_t *decision,
                                       Integer forwardIntrvFlg,
                                       Real forwardSafeLen,
                                       Integer reverseIntrvFlg,
                                       Real reverseSafeLen,
                                       GeoHtMatlabMainDrFixResult_t *result) {
  Integer signTheta0;
  Integer reverseTrnD0;
  Boolean runForward0;
  Boolean runReverse0;

  geoHtMatlabMainInitDrFixResult(input, decision, result);
  signTheta0 = 1;
  if (decision->RelTheta < 0.0f) {
    signTheta0 = -1;
  }
  reverseTrnD0 = -signTheta0;
  runForward0 = false;
  runReverse0 = false;

  if (decision->DrFixAngFlg != 0) {
    if (decision->InitialGoDir == 1) {
      runForward0 = true;
    } else {
      runReverse0 = true;
    }
  }

  if (runForward0) {
    if (forwardIntrvFlg == 2) {
      result->PlanCmd = 5;
      result->FailRs = 8;
    } else if (forwardIntrvFlg == 1) {
      result->DrFixAngFlg = 2;
      runReverse0 = true;
    } else {
      geoHtMatlabMainWriteDrFixSuccess(input, decision, 1, signTheta0, forwardSafeLen, result);
    }
  }

  if (runReverse0) {
    if (result->PlanCmd != 5) {
      if (reverseIntrvFlg == 2) {
        result->PlanCmd = 5;
        result->FailRs = 8;
      } else if (reverseIntrvFlg == 1) {
        result->PlanCmd = 5;
        result->FailRs = 7;
        result->DrFixAngFlg = 2;
      } else {
        geoHtMatlabMainWriteDrFixSuccess(input, decision, -1, reverseTrnD0, reverseSafeLen, result);
      }
    }
  }
}

static inline void
geoHtMatlabMainResolveInitialOffsetDecision(const GeoHtPathPlanningInput_t *input,
                                            Integer prkDir,
                                            Integer drFixAngFlg,
                                            Real plnEdX,
                                            Real plnEdY,
                                            Real plnEdTheta,
                                            Real leftSpace,
                                            Real rightSpace,
                                            GeoHtMatlabMainInitialOffsetDecision_t *decision) {
  Real relX0;
  Real relY0;
  Real relTheta0;
  Real spaceSum0;
  Boolean totalSpaceEnough0;
  Boolean rightSideTight0;
  Boolean leftSideTight0;
  Boolean desireOffset0;
  Real iniOfst0;
  Real thetaSin0;
  Real thetaCos0;
  Integer firstArcIndex0;
  Integer secondArcIndex0;

  geoAxisTransToTarget(plnEdX,
                       plnEdY,
                       plnEdTheta,
                       input->TarX,
                       input->TarY,
                       input->TarTheta,
                       &relX0,
                       &relY0,
                       &relTheta0);
  (void)relY0;
  (void)relTheta0;
  spaceSum0 = leftSpace + rightSpace;
  totalSpaceEnough0 = (spaceSum0 > 2.5f);
  rightSideTight0 = (rightSpace < 1.3f);
  leftSideTight0 = (leftSpace < 1.3f);
  desireOffset0 = false;
  iniOfst0 = 0.0f;

  if (prkDir > 0) {
    if (totalSpaceEnough0) {
      if (rightSideTight0) {
        desireOffset0 = true;
        iniOfst0 = 1.2f - rightSpace;
      }
    }
  } else {
    if (totalSpaceEnough0) {
      if (leftSideTight0) {
        desireOffset0 = true;
        iniOfst0 = -(1.2f - leftSpace);
      }
    }
  }

  firstArcIndex0 = 1;
  secondArcIndex0 = 2;
  if (drFixAngFlg == 3) {
    firstArcIndex0 = 2;
    secondArcIndex0 = 3;
  }

  thetaSin0 = sinf(plnEdTheta);
  thetaCos0 = cosf(plnEdTheta);
  decision->PlanCmd = 0;
  decision->FailRs = 0;
  decision->IniOfstDesir = 0;
  decision->IniOfstFlg = 0;
  decision->AddSegmentCount = 0;
  decision->FirstArcIndex = firstArcIndex0;
  decision->SecondArcIndex = secondArcIndex0;
  decision->PlnChkCnt = 0;
  decision->CheckLen = 6.0f - relX0;
  decision->IniOfst = 0.0f;
  decision->StartX = plnEdX;
  decision->StartY = plnEdY;
  decision->StartTheta = plnEdTheta;
  decision->TargetX = plnEdX;
  decision->TargetY = plnEdY;
  decision->TargetTheta = plnEdTheta;
  decision->PlnEndX = plnEdX;
  decision->PlnEndY = plnEdY;
  decision->PlnEndTheta = plnEdTheta;
  decision->FirstSafeLen = 0.0f;
  decision->SecondSafeLen = 0.0f;

  if (desireOffset0) {
    decision->IniOfstDesir = 1;
    decision->IniOfst = iniOfst0;
    decision->TargetX = plnEdX - (iniOfst0 * thetaSin0) + (2.0f * thetaCos0);
    decision->TargetY = plnEdY + (iniOfst0 * thetaCos0) + (2.0f * thetaSin0);
    decision->TargetTheta = plnEdTheta;
  }
}

static inline void
geoHtMatlabMainApplyInitialOffsetProbeDecision(GeoHtMatlabMainInitialOffsetDecision_t *decision,
                                               Integer linkFailFlg,
                                               Integer firstIntrvFlg,
                                               Integer secondIntrvFlg,
                                               Real firstSafeLen,
                                               Real secondSafeLen,
                                               const Geo2CircleLinkHTOutput_t *link) {
  Boolean desireOffset0;
  Boolean linkOk0;
  Boolean firstArcOk0;
  Boolean secondArcOk0;
  Boolean offsetComplete0;

  desireOffset0 = (decision->IniOfstDesir == 1);
  linkOk0 = (linkFailFlg == 0);
  firstArcOk0 = (firstIntrvFlg == 0);
  secondArcOk0 = (secondIntrvFlg == 0);
  offsetComplete0 = false;
  if (desireOffset0) {
    if (linkOk0) {
      if (firstArcOk0) {
        if (secondArcOk0) {
          offsetComplete0 = true;
        }
      }
    }
  }

  decision->IniOfstFlg = 0;
  decision->AddSegmentCount = 0;
  decision->FirstSafeLen = 0.0f;
  decision->SecondSafeLen = 0.0f;
  if (offsetComplete0) {
    decision->IniOfstFlg = 1;
    decision->AddSegmentCount = 2;
    decision->FirstSafeLen = firstSafeLen;
    decision->SecondSafeLen = secondSafeLen;

    decision->PlnEndX = link->St3X;
    decision->PlnEndY = link->St3Y;
    decision->PlnEndTheta = link->St3Theta;
  }
}

static void geoHtMatlabMainBuildStPtSpcInput(const GeoHtPathPlanningInput_t *input,
                                             const GeoHtPathPlanningParam_t *param,
                                             Real vehX,
                                             Real vehY,
                                             Real vehTheta,
                                             Real checkLen,
                                             GeoStPtSpcVtInput_t *spcInput) {
  *spcInput = (GeoStPtSpcVtInput_t){0};
  spcInput->VehPosX = vehX;
  spcInput->VehPosY = vehY;
  spcInput->VehPosTheta = vehTheta;
  spcInput->TarPtX = input->TarX;
  spcInput->TarPtY = input->TarY;
  spcInput->TarPtTheta = input->TarTheta;
  spcInput->SltPt1X = input->SltPt1X;
  spcInput->SltPt1Y = input->SltPt1Y;
  spcInput->SltPt2X = input->SltPt2X;
  spcInput->SltPt2Y = input->SltPt2Y;
  spcInput->SltPt3X = input->SltPt3X;
  spcInput->SltPt3Y = input->SltPt3Y;
  spcInput->SltPt4X = input->SltPt4X;
  spcInput->SltPt4Y = input->SltPt4Y;
  spcInput->SltEntr = input->SltEntr;
  spcInput->CheckLen = checkLen;
  spcInput->SltType = VCC_SLOT_CLASS_PARALLEL;
  spcInput->FallbackSpace = 4.1f;
  spcInput->UseCollisionCheck = true;
  geoHtMatlabMainBuildTargetPrecheckCollisionInput(
      input, param, vehX, vehY, vehTheta, &spcInput->VccTemplate);
}

static void geoHtMatlabMainProbeDrFixArc(const GeoHtPathPlanningInput_t *input,
                                         const GeoHtPathPlanningParam_t *param,
                                         Integer goDir,
                                         Integer trnD,
                                         GeoCircleIntrvCheckOutput_t *arcOutput) {
  GeoCircleIntrvCheckInput_t arcInput;
  Real rd0;
  Real trnDReal0;
  Real vehSin0;
  Real vehCos0;
  Real tarSin0;
  Real tarCos0;
  Real centerX0;
  Real centerY0;
  Real endX0;
  Real endY0;

  rd0 = input->Rmin;
  if (rd0 <= GEO_EPSILON) {
    rd0 = 5.0f;
  }
  trnDReal0 = (Real)trnD;
  vehSin0 = sinf(input->VehTheta);
  vehCos0 = cosf(input->VehTheta);
  tarSin0 = sinf(input->TarTheta);
  tarCos0 = cosf(input->TarTheta);
  centerX0 = input->VehX - (trnDReal0 * rd0 * vehSin0);
  centerY0 = input->VehY + (trnDReal0 * rd0 * vehCos0);
  endX0 = centerX0 + (trnDReal0 * rd0 * tarSin0);
  endY0 = centerY0 - (trnDReal0 * rd0 * tarCos0);
  geoHtFillCircleInput(input,
                       param,
                       centerX0,
                       centerY0,
                       rd0,
                       input->VehX,
                       input->VehY,
                       input->VehTheta,
                       endX0,
                       endY0,
                       input->TarTheta,
                       goDir,
                       trnD,
                       0.1f,
                       &arcInput);
  *arcOutput = (GeoCircleIntrvCheckOutput_t){0};
  geoCircleIntrvCheckWithScene(&arcInput, &param->SceneQuery, arcOutput);
}

static void geoHtMatlabMainWriteDrFixPrefixSegment(const GeoHtPathPlanningInput_t *input,
                                                   const GeoHtMatlabMainDrFixResult_t *drFixResult,
                                                   HtSegWorkspace_t *ws) {
  Real rd0;

  rd0 = input->Rmin;
  if (rd0 <= GEO_EPSILON) {
    rd0 = 5.0f;
  }
  if (drFixResult->AddSegment != 0) {
    geoHtWriteArcSeg(ws,
                     0,
                     drFixResult->SegmentStartX,
                     drFixResult->SegmentStartY,
                     drFixResult->SegmentStartTheta,
                     drFixResult->SegmentGoDir,
                     drFixResult->SegmentTrnD,
                     rd0,
                     drFixResult->SegmentTht,
                     drFixResult->SegmentLen,
                     drFixResult->SegmentCenterX,
                     drFixResult->SegmentCenterY);
  }
}

static void geoHtMatlabMainRunDrFixRuntime(const GeoHtPathPlanningInput_t *input,
                                           const GeoHtPathPlanningParam_t *param,
                                           HtSegWorkspace_t *ws,
                                           GeoHtMatlabMainDrFixResult_t *drFixResult,
                                           GeoHtMatlabMainLoopPose_t *loopEndPose) {
  GeoHtMatlabMainDrFixDecision_t drFixDecision;
  GeoCircleIntrvCheckOutput_t forwardOutput;
  GeoCircleIntrvCheckOutput_t reverseOutput;
  Integer turnSign0;
  Integer reverseTurn0;

  drFixDecision = (GeoHtMatlabMainDrFixDecision_t){0};
  forwardOutput = (GeoCircleIntrvCheckOutput_t){0};
  reverseOutput = (GeoCircleIntrvCheckOutput_t){0};
  geoHtMatlabMainResolveDrFixDecision(input, false, &drFixDecision);
  turnSign0 = apaMathSign(drFixDecision.RelTheta);
  if (turnSign0 == 0) {
    turnSign0 = 1;
  }
  reverseTurn0 = -turnSign0;
  if (drFixDecision.DrFixAngFlg != 0) {
    geoHtMatlabMainProbeDrFixArc(input, param, 1, turnSign0, &forwardOutput);
    geoHtMatlabMainProbeDrFixArc(input, param, -1, reverseTurn0, &reverseOutput);
    ws->PlnChkCnt = ws->PlnChkCnt + forwardOutput.CheckCount;
    ws->PlnChkCnt = ws->PlnChkCnt + reverseOutput.CheckCount;
  }
  geoHtMatlabMainApplyDrFixProbeDecision(input,
                                         &drFixDecision,
                                         forwardOutput.IntrvFlg,
                                         forwardOutput.SafeLen,
                                         reverseOutput.IntrvFlg,
                                         reverseOutput.SafeLen,
                                         drFixResult);
  if (drFixResult->PlanCmd == 5) {
    ws->PlanCmd = 5;
    ws->FailRs = drFixResult->FailRs;
  }
  if (drFixResult->AddSegment != 0) {
    geoHtMatlabMainWriteDrFixPrefixSegment(input, drFixResult, ws);
    loopEndPose->X = drFixResult->PlnEndX;
    loopEndPose->Y = drFixResult->PlnEndY;
    loopEndPose->Theta = drFixResult->PlnEndTheta;
  }
}

static void
geoHtMatlabMainProbeInitialOffsetLink(const GeoHtPathPlanningInput_t *input,
                                      const GeoHtPathPlanningParam_t *param,
                                      const GeoHtMatlabMainInitialOffsetDecision_t *offsetDecision,
                                      Geo2CircleLinkHTInput_t *linkInput,
                                      Geo2CircleLinkHTOutput_t *linkOutput,
                                      GeoCircleIntrvCheckOutput_t *firstOutput,
                                      GeoCircleIntrvCheckOutput_t *secondOutput,
                                      Integer *linkFailFlg,
                                      Integer *firstIntrvFlg,
                                      Integer *secondIntrvFlg,
                                      HtSegWorkspace_t *ws) {
  GeoCircleIntrvCheckInput_t arcInput;
  Integer turnDir0;
  Boolean canProbe0;
  Boolean linkOk0;

  *linkInput = (Geo2CircleLinkHTInput_t){0};
  *linkOutput = (Geo2CircleLinkHTOutput_t){0};
  *firstOutput = (GeoCircleIntrvCheckOutput_t){0};
  *secondOutput = (GeoCircleIntrvCheckOutput_t){0};
  *linkFailFlg = 1;
  *firstIntrvFlg = 1;
  *secondIntrvFlg = 1;
  turnDir0 = apaMathSign(offsetDecision->IniOfst);
  canProbe0 = false;
  if (offsetDecision->IniOfstDesir == 1) {
    if (turnDir0 != 0) {
      if (input->Rmin > GEO_EPSILON) {
        canProbe0 = true;
      }
    }
  }

  if (canProbe0) {
    linkInput->Pt1X = offsetDecision->StartX;
    linkInput->Pt1Y = offsetDecision->StartY;
    linkInput->Pt1Theta = offsetDecision->StartTheta;
    linkInput->Pt2X = offsetDecision->TargetX;
    linkInput->Pt2Y = offsetDecision->TargetY;
    linkInput->Pt2Theta = offsetDecision->TargetTheta;
    linkInput->Dir = turnDir0;
    linkInput->R1 = input->Rmin;
    linkInput->R2 = input->Rmin;
    geo2CircleLinkHT(linkInput, linkOutput);
    *linkFailFlg = linkOutput->FailFlag;
  }

  linkOk0 = (*linkFailFlg == 0);
  if (linkOk0) {
    geoHtFillCircleInput(input,
                         param,
                         linkOutput->Ct1X,
                         linkOutput->Ct1Y,
                         input->Rmin,
                         linkInput->Pt1X,
                         linkInput->Pt1Y,
                         linkInput->Pt1Theta,
                         linkOutput->St2X,
                         linkOutput->St2Y,
                         linkOutput->St2Theta,
                         1,
                         linkOutput->TrnD1,
                         0.2f,
                         &arcInput);
    geoCircleIntrvCheckWithScene(&arcInput, &param->SceneQuery, firstOutput);
    ws->PlnChkCnt = ws->PlnChkCnt + firstOutput->CheckCount;
    *firstIntrvFlg = firstOutput->IntrvFlg;
  }

  if (*firstIntrvFlg == 0) {
    geoHtFillCircleInput(input,
                         param,
                         linkOutput->Ct2X,
                         linkOutput->Ct2Y,
                         input->Rmin,
                         linkOutput->St2X,
                         linkOutput->St2Y,
                         linkOutput->St2Theta,
                         linkOutput->St3X,
                         linkOutput->St3Y,
                         linkOutput->St3Theta,
                         1,
                         linkOutput->TrnD2,
                         0.2f,
                         &arcInput);
    geoCircleIntrvCheckWithScene(&arcInput, &param->SceneQuery, secondOutput);
    ws->PlnChkCnt = ws->PlnChkCnt + secondOutput->CheckCount;
    *secondIntrvFlg = secondOutput->IntrvFlg;
  }
}

static void geoHtMatlabMainWriteInitialOffsetPrefix(
    const GeoHtPathPlanningInput_t *input,
    const GeoHtMatlabMainInitialOffsetDecision_t *offsetDecision,
    const Geo2CircleLinkHTInput_t *linkInput,
    const Geo2CircleLinkHTOutput_t *linkOutput,
    HtSegWorkspace_t *ws) {
  Integer firstIndex0;
  Integer secondIndex0;
  Real firstLen0;
  Real secondLen0;

  if (offsetDecision->IniOfstFlg == 1) {
    firstIndex0 = offsetDecision->FirstArcIndex - 1;
    secondIndex0 = offsetDecision->SecondArcIndex - 1;
    firstIndex0 = apaClampCount(firstIndex0, HT_SEG_MAX - 1);
    secondIndex0 = apaClampCount(secondIndex0, HT_SEG_MAX - 1);
    firstLen0 = apaMathResolvePositiveOrFallback(offsetDecision->FirstSafeLen, linkOutput->Len1);
    secondLen0 = apaMathResolvePositiveOrFallback(offsetDecision->SecondSafeLen, linkOutput->Len2);
    geoHtWriteArcSeg(ws,
                     firstIndex0,
                     linkInput->Pt1X,
                     linkInput->Pt1Y,
                     linkInput->Pt1Theta,
                     1,
                     linkOutput->TrnD1,
                     input->Rmin,
                     linkOutput->St2Theta - linkInput->Pt1Theta,
                     firstLen0,
                     linkOutput->Ct1X,
                     linkOutput->Ct1Y);
    geoHtWriteArcSeg(ws,
                     secondIndex0,
                     linkOutput->St2X,
                     linkOutput->St2Y,
                     linkOutput->St2Theta,
                     1,
                     linkOutput->TrnD2,
                     input->Rmin,
                     linkOutput->St3Theta - linkOutput->St2Theta,
                     secondLen0,
                     linkOutput->Ct2X,
                     linkOutput->Ct2Y);
  }
}

static void geoHtMatlabMainRunInitialOffsetRuntime(
    const GeoHtPathPlanningInput_t *input,
    const GeoHtPathPlanningParam_t *param,
    const GeoHtMatlabMainNormalEntryDecision_t *normalEntryDecision,
    const GeoHtMatlabMainDrFixResult_t *drFixResult,
    HtSegWorkspace_t *ws,
    GeoHtMatlabMainInitialOffsetDecision_t *offsetDecision,
    GeoHtMatlabMainLoopPose_t *loopEndPose) {
  GeoHtMatlabMainInitialOffsetDecision_t checkDecision;
  GeoStPtSpcVtInput_t spcInput;
  GeoStPtSpcVtOutput_t spcOutput;
  Geo2CircleLinkHTInput_t linkInput;
  Geo2CircleLinkHTOutput_t linkOutput;
  GeoCircleIntrvCheckOutput_t firstOutput;
  GeoCircleIntrvCheckOutput_t secondOutput;
  Integer linkFailFlg;
  Integer firstIntrvFlg;
  Integer secondIntrvFlg;

  checkDecision = (GeoHtMatlabMainInitialOffsetDecision_t){0};
  spcInput = (GeoStPtSpcVtInput_t){0};
  spcOutput = (GeoStPtSpcVtOutput_t){0};
  geoHtMatlabMainResolveInitialOffsetDecision(input,
                                              normalEntryDecision->PrkDir,
                                              drFixResult->DrFixAngFlg,
                                              loopEndPose->X,
                                              loopEndPose->Y,
                                              loopEndPose->Theta,
                                              4.1f,
                                              4.1f,
                                              &checkDecision);
  geoHtMatlabMainBuildStPtSpcInput(input,
                                   param,
                                   loopEndPose->X,
                                   loopEndPose->Y,
                                   loopEndPose->Theta,
                                   checkDecision.CheckLen,
                                   &spcInput);
  geoStPtSpcVtWithScene(&spcInput, &param->SceneQuery, &spcOutput);
  ws->PlnChkCnt = ws->PlnChkCnt + spcOutput.CheckCount;
  geoHtMatlabMainResolveInitialOffsetDecision(input,
                                              normalEntryDecision->PrkDir,
                                              drFixResult->DrFixAngFlg,
                                              loopEndPose->X,
                                              loopEndPose->Y,
                                              loopEndPose->Theta,
                                              spcOutput.LeftSpace,
                                              spcOutput.RightSpace,
                                              offsetDecision);
  geoHtMatlabMainProbeInitialOffsetLink(input,
                                        param,
                                        offsetDecision,
                                        &linkInput,
                                        &linkOutput,
                                        &firstOutput,
                                        &secondOutput,
                                        &linkFailFlg,
                                        &firstIntrvFlg,
                                        &secondIntrvFlg,
                                        ws);
  geoHtMatlabMainApplyInitialOffsetProbeDecision(offsetDecision,
                                                 linkFailFlg,
                                                 firstIntrvFlg,
                                                 secondIntrvFlg,
                                                 firstOutput.SafeLen,
                                                 secondOutput.SafeLen,
                                                 &linkOutput);
  if (offsetDecision->IniOfstFlg == 1) {
    geoHtMatlabMainWriteInitialOffsetPrefix(input, offsetDecision, &linkInput, &linkOutput, ws);
    loopEndPose->X = offsetDecision->PlnEndX;
    loopEndPose->Y = offsetDecision->PlnEndY;
    loopEndPose->Theta = offsetDecision->PlnEndTheta;
  }
}

static void
geoHtMatlabMainRunPrefixRuntime(const GeoHtPathPlanningInput_t *input,
                                const GeoHtPathPlanningParam_t *param,
                                const GeoHtMatlabMainNormalEntryDecision_t *normalEntryDecision,
                                HtSegWorkspace_t *ws,
                                Integer *iniSegN,
                                GeoHtMatlabMainLoopPose_t *loopEndPose) {
  GeoHtMatlabMainDrFixResult_t drFixResult;
  GeoHtMatlabMainInitialOffsetDecision_t offsetDecision;

  *iniSegN = 0;
  loopEndPose->X = input->VehX;
  loopEndPose->Y = input->VehY;
  loopEndPose->Theta = input->VehTheta;
  drFixResult = (GeoHtMatlabMainDrFixResult_t){0};
  offsetDecision = (GeoHtMatlabMainInitialOffsetDecision_t){0};
  geoHtMatlabMainRunDrFixRuntime(input, param, ws, &drFixResult, loopEndPose);
  if (ws->PlanCmd != 5) {
    geoHtMatlabMainRunInitialOffsetRuntime(
        input, param, normalEntryDecision, &drFixResult, ws, &offsetDecision, loopEndPose);
    *iniSegN = geoHtMatlabMainResolveIniSegN(drFixResult.DrFixAngFlg, offsetDecision.IniOfstFlg);
  }
}

static inline Boolean
geoHtMatlabMainHasStopperBarrier(Real barrier1, Real barrier2, Real barrier3, Real barrier4) {
  Boolean hasBarrier0;

  hasBarrier0 = false;
  if (fabsf(barrier1) > 0.01f) {
    hasBarrier0 = true;
  }
  if (fabsf(barrier2) > 0.01f) {
    hasBarrier0 = true;
  }
  if (fabsf(barrier3) > 0.01f) {
    hasBarrier0 = true;
  }
  if (fabsf(barrier4) > 0.01f) {
    hasBarrier0 = true;
  }
  return hasBarrier0;
}

static inline void geoHtMatlabMainInitPt1Decision(const GeoHtPathPlanningInput_t *input,
                                                  GeoHtMatlabMainPt1Decision_t *decision) {
  decision->PlanCmd = 0;
  decision->FailRs = 0;
  decision->Stp1Done = 0;
  decision->HasStopperBarrier = 0;
  decision->NeedsRearProbe = 0;
  decision->AddSegment = 0;
  decision->SegmentGoDir = 0;
  decision->DangerFlag = 0;
  decision->PlnChkCnt = 0;
  decision->Pt1X = input->TarX;
  decision->Pt1Y = input->TarY;
  decision->Pt1Theta = input->TarTheta;
  decision->DangerX = 0.0f;
  decision->DangerY = 0.0f;
  decision->SegmentStartX = 0.0f;
  decision->SegmentStartY = 0.0f;
  decision->SegmentStartTheta = 0.0f;
  decision->SegmentLen = 0.0f;
  decision->FrontSafeLen = 0.0f;
  decision->RearSafeLen = 0.0f;
}

static inline void geoHtMatlabMainSetPt1ReverseNudge(const GeoHtPathPlanningInput_t *input,
                                                     GeoHtMatlabMainPt1Decision_t *decision) {
  Real segmentLen0;

  segmentLen0 = 0.25f;
  decision->AddSegment = 1;
  decision->SegmentGoDir = -1;
  decision->SegmentLen = segmentLen0;
  decision->SegmentStartX = input->TarX + (segmentLen0 * cosf(input->TarTheta));
  decision->SegmentStartY = input->TarY + (segmentLen0 * sinf(input->TarTheta));
  decision->SegmentStartTheta = input->TarTheta;
  decision->Pt1X = decision->SegmentStartX;
  decision->Pt1Y = decision->SegmentStartY;
  decision->Pt1Theta = decision->SegmentStartTheta;
}

static inline void geoHtMatlabMainSetPt1RearProbeSegment(Real rearSafeLen,
                                                         Real rearAvlX,
                                                         Real rearAvlY,
                                                         Real rearAvlTheta,
                                                         GeoHtMatlabMainPt1Decision_t *decision) {
  decision->AddSegment = 1;
  decision->SegmentGoDir = 1;
  decision->SegmentLen = rearSafeLen;
  decision->SegmentStartX = rearAvlX;
  decision->SegmentStartY = rearAvlY;
  decision->SegmentStartTheta = rearAvlTheta;
  decision->Pt1X = rearAvlX;
  decision->Pt1Y = rearAvlY;
  decision->Pt1Theta = rearAvlTheta;
}

static inline void geoHtMatlabMainApplyPt1ProbeDecision(const GeoHtPathPlanningInput_t *input,
                                                        Integer stp1Done,
                                                        Boolean hasStopperBarrier,
                                                        Integer frontIntrvFlg,
                                                        Real frontSafeLen,
                                                        Real frontAvlX,
                                                        Real frontAvlY,
                                                        Real frontAvlTheta,
                                                        Integer rearIntrvFlg,
                                                        Real rearSafeLen,
                                                        Real rearAvlX,
                                                        Real rearAvlY,
                                                        Real rearAvlTheta,
                                                        GeoHtMatlabMainPt1Decision_t *decision) {
  Boolean stopperOrDone0;
  Boolean frontShort0;

  (void)frontAvlX;
  (void)frontAvlY;
  (void)frontAvlTheta;
  geoHtMatlabMainInitPt1Decision(input, decision);
  decision->Stp1Done = stp1Done;
  if (hasStopperBarrier) {
    decision->HasStopperBarrier = 1;
  }
  decision->FrontSafeLen = frontSafeLen;
  decision->RearSafeLen = rearSafeLen;
  stopperOrDone0 = hasStopperBarrier;
  if (stp1Done != 0) {
    stopperOrDone0 = true;
  }

  if (frontIntrvFlg == 2) {
    decision->PlanCmd = 5;
    decision->FailRs = 8;
  } else {
    if (stopperOrDone0) {
      frontShort0 = false;
      if (frontIntrvFlg == 1) {
        if (frontSafeLen < 0.7f) {
          frontShort0 = true;
        }
      }
      if (!frontShort0) {
        geoHtMatlabMainSetPt1ReverseNudge(input, decision);
      }
    } else {
      if (frontIntrvFlg == 1) {
        decision->NeedsRearProbe = 1;
        if (rearIntrvFlg == 2) {
          decision->PlanCmd = 5;
          decision->FailRs = 8;
        } else {
          geoHtMatlabMainSetPt1RearProbeSegment(
              rearSafeLen, rearAvlX, rearAvlY, rearAvlTheta, decision);
        }
      }
    }
  }
}

static void geoHtFillLineInput(const GeoHtPathPlanningInput_t *input,
                               const GeoHtPathPlanningParam_t *param,
                               Real stX,
                               Real stY,
                               Real stTheta,
                               Real edX,
                               Real edY,
                               Real edTheta,
                               Real checkStep,
                               GeoLineIntrvCheckInput_t *lineInput) {
  Integer idx;
  Integer lineCount;
  Integer virtualLineCount;
  Integer sepPointCount;
  Integer stopperPointCount;
  Integer trackStopperCount;

  *lineInput = (GeoLineIntrvCheckInput_t){0};
  lineInput->UsePlanningCheck = true;
  lineInput->SlotClass = VCC_SLOT_CLASS_PARALLEL;
  lineInput->DisableSlotBoundaryFallback = true;
  lineInput->Pt1X = stX;
  lineInput->Pt1Y = stY;
  lineInput->Pt1Theta = stTheta;
  lineInput->Pt2X = edX;
  lineInput->Pt2Y = edY;
  lineInput->Pt2Theta = edTheta;
  lineInput->CheckStep = checkStep;
  lineInput->GoalX = input->TarX;
  lineInput->GoalY = input->TarY;
  lineInput->GoalYaw = input->TarTheta;
  lineInput->SlotP1X = input->SltPt1X;
  lineInput->SlotP1Y = input->SltPt1Y;
  lineInput->SlotP2X = input->SltPt2X;
  lineInput->SlotP2Y = input->SltPt2Y;
  lineInput->SlotP3X = input->SltPt3X;
  lineInput->SlotP3Y = input->SltPt3Y;
  lineInput->SlotP4X = input->SltPt4X;
  lineInput->SlotP4Y = input->SltPt4Y;
  apaSlotOrderFromEntrance(input->SltEntr,
                           &lineInput->SlotP1X,
                           &lineInput->SlotP1Y,
                           &lineInput->SlotP2X,
                           &lineInput->SlotP2Y,
                           &lineInput->SlotP3X,
                           &lineInput->SlotP3Y,
                           &lineInput->SlotP4X,
                           &lineInput->SlotP4Y);
  lineInput->VehWidth = param->VehW;
  lineInput->VehFrontOffset = geoHtResolveVehFrontOffset(param);
  lineInput->VehRearOffset = geoHtResolveVehRearOffset(param);
  lineCount = input->BoundaryLineNum;
  if (lineCount > GEO_BOUNDARY_LINE_MAX) {
    lineCount = GEO_BOUNDARY_LINE_MAX;
  }
  if (lineCount < 0) {
    lineCount = 0;
  }
  lineInput->BoundaryLineNum = lineCount;
  idx = 0;
  while (idx < lineCount) {
    lineInput->BoundaryLineX1[idx] = input->BoundaryLineX1[idx];
    lineInput->BoundaryLineY1[idx] = input->BoundaryLineY1[idx];
    lineInput->BoundaryLineX2[idx] = input->BoundaryLineX2[idx];
    lineInput->BoundaryLineY2[idx] = input->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  virtualLineCount = input->VirtualBoundaryLineNum;
  if (virtualLineCount > GEO_BOUNDARY_LINE_MAX) {
    virtualLineCount = GEO_BOUNDARY_LINE_MAX;
  }
  if (virtualLineCount > VCC_MAX_VIRTUAL_LINES) {
    virtualLineCount = VCC_MAX_VIRTUAL_LINES;
  }
  if (virtualLineCount < 0) {
    virtualLineCount = 0;
  }
  lineInput->VirtualBoundaryLineNum = virtualLineCount;
  idx = 0;
  while (idx < virtualLineCount) {
    lineInput->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    lineInput->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    lineInput->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    lineInput->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }

  sepPointCount = input->BoundarySepPtNum;
  if (sepPointCount > VCC_MAX_SEPARATE_PTS) {
    sepPointCount = VCC_MAX_SEPARATE_PTS;
  }
  if (sepPointCount < 0) {
    sepPointCount = 0;
  }
  lineInput->BoundarySepPtNum = sepPointCount;
  idx = 0;
  while (idx < sepPointCount) {
    lineInput->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    lineInput->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  stopperPointCount = input->StopperPtNum;
  if (stopperPointCount > VCC_MAX_SEPARATE_PTS) {
    stopperPointCount = VCC_MAX_SEPARATE_PTS;
  }
  if (stopperPointCount < 0) {
    stopperPointCount = 0;
  }
  lineInput->SeparateStopperNum = stopperPointCount;
  idx = 0;
  while (idx < stopperPointCount) {
    lineInput->SeparateStopperX[idx] = input->StopperPtX[idx];
    lineInput->SeparateStopperY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }

  trackStopperCount = apaClampCountToLimits(
      input->SlotTrackStopperNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_TRACK_STOPPER);
  lineInput->SlotTrackStopperNum = trackStopperCount;
  idx = 0;
  while (idx < trackStopperCount) {
    lineInput->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    lineInput->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    lineInput->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    lineInput->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    idx = idx + 1;
  }
}

static void geoHtMatlabMainRunLineProbe(const GeoHtPathPlanningInput_t *input,
                                        const GeoHtPathPlanningParam_t *param,
                                        Real stX,
                                        Real stY,
                                        Real stTheta,
                                        Real edX,
                                        Real edY,
                                        Real edTheta,
                                        GeoLineIntrvCheckOutput_t *lineOutput) {
  GeoLineIntrvCheckInput_t lineInput;

  geoHtFillLineInput(input, param, stX, stY, stTheta, edX, edY, edTheta, 0.8f, &lineInput);
  *lineOutput = (GeoLineIntrvCheckOutput_t){0};
  geoLineIntrvCheckWithScene(&lineInput, &param->SceneQuery, lineOutput);
}

static inline Integer geoHtMatlabMainResolvePrkTypeFromRelY(Real relY) {
  Integer prkType0;

  prkType0 = 1;
  if (relY > 0.0f) {
    prkType0 = 2;
  }
  return prkType0;
}

static inline Integer geoHtMatlabMainResolveStp1Done(const GeoHtPathPlanningInput_t *input,
                                                     const GeoHtPathPlanningParam_t *param,
                                                     Real relY,
                                                     Integer dangerFlag,
                                                     Real dangerX,
                                                     Real dangerY) {
  Real latSafeOffset0;
  Real rfout0;
  Real centerX0;
  Real centerY0;
  Real deltaX0;
  Real deltaY0;
  Real distSq0;
  Real limit0;
  Real limitSq0;
  Integer stp1Done0;

  stp1Done0 = 1;
  if (dangerFlag > 0) {
    latSafeOffset0 = 0.6f;
    if (dangerFlag == 2) {
      latSafeOffset0 = 0.0f;
    }
    rfout0 = param->RfoutMin;
    if (rfout0 <= GEO_EPSILON) {

      rfout0 = hypotf(param->VehLlF, input->Rmin + param->VehW * 0.5f) - 0.2f;
    }
    if (relY > 0.0f) {
      centerX0 = input->TarX - (input->Rmin * sinf(input->TarTheta));
      centerY0 = input->TarY + (input->Rmin * cosf(input->TarTheta));
    } else {
      centerX0 = input->TarX + (input->Rmin * sinf(input->TarTheta));
      centerY0 = input->TarY - (input->Rmin * cosf(input->TarTheta));
    }
    deltaX0 = centerX0 - dangerX;
    deltaY0 = centerY0 - dangerY;
    distSq0 = (deltaX0 * deltaX0) + (deltaY0 * deltaY0);
    limit0 = rfout0 + latSafeOffset0;
    limitSq0 = limit0 * limit0;
    if (distSq0 < limitSq0) {
      stp1Done0 = 0;
    }
  }
  return stp1Done0;
}

static void geoHtMatlabMainResolvePt1Decision(const GeoHtPathPlanningInput_t *input,
                                              const GeoHtPathPlanningParam_t *param,
                                              GeoHtMatlabMainPt1Decision_t *decision) {
  GeoHtDangerPtInput_t dangerInput;
  GeoHtDangerPtOutput_t dangerOutput;
  GeoLineIntrvCheckOutput_t frontOutput;
  GeoLineIntrvCheckOutput_t rearOutput;
  Real relX0;
  Real relY0;
  Real relTheta0;
  Real frontEndX0;
  Real frontEndY0;
  Real rearDist0;
  Real rearEndX0;
  Real rearEndY0;
  Integer prkType0;
  Integer stp1Done0;
  Boolean hasStopperBarrier0;
  Boolean needsRearProbe0;

  geoHtMatlabMainInitPt1Decision(input, decision);
  geoAxisTransToTarget(input->VehX,
                       input->VehY,
                       input->VehTheta,
                       input->TarX,
                       input->TarY,
                       input->TarTheta,
                       &relX0,
                       &relY0,
                       &relTheta0);
  (void)relX0;
  (void)relTheta0;
  prkType0 = geoHtMatlabMainResolvePrkTypeFromRelY(relY0);
  geoHtFillDangerInput(
      input, param, prkType0, input->TarX, input->TarY, input->TarTheta, &dangerInput);

  dangerOutput = (GeoHtDangerPtOutput_t){0};
  geoHtDangerPt(&dangerInput, &dangerOutput);
  stp1Done0 = geoHtMatlabMainResolveStp1Done(
      input, param, relY0, dangerOutput.PtEx, dangerOutput.DangerPtX, dangerOutput.DangerPtY);
  hasStopperBarrier0 = geoHtMatlabMainHasStopperBarrier(input->StopperBarrier1,
                                                        input->StopperBarrier2,
                                                        input->StopperBarrier3,
                                                        input->StopperBarrier4);
  frontEndX0 = input->TarX + (2.55f * cosf(input->TarTheta));
  frontEndY0 = input->TarY + (2.55f * sinf(input->TarTheta));
  geoHtMatlabMainRunLineProbe(input,
                              param,
                              input->TarX,
                              input->TarY,
                              input->TarTheta,
                              frontEndX0,
                              frontEndY0,
                              input->TarTheta,
                              &frontOutput);
  rearOutput = (GeoLineIntrvCheckOutput_t){0};
  needsRearProbe0 = false;
  if (!hasStopperBarrier0) {
    if (stp1Done0 == 0) {
      if (frontOutput.IntrvFlg == 1) {
        needsRearProbe0 = true;
      }
    }
  }
  if (needsRearProbe0) {
    rearDist0 = 2.55f - frontOutput.SafeLen;
    if (rearDist0 < 0.0f) {
      rearDist0 = 0.0f;
    }
    rearEndX0 = input->TarX - (rearDist0 * cosf(input->TarTheta));
    rearEndY0 = input->TarY - (rearDist0 * sinf(input->TarTheta));
    geoHtMatlabMainRunLineProbe(input,
                                param,
                                input->TarX,
                                input->TarY,
                                input->TarTheta,
                                rearEndX0,
                                rearEndY0,
                                input->TarTheta,
                                &rearOutput);
  }
  geoHtMatlabMainApplyPt1ProbeDecision(input,
                                       stp1Done0,
                                       hasStopperBarrier0,
                                       frontOutput.IntrvFlg,
                                       frontOutput.SafeLen,
                                       frontOutput.AvlPosX,
                                       frontOutput.AvlPosY,
                                       frontOutput.AvlPosTheta,
                                       rearOutput.IntrvFlg,
                                       rearOutput.SafeLen,
                                       rearOutput.AvlPosX,
                                       rearOutput.AvlPosY,
                                       rearOutput.AvlPosTheta,
                                       decision);
  decision->DangerFlag = dangerOutput.PtEx;
  decision->DangerX = dangerOutput.DangerPtX;
  decision->DangerY = dangerOutput.DangerPtY;
  decision->PlnChkCnt = frontOutput.CheckCount + rearOutput.CheckCount;
}

static void geoHtWriteReverseArcFromPose(HtSegWorkspace_t *ws,
                                         const GeoHtPathPlanningInput_t *input,
                                         Integer idx,
                                         Real poseX,
                                         Real poseY,
                                         Real poseTheta,
                                         Integer goDir,
                                         Integer trnD,
                                         Real rd,
                                         Real len,
                                         Real ccX,
                                         Real ccY) {
  Real nextTheta;
  Real turnTheta;

  if (ws->SegNum > 0) {
    nextTheta = ws->StTh[idx + 1];
  } else {
    nextTheta = input->TarTheta;
  }
  turnTheta = nextTheta - poseTheta;
  geoHtWriteArcSeg(ws, idx, poseX, poseY, poseTheta, goDir, trnD, rd, turnTheta, len, ccX, ccY);
}

static Boolean geoHtMatlabMainHasTailCapacity(const HtSegWorkspace_t *ws) {
  return ws->SegNum < HT_SEG_MAX;
}

static void geoHtMatlabMainSetCapacityFail(HtSegWorkspace_t *ws) {
  ws->PlanCmd = 5;
  ws->FailRs = 9;
}

static void geoHtMatlabMainSetContinueAfterTailRecord(HtSegWorkspace_t *ws) {
  if (ws->PlanCmd != 5) {
    if (ws->SegNum >= HT_SEG_MAX) {
      geoHtMatlabMainSetCapacityFail(ws);
    } else {
      ws->PlanCmd = 2;
    }
  }
}

static void geoHtMatlabMainSetSuccessAfterTailRecord(HtSegWorkspace_t *ws) {
  if (ws->PlanCmd != 5) {
    ws->PlanCmd = 1;
  }
}

static void geoHtExternalRecordArc(HtSegWorkspace_t *ws,
                                   const GeoHtPathPlanningInput_t *input,
                                   Real poseX,
                                   Real poseY,
                                   Real poseTheta,
                                   Integer goDir,
                                   Integer trnD,
                                   Real rd,
                                   Real len,
                                   Real ccX,
                                   Real ccY) {
  Integer idx;

  if (!isfinite(len)) {
    ws->PlanCmd = 5;
    ws->FailRs = 8;
    return;
  }

  if (!apaPlanningMotionHasProgress(len))
    return;
  if (geoHtMatlabMainHasTailCapacity(ws)) {
    idx = HT_SEG_MAX - ws->SegNum - 1;
    geoHtWriteReverseArcFromPose(
        ws, input, idx, poseX, poseY, poseTheta, goDir, trnD, rd, len, ccX, ccY);
    ws->SegNum = ws->SegNum + 1;
  } else {
    geoHtMatlabMainSetCapacityFail(ws);
  }
}

static void geoHtMatlabMainRecordLineToLoopEnd(HtSegWorkspace_t *ws,
                                               Real linkX,
                                               Real linkY,
                                               Real linkTheta,
                                               Real endX,
                                               Real endY,
                                               Real endTheta) {
  Integer idx;
  Integer lineDir;
  Real relX;
  Real relY;
  Real relTheta;
  Real deltaX;
  Real deltaY;
  Real lineLen;

  geoAxisTransToTarget(linkX, linkY, linkTheta, endX, endY, endTheta, &relX, &relY, &relTheta);
  (void)relY;
  (void)relTheta;
  if (fabsf(relX) > 0.2f) {
    lineDir = 1;
    if (relX < 0.0f) {
      lineDir = -1;
    }
    deltaX = endX - linkX;
    deltaY = endY - linkY;
    lineLen = sqrtf((deltaX * deltaX) + (deltaY * deltaY));
    if (geoHtMatlabMainHasTailCapacity(ws)) {
      idx = HT_SEG_MAX - ws->SegNum - 1;
      geoHtWriteLineSeg(ws, idx, endX, endY, endTheta, lineDir, lineLen);
      ws->SegNum = ws->SegNum + 1;
    } else {
      geoHtMatlabMainSetCapacityFail(ws);
    }
  }
}

static void geoHtFillOneRFixInput(const GeoHtPathPlanningInput_t *input,
                                  const GeoHtPathPlanningParam_t *param,
                                  Real p0X,
                                  Real p0Y,
                                  Real p0Theta,
                                  Real p1X,
                                  Real p1Y,
                                  Real p1Theta,
                                  Real centerX,
                                  Real centerY,
                                  Integer turnDir,
                                  Real curve,
                                  Real pendX,
                                  Real pendY,
                                  Real pendTheta,
                                  GeoHt1RFixInput_t *fixInput) {
  Integer idx;
  Integer count;

  *fixInput = (GeoHt1RFixInput_t){0};
  fixInput->P0X = p0X;
  fixInput->P0Y = p0Y;
  fixInput->P0Theta = p0Theta;
  fixInput->P1X = p1X;
  fixInput->P1Y = p1Y;
  fixInput->P1Theta = p1Theta;
  fixInput->CenterX = centerX;
  fixInput->CenterY = centerY;
  fixInput->TurnDir = turnDir;
  fixInput->Curve = curve;
  fixInput->PendX = pendX;
  fixInput->PendY = pendY;
  fixInput->PendTheta = pendTheta;
  fixInput->RdLs1 = input->Rmin;
  fixInput->RdLs2 = input->Rmin;
  fixInput->SceneQuery = param->SceneQuery;
  fixInput->VehWidth = param->VehW;
  fixInput->VehFrontOffset = geoHtResolveVehFrontOffset(param);
  fixInput->VehRearOffset = geoHtResolveVehRearOffset(param);
  fixInput->SlotP1X = input->SltPt1X;
  fixInput->SlotP1Y = input->SltPt1Y;
  fixInput->SlotP2X = input->SltPt2X;
  fixInput->SlotP2Y = input->SltPt2Y;
  fixInput->SlotP3X = input->SltPt3X;
  fixInput->SlotP3Y = input->SltPt3Y;
  fixInput->SlotP4X = input->SltPt4X;
  fixInput->SlotP4Y = input->SltPt4Y;
  apaSlotOrderFromEntrance(input->SltEntr,
                           &fixInput->SlotP1X,
                           &fixInput->SlotP1Y,
                           &fixInput->SlotP2X,
                           &fixInput->SlotP2Y,
                           &fixInput->SlotP3X,
                           &fixInput->SlotP3Y,
                           &fixInput->SlotP4X,
                           &fixInput->SlotP4Y);

  count =
      apaClampCountToLimits(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX, GEO_BOUNDARY_LINE_MAX);
  fixInput->BoundaryLineNum = count;
  idx = 0;
  while (idx < count) {
    fixInput->BoundaryLineX1[idx] = input->BoundaryLineX1[idx];
    fixInput->BoundaryLineY1[idx] = input->BoundaryLineY1[idx];
    fixInput->BoundaryLineX2[idx] = input->BoundaryLineX2[idx];
    fixInput->BoundaryLineY2[idx] = input->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  count = apaClampCountToLimits(
      input->VirtualBoundaryLineNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_VIRTUAL_LINES);
  fixInput->VirtualBoundaryLineNum = count;
  idx = 0;
  while (idx < count) {
    fixInput->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    fixInput->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    fixInput->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    fixInput->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }

  count = apaClampCountToLimits(input->BoundarySepPtNum, DGR_PT_MAX, VCC_MAX_SEPARATE_PTS);
  fixInput->BoundarySepPtNum = count;
  idx = 0;
  while (idx < count) {
    fixInput->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    fixInput->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  count = apaClampCountToLimits(input->StopperPtNum, DGR_PT_MAX, VCC_MAX_SEPARATE_PTS);
  fixInput->StopperPtNum = count;
  idx = 0;
  while (idx < count) {
    fixInput->StopperPtX[idx] = input->StopperPtX[idx];
    fixInput->StopperPtY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }
}

static Boolean geoHtTryApplyOneRFix(const GeoHtPathPlanningInput_t *input,
                                    const GeoHtPathPlanningParam_t *param,
                                    HtSegWorkspace_t *ws,
                                    Integer firstArcIdx,
                                    Real pendX,
                                    Real pendY,
                                    Real pendTheta) {
  GeoHt1RFixInput_t fixInput;
  GeoHt1RFixOutput_t fixOutput;
  Real p0X;
  Real p0Y;
  Real p0Theta;
  Integer index0;
  Integer p1Index;
  Integer writeIndex;
  Integer fixIndex;
  Boolean applied;

  applied = false;
  if (ws->SegNum >= 3) {
    index0 = firstArcIdx + 3;
    p1Index = index0 - 1;

    if (((ws->SegNum == 3) || (index0 < HT_SEG_MAX)) && (p1Index >= 0) && (p1Index < HT_SEG_MAX)) {
      if (ws->SegNum == 3) {
        p0X = input->TarX;
        p0Y = input->TarY;
        p0Theta = input->TarTheta;
      } else {
        p0X = ws->Stx[index0];
        p0Y = ws->Sty[index0];
        p0Theta = ws->StTh[index0];
      }
      geoHtFillOneRFixInput(input,
                            param,
                            p0X,
                            p0Y,
                            p0Theta,
                            ws->Stx[p1Index],
                            ws->Sty[p1Index],
                            ws->StTh[p1Index],
                            ws->CcX[p1Index],
                            ws->CcY[p1Index],
                            ws->TrnDir[p1Index],
                            ws->Crv[p1Index],
                            pendX,
                            pendY,
                            pendTheta,
                            &fixInput);
      fixOutput = (GeoHt1RFixOutput_t){0};
      geoHt1RFix(&fixInput, &fixOutput);
      writeIndex = index0 - fixOutput.SegNum;
      if ((fixOutput.Success != 0) && (fixOutput.SegNum > 0) && (writeIndex >= 0)) {
        ws->SegNum = ws->SegNum - 3;
        fixIndex = 0;
        while (fixIndex < fixOutput.SegNum) {
          Integer dstIndex;

          dstIndex = writeIndex + fixIndex;
          ws->Stx[dstIndex] = fixOutput.AnchorX[fixIndex];
          ws->Sty[dstIndex] = fixOutput.AnchorY[fixIndex];
          ws->StTh[dstIndex] = fixOutput.AnchorTheta[fixIndex];
          ws->GoDir[dstIndex] = fixOutput.GoDirs[fixIndex];
          ws->TrnDir[dstIndex] = fixOutput.TurnDirs[fixIndex];
          ws->Crv[dstIndex] = fixOutput.Curves[fixIndex];
          ws->Tht[dstIndex] = fixOutput.DThetas[fixIndex];
          ws->Len[dstIndex] = fixOutput.Lengths[fixIndex];
          ws->CcX[dstIndex] = fixOutput.CtrX[fixIndex];
          ws->CcY[dstIndex] = fixOutput.CtrY[fixIndex];
          ws->SegNum = ws->SegNum + 1;
          fixIndex = fixIndex + 1;
        }
        applied = true;
      }
    }
  }
  return applied;
}

static Boolean geoHtTryExternalReverseArc(const GeoHtPathPlanningInput_t *input,
                                          const GeoHtPathPlanningParam_t *param,
                                          HtSegWorkspace_t *ws,
                                          Real stX,
                                          Real stY,
                                          Real stTheta,
                                          Real dangerX,
                                          Real dangerY,
                                          Integer dirS,
                                          Integer trnD) {
  GeoCircleIntrvCheckInput_t arcInput;
  GeoCircleIntrvCheckOutput_t arcOutput;
  Real dngRelX;
  Real dngRelY;
  Real dngRelTheta;
  Real radius;
  Real widthHalf;
  Real latGuard;
  Real ccX;
  Real ccY;
  Real centerToDanger;
  Real thetaA;
  Real thetaB;
  Real edTheta;
  Real edX;
  Real edY;
  Boolean planned;

  (void)dirS;
  planned = false;
  radius = input->Rmin;
  widthHalf = param->VehW * 0.5f;
  latGuard = 0.5f;
  geoAxisTransToTarget(dangerX, dangerY, 0.0f, stX, stY, stTheta, &dngRelX, &dngRelY, &dngRelTheta);
  (void)dngRelX;
  (void)dngRelTheta;

  if (trnD > 0) {
    ccX = stX - (radius * sinf(stTheta));
    ccY = stY + (radius * cosf(stTheta));
    centerToDanger =
        sqrtf(((ccX - dangerX) * (ccX - dangerX)) + ((ccY - dangerY) * (ccY - dangerY)));
    if ((dngRelY <= (widthHalf + latGuard)) && (fabsf(radius - dngRelY) < centerToDanger) &&
        (fabsf(radius - widthHalf - latGuard) < centerToDanger)) {
      thetaA = acosf(apaMathClampUnit((radius - dngRelY) / centerToDanger));
      thetaB = acosf(apaMathClampUnit((radius - widthHalf - latGuard) / centerToDanger));
      edTheta = stTheta - thetaB + thetaA;
      edTheta = apaCoordWrapToPi(edTheta);
      edX = ccX + (radius * sinf(edTheta));
      edY = ccY - (radius * cosf(edTheta));
      geoHtFillCircleInput(input,
                           param,
                           ccX,
                           ccY,
                           radius,
                           stX,
                           stY,
                           stTheta,
                           edX,
                           edY,
                           edTheta,
                           -1,
                           trnD,
                           0.1f,
                           &arcInput);
      arcOutput = (GeoCircleIntrvCheckOutput_t){0};
      geoCircleIntrvCheckWithScene(&arcInput, &param->SceneQuery, &arcOutput);
      ws->PlnChkCnt = ws->PlnChkCnt + arcOutput.CheckCount;
      if (arcOutput.IntrvFlg != 2) {
        geoHtExternalRecordArc(ws,
                               input,
                               arcOutput.AvlPosX,
                               arcOutput.AvlPosY,
                               arcOutput.AvlPosTheta,
                               1,
                               trnD,
                               radius,
                               arcOutput.SafeLen,
                               ccX,
                               ccY);
        planned = true;
      } else {
        ws->PlanCmd = 5;
        ws->FailRs = 8;
      }
    }
  } else {
    ccX = stX + (radius * sinf(stTheta));
    ccY = stY - (radius * cosf(stTheta));
    centerToDanger =
        sqrtf(((ccX - dangerX) * (ccX - dangerX)) + ((ccY - dangerY) * (ccY - dangerY)));
    if ((dngRelY >= (-widthHalf - latGuard)) && (fabsf(radius + dngRelY) < centerToDanger) &&
        (fabsf(radius - widthHalf - latGuard) < centerToDanger)) {
      thetaA = acosf(apaMathClampUnit((radius + dngRelY) / centerToDanger));
      thetaB = acosf(apaMathClampUnit((radius - widthHalf - latGuard) / centerToDanger));
      edTheta = stTheta + thetaB - thetaA;
      edTheta = apaCoordWrapToPi(edTheta);
      edX = ccX - (radius * sinf(edTheta));
      edY = ccY + (radius * cosf(edTheta));
      geoHtFillCircleInput(input,
                           param,
                           ccX,
                           ccY,
                           radius,
                           stX,
                           stY,
                           stTheta,
                           edX,
                           edY,
                           edTheta,
                           -1,
                           trnD,
                           0.1f,
                           &arcInput);
      arcOutput = (GeoCircleIntrvCheckOutput_t){0};
      geoCircleIntrvCheckWithScene(&arcInput, &param->SceneQuery, &arcOutput);
      ws->PlnChkCnt = ws->PlnChkCnt + arcOutput.CheckCount;
      if (arcOutput.IntrvFlg != 2) {
        geoHtExternalRecordArc(ws,
                               input,
                               arcOutput.AvlPosX,
                               arcOutput.AvlPosY,
                               arcOutput.AvlPosTheta,
                               1,
                               trnD,
                               radius,
                               arcOutput.SafeLen,
                               ccX,
                               ccY);
        planned = true;
      } else {
        ws->PlanCmd = 5;
        ws->FailRs = 8;
      }
    }
  }
  if ((!planned) && (ws->PlanCmd != 5)) {
    planned = true;
  }
  return planned;
}

static Boolean geoHtTryMatlabNormalLoop(const GeoHtPathPlanningInput_t *input,
                                        const GeoHtPathPlanningParam_t *param,
                                        GeoHtPathPlanningOutput_t *output) {
  HtSegWorkspace_t ws;
  GeoHtMatlabMainPt1Decision_t pt1Decision;
  GeoHtMatlabMainNormalEntryDecision_t normalEntryDecision;
  GeoHtMatlabMainLoopPose_t loopEndPose;
  Real relX;
  Real relY;
  Real relTheta;
  Real dangerX;
  Real dangerY;
  Real pt1X;
  Real pt1Y;
  Real pt1Theta;
  Integer dirS;
  Integer trnD;
  Integer calcCount;
  Integer iniSegN;
  Integer writeIndex0;
  Boolean canRunLoop;
  Boolean success;
  Boolean loopCond;

  success = false;
  loopCond = false;
  canRunLoop = true;
  dangerX = 0.0f;
  dangerY = 0.0f;
  iniSegN = 0;
  loopEndPose = (GeoHtMatlabMainLoopPose_t){0};
  geoHtMatlabMainInitWorkspace(&ws);
  geoAxisTransToTarget(input->VehX,
                       input->VehY,
                       input->VehTheta,
                       input->TarX,
                       input->TarY,
                       input->TarTheta,
                       &relX,
                       &relY,
                       &relTheta);
  (void)relX;
  (void)relTheta;
  (void)relY;

  pt1Decision = (GeoHtMatlabMainPt1Decision_t){0};
  geoHtMatlabMainResolvePt1Decision(input, param, &pt1Decision);
  ws.PlnChkCnt = ws.PlnChkCnt + pt1Decision.PlnChkCnt;
  if (pt1Decision.PlanCmd == 5) {
    ws.PlanCmd = 5;
    ws.FailRs = pt1Decision.FailRs;
    canRunLoop = false;
  }
  if (canRunLoop) {
    GeoHtDangerPtInput_t dangerInput;
    GeoHtDangerPtOutput_t dangerOutput;

    pt1X = pt1Decision.Pt1X;
    pt1Y = pt1Decision.Pt1Y;
    pt1Theta = pt1Decision.Pt1Theta;
    if (pt1Decision.AddSegment != 0) {
      writeIndex0 = geoHtMatlabMainTailWriteIndex(&ws);
      geoHtWriteLineSeg(&ws,
                        writeIndex0,
                        pt1Decision.SegmentStartX,
                        pt1Decision.SegmentStartY,
                        pt1Decision.SegmentStartTheta,
                        pt1Decision.SegmentGoDir,
                        pt1Decision.SegmentLen);
      ws.SegNum = ws.SegNum + 1;
    }

    geoHtFillDangerInput(input,
                         param,
                         geoHtMatlabMainResolvePrkTypeFromRelY(relY),
                         pt1X,
                         pt1Y,
                         pt1Theta,
                         &dangerInput);
    dangerOutput = (GeoHtDangerPtOutput_t){0};
    geoHtDangerPt(&dangerInput, &dangerOutput);
    normalEntryDecision = (GeoHtMatlabMainNormalEntryDecision_t){0};
    geoHtMatlabMainResolveNormalEntryDecision(input,
                                              param,
                                              pt1X,
                                              pt1Y,
                                              pt1Theta,
                                              dangerOutput.PtEx,
                                              dangerOutput.DangerPtX,
                                              dangerOutput.DangerPtY,
                                              &normalEntryDecision);
    dangerX = normalEntryDecision.DangerX;
    dangerY = normalEntryDecision.DangerY;
    dirS = normalEntryDecision.DirS;
    trnD = normalEntryDecision.TrnD;
    geoHtMatlabMainRunPrefixRuntime(
        input, param, &normalEntryDecision, &ws, &iniSegN, &loopEndPose);
    if (ws.PlanCmd == 5) {
      loopCond = false;
    } else {
      loopCond = true;
    }
  } else {
    dirS = 1;
    trnD = -1;
  }

  calcCount = 0;
  while (loopCond && (ws.SegNum < HT_SEG_MAX - iniSegN)) {
    Real stX;
    Real stY;
    Real stTheta;
    Integer prevSegNum;
    Integer previousDir;
    Integer previousTurn;
    Boolean allowNoSegmentSwitch;

    prevSegNum = ws.SegNum;
    previousDir = dirS;
    previousTurn = trnD;
    allowNoSegmentSwitch = false;
    if (ws.SegNum > 0) {
      Integer prevIdx;
      prevIdx = HT_SEG_MAX - ws.SegNum;
      stX = ws.Stx[prevIdx];
      stY = ws.Sty[prevIdx];
      stTheta = ws.StTh[prevIdx];
    } else {
      stX = input->TarX;
      stY = input->TarY;
      stTheta = input->TarTheta;
    }
    if (dirS > 0) {
      Geo2CircleLinkHTInput_t linkInput;
      Geo2CircleLinkHTOutput_t linkOutput;
      GeoCircleIntrvCheckInput_t arcInput;
      GeoCircleIntrvCheckOutput_t arcOutput;
      Integer firstArcIdx;

      linkInput = (Geo2CircleLinkHTInput_t){0};
      linkInput.Pt1X = stX;
      linkInput.Pt1Y = stY;
      linkInput.Pt1Theta = stTheta;
      linkInput.Pt2X = loopEndPose.X;
      linkInput.Pt2Y = loopEndPose.Y;
      linkInput.Pt2Theta = loopEndPose.Theta;
      linkInput.Dir = trnD;
      linkInput.R1 = input->Rmin;
      linkInput.R2 = input->Rmin;
      linkOutput = (Geo2CircleLinkHTOutput_t){0};
      geo2CircleLinkHT(&linkInput, &linkOutput);
      if (linkOutput.FailFlag != 0) {
        ws.PlanCmd = 5;
        ws.FailRs = 8;
      } else {
        geoHtFillCircleInput(input,
                             param,
                             linkOutput.Ct1X,
                             linkOutput.Ct1Y,
                             input->Rmin,
                             stX,
                             stY,
                             stTheta,
                             linkOutput.St2X,
                             linkOutput.St2Y,
                             linkOutput.St2Theta,
                             dirS,
                             linkOutput.TrnD1,
                             0.1f,
                             &arcInput);
        arcOutput = (GeoCircleIntrvCheckOutput_t){0};
        geoCircleIntrvCheckWithScene(&arcInput, &param->SceneQuery, &arcOutput);
        ws.PlnChkCnt = ws.PlnChkCnt + arcOutput.CheckCount;
        if (arcOutput.IntrvFlg == 2) {
          ws.PlanCmd = 5;
          ws.FailRs = 8;
        } else if (arcOutput.IntrvFlg == 0) {
          firstArcIdx = HT_SEG_MAX - ws.SegNum - 1;
          geoHtExternalRecordArc(&ws,
                                 input,
                                 linkOutput.St2X,
                                 linkOutput.St2Y,
                                 linkOutput.St2Theta,
                                 -dirS,
                                 linkOutput.TrnD1,
                                 input->Rmin,
                                 arcOutput.SafeLen,
                                 linkOutput.Ct1X,
                                 linkOutput.Ct1Y);

          if (ws.SegNum >= HT_SEG_MAX) {
            geoHtMatlabMainSetCapacityFail(&ws);
          }
          if (ws.PlanCmd != 5) {
            geoHtFillCircleInput(input,
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
                                 dirS,
                                 linkOutput.TrnD2,
                                 0.1f,
                                 &arcInput);
            arcOutput = (GeoCircleIntrvCheckOutput_t){0};
            geoCircleIntrvCheckWithScene(&arcInput, &param->SceneQuery, &arcOutput);
            ws.PlnChkCnt = ws.PlnChkCnt + arcOutput.CheckCount;
            if (arcOutput.IntrvFlg == 2) {
              ws.PlanCmd = 5;
              ws.FailRs = 8;
            } else if (arcOutput.IntrvFlg == 0) {
              Boolean fixOk;

              fixOk = geoHtTryApplyOneRFix(
                  input, param, &ws, firstArcIdx, loopEndPose.X, loopEndPose.Y, loopEndPose.Theta);
              if (fixOk) {
                geoHtMatlabMainSetSuccessAfterTailRecord(&ws);
              } else {
                geoHtExternalRecordArc(&ws,
                                       input,
                                       linkOutput.St3X,
                                       linkOutput.St3Y,
                                       linkOutput.St3Theta,
                                       -dirS,
                                       linkOutput.TrnD2,
                                       input->Rmin,
                                       arcOutput.SafeLen,
                                       linkOutput.Ct2X,
                                       linkOutput.Ct2Y);
                if (ws.PlanCmd != 5) {
                  geoHtMatlabMainRecordLineToLoopEnd(&ws,
                                                     linkOutput.St3X,
                                                     linkOutput.St3Y,
                                                     linkOutput.St3Theta,
                                                     loopEndPose.X,
                                                     loopEndPose.Y,
                                                     loopEndPose.Theta);
                }
                geoHtMatlabMainSetSuccessAfterTailRecord(&ws);
              }
            } else {
              geoHtExternalRecordArc(&ws,
                                     input,
                                     arcOutput.AvlPosX,
                                     arcOutput.AvlPosY,
                                     arcOutput.AvlPosTheta,
                                     -dirS,
                                     linkOutput.TrnD2,
                                     input->Rmin,
                                     arcOutput.SafeLen,
                                     linkOutput.Ct2X,
                                     linkOutput.Ct2Y);
              geoHtMatlabMainSetContinueAfterTailRecord(&ws);
            }
          }
        } else {
          geoHtExternalRecordArc(&ws,
                                 input,
                                 arcOutput.AvlPosX,
                                 arcOutput.AvlPosY,
                                 arcOutput.AvlPosTheta,
                                 -dirS,
                                 linkOutput.TrnD1,
                                 input->Rmin,
                                 arcOutput.SafeLen,
                                 linkOutput.Ct1X,
                                 linkOutput.Ct1Y);
          geoHtMatlabMainSetContinueAfterTailRecord(&ws);
        }
      }
    } else {
      Boolean reverseOk;

      reverseOk = geoHtTryExternalReverseArc(
          input, param, &ws, stX, stY, stTheta, dangerX, dangerY, dirS, trnD);
      if (reverseOk && (ws.PlanCmd != 5)) {
        ws.PlanCmd = 2;
        if (ws.SegNum == prevSegNum) {
          allowNoSegmentSwitch = true;
        }
      } else if (ws.PlanCmd != 5) {
        ws.PlanCmd = 5;
        ws.FailRs = 7;
      }
    }

    if (ws.PlanCmd == 1) {
      success = true;
      loopCond = false;
    } else if (ws.PlanCmd == 5) {
      loopCond = false;
    } else if (ws.PlanCmd == 2) {
      if ((ws.SegNum <= prevSegNum) && (!allowNoSegmentSwitch)) {
        ws.PlanCmd = 5;
        ws.FailRs = 9;
        loopCond = false;
      } else {
        Integer planCmd2;

        planCmd2 = ws.PlanCmd;
        geoHtReadTailStateForPlanCmd2(&ws, &dirS, &trnD, &planCmd2);
        ws.PlanCmd = planCmd2;
        if ((ws.SegNum <= prevSegNum) && (dirS == previousDir) && (trnD == previousTurn)) {
          ws.PlanCmd = 5;
          ws.FailRs = 9;
          loopCond = false;
        }
      }
    }

    calcCount = calcCount + 1;
    if (calcCount > 10) {
      ws.FailRs = 9;
      loopCond = false;
    }
    if (ws.SegNum >= HT_SEG_MAX) {
      loopCond = false;
    }
  }

  if (success) {
    success = geoHtAssembleOutputWithPrefix(&ws, iniSegN, output);
    if (success) {
      output->PlannerCase = GEO_HT_PLANNER_CASE_MAIN;
      output->PlnChkCnt = ws.PlnChkCnt;
      output->FailRs0 = ws.FailRs;
      output->DngPtX = dangerX;
      output->DngPtY = dangerY;
    }
  } else {
    *output = (GeoHtPathPlanningOutput_t){0};
    output->PlanFlg = 0;
    output->SegNum = 0;
    output->PlannerCase = ws.PlanCmd;
    output->FailRs0 = ws.FailRs;
    output->PlnChkCnt = ws.PlnChkCnt;
    output->DngPtX = dangerX;
    output->DngPtY = dangerY;
  }
  return success;
}

static Boolean geoHtTryMatlabExternalLongEdgeLoop(const GeoHtPathPlanningInput_t *input,
                                                  const GeoHtPathPlanningParam_t *param,
                                                  GeoHtPathPlanningOutput_t *output) {
  Boolean success0;

  success0 = false;
  if (geoHtIsLongEdgeEntrySlot(input)) {
    success0 = geoHtTryMatlabNormalLoop(input, param, output);
  }
  return success0;
}

static void geoHtBuildParkingTypeInput(const GeoHtPathPlanningInput_t *input,
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

static Integer geoHtResolveParkingType(const GeoHtPathPlanningInput_t *input) {
  CheckParkingTypeInput_t typeInput;
  CheckParkingTypeOutput_t typeOutput;

  geoHtBuildParkingTypeInput(input, &typeInput);
  typeOutput = (CheckParkingTypeOutput_t){0};
  checkParkingType(&typeInput, &typeOutput);
  return typeOutput.ParkingType;
}

static void geoHtHybridBuildSlot(const GeoHtPathPlanningInput_t *input,
                                 const GeoHtPathPlanningParam_t *param,
                                 ApaSlot_t *slot) {
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
  parkingType = input->PrkType;
  if (parkingType != APA_PARKING_TYPE_LEFT && parkingType != APA_PARKING_TYPE_RIGHT)
    parkingType = geoHtResolveParkingType(input);
  apaPlannerHybridInitSlot(slot,
                           input->TarX,
                           input->TarY,
                           input->TarTheta,
                           input->SltEntr,
                           slotPts,
                           parkingType,
                           input->SurdType,
                           APA_PARKING_DIREC_REAR);
  slot->SlotStatus = param->SltState;
}

static void geoHtHybridInitCollisionContext(const GeoHtPathPlanningInput_t *input,
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
  while ((idx < input->BoundarySepPtNum) && (idx < DGR_PT_MAX)) {
    (void)apaPlannerHybridCollisionAddBoundaryPoint(
        context, input->BoundarySepPtX[idx], input->BoundarySepPtY[idx]);
    idx = idx + 1;
  }
  idx = 0;
  while ((idx < input->StopperPtNum) && (idx < DGR_PT_MAX)) {
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

static void geoHtHybridCopySegmentsToOutput(const ApaPlannerHybridSegments_t *segments,
                                            GeoHtPathPlanningOutput_t *output) {
  Integer idx;

  *output = (GeoHtPathPlanningOutput_t){0};
  output->PlanFlg = 1;
  output->SegNum = segments->SegNum;
  output->PathLen = segments->PathLen;
  output->PlnChkCnt = segments->PlnChkCnt;
  output->StpN = segments->StpN;
  output->PlannerCase = GEO_HT_PLANNER_CASE_HYBRID_FALLBACK;
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
}

static Boolean geoHtHasValidCandidate(const GeoHtPathPlanningOutput_t *candidate) {
  return (candidate->PlanFlg == 1) && (candidate->SegNum > 0) &&
         (candidate->PathLen > APA_PLANNER_HYBRID_EPS);
}

static Boolean geoHtCandidateIsHybridFallback(const GeoHtPathPlanningOutput_t *candidate) {
  return candidate->PlannerCase == GEO_HT_PLANNER_CASE_HYBRID_FALLBACK;
}

static Boolean geoHtIsLongEdgeEntrySlot(const GeoHtPathPlanningInput_t *input) {
  Real entryDx;
  Real entryDy;
  Real depthDx;
  Real depthDy;
  Real entryLen;
  Real depthLen;

  entryDx = input->SltPt2X - input->SltPt1X;
  entryDy = input->SltPt2Y - input->SltPt1Y;
  depthDx = input->SltPt3X - input->SltPt2X;
  depthDy = input->SltPt3Y - input->SltPt2Y;
  entryLen = sqrtf((entryDx * entryDx) + (entryDy * entryDy));
  depthLen = sqrtf((depthDx * depthDx) + (depthDy * depthDy));
  return (input->SltEntr == 1) && (entryLen > (depthLen + 0.1f));
}

static Integer geoHtResolveStepCount(const GeoHtPathPlanningOutput_t *candidate) {
  Integer idx;
  Integer stpN0;
  Integer prevDir;
  Boolean calculate0;

  stpN0 = candidate->StpN;
  if (candidate->StpN > 0) {
    calculate0 = false;
  } else {
    calculate0 = (candidate->SegNum > 0);
  }

  if (calculate0) {
    stpN0 = 0;
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

static Boolean
geoHtHybridHasSignificantPathAdvantage(const GeoHtPathPlanningOutput_t *hybridCandidate,
                                       const GeoHtPathPlanningOutput_t *otherCandidate) {
  Integer stepDelta;
  Real pathGain;
  Real pathGainRatio;

  stepDelta = geoHtResolveStepCount(hybridCandidate) - geoHtResolveStepCount(otherCandidate);
  pathGain = otherCandidate->PathLen - hybridCandidate->PathLen;
  pathGainRatio = 0.0f;
  if (otherCandidate->PathLen > APA_PLANNER_HYBRID_EPS) {
    pathGainRatio = pathGain / otherCandidate->PathLen;
  }
  return (stepDelta > 0) && (stepDelta <= GEO_HT_HYBRID_OVERRIDE_STEP_DELTA_MAX) &&
         (pathGain > GEO_HT_HYBRID_OVERRIDE_PATH_GAIN_MIN) &&
         (pathGainRatio > GEO_HT_HYBRID_OVERRIDE_PATH_RATIO_MIN);
}

static Integer
geoHtResolveCandidateDecisionReason(const GeoHtPathPlanningOutput_t *currentCandidate,
                                    const GeoHtPathPlanningOutput_t *nextCandidate,
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

  currentValid = geoHtHasValidCandidate(currentCandidate);
  nextValid = geoHtHasValidCandidate(nextCandidate);
  *replaceCandidate = false;
  decisionReason0 = GEO_HT_FALLBACK_DECISION_NONE;

  if (!nextValid) {
    decisionReason0 = GEO_HT_FALLBACK_DECISION_NONE;
  } else if (!currentValid) {
    *replaceCandidate = true;
    decisionReason0 = GEO_HT_FALLBACK_DECISION_VALID_ONLY;
  } else {
    currentHybrid = geoHtCandidateIsHybridFallback(currentCandidate);
    nextHybrid = geoHtCandidateIsHybridFallback(nextCandidate);
    currentStpN = geoHtResolveStepCount(currentCandidate);
    nextStpN = geoHtResolveStepCount(nextCandidate);
    shorterStepCount = (nextStpN < currentStpN);
    longerStepCount = (nextStpN > currentStpN);
    shorterPath =
        ((nextCandidate->PathLen + GEO_HT_CANDIDATE_PATH_EPS) < currentCandidate->PathLen);
    longerPath = ((currentCandidate->PathLen + GEO_HT_CANDIDATE_PATH_EPS) < nextCandidate->PathLen);
    fewerSegments = (nextCandidate->SegNum < currentCandidate->SegNum);
    moreSegments = (nextCandidate->SegNum > currentCandidate->SegNum);
    fewerChecks = (nextCandidate->PlnChkCnt < currentCandidate->PlnChkCnt);
    moreChecks = (nextCandidate->PlnChkCnt > currentCandidate->PlnChkCnt);
    preferHybridPathQuality = false;

    if ((!currentHybrid) && nextHybrid) {
      preferHybridPathQuality =
          geoHtHybridHasSignificantPathAdvantage(nextCandidate, currentCandidate);
    } else if (currentHybrid && (!nextHybrid)) {
      if (geoHtHybridHasSignificantPathAdvantage(currentCandidate, nextCandidate)) {
        decisionReason0 = GEO_HT_FALLBACK_DECISION_PATH_LEN;
      }
    }

    if (preferHybridPathQuality) {
      *replaceCandidate = true;
      decisionReason0 = GEO_HT_FALLBACK_DECISION_PATH_LEN;
    } else if (decisionReason0 == GEO_HT_FALLBACK_DECISION_PATH_LEN) {
      decisionReason0 = GEO_HT_FALLBACK_DECISION_PATH_LEN;
    } else if (shorterStepCount) {
      *replaceCandidate = true;
      decisionReason0 = GEO_HT_FALLBACK_DECISION_STEP_COUNT;
    } else if (longerStepCount) {
      decisionReason0 = GEO_HT_FALLBACK_DECISION_STEP_COUNT;
    } else if (shorterPath) {
      *replaceCandidate = true;
      decisionReason0 = GEO_HT_FALLBACK_DECISION_PATH_LEN;
    } else if (longerPath) {
      decisionReason0 = GEO_HT_FALLBACK_DECISION_PATH_LEN;
    } else if (fewerSegments) {
      *replaceCandidate = true;
      decisionReason0 = GEO_HT_FALLBACK_DECISION_SEG_NUM;
    } else if (moreSegments) {
      decisionReason0 = GEO_HT_FALLBACK_DECISION_SEG_NUM;
    } else if (fewerChecks) {
      *replaceCandidate = true;
      decisionReason0 = GEO_HT_FALLBACK_DECISION_PLN_CHK_CNT;
    } else if (moreChecks) {
      decisionReason0 = GEO_HT_FALLBACK_DECISION_PLN_CHK_CNT;
    }
  }
  return decisionReason0;
}

static Boolean geoHtShouldReplaceCandidate(const GeoHtPathPlanningOutput_t *currentCandidate,
                                           const GeoHtPathPlanningOutput_t *nextCandidate,
                                           const GeoHtPathPlanningParam_t *param) {
  if (param->HybridPolicy.CompareCandidates && currentCandidate->PlanFlg &&
      nextCandidate->PlanFlg) {
    ApaPlannerHybridSegments_t first, second;
    if (apaPlannerHybridReadGeometry(
            APA_HYBRID_GEOMETRY_FIELDS(currentCandidate, HT_SEG_OUT_MAX), &first, 0) &&
        apaPlannerHybridReadGeometry(
            APA_HYBRID_GEOMETRY_FIELDS(nextCandidate, HT_SEG_OUT_MAX), &second, 0))
      return param->HybridPolicy.CompareCandidates(&first, &second, param->HybridPolicy.UserData) ==
             APA_PATH_COMP_NEW_PATH_BETTER;
  }

  Boolean replaceCandidate;

  replaceCandidate = false;
  (void)geoHtResolveCandidateDecisionReason(currentCandidate, nextCandidate, &replaceCandidate);
  return replaceCandidate;
}

static void geoHtStoreFallbackDiagFromCandidate(Boolean candidateOk,
                                                const GeoHtPathPlanningOutput_t *candidate,
                                                Integer *stpN,
                                                Real *pathLen,
                                                Integer *segNum,
                                                Integer *plnChkCnt) {
  if (candidateOk) {
    *stpN = geoHtResolveStepCount(candidate);
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

static Integer geoHtResolveFallbackWinnerKind(Boolean geometricOk,
                                              Boolean hybridOk,
                                              const GeoHtPathPlanningOutput_t *bestCandidate) {
  Integer winnerKind0;

  if (!geoHtHasValidCandidate(bestCandidate)) {
    winnerKind0 = GEO_HT_FALLBACK_WINNER_NONE;
  } else if (hybridOk && (bestCandidate->PlannerCase == GEO_HT_PLANNER_CASE_HYBRID_FALLBACK)) {
    winnerKind0 = GEO_HT_FALLBACK_WINNER_HYBRID;
  } else if (geometricOk) {
    winnerKind0 = GEO_HT_FALLBACK_WINNER_GEOM;
  } else {
    winnerKind0 = GEO_HT_FALLBACK_WINNER_NONE;
  }
  return winnerKind0;
}

static Integer
geoHtResolveFallbackDecisionReason(Boolean geometricOk,
                                   Boolean hybridOk,
                                   const GeoHtPathPlanningOutput_t *geometricCandidate,
                                   const GeoHtPathPlanningOutput_t *hybridCandidate,
                                   const GeoHtPathPlanningParam_t *param) {
  if (geometricOk && hybridOk && param->HybridPolicy.CompareCandidates)
    return 6;

  Boolean replaceCandidate;
  Integer decisionReason0;

  if (!geometricOk && !hybridOk) {
    decisionReason0 = GEO_HT_FALLBACK_DECISION_NONE;
  } else {
    replaceCandidate = false;
    decisionReason0 =
        geoHtResolveCandidateDecisionReason(geometricCandidate, hybridCandidate, &replaceCandidate);
  }
  return decisionReason0;
}

static Boolean geoHtRunHybridFallback(const GeoHtPathPlanningInput_t *input,
                                      const GeoHtPathPlanningParam_t *param,
                                      Boolean allowManeuver,
                                      GeoHtPathPlanningOutput_t *output) {
  ApaPlannerGlobalState_t global = APA_PLANNER_GLOBAL_STATE_INIT;
  ApaPlannerHybridCollisionContext_t collision = {0};
  ApaHybridAStar_t planner = {0};
  ApaSlot_t slot = {0};
  ApaPlannerHybridSegments_t segments;
  const Real start[3] = {input->VehX, input->VehY, input->VehTheta};
  const Real goal[3] = {input->TarX, input->TarY, input->TarTheta};
  Real relativeX, relativeY, relativeTheta;
  Boolean reversePlan = true;
  apaPlannerHybridInitPlannerGlobal(&global, param->VehW, input->Rmin, param->NeedHybridAlgFusion);

  const Boolean target = param->SltState == APA_SLOT_STATUS_TARGET_SLOT ||
                         param->SltState == APA_SLOT_STATUS_TARGET_SLOT_REPLAN;
  global.Planner.Conf.PlanningCalPowerMaxPerFrame =
      param->SltState == APA_SLOT_STATUS_FAIL_PLAN ? 1000 : target ? 600 : 180;
  global.Planner.Conf.PlanningCalPowerMaxPerSlot =
      param->SltState == APA_SLOT_STATUS_FAIL_PLAN ? 900 : target ? 600 : 150;

  apaPlannerGlobalSetXyGridResolution(&global, 0.2f);
  apaPlannerGlobalSetYawGridResolution(&global, 5.0f * APA_PI / 180.0f);
  global.Planner.Map.NSteer = 20.0f;
  global.Planner.Map.MotionResolution = 0.1f;
  if (input->SurdType == APA_SURROUND_OBS_EXIST_BOTH &&
      (param->SltState == APA_SLOT_STATUS_TARGET_SLOT ||
       param->SltState == APA_SLOT_STATUS_TARGET_SLOT_REPLAN ||
       param->SltState == APA_SLOT_STATUS_FAIL_PLAN)) {
    apaPlannerGlobalSetXyGridResolution(&global, 0.05f);
    global.Planner.Conf.CollisionModel = APA_COLLISION_MODEL_EIGHT_EDGES;
  }

#if APA_ENABLE_HT_SEARCH_MODEL
  global.Planner.Conf.ParkingSearch = true;
  global.Planner.Conf.PlanningCalPowerMaxPerFrame = 12000;
  global.Planner.Conf.PlanningCalPowerMaxPerSlot = 12000;
  const Real primitive = global.Planner.Conf.PathLenMin;
  const Real xy = primitive / 4.0f;
  const Integer yawBins = (Integer)ceilf(4.0f * APA_PI * input->Rmin / primitive);
  apaPlannerGlobalSetXyGridResolution(&global, xy);
  global.Planner.Map.MapEdgePixNum = (Integer)ceilf(global.Planner.Map.MapEdgeLen / xy);
  apaPlannerGlobalSetYawGridResolution(&global, 2.0f * APA_PI / (Real)yawBins);
  global.Planner.Map.NSteer = 2.0f;
  global.Planner.Map.MotionResolution = xy;
  global.Planner.Map.CollisionDetectStepLength = xy;
  global.Planner.Map.HCost = 1.5f;
  global.Planner.Map.SbCost = 0.4f;
  global.Planner.Map.SteerChangeCost = 0.1f;
  global.Planner.Map.SteerCost = 0.05f;
#endif
  global.Planner.Veh.LF = param->VehLlF;
  global.Planner.Veh.LB = param->VehLr;
  global.Planner.Veh.LL = param->VehLlF + param->VehLr;
  global.Planner.Veh.Lf = param->VehLlF - global.Planner.Veh.WB;

  global.Planner.Conf.UseMoreCalUnits =
      !global.Planner.Conf.ParkingSearch && param->SltState != APA_SLOT_STATUS_TARGET_SLOT_REPLAN;
  geoHtHybridBuildSlot(input, param, &slot);
  geoHtHybridInitCollisionContext(input, &slot, &global, &collision);
  apaVehicleCollisionCheckContextSetCollisionDemand(&collision.CollisionContext,
                                                    APA_COLLISION_DEMAND_SIMPLIFY);
  collision.SceneTemplate.CollisionModel = global.Planner.Conf.CollisionModel;
  geoAxisTransToTarget(input->VehX,
                       input->VehY,
                       input->VehTheta,
                       input->TarX,
                       input->TarY,
                       input->TarTheta,
                       &relativeX,
                       &relativeY,
                       &relativeTheta);
  if (apaMathLess(fabsf(relativeTheta), 15.0f * APA_PI / 180.0f) && apaMathLess(relativeX, 1.5f) &&
      apaMathLess(relativeY, 0.5f))
    reversePlan = false;

  const Boolean vehicleInSlot =
      apaSlotCalcVehInSlotFlag(start, slot.SlotPts, 0, &global.Planner.Veh);
  const Boolean parkingOutReplan = param->SltState == APA_SLOT_STATUS_TARGET_SLOT_REPLAN &&
                                   vehicleInSlot && fabsf(relativeY) > 0.5f;
  if (allowManeuver && global.Planner.Conf.EnableParallelPlanInSlot &&
      (!vehicleInSlot || parkingOutReplan)) {
    ApaHybridAStarMotionPath_t tail;
    ApaParallelParkinManeuverDiag_t maneuverDiag;
    apaPlannerHybridBindScene(&collision, &param->SceneQuery);
    const Boolean generated = apaParallelParkinManeuverInRegion(
        &slot, start, param->HybridPolicy.GoalPositionTolerance, &collision, &tail, &maneuverDiag);
    fprintf(stderr,
            "[APA_HT_MANEUVER] generated=%d reason=%s loops=%ld adjusted=%d points=%ld\n",
            (int)generated,
            maneuverDiag.Reason,
            (long)maneuverDiag.Loops,
            (int)maneuverDiag.Adjusted,
            (long)(generated ? tail.Path.PathN : 0));
    if (generated) {
      const Real connectorStart[3] = {tail.Path.X[0], tail.Path.Y[0], tail.Path.Tht[0]};
      const Integer available = global.Planner.Conf.PlanningCalPowerMaxPerSlot;
      global.Planner.Conf.UseMoreCalUnits = false;
      collision.ReverseOutput = true;
      apaHybridAStarConfigure(&planner, &global, connectorStart, start, &slot, true);
      fprintf(stderr,
              "[APA_HT_CONNECT] phase=begin budget=%ld tail_points=%ld\n",
              (long)available,
              (long)tail.Path.PathN);
      const Boolean connected = apaPlannerHybridSearchWithTail(&planner,
                                                               &collision,
                                                               &param->SceneQuery,
                                                               &param->HybridPolicy,
                                                               param->Diagnostics,
                                                               HT_SEG_OUT_MAX,
                                                               &tail,
                                                               &segments,
                                                               0);
      const Integer consumed = planner.NodePool.ValidNodeCnt;
      fprintf(stderr,
              "[APA_HT_CONNECT] accepted=%d consumed=%ld remaining=%ld fault=%ld\n",
              (int)connected,
              (long)consumed,
              (long)(available - consumed),
              (long)planner.Fault);
      if (connected) {
        geoHtHybridCopySegmentsToOutput(&segments, output);
        return true;
      }
      if (consumed >= available ||
          (!global.Planner.Conf.ParkingSearch &&
           planner.Fault == APA_HYBRID_FAULT_EXCEED_CAL_POWER && consumed > 300)) {
        return false;
      }
      global.Planner.Conf.PlanningCalPowerMaxPerSlot = available - consumed;
      global.Planner.Conf.UseMoreCalUnits = !global.Planner.Conf.ParkingSearch &&
                                            param->SltState != APA_SLOT_STATUS_TARGET_SLOT_REPLAN;
    }
  }
  fprintf(stderr,
          "[APA_HT_CONNECT] phase=direct_search engine=matlab budget=%ld\n",
          (long)global.Planner.Conf.PlanningCalPowerMaxPerSlot);

  collision.ReverseOutput = reversePlan;
  if (reversePlan)
    apaHybridAStarConfigure(&planner, &global, goal, start, &slot, true);
  else
    apaHybridAStarConfigure(&planner, &global, start, goal, &slot, false);
  const Boolean found = apaPlannerHybridSearch(&planner,
                                               &collision,
                                               &param->SceneQuery,
                                               &param->HybridPolicy,
                                               param->Diagnostics,
                                               HT_SEG_OUT_MAX,
                                               &segments,
                                               0);
  if (!found)
    return false;
  geoHtHybridCopySegmentsToOutput(&segments, output);
  return true;
}

static Boolean geoHtTryHybridFallback(const GeoHtPathPlanningInput_t *input,
                                      const GeoHtPathPlanningParam_t *param,
                                      Boolean allowManeuver,
                                      GeoHtPathPlanningOutput_t *output) {
  if (!apaPlannerHybridResolveFusion(param->NeedHybridAlgFusion))
    return false;
  return geoHtRunHybridFallback(input, param, allowManeuver, output);
}

static void geoHtMatlabModifyInitialTurn(const GeoHtPathPlanningInput_t *input,
                                         const GeoHtPathPlanningParam_t *param,
                                         GeoHtPathPlanningOutput_t *candidate) {
  if (!candidate->PlanFlg || candidate->SegNum < 3 || candidate->SegNum >= HT_SEG_OUT_MAX ||
      candidate->SegGoDir[0] != 1 || candidate->SegTrnDir[0] != 0 || candidate->SegGoDir[1] != -1 ||
      candidate->SegTrnDir[1] == 0)
    return;

  Real relX, relY, relYaw, centerX, centerY, centerYaw;
  geoAxisTransToTarget(input->TarX,
                       input->TarY,
                       input->TarTheta,
                       input->VehX,
                       input->VehY,
                       input->VehTheta,
                       &relX,
                       &relY,
                       &relYaw);
  geoAxisTransToTarget(candidate->SegCcX[1],
                       candidate->SegCcY[1],
                       0.0f,
                       input->VehX,
                       input->VehY,
                       input->VehTheta,
                       &centerX,
                       &centerY,
                       &centerYaw);
  const Boolean left = relY > 0.0f && candidate->SegTrnDir[1] == 1;
  const Boolean right = relY < 0.0f && candidate->SegTrnDir[1] == -1;
  if (!left && !right)
    return;

  GeoStPtSpcVtInput_t spaceInput;
  GeoStPtSpcVtOutput_t space = {0};
  geoHtMatlabMainBuildStPtSpcInput(input,
                                   param,
                                   candidate->SegStX[1],
                                   candidate->SegStY[1],
                                   candidate->SegStTh[1],
                                   0.0f,
                                   &spaceInput);
  geoStPtSpcVtWithScene(&spaceInput, &param->SceneQuery, &space);
  candidate->PlnChkCnt += space.CheckCount;

  Real factor = 1.0f;
  if (candidate->SegLen[0] <= 2.2f) {
    const Real first = candidate->SegLen[0] > 0.5f ? candidate->SegLen[0] * 0.7f - 0.6f : 0.1f;
    const Real second = candidate->SegLen[1] < 2.5f
                            ? 1.0f
                            : candidate->SegLen[1] < 4.0f
                                  ? 2.5f / (candidate->SegLen[1] * candidate->SegLen[1])
                                  : 0.1f;
    factor = first * second;
  }
  const Real radiusFactor = fminf(1.5f, fmaxf(1.06f, 1.0f + factor));
  const Real radius2 = 1.1f * input->Rmin;
  const Real front = param->VehLlF + param->VehLfMin * 0.5f;
  const Real available = left ? space.RightSpace + param->VehW * 0.5f + centerY
                              : space.LeftSpace + param->VehW * 0.5f - centerY;
  const Real radiusLimit =
      available > front
          ? fmaxf(radius2, sqrtf(available * available - front * front) - param->VehW * 0.5f)
          : radius2;

  for (Integer trial = 0; trial < 2; ++trial) {
    Real radius1 = radiusFactor * input->Rmin;
    if (trial != 0)
      radius1 = fminf(radius1, radiusLimit);
    GeoRev2CircleLinkInput_t linkInput = {0};
    GeoRev2CircleLinkOutput_t link = {0};
    linkInput.Pt1X = candidate->SegStX[2];
    linkInput.Pt1Y = candidate->SegStY[2];
    linkInput.Pt1Theta = candidate->SegStTh[2];
    linkInput.Pt2X = input->VehX;
    linkInput.Pt2Y = input->VehY;
    linkInput.Pt2Theta = input->VehTheta;
    linkInput.Dir = 1;
    linkInput.R1 = radius1;
    linkInput.R2 = radius2;
    geoRev2CircleLink(&linkInput, &link);
    if (link.GeoOk != 1)
      continue;

    GeoCircleIntrvCheckInput_t arcInput;
    GeoCircleIntrvCheckOutput_t arc = {0};
    geoHtFillCircleInput(input,
                         param,
                         link.Ct1X,
                         link.Ct1Y,
                         radius1,
                         linkInput.Pt1X,
                         linkInput.Pt1Y,
                         linkInput.Pt1Theta,
                         link.St2X,
                         link.St2Y,
                         link.St2Theta,
                         1,
                         link.TrnD1,
                         0.2f,
                         &arcInput);
    geoCircleIntrvCheckWithScene(&arcInput, &param->SceneQuery, &arc);
    candidate->PlnChkCnt += arc.CheckCount;
    geoAxisTransToTarget(link.St3X,
                         link.St3Y,
                         link.St3Theta,
                         input->VehX,
                         input->VehY,
                         input->VehTheta,
                         &relX,
                         &relY,
                         &relYaw);
    if (arc.IntrvFlg != 0 || relX < 0.0f)
      continue;

    geoHtFillCircleInput(input,
                         param,
                         link.Ct2X,
                         link.Ct2Y,
                         radius2,
                         link.St2X,
                         link.St2Y,
                         link.St2Theta,
                         link.St3X,
                         link.St3Y,
                         link.St3Theta,
                         -1,
                         link.TrnD2,
                         0.2f,
                         &arcInput);
    geoCircleIntrvCheckWithScene(&arcInput, &param->SceneQuery, &arc);
    candidate->PlnChkCnt += arc.CheckCount;
    if (arc.IntrvFlg != 0)
      continue;

    for (Integer i = candidate->SegNum; i > 0; --i) {
      candidate->SegGoDir[i] = candidate->SegGoDir[i - 1];
      candidate->SegTrnDir[i] = candidate->SegTrnDir[i - 1];
      candidate->SegCrv[i] = candidate->SegCrv[i - 1];
      candidate->SegTht[i] = candidate->SegTht[i - 1];
      candidate->SegLen[i] = candidate->SegLen[i - 1];
      candidate->SegStX[i] = candidate->SegStX[i - 1];
      candidate->SegStY[i] = candidate->SegStY[i - 1];
      candidate->SegStTh[i] = candidate->SegStTh[i - 1];
      candidate->SegCcX[i] = candidate->SegCcX[i - 1];
      candidate->SegCcY[i] = candidate->SegCcY[i - 1];
    }
    ++candidate->SegNum;
    geoHtCommitDirectLineSeg(
        candidate, 0, input->VehX, input->VehY, input->VehTheta, 1, fabsf(relX));
    geoHtCommitInSlotArcSeg(candidate,
                            1,
                            link.St3X,
                            link.St3Y,
                            link.St3Theta,
                            1,
                            link.TrnD2,
                            radius2,
                            link.St2Theta,
                            link.Len2,
                            link.Ct2X,
                            link.Ct2Y);
    geoHtCommitInSlotArcSeg(candidate,
                            2,
                            link.St2X,
                            link.St2Y,
                            link.St2Theta,
                            -1,
                            link.TrnD1,
                            radius1,
                            linkInput.Pt1Theta,
                            link.Len1,
                            link.Ct1X,
                            link.Ct1Y);
    candidate->SegTht[1] = link.Tht2;
    candidate->SegTht[2] = link.Tht1;
    candidate->PathLen = 0.0f;
    for (Integer i = 0; i < candidate->SegNum; ++i)
      candidate->PathLen += candidate->SegLen[i];
    candidate->StpN = geoHtMatlabMainCountOutputStpN(candidate);
    return;
  }
}

static Boolean geoHtCandidateAllowed(const GeoHtPathPlanningOutput_t *candidate,
                                     const GeoHtPathPlanningParam_t *param) {
  return candidate->PlanFlg != 0 && apaPlannerHybridCheckGeometry(candidate->SegNum,
                                                                  HT_SEG_OUT_MAX,
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

/**
 * @brief HT(水平泊入)路径规划器 — 编排层
 * @en_name geoHtPathPlanning
 * @cn_name HT路径规划器
 * @type widget_module
 * @config cn_name=车位状态, en_name=SltState, desc=APA车位状态输入, unit=1, type=Integer, range=[0,6], default=1
 * @config cn_name=车宽, en_name=VehW, desc=车辆宽度, unit=m, type=Real, range=[1.0,3.0], default=1.9
 * @config cn_name=前悬到参考点, en_name=VehLlF, desc=前悬到规划参考点长度, unit=m, type=Real, range=[0.1,3.0], default=0.9
 * @config cn_name=最小出口半径, en_name=RfoutMin, desc=水平泊入最小出口转弯半径, unit=m, type=Real, range=[1.0,10.0], default=5.5
 * @config cn_name=Hybrid融合开关, en_name=NeedHybridAlgFusion, desc=是否允许在几何规划后评估HybridA*候选, unit=1, type=Integer, range=[0,1], default=1
 * @param[IN] Real VehX 车辆X
 * @param[IN] Real VehY 车辆Y
 * @param[IN] Real VehTheta 车辆航向
 * @param[IN] Real TarX 目标X
 * @param[IN] Real TarY 目标Y
 * @param[IN] Real TarTheta 目标航向
 * @param[IN] Real Rmin 最小转弯半径
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
 * @param[OUT] Integer[30] SegGoDir 各段行驶方向
 * @param[OUT] Integer[30] SegTrnDir 各段转弯方向
 * @param[OUT] Real[30] SegCrv 各段曲率
 * @param[OUT] Real[30] SegTht 各段转角
 * @param[OUT] Real[30] SegLen 各段长度
 * @param[OUT] Real[30] SegStX 各段起点X
 * @param[OUT] Real[30] SegStY 各段起点Y
 * @param[OUT] Real[30] SegStTh 各段起点航向
 * @param[OUT] Real[30] SegCcX 各段圆心X
 * @param[OUT] Real[30] SegCcY 各段圆心Y
 * @param[OUT] Real DngPtX 危险点X
 * @param[OUT] Real DngPtY 危险点Y
 * @param[OUT] Integer PlnChkCnt 规划检查次数
 * @param[OUT] Integer FailRs0 失败原因首项
 * @param[OUT] Integer StpN 换向次数
 * @param[OUT] Integer PlannerCase 规划分支编号
 * @param[OUT] Integer FallbackGeomCandidateFlg 几何候选存在标志
 * @param[OUT] Integer FallbackHybridCandidateFlg Hybrid候选存在标志
 * @param[OUT] Integer FallbackWinnerKind 回退赢家0无1几何2Hybrid
 * @param[OUT] Integer FallbackDecisionReason 回退决策原因0无1仅有效性2换向次数3路径长度4段数5检查次数6原生路径质量
 * @param[OUT] Integer FallbackGeomStpN 几何候选换向次数
 * @param[OUT] Real FallbackGeomPathLen 几何候选路径总长
 * @param[OUT] Integer FallbackGeomSegNum 几何候选段数
 * @param[OUT] Integer FallbackGeomPlnChkCnt 几何候选检查次数
 * @param[OUT] Integer FallbackHybridStpN Hybrid候选换向次数
 * @param[OUT] Real FallbackHybridPathLen Hybrid候选路径总长
 * @param[OUT] Integer FallbackHybridSegNum Hybrid候选段数
 * @param[OUT] Integer FallbackHybridPlnChkCnt Hybrid候选检查次数
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 HT水平泊入
 * @source matlab_rewrite geo_HT_PathPlanning.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
void geoHtPathPlanning(const GeoHtPathPlanningInput_t *input,
                       const GeoHtPathPlanningParam_t *param,
                       GeoHtPathPlanningOutput_t *output) {
  GeoHtPathPlanningInput_t resolvedInput;
  Boolean inSlotReplanInput;
  Boolean inSlotReplanCandidateOk;
  Boolean longEdgeInput;
  Boolean longEdgeCandidateOk;
  Boolean matlabCandidateOk;
  Boolean matlabMainLocked;
  Boolean targetPrecheckSafe;
  GeoHtPathPlanningOutput_t inSlotReplanCandidate;
  GeoHtPathPlanningOutput_t longEdgeCandidate;
  GeoHtPathPlanningOutput_t matlabCandidate;

  *output = (GeoHtPathPlanningOutput_t){0};

  if (input->SurdType == APA_SURROUND_UNKNOWN) {
    resolvedInput = *input;
    resolvedInput.SurdType = geoHtResolveUnknownSurroundingType(input, param);
    input = &resolvedInput;
  }

  targetPrecheckSafe = geoHtMatlabMainRunTargetPrecheck(input, param, output);
  if (!targetPrecheckSafe)
    apaPlanningRecordFailure(
        param->Diagnostics, APA_FAULT_COLLISION_AT_TARGET_SLOT, "ht_target_precheck", 5);
  inSlotReplanInput = false;
  longEdgeInput = false;
  matlabMainLocked = false;
  inSlotReplanCandidate = (GeoHtPathPlanningOutput_t){0};
  longEdgeCandidate = (GeoHtPathPlanningOutput_t){0};
  matlabCandidate = (GeoHtPathPlanningOutput_t){0};

  if (targetPrecheckSafe) {
    inSlotReplanInput = geoHtMatlabMainShouldUseInSlotReplan(input, param);
    longEdgeInput = geoHtIsLongEdgeEntrySlot(input);
  }

  if (targetPrecheckSafe && inSlotReplanInput) {
    inSlotReplanCandidateOk = geoHtTryMatlabInSlotReplan(input, param, &inSlotReplanCandidate);
    matlabMainLocked = inSlotReplanCandidateOk;
    if (!inSlotReplanCandidateOk) {
      inSlotReplanCandidate.PlannerCase = GEO_HT_PLANNER_CASE_NONE;
    }
    *output = inSlotReplanCandidate;
  }
  if (targetPrecheckSafe && (!inSlotReplanInput) && longEdgeInput) {
    longEdgeCandidateOk = geoHtTryMatlabExternalLongEdgeLoop(input, param, &longEdgeCandidate);
    matlabMainLocked = longEdgeCandidateOk;
    *output = longEdgeCandidate;
  }
  if (targetPrecheckSafe && (!inSlotReplanInput) && (!longEdgeInput)) {
    matlabCandidateOk = geoHtTryMatlabNormalLoop(input, param, &matlabCandidate);
    matlabMainLocked = matlabCandidateOk;
    *output = matlabCandidate;
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
    GeoHtPathPlanningOutput_t geometricCandidate;
    GeoHtPathPlanningOutput_t hybridCandidate;
    GeoHtPathPlanningOutput_t bestCandidate;

    allowHybridFallback = false;
    if (!matlabMainLocked) {
      allowHybridFallback = (apaPlannerHybridResolveFusion(param->NeedHybridAlgFusion) != 0);
    }
    geoHtMatlabModifyInitialTurn(input, param, output);
    geometricCandidate = *output;
    hybridCandidate = (GeoHtPathPlanningOutput_t){0};
    bestCandidate = geometricCandidate;
    geometricOk = geoHtHasValidCandidate(&geometricCandidate) &&
                  geoHtCandidateAllowed(&geometricCandidate, param);
    hybridOk = false;
    if (!geometricOk) {
      geometricCandidate = (GeoHtPathPlanningOutput_t){0};
      bestCandidate = geometricCandidate;
      output->PlanFlg = 0;
      allowHybridFallback = apaPlannerHybridResolveFusion(param->NeedHybridAlgFusion) != 0;
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

    geoHtStoreFallbackDiagFromCandidate(geometricOk,
                                        &geometricCandidate,
                                        &fallbackGeomStpN,
                                        &fallbackGeomPathLen,
                                        &fallbackGeomSegNum,
                                        &fallbackGeomPlnChkCnt);

    if (allowHybridFallback && (!matlabMainLocked || !geometricOk)) {
      hybridOk = geoHtTryHybridFallback(input, param, !geometricOk, &hybridCandidate);
      if (hybridOk) {
        fallbackHybridCandidateFlg = 1;
      } else {
        fallbackHybridCandidateFlg = 0;
      }
      geoHtStoreFallbackDiagFromCandidate(hybridOk,
                                          &hybridCandidate,
                                          &fallbackHybridStpN,
                                          &fallbackHybridPathLen,
                                          &fallbackHybridSegNum,
                                          &fallbackHybridPlnChkCnt);
      if (!param->HybridPolicy.CompareCandidates && hybridOk && geoHtIsLongEdgeEntrySlot(input)) {
        bestCandidate = hybridCandidate;
      } else if (geoHtShouldReplaceCandidate(&bestCandidate, &hybridCandidate, param)) {
        bestCandidate = hybridCandidate;
      }
    }

    fallbackWinnerKind = geoHtResolveFallbackWinnerKind(geometricOk, hybridOk, &bestCandidate);
    fallbackDecisionReason = geoHtResolveFallbackDecisionReason(
        geometricOk, hybridOk, &geometricCandidate, &hybridCandidate, param);
    if (geoHtHasValidCandidate(&bestCandidate)) {
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
