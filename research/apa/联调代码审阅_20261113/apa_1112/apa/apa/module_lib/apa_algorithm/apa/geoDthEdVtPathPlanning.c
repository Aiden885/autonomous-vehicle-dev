#include "geoDthEdVtPathPlanningTypes.h"
#include "apaVehicleDefaults.h"

#include "apaCountUtils.h"
#include "apaPlannerHybridParity.h"
#include "apaPlannerUtils.h"
#include "apaSlotTypes.h"
#include "checkParkingTypeTypes.h"
#include "geo2CircleLinkTypes.h"
#include "geoCircle1pt1lineTypes.h"
#include "geoCircle2lineRTypes.h"
#include "geoCircleIntrvCheckTypes.h"
#include "geoLineIntrvCheckTypes.h"
#include "geoPosTypClassPkInTypes.h"
#include "geoRev2CircleLinkTypes.h"
#include "geoStPtSpcDthEdVtTypes.h"
#include "geoVtEntrPtDngPtTypes.h"
#include "vehicleCollisionCheckTypes.h"
#include <math.h>

#ifndef GEO_DTH_ED_VT_EPS
#define GEO_DTH_ED_VT_EPS 1e-6f
#endif

#ifndef GEO_DTH_ED_VT_DEFAULT_RADIUS
#define GEO_DTH_ED_VT_DEFAULT_RADIUS 5.0f
#endif

#ifndef GEO_DTH_ED_VT_DEFAULT_WHEELBASE
#define GEO_DTH_ED_VT_DEFAULT_WHEELBASE APA_VEHICLE_WHEELBASE_M
#endif

#ifndef GEO_DTH_ED_VT_PRECHECK_FORWARD
#define GEO_DTH_ED_VT_PRECHECK_FORWARD 1.5f
#endif

#ifndef GEO_DTH_ED_VT_COMPRESS_LEN_EPS
#define GEO_DTH_ED_VT_COMPRESS_LEN_EPS 0.1f
#endif

#ifndef GEO_DTH_ED_VT_COMPRESS_CRV_EPS
#define GEO_DTH_ED_VT_COMPRESS_CRV_EPS 0.08f
#endif

#ifndef GEO_DTH_ED_VT_COMPRESS_CC_EPS
#define GEO_DTH_ED_VT_COMPRESS_CC_EPS 0.4f
#endif

#ifndef GEO_DTH_ED_VT_APPEND_CRV_EPS
#define GEO_DTH_ED_VT_APPEND_CRV_EPS 0.1f
#endif

#ifndef GEO_DTH_ED_VT_STRAIGHT_LAT_EPS
#define GEO_DTH_ED_VT_STRAIGHT_LAT_EPS 0.05f
#endif

#ifndef GEO_DTH_ED_VT_STRAIGHT_MIN_LEN
#define GEO_DTH_ED_VT_STRAIGHT_MIN_LEN 0.05f
#endif

#ifndef GEO_DTH_ED_VT_LOW_RADIUS_DELTA
#define GEO_DTH_ED_VT_LOW_RADIUS_DELTA 0.2f
#endif

#ifndef GEO_DTH_ED_VT_CIR_LEN_MAX
#define GEO_DTH_ED_VT_CIR_LEN_MAX 10.0f
#endif

#ifndef GEO_DTH_ED_VT_CIR_EDX_MAX
#define GEO_DTH_ED_VT_CIR_EDX_MAX 5.0f
#endif

#ifndef GEO_DTH_ED_VT_CIR_EDY_MAX
#define GEO_DTH_ED_VT_CIR_EDY_MAX 10.0f
#endif

#ifndef GEO_DTH_ED_VT_CIR_THETA_MAX
#define GEO_DTH_ED_VT_CIR_THETA_MAX (APA_PI / 4.0f)
#endif

#ifndef GEO_DTH_ED_VT_STAGE_MAX
#define GEO_DTH_ED_VT_STAGE_MAX 30
#endif

#ifndef GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_NONE
#define GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_NONE 0
#endif

#ifndef GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_1
#define GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_1 1
#endif

#ifndef GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_2
#define GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_2 2
#endif

#ifndef GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_3
#define GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_3 3
#endif

static Real geoDthEdVtResolveRadius(const GeoDthEdVtPathPlanningInput_t *input) {
  Real radius0;

  radius0 = input->Rmin;
  if (radius0 <= GEO_DTH_ED_VT_EPS) {
    radius0 = GEO_DTH_ED_VT_DEFAULT_RADIUS;
  }
  return radius0;
}

static void geoDthEdVtBuildParkingTypeInput(const GeoDthEdVtPathPlanningInput_t *input,
                                            CheckParkingTypeInput_t *parkingInput) {
  *parkingInput = (CheckParkingTypeInput_t){0};
  parkingInput->VehX = input->VehX;
  parkingInput->VehY = input->VehY;
  parkingInput->VehTheta = input->VehTheta;
  parkingInput->TarX = input->TarX;
  parkingInput->TarY = input->TarY;
  parkingInput->TarTheta = input->TarTheta;
  parkingInput->SltPt1X = input->SltPt1X;
  parkingInput->SltPt1Y = input->SltPt1Y;
  parkingInput->SltPt2X = input->SltPt2X;
  parkingInput->SltPt2Y = input->SltPt2Y;
  parkingInput->SltPt3X = input->SltPt3X;
  parkingInput->SltPt3Y = input->SltPt3Y;
  parkingInput->SltPt4X = input->SltPt4X;
  parkingInput->SltPt4Y = input->SltPt4Y;
  parkingInput->EnterEdge = input->SltEntr;
}

static Integer geoDthEdVtResolveParkingType(const GeoDthEdVtPathPlanningInput_t *input) {
  CheckParkingTypeInput_t parkingInput;
  CheckParkingTypeOutput_t parkingOutput;
  Integer parkingType0;

  parkingType0 = input->PrkType;
  if (parkingType0 == 0) {
    geoDthEdVtBuildParkingTypeInput(input, &parkingInput);
    parkingOutput = (CheckParkingTypeOutput_t){0};
    checkParkingType(&parkingInput, &parkingOutput);
    parkingType0 = parkingOutput.ParkingType;
  }
  return parkingType0;
}

static void geoDthEdVtCalcEntranceMidpoint(const GeoDthEdVtPathPlanningInput_t *input,
                                           Real *entrX,
                                           Real *entrY) {
  Real p1x0;
  Real p1y0;
  Real p2x0;
  Real p2y0;

  p1x0 = input->SltPt1X;
  p1y0 = input->SltPt1Y;
  p2x0 = input->SltPt2X;
  p2y0 = input->SltPt2Y;
  if (input->SltEntr == 2) {
    p1x0 = input->SltPt2X;
    p1y0 = input->SltPt2Y;
    p2x0 = input->SltPt3X;
    p2y0 = input->SltPt3Y;
  } else if (input->SltEntr == 3) {
    p1x0 = input->SltPt3X;
    p1y0 = input->SltPt3Y;
    p2x0 = input->SltPt4X;
    p2y0 = input->SltPt4Y;
  } else if (input->SltEntr == 4) {
    p1x0 = input->SltPt4X;
    p1y0 = input->SltPt4Y;
    p2x0 = input->SltPt1X;
    p2y0 = input->SltPt1Y;
  }
  *entrX = (p1x0 + p2x0) * 0.5f;
  *entrY = (p1y0 + p2y0) * 0.5f;
}

static void geoDthEdVtBuildEntranceInput(const GeoDthEdVtPathPlanningInput_t *input,
                                         GeoVtEntrPtDngPtInput_t *entrInput) {
  Integer boundaryLineCount0;
  Integer boundarySepCount0;
  Integer idx;
  Integer stopperCount0;
  Integer virtualLineCount0;

  *entrInput = (GeoVtEntrPtDngPtInput_t){0};
  boundaryLineCount0 =
      apaClampCountToLimits(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX, DGR_LINE_MAX);
  virtualLineCount0 =
      apaClampCountToLimits(input->VirtualBoundaryLineNum, GEO_BOUNDARY_LINE_MAX, DGR_LINE_MAX);
  boundarySepCount0 =
      apaClampCountToLimits(input->BoundarySepPtNum, GEO_BOUNDARY_POINT_MAX, DGR_PT_MAX);
  stopperCount0 = apaClampCountToLimits(input->StopperPtNum, GEO_BOUNDARY_POINT_MAX, DGR_PT_MAX);
  entrInput->SltPt1X = input->SltPt1X;
  entrInput->SltPt1Y = input->SltPt1Y;
  entrInput->SltPt2X = input->SltPt2X;
  entrInput->SltPt2Y = input->SltPt2Y;
  entrInput->SltPt3X = input->SltPt3X;
  entrInput->SltPt3Y = input->SltPt3Y;
  entrInput->SltPt4X = input->SltPt4X;
  entrInput->SltPt4Y = input->SltPt4Y;
  entrInput->SltEntr = input->SltEntr;
  entrInput->VehX = input->VehX;
  entrInput->VehY = input->VehY;
  entrInput->VehTheta = input->VehTheta;
  entrInput->TgtX = input->TarX;
  entrInput->TgtY = input->TarY;
  entrInput->TgtTheta = input->TarTheta;
  entrInput->Radius = geoDthEdVtResolveRadius(input);
  entrInput->PrkType = geoDthEdVtResolveParkingType(input);
  entrInput->SurdType = input->SurdType;
  entrInput->AgrOfst = input->AgrOfst;
  entrInput->BoundaryLineNum = boundaryLineCount0;
  entrInput->VirtualBoundaryLineNum = virtualLineCount0;
  entrInput->BoundarySepPtNum = boundarySepCount0;
  entrInput->StopperPtNum = stopperCount0;

  idx = 0;
  while (idx < boundaryLineCount0) {
    entrInput->BoundaryP1X[idx] = input->BoundaryLineX1[idx];
    entrInput->BoundaryP1Y[idx] = input->BoundaryLineY1[idx];
    entrInput->BoundaryP2X[idx] = input->BoundaryLineX2[idx];
    entrInput->BoundaryP2Y[idx] = input->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < virtualLineCount0) {
    entrInput->VirtualBoundaryP1X[idx] = input->VirtualBoundaryP1X[idx];
    entrInput->VirtualBoundaryP1Y[idx] = input->VirtualBoundaryP1Y[idx];
    entrInput->VirtualBoundaryP2X[idx] = input->VirtualBoundaryP2X[idx];
    entrInput->VirtualBoundaryP2Y[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < boundarySepCount0) {
    entrInput->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    entrInput->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < stopperCount0) {
    entrInput->StopperPtX[idx] = input->StopperPtX[idx];
    entrInput->StopperPtY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }
}

static void geoDthEdVtResolveEntrance(const GeoDthEdVtPathPlanningInput_t *input,
                                      GeoDthEdVtPathPlanningOutput_t *output) {
  GeoVtEntrPtDngPtInput_t entrInput;
  GeoVtEntrPtDngPtOutput_t entrOutput;
  Boolean hasExplicitEntrance0;
  Boolean hasPlanEntrance0;

  geoDthEdVtCalcEntranceMidpoint(input, &output->Pt1X, &output->Pt1Y);
  hasExplicitEntrance0 =
      (fabsf(input->EntrPtX) > GEO_DTH_ED_VT_EPS) || (fabsf(input->EntrPtY) > GEO_DTH_ED_VT_EPS);
  if (hasExplicitEntrance0) {
    output->Pt1X = input->EntrPtX;
    output->Pt1Y = input->EntrPtY;
  }

  geoDthEdVtBuildEntranceInput(input, &entrInput);
  entrOutput = (GeoVtEntrPtDngPtOutput_t){0};
  geoVtEntrPtDngPt(&entrInput, &entrOutput);
  hasPlanEntrance0 = (fabsf(entrOutput.EntrPlanX) > GEO_DTH_ED_VT_EPS) ||
                     (fabsf(entrOutput.EntrPlanY) > GEO_DTH_ED_VT_EPS);
  if (hasPlanEntrance0) {
    output->Pt1X = entrOutput.EntrPlanX;
    output->Pt1Y = entrOutput.EntrPlanY;
  }
  output->DngPtX = entrOutput.DgrPtX;
  output->DngPtY = entrOutput.DgrPtY;
  output->EntrDgrPtFlg = entrOutput.DgrPtFlg;
  output->EntrCheckCnt = entrOutput.CheckCount;
}

static void geoDthEdVtBuildSpaceInput(const GeoDthEdVtPathPlanningInput_t *input,
                                      const GeoDthEdVtPathPlanningParam_t *param,
                                      const GeoDthEdVtPathPlanningOutput_t *output,
                                      GeoStPtSpcDthEdVtInput_t *spaceInput) {
  Integer idx;
  Integer count0;
  Integer stopperPointCount0;
  VehicleCollisionCheckInput_t *vccTemplate;

  *spaceInput = (GeoStPtSpcDthEdVtInput_t){0};
  stopperPointCount0 = apaClampCount(input->StopperPtNum, GEO_BOUNDARY_POINT_MAX);
  stopperPointCount0 = apaClampCount(stopperPointCount0, VCC_MAX_SEPARATE_PTS);
  spaceInput->StPtX = output->Pt1X;
  spaceInput->StPtY = output->Pt1Y;
  spaceInput->StPtTheta = input->TarTheta;
  spaceInput->TarPtX = input->TarX;
  spaceInput->TarPtY = input->TarY;
  spaceInput->TarPtTheta = input->TarTheta;
  spaceInput->SeparateStopperNum = stopperPointCount0;
  spaceInput->SlotPointsX[0] = input->SltPt1X;
  spaceInput->SlotPointsY[0] = input->SltPt1Y;
  spaceInput->SlotPointsX[1] = input->SltPt2X;
  spaceInput->SlotPointsY[1] = input->SltPt2Y;
  spaceInput->SlotPointsX[2] = input->SltPt3X;
  spaceInput->SlotPointsY[2] = input->SltPt3Y;
  spaceInput->SlotPointsX[3] = input->SltPt4X;
  spaceInput->SlotPointsY[3] = input->SltPt4Y;
  spaceInput->VehLF = param->VehLlF;
  spaceInput->VehLB = param->VehLr;
  spaceInput->VehW = param->VehW;
  spaceInput->UseVccTemplate = true;

  vccTemplate = &spaceInput->VccTemplate;
  vccTemplate->CheckType = VCC_CHECK_TYPE_PLANNING;
  vccTemplate->CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  vccTemplate->CollisionDemand = VCC_COLLISION_DEMAND_ALL;
  vccTemplate->UseAutoMargin = true;
  vccTemplate->SlotClass = VCC_SLOT_CLASS_DIAGONAL;
  vccTemplate->SlotPtsValid = true;
  vccTemplate->SlotPts[0] = input->SltPt1X;
  vccTemplate->SlotPts[1] = input->SltPt1Y;
  vccTemplate->SlotPts[2] = input->SltPt2X;
  vccTemplate->SlotPts[3] = input->SltPt2Y;
  vccTemplate->SlotPts[4] = input->SltPt3X;
  vccTemplate->SlotPts[5] = input->SltPt3Y;
  vccTemplate->SlotPts[6] = input->SltPt4X;
  vccTemplate->SlotPts[7] = input->SltPt4Y;
  apaSlotOrderFromEntrance(input->SltEntr,
                           &vccTemplate->SlotPts[0],
                           &vccTemplate->SlotPts[1],
                           &vccTemplate->SlotPts[2],
                           &vccTemplate->SlotPts[3],
                           &vccTemplate->SlotPts[4],
                           &vccTemplate->SlotPts[5],
                           &vccTemplate->SlotPts[6],
                           &vccTemplate->SlotPts[7]);
  vccTemplate->TarPosValid = true;
  vccTemplate->TarPosX = input->TarX;
  vccTemplate->TarPosY = input->TarY;
  vccTemplate->TarPosTheta = input->TarTheta;
  vccTemplate->VehLF = spaceInput->VehLF;
  vccTemplate->VehLB = spaceInput->VehLB;
  vccTemplate->VehW = spaceInput->VehW;
  vccTemplate->VehWB = GEO_DTH_ED_VT_DEFAULT_WHEELBASE;
  vccTemplate->VehLL = spaceInput->VehLF + spaceInput->VehLB;
  vehicleCollisionCheckLoadDefaultMargins(vccTemplate);
  vccTemplate->BoundaryLineNum = 3;
  vccTemplate->BoundaryLineX1[0] = input->SltPt1X;
  vccTemplate->BoundaryLineY1[0] = input->SltPt1Y;
  vccTemplate->BoundaryLineX2[0] = input->SltPt4X;
  vccTemplate->BoundaryLineY2[0] = input->SltPt4Y;
  vccTemplate->BoundaryLineX1[1] = input->SltPt4X;
  vccTemplate->BoundaryLineY1[1] = input->SltPt4Y;
  vccTemplate->BoundaryLineX2[1] = input->SltPt3X;
  vccTemplate->BoundaryLineY2[1] = input->SltPt3Y;
  vccTemplate->BoundaryLineX1[2] = input->SltPt3X;
  vccTemplate->BoundaryLineY1[2] = input->SltPt3Y;
  vccTemplate->BoundaryLineX2[2] = input->SltPt2X;
  vccTemplate->BoundaryLineY2[2] = input->SltPt2Y;

  idx = 0;
  while ((idx < input->BoundaryLineNum) && (idx < GEO_BOUNDARY_LINE_MAX) &&
         (vccTemplate->BoundaryLineNum < VCC_MAX_BOUNDARY_LINES)) {
    count0 = vccTemplate->BoundaryLineNum;
    vccTemplate->BoundaryLineX1[count0] = input->BoundaryLineX1[idx];
    vccTemplate->BoundaryLineY1[count0] = input->BoundaryLineY1[idx];
    vccTemplate->BoundaryLineX2[count0] = input->BoundaryLineX2[idx];
    vccTemplate->BoundaryLineY2[count0] = input->BoundaryLineY2[idx];
    vccTemplate->BoundaryLineNum = count0 + 1;
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->BoundarySepPtNum) && (idx < GEO_BOUNDARY_POINT_MAX) &&
         (idx < VCC_MAX_SEPARATE_PTS)) {
    vccTemplate->BoundarySepPtsX[idx] = input->BoundarySepPtX[idx];
    vccTemplate->BoundarySepPtsY[idx] = input->BoundarySepPtY[idx];
    vccTemplate->BoundarySepPtsNum = idx + 1;
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->SlotTrackStopperNum) && (idx < GEO_BOUNDARY_LINE_MAX) &&
         (idx < VCC_MAX_TRACK_STOPPER)) {
    vccTemplate->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    vccTemplate->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    vccTemplate->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    vccTemplate->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    vccTemplate->SlotTrackStopperNum = idx + 1;
    idx = idx + 1;
  }

  idx = 0;
  while (idx < stopperPointCount0) {
    spaceInput->SeparateStopperX[idx] = input->StopperPtX[idx];
    spaceInput->SeparateStopperY[idx] = input->StopperPtY[idx];
    vccTemplate->SeparateStopperX[idx] = input->StopperPtX[idx];
    vccTemplate->SeparateStopperY[idx] = input->StopperPtY[idx];
    vccTemplate->SeparateStopperNum = idx + 1;
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->VirtualBoundaryLineNum) && (idx < GEO_BOUNDARY_LINE_MAX) &&
         (idx < VCC_MAX_VIRTUAL_LINES)) {
    vccTemplate->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    vccTemplate->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    vccTemplate->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    vccTemplate->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
    vccTemplate->VirtualBoundaryLineNum = idx + 1;
    idx = idx + 1;
  }
}

static void geoDthEdVtStoreSpaceOutput(const GeoStPtSpcDthEdVtOutput_t *spaceOutput,
                                       GeoDthEdVtPathPlanningOutput_t *output) {
  output->DthFtSpc = spaceOutput->FtSpc;
  output->DthLtSpc = spaceOutput->LtSpc;
  output->DthRtSpc = spaceOutput->RtSpc;
  output->DthSpaceCheckCnt = spaceOutput->ChkCnt;
  output->DthIntrvRS = spaceOutput->IntrvRS;
  output->DthCrshPosX = spaceOutput->CrshPosX;
  output->DthCrshPosY = spaceOutput->CrshPosY;
  output->DthCrshPosTheta = spaceOutput->CrshPosTheta;
  output->DthCheckPosCX = spaceOutput->ChckPosCX;
  output->DthCheckPosCY = spaceOutput->ChckPosCY;
  output->DthCheckPosCTheta = spaceOutput->ChckPosCTheta;
}

static void geoDthEdVtRunSpaceProbe(const GeoDthEdVtPathPlanningInput_t *input,
                                    const GeoDthEdVtPathPlanningParam_t *param,
                                    GeoDthEdVtPathPlanningOutput_t *output) {
  GeoStPtSpcDthEdVtInput_t spaceInput;
  GeoStPtSpcDthEdVtOutput_t spaceOutput;

  geoDthEdVtBuildSpaceInput(input, param, output, &spaceInput);
  spaceOutput = (GeoStPtSpcDthEdVtOutput_t){0};
  geoStPtSpcDthEdVtWithScene(&spaceInput, &param->SceneQuery, &spaceOutput);
  geoDthEdVtStoreSpaceOutput(&spaceOutput, output);
}

static void geoDthEdVtBuildCollisionInput(const GeoDthEdVtPathPlanningInput_t *input,
                                          const GeoDthEdVtPathPlanningParam_t *param,
                                          VehicleCollisionCheckInput_t *collisionInput) {
  Integer boundaryLineCount0;
  Integer boundarySepPointCount0;
  Integer idx;
  Integer slotTrackStopperCount0;
  Integer stopperPointCount0;
  Integer virtualBoundaryLineCount0;

  *collisionInput = (VehicleCollisionCheckInput_t){0};
  boundaryLineCount0 =
      apaClampCountToLimits(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_BOUNDARY_LINES);
  boundarySepPointCount0 =
      apaClampCountToLimits(input->BoundarySepPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  slotTrackStopperCount0 = apaClampCountToLimits(
      input->SlotTrackStopperNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_TRACK_STOPPER);
  stopperPointCount0 =
      apaClampCountToLimits(input->StopperPtNum, GEO_BOUNDARY_POINT_MAX, VCC_MAX_SEPARATE_PTS);
  virtualBoundaryLineCount0 = apaClampCountToLimits(
      input->VirtualBoundaryLineNum, GEO_BOUNDARY_LINE_MAX, VCC_MAX_VIRTUAL_LINES);
  collisionInput->CheckType = VCC_CHECK_TYPE_PLANNING;
  collisionInput->CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  collisionInput->CollisionDemand = VCC_COLLISION_DEMAND_ALL;
  collisionInput->UseAutoMargin = true;
  collisionInput->SlotClass = VCC_SLOT_CLASS_DIAGONAL;
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
  collisionInput->VehPosX = input->TarX + GEO_DTH_ED_VT_PRECHECK_FORWARD * cosf(input->TarTheta);
  collisionInput->VehPosY = input->TarY + GEO_DTH_ED_VT_PRECHECK_FORWARD * sinf(input->TarTheta);
  collisionInput->VehPosTheta = input->TarTheta;
  collisionInput->VehLF = param->VehLlF;
  collisionInput->VehLB = param->VehLr;
  collisionInput->VehW = param->VehW;
  collisionInput->VehWB = GEO_DTH_ED_VT_DEFAULT_WHEELBASE;
  collisionInput->VehLL = collisionInput->VehLF + collisionInput->VehLB;
  vehicleCollisionCheckLoadDefaultMargins(collisionInput);
  collisionInput->BoundaryLineNum = boundaryLineCount0;
  collisionInput->BoundarySepPtsNum = boundarySepPointCount0;
  collisionInput->SlotTrackStopperNum = slotTrackStopperCount0;
  collisionInput->SeparateStopperNum = stopperPointCount0;
  collisionInput->VirtualBoundaryLineNum = virtualBoundaryLineCount0;

  idx = 0;
  while (idx < boundaryLineCount0) {
    collisionInput->BoundaryLineX1[idx] = input->BoundaryLineX1[idx];
    collisionInput->BoundaryLineY1[idx] = input->BoundaryLineY1[idx];
    collisionInput->BoundaryLineX2[idx] = input->BoundaryLineX2[idx];
    collisionInput->BoundaryLineY2[idx] = input->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < boundarySepPointCount0) {
    collisionInput->BoundarySepPtsX[idx] = input->BoundarySepPtX[idx];
    collisionInput->BoundarySepPtsY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < slotTrackStopperCount0) {
    collisionInput->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    collisionInput->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    collisionInput->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    collisionInput->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < stopperPointCount0) {
    collisionInput->SeparateStopperX[idx] = input->StopperPtX[idx];
    collisionInput->SeparateStopperY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < virtualBoundaryLineCount0) {
    collisionInput->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    collisionInput->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    collisionInput->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    collisionInput->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }
}

static Integer geoDthEdVtPrecheckCollisionType(const GeoDthEdVtPathPlanningInput_t *input,
                                               const GeoDthEdVtPathPlanningParam_t *param) {
  VehicleCollisionCheckInput_t collisionInput;
  VehicleCollisionCheckOutput_t collisionOutput;
  Integer collisionType0;

  geoDthEdVtBuildCollisionInput(input, param, &collisionInput);
  collisionOutput = (VehicleCollisionCheckOutput_t){0};
  apaPlanningSceneCollisionCheck(&collisionInput, &param->SceneQuery, &collisionOutput);
  collisionType0 = VCC_COLLISION_NONE;
  if (collisionOutput.IsCollision) {
    collisionType0 = collisionOutput.CollisionType;
  }
  return collisionType0;
}

static void geoDthEdVtBuildVtInput(const GeoDthEdVtPathPlanningInput_t *input,
                                   const GeoDthEdVtPathPlanningOutput_t *output,
                                   GeoVtPathPlanningInput_t *vtInput) {
  Integer boundaryLineCount0;
  Integer boundarySepCount0;
  Integer idx;
  Integer stopperCount0;
  Integer virtualLineCount0;

  *vtInput = (GeoVtPathPlanningInput_t){0};
  boundaryLineCount0 = apaClampCount(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX);
  virtualLineCount0 = apaClampCount(input->VirtualBoundaryLineNum, GEO_BOUNDARY_LINE_MAX);
  boundarySepCount0 = apaClampCount(input->BoundarySepPtNum, GEO_BOUNDARY_POINT_MAX);
  stopperCount0 = apaClampCount(input->StopperPtNum, GEO_BOUNDARY_POINT_MAX);
  vtInput->VehX = input->VehX;
  vtInput->VehY = input->VehY;
  vtInput->VehTheta = input->VehTheta;
  vtInput->TarX = input->TarX;
  vtInput->TarY = input->TarY;
  vtInput->TarTheta = input->TarTheta;
  vtInput->Rmin = geoDthEdVtResolveRadius(input);
  vtInput->EntrPtX = output->Pt1X;
  vtInput->EntrPtY = output->Pt1Y;
  vtInput->SltPt1X = input->SltPt1X;
  vtInput->SltPt1Y = input->SltPt1Y;
  vtInput->SltPt2X = input->SltPt2X;
  vtInput->SltPt2Y = input->SltPt2Y;
  vtInput->SltPt3X = input->SltPt3X;
  vtInput->SltPt3Y = input->SltPt3Y;
  vtInput->SltPt4X = input->SltPt4X;
  vtInput->SltPt4Y = input->SltPt4Y;
  vtInput->SltEntr = input->SltEntr;
  vtInput->PrkType = geoDthEdVtResolveParkingType(input);
  vtInput->SurdType = input->SurdType;
  vtInput->AgrOfst = input->AgrOfst;
  vtInput->BoundaryLineNum = boundaryLineCount0;
  vtInput->VirtualBoundaryLineNum = virtualLineCount0;
  vtInput->BoundarySepPtNum = boundarySepCount0;
  vtInput->StopperPtNum = stopperCount0;

  idx = 0;
  while (idx < boundaryLineCount0) {
    vtInput->BoundaryLineX1[idx] = input->BoundaryLineX1[idx];
    vtInput->BoundaryLineY1[idx] = input->BoundaryLineY1[idx];
    vtInput->BoundaryLineX2[idx] = input->BoundaryLineX2[idx];
    vtInput->BoundaryLineY2[idx] = input->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < virtualLineCount0) {
    vtInput->VirtualBoundaryP1X[idx] = input->VirtualBoundaryP1X[idx];
    vtInput->VirtualBoundaryP1Y[idx] = input->VirtualBoundaryP1Y[idx];
    vtInput->VirtualBoundaryP2X[idx] = input->VirtualBoundaryP2X[idx];
    vtInput->VirtualBoundaryP2Y[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < boundarySepCount0) {
    vtInput->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    vtInput->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < stopperCount0) {
    vtInput->StopperPtX[idx] = input->StopperPtX[idx];
    vtInput->StopperPtY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }
}

static void geoDthEdVtBuildVtParam(const GeoDthEdVtPathPlanningParam_t *param,
                                   GeoVtPathPlanningParam_t *vtParam) {
  *vtParam = (GeoVtPathPlanningParam_t){0};
  vtParam->SltState = param->SltState;
  vtParam->VehW = param->VehW;
  vtParam->VehLlF = param->VehLlF;
  vtParam->VehLfMin = param->VehLfMin;
  vtParam->VehLr = param->VehLr;
  vtParam->FallbackSpace = param->FallbackSpace;
  vtParam->NeedHybridAlgFusion = param->NeedHybridAlgFusion;
  vtParam->SceneQuery = param->SceneQuery;
  vtParam->Diagnostics = param->Diagnostics;
  vtParam->HybridPolicy = param->HybridPolicy;
}

static void geoDthEdVtCopyOutputSegment(GeoDthEdVtPathPlanningOutput_t *output,
                                        Integer dstIdx,
                                        Integer srcIdx) {
  output->SegGoDir[dstIdx] = output->SegGoDir[srcIdx];
  output->SegTrnDir[dstIdx] = output->SegTrnDir[srcIdx];
  output->SegCrv[dstIdx] = output->SegCrv[srcIdx];
  output->SegTht[dstIdx] = output->SegTht[srcIdx];
  output->SegLen[dstIdx] = output->SegLen[srcIdx];
  output->SegStX[dstIdx] = output->SegStX[srcIdx];
  output->SegStY[dstIdx] = output->SegStY[srcIdx];
  output->SegStTh[dstIdx] = output->SegStTh[srcIdx];
  output->SegCcX[dstIdx] = output->SegCcX[srcIdx];
  output->SegCcY[dstIdx] = output->SegCcY[srcIdx];
}

static void geoDthEdVtClearOutputSegment(GeoDthEdVtPathPlanningOutput_t *output, Integer idx) {
  output->SegGoDir[idx] = 0;
  output->SegTrnDir[idx] = 0;
  output->SegCrv[idx] = 0.0f;
  output->SegTht[idx] = 0.0f;
  output->SegLen[idx] = 0.0f;
  output->SegStX[idx] = 0.0f;
  output->SegStY[idx] = 0.0f;
  output->SegStTh[idx] = 0.0f;
  output->SegCcX[idx] = 0.0f;
  output->SegCcY[idx] = 0.0f;
}

static void geoDthEdVtDeleteOutputSegments(GeoDthEdVtPathPlanningOutput_t *output,
                                           Integer firstIdx,
                                           Integer deleteCount) {
  Integer oldSegNum;
  Integer srcIdx;
  Integer dstIdx;

  oldSegNum = output->SegNum;
  srcIdx = firstIdx + deleteCount;
  dstIdx = firstIdx;
  while (srcIdx < oldSegNum) {
    geoDthEdVtCopyOutputSegment(output, dstIdx, srcIdx);
    dstIdx = dstIdx + 1;
    srcIdx = srcIdx + 1;
  }
  output->SegNum = oldSegNum - deleteCount;
  while (dstIdx < oldSegNum) {
    geoDthEdVtClearOutputSegment(output, dstIdx);
    dstIdx = dstIdx + 1;
  }
}

static Integer geoDthEdVtCountOutputStpN(const GeoDthEdVtPathPlanningOutput_t *output) {
  Integer stpN0;
  Integer dirTmp0;
  Integer idx;

  stpN0 = 0;
  dirTmp0 = 0;
  if (output->SegNum > 0) {
    dirTmp0 = output->SegGoDir[0];
    if (dirTmp0 < 0) {
      stpN0 = 1;
    }
  }
  idx = 1;
  while (idx < output->SegNum) {
    if ((output->SegGoDir[idx] * dirTmp0) < 0) {
      stpN0 = stpN0 + 1;
      dirTmp0 = output->SegGoDir[idx];
    }
    idx = idx + 1;
  }
  return stpN0;
}

typedef struct {
  Integer GoDir;
  Integer TrnDir;
  Real Crv;
  Real Tht;
  Real Len;
  Real Stx;
  Real Sty;
  Real StTh;
  Real CcX;
  Real CcY;
} GeoDthEdVtStageSegment_t;

typedef struct {
  Integer IntrvFlg;
  Integer IntrvRS;
  Real AvlLen;
  Integer CheckCount;
  Real AvlPos[3];
  Real CrashPos[3];
} GeoDthEdVtCircleIntrvResult_t;

typedef struct {
  Integer PlanCmd;
  Integer SegNum;
  Integer IniSegN;
  Integer DirS;
  Integer CalcCnt;
  Integer PlnChkCnt;
  Real CurrentPose[3];
  Integer StpPthTypLN;
  Integer StpPthTypList[GEO_DTH_ED_VT_PATH_LIST_MAX];
  Integer GoDir[GEO_DTH_ED_VT_STAGE_MAX];
  Integer TrnDir[GEO_DTH_ED_VT_STAGE_MAX];
  Real Crv[GEO_DTH_ED_VT_STAGE_MAX];
  Real Tht[GEO_DTH_ED_VT_STAGE_MAX];
  Real Len[GEO_DTH_ED_VT_STAGE_MAX];
  Real Stx[GEO_DTH_ED_VT_STAGE_MAX];
  Real Sty[GEO_DTH_ED_VT_STAGE_MAX];
  Real StTh[GEO_DTH_ED_VT_STAGE_MAX];
  Real CcX[GEO_DTH_ED_VT_STAGE_MAX];
  Real CcY[GEO_DTH_ED_VT_STAGE_MAX];
  Integer FailRsOut[GEO_DTH_ED_VT_PATH_LIST_MAX];
} GeoDthEdVtStageWork_t;

static void geoDthEdVtStageWorkInit(GeoDthEdVtStageWork_t *work) {
  Integer idx;

  work->PlanCmd = 0;
  work->SegNum = 0;
  work->IniSegN = 0;
  work->DirS = 0;
  work->CalcCnt = 0;
  work->PlnChkCnt = 0;
  work->CurrentPose[0] = 0.0f;
  work->CurrentPose[1] = 0.0f;
  work->CurrentPose[2] = 0.0f;
  work->StpPthTypLN = 0;
  idx = 0;
  while (idx < GEO_DTH_ED_VT_PATH_LIST_MAX) {
    work->StpPthTypList[idx] = 0;
    idx = idx + 1;
  }
  idx = 0;
  while (idx < GEO_DTH_ED_VT_STAGE_MAX) {
    work->GoDir[idx] = 0;
    work->TrnDir[idx] = 0;
    work->Crv[idx] = 0.0f;
    work->Tht[idx] = 0.0f;
    work->Len[idx] = 0.0f;
    work->Stx[idx] = 0.0f;
    work->Sty[idx] = 0.0f;
    work->StTh[idx] = 0.0f;
    work->CcX[idx] = 0.0f;
    work->CcY[idx] = 0.0f;
    idx = idx + 1;
  }
  idx = 0;
  while (idx < GEO_DTH_ED_VT_PATH_LIST_MAX) {
    work->FailRsOut[idx] = 0;
    idx = idx + 1;
  }
}

static void geoDthEdVtRecordPlanLoopPathType(GeoDthEdVtStageWork_t *work, Integer pathType) {
  Integer dstIdx0;

  if (work->StpPthTypLN < GEO_DTH_ED_VT_PATH_LIST_MAX) {
    work->StpPthTypLN = work->StpPthTypLN + 1;
  }
  dstIdx0 = work->StpPthTypLN - 1;
  if ((dstIdx0 >= 0) && (dstIdx0 < GEO_DTH_ED_VT_PATH_LIST_MAX)) {
    work->StpPthTypList[dstIdx0] = pathType;
  }
}

static Boolean geoDthEdVtCheckStageAppendFailure(GeoDthEdVtStageWork_t *work, Integer stageIdx) {
  Boolean failed0;

  failed0 = false;
  if (work->FailRsOut[0] != 0) {
    failed0 = true;
  } else if (work->SegNum >= GEO_DTH_ED_VT_STAGE_MAX) {
    work->FailRsOut[0] = 9;
    failed0 = true;
  } else if ((stageIdx > 0) && (stageIdx < GEO_DTH_ED_VT_STAGE_MAX)) {
    if (((work->GoDir[stageIdx] * work->GoDir[stageIdx - 1]) <= 0) &&
        (work->TrnDir[stageIdx] != 0) && (work->TrnDir[stageIdx] == work->TrnDir[stageIdx - 1]) &&
        (fabsf(work->Crv[stageIdx] - work->Crv[stageIdx - 1]) < GEO_DTH_ED_VT_APPEND_CRV_EPS)) {
      work->FailRsOut[0] = 13;
      failed0 = true;
    }
  }
  return failed0;
}

static Boolean geoDthEdVtAppendStageSegment(GeoDthEdVtStageWork_t *work,
                                            const GeoDthEdVtStageSegment_t *segment,
                                            Integer *stageIdxOut) {
  Integer stageIdx0;
  Boolean ok0;

  stageIdx0 = GEO_DTH_ED_VT_STAGE_MAX - work->SegNum - 1;
  if (stageIdxOut != (Integer *)0) {
    *stageIdxOut = stageIdx0;
  }

  ok0 = false;
  if ((stageIdx0 < 0) || (stageIdx0 >= GEO_DTH_ED_VT_STAGE_MAX)) {
    if (work->FailRsOut[0] == 0) {
      work->FailRsOut[0] = 9;
    }
  } else {
    work->GoDir[stageIdx0] = segment->GoDir;
    work->TrnDir[stageIdx0] = segment->TrnDir;
    work->Crv[stageIdx0] = segment->Crv;
    work->Tht[stageIdx0] = segment->Tht;
    work->Len[stageIdx0] = segment->Len;
    work->Stx[stageIdx0] = segment->Stx;
    work->Sty[stageIdx0] = segment->Sty;
    work->StTh[stageIdx0] = segment->StTh;
    work->CcX[stageIdx0] = segment->CcX;
    work->CcY[stageIdx0] = segment->CcY;
    work->SegNum = work->SegNum + 1;
    ok0 = !geoDthEdVtCheckStageAppendFailure(work, stageIdx0);
  }
  return ok0;
}

static Boolean geoDthEdVtSeedTargetTailSegment(GeoDthEdVtStageWork_t *work,
                                               Real pt1X,
                                               Real pt1Y,
                                               Real tarX,
                                               Real tarY,
                                               Real tarTheta,
                                               Integer *stageIdxOut) {
  GeoDthEdVtStageSegment_t segment;
  Real dx0;
  Real dy0;
  Real lenSq0;
  Boolean ok0;

  dx0 = pt1X - tarX;
  dy0 = pt1Y - tarY;
  lenSq0 = (dx0 * dx0) + (dy0 * dy0);

  segment = (GeoDthEdVtStageSegment_t){0};
  segment.GoDir = -1;
  segment.TrnDir = 0;
  segment.Crv = 0.0f;
  segment.Tht = 0.0f;
  segment.Len = sqrtf(lenSq0);
  segment.Stx = pt1X;
  segment.Sty = pt1Y;
  segment.StTh = tarTheta;
  segment.CcX = 0.0f;
  segment.CcY = 0.0f;

  ok0 = geoDthEdVtAppendStageSegment(work, &segment, stageIdxOut);
  if (ok0) {
    work->DirS = 1;
  }
  return ok0;
}

static Boolean geoDthEdVtAppendFinalStraightSegment(GeoDthEdVtStageWork_t *work,
                                                    Real startX,
                                                    Real startY,
                                                    Real startTheta,
                                                    Real relativeX,
                                                    Integer *stageIdxOut) {
  GeoDthEdVtStageSegment_t segment;
  Integer nextIdx0;
  Real dx0;
  Real dy0;
  Real lenSq0;
  Boolean ok0;

  ok0 = true;
  if (stageIdxOut != (Integer *)0) {
    *stageIdxOut = -1;
  }
  if (fabsf(relativeX) > 0.2f) {
    nextIdx0 = GEO_DTH_ED_VT_STAGE_MAX - work->SegNum;
    ok0 = false;
    if ((work->SegNum <= 0) || (nextIdx0 < 0) || (nextIdx0 >= GEO_DTH_ED_VT_STAGE_MAX)) {
      if (work->FailRsOut[0] == 0) {
        work->FailRsOut[0] = 9;
      }
    } else {
      dx0 = startX - work->Stx[nextIdx0];
      dy0 = startY - work->Sty[nextIdx0];
      lenSq0 = (dx0 * dx0) + (dy0 * dy0);

      segment = (GeoDthEdVtStageSegment_t){0};
      if (relativeX > 0.0f) {
        segment.GoDir = 1;
      } else {
        segment.GoDir = -1;
      }
      segment.TrnDir = 0;
      segment.Crv = 0.0f;
      segment.Tht = 0.0f;
      segment.Len = sqrtf(lenSq0);
      segment.Stx = startX;
      segment.Sty = startY;
      segment.StTh = startTheta;
      segment.CcX = 0.0f;
      segment.CcY = 0.0f;

      ok0 = geoDthEdVtAppendStageSegment(work, &segment, stageIdxOut);
    }
  }
  return ok0;
}

static Boolean geoDthEdVtAppendCircleSegment(GeoDthEdVtStageWork_t *work,
                                             Real endX,
                                             Real endY,
                                             Real endTheta,
                                             Integer goDir,
                                             Integer trnDir,
                                             Real radius,
                                             Real availableLen,
                                             Real centerX,
                                             Real centerY,
                                             Integer *stageIdxOut) {
  GeoDthEdVtStageSegment_t segment;
  Integer nextIdx0;
  Boolean ok0;

  ok0 = false;
  if (stageIdxOut != (Integer *)0) {
    *stageIdxOut = -1;
  }
  nextIdx0 = GEO_DTH_ED_VT_STAGE_MAX - work->SegNum;
  if ((work->SegNum <= 0) || (nextIdx0 < 0) || (nextIdx0 >= GEO_DTH_ED_VT_STAGE_MAX)) {
    if (work->FailRsOut[0] == 0) {
      work->FailRsOut[0] = 9;
    }
  } else if (fabsf(radius) <= GEO_DTH_ED_VT_EPS) {
    if (work->FailRsOut[0] == 0) {
      work->FailRsOut[0] = 14;
    }
  } else {
    segment = (GeoDthEdVtStageSegment_t){0};
    segment.GoDir = goDir;
    segment.TrnDir = trnDir;
    segment.Crv = ((Real)trnDir) / radius;
    segment.Tht = work->StTh[nextIdx0] - endTheta;
    segment.Len = availableLen;
    segment.Stx = endX;
    segment.Sty = endY;
    segment.StTh = endTheta;
    segment.CcX = centerX;
    segment.CcY = centerY;
    ok0 = geoDthEdVtAppendStageSegment(work, &segment, stageIdxOut);
  }
  return ok0;
}

static void geoDthEdVtBuildCircleIntrvCheckInput(const GeoDthEdVtPathPlanningInput_t *input,
                                                 const GeoDthEdVtPathPlanningParam_t *param,
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
                                                 GeoCircleIntrvCheckInput_t *checkInput) {
  Integer idx;

  *checkInput = (GeoCircleIntrvCheckInput_t){0};
  checkInput->CptX = centerX;
  checkInput->CptY = centerY;
  checkInput->Radius = radius;
  checkInput->Pt1X = startX;
  checkInput->Pt1Y = startY;
  checkInput->Pt1Theta = startTheta;
  checkInput->Pt2X = endX;
  checkInput->Pt2Y = endY;
  checkInput->Pt2Theta = endTheta;
  checkInput->GoDir = goDir;
  checkInput->TrnDir = trnDir;
  checkInput->CheckStep = 0.2f;
  checkInput->GoalX = input->TarX;
  checkInput->GoalY = input->TarY;
  checkInput->GoalYaw = input->TarTheta;
  checkInput->SlotP1X = input->SltPt1X;
  checkInput->SlotP1Y = input->SltPt1Y;
  checkInput->SlotP2X = input->SltPt2X;
  checkInput->SlotP2Y = input->SltPt2Y;
  checkInput->SlotP3X = input->SltPt3X;
  checkInput->SlotP3Y = input->SltPt3Y;
  checkInput->SlotP4X = input->SltPt4X;
  checkInput->SlotP4Y = input->SltPt4Y;
  apaSlotOrderFromEntrance(input->SltEntr,
                           &checkInput->SlotP1X,
                           &checkInput->SlotP1Y,
                           &checkInput->SlotP2X,
                           &checkInput->SlotP2Y,
                           &checkInput->SlotP3X,
                           &checkInput->SlotP3Y,
                           &checkInput->SlotP4X,
                           &checkInput->SlotP4Y);
  checkInput->SlotClass = VCC_SLOT_CLASS_DIAGONAL;
  checkInput->VehWidth = param->VehW;
  checkInput->VehFrontOffset = param->VehLlF;
  checkInput->VehRearOffset = param->VehLr;
  checkInput->UseCollisionContract = true;
  checkInput->CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  checkInput->CollisionDemand = VCC_COLLISION_DEMAND_SIMPLIFY;
  checkInput->UsePlanningCheck = true;
  checkInput->DisableSlotBoundaryFallback = true;
  checkInput->ParkingOutFlag = false;
  checkInput->DirOut = 0;

  idx = 0;
  while ((idx < input->BoundaryLineNum) && (idx < GEO_BOUNDARY_LINE_MAX) &&
         (idx < GEO_BOUNDARY_LINE_MAX)) {
    checkInput->BoundaryLineX1[idx] = input->BoundaryLineX1[idx];
    checkInput->BoundaryLineY1[idx] = input->BoundaryLineY1[idx];
    checkInput->BoundaryLineX2[idx] = input->BoundaryLineX2[idx];
    checkInput->BoundaryLineY2[idx] = input->BoundaryLineY2[idx];
    checkInput->BoundaryLineNum = idx + 1;
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->VirtualBoundaryLineNum) && (idx < GEO_BOUNDARY_LINE_MAX) &&
         (idx < VCC_MAX_VIRTUAL_LINES)) {
    checkInput->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    checkInput->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    checkInput->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    checkInput->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
    checkInput->VirtualBoundaryLineNum = idx + 1;
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->BoundarySepPtNum) && (idx < GEO_BOUNDARY_POINT_MAX) &&
         (idx < VCC_MAX_SEPARATE_PTS)) {
    checkInput->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    checkInput->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    checkInput->BoundarySepPtNum = idx + 1;
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->StopperPtNum) && (idx < GEO_BOUNDARY_POINT_MAX) &&
         (idx < VCC_MAX_SEPARATE_PTS)) {
    checkInput->StopperPtX[idx] = input->StopperPtX[idx];
    checkInput->StopperPtY[idx] = input->StopperPtY[idx];
    checkInput->StopperPtNum = idx + 1;
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->SlotTrackStopperNum) && (idx < GEO_BOUNDARY_LINE_MAX) &&
         (idx < VCC_MAX_TRACK_STOPPER)) {
    checkInput->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    checkInput->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    checkInput->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    checkInput->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    checkInput->SlotTrackStopperNum = idx + 1;
    idx = idx + 1;
  }
}

static GeoDthEdVtCircleIntrvResult_t
geoDthEdVtRunLineIntrvCheck(const GeoDthEdVtPathPlanningInput_t *input,
                            const GeoDthEdVtPathPlanningParam_t *param,
                            Real startX,
                            Real startY,
                            Real startTheta,
                            Real endX,
                            Real endY,
                            Real endTheta,
                            Real checkStep) {
  GeoLineIntrvCheckInput_t checkInput;
  GeoLineIntrvCheckOutput_t checkOutput;
  GeoDthEdVtCircleIntrvResult_t result;
  Integer idx;
  Integer lineCount;
  Integer virtualLineCount;
  Integer boundarySepCount;
  Integer stopperCount;
  Integer slotTrackStopperCount;

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
  checkInput.VehWidth = param->VehW;
  checkInput.VehFrontOffset = param->VehLlF;
  checkInput.VehRearOffset = param->VehLr;
  checkInput.UsePlanningCheck = true;
  checkInput.DisableSlotBoundaryFallback = true;
  checkInput.ParkingOutFlag = false;
  checkInput.DirOut = 0;
  lineCount = input->BoundaryLineNum;
  if (lineCount > GEO_BOUNDARY_LINE_MAX) {
    lineCount = GEO_BOUNDARY_LINE_MAX;
  }
  if (lineCount < 0) {
    lineCount = 0;
  }
  checkInput.BoundaryLineNum = lineCount;
  idx = 0;
  while (idx < lineCount) {
    checkInput.BoundaryLineX1[idx] = input->BoundaryLineX1[idx];
    checkInput.BoundaryLineY1[idx] = input->BoundaryLineY1[idx];
    checkInput.BoundaryLineX2[idx] = input->BoundaryLineX2[idx];
    checkInput.BoundaryLineY2[idx] = input->BoundaryLineY2[idx];
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
  checkInput.VirtualBoundaryLineNum = virtualLineCount;
  idx = 0;
  while (idx < virtualLineCount) {
    checkInput.VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    checkInput.VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    checkInput.VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    checkInput.VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }

  boundarySepCount = input->BoundarySepPtNum;
  if (boundarySepCount > GEO_BOUNDARY_POINT_MAX) {
    boundarySepCount = GEO_BOUNDARY_POINT_MAX;
  }
  if (boundarySepCount > VCC_MAX_SEPARATE_PTS) {
    boundarySepCount = VCC_MAX_SEPARATE_PTS;
  }
  if (boundarySepCount < 0) {
    boundarySepCount = 0;
  }
  checkInput.BoundarySepPtNum = boundarySepCount;
  idx = 0;
  while (idx < boundarySepCount) {
    checkInput.BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    checkInput.BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  stopperCount = input->StopperPtNum;
  if (stopperCount > GEO_BOUNDARY_POINT_MAX) {
    stopperCount = GEO_BOUNDARY_POINT_MAX;
  }
  if (stopperCount > VCC_MAX_SEPARATE_PTS) {
    stopperCount = VCC_MAX_SEPARATE_PTS;
  }
  if (stopperCount < 0) {
    stopperCount = 0;
  }
  checkInput.SeparateStopperNum = stopperCount;
  idx = 0;
  while (idx < stopperCount) {
    checkInput.SeparateStopperX[idx] = input->StopperPtX[idx];
    checkInput.SeparateStopperY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }

  slotTrackStopperCount = input->SlotTrackStopperNum;
  if (slotTrackStopperCount > GEO_BOUNDARY_LINE_MAX) {
    slotTrackStopperCount = GEO_BOUNDARY_LINE_MAX;
  }
  if (slotTrackStopperCount > VCC_MAX_TRACK_STOPPER) {
    slotTrackStopperCount = VCC_MAX_TRACK_STOPPER;
  }
  if (slotTrackStopperCount < 0) {
    slotTrackStopperCount = 0;
  }
  checkInput.SlotTrackStopperNum = slotTrackStopperCount;
  idx = 0;
  while (idx < slotTrackStopperCount) {
    checkInput.SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    checkInput.SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    checkInput.SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    checkInput.SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    idx = idx + 1;
  }
  checkOutput = (GeoLineIntrvCheckOutput_t){0};
  geoLineIntrvCheckWithScene(&checkInput, &param->SceneQuery, &checkOutput);

  result = (GeoDthEdVtCircleIntrvResult_t){0};
  result.IntrvFlg = checkOutput.IntrvFlg;
  result.IntrvRS = checkOutput.IntrvRS;
  result.AvlLen = checkOutput.SafeLen;
  result.CheckCount = checkOutput.CheckCount;
  result.AvlPos[0] = checkOutput.AvlPosX;
  result.AvlPos[1] = checkOutput.AvlPosY;
  result.AvlPos[2] = checkOutput.AvlPosTheta;
  result.CrashPos[0] = checkOutput.CrshPosX;
  result.CrashPos[1] = checkOutput.CrshPosY;
  result.CrashPos[2] = checkOutput.CrshPosTheta;
  return result;
}

static GeoDthEdVtCircleIntrvResult_t
geoDthEdVtRunCircleIntrvCheck(const GeoDthEdVtPathPlanningInput_t *input,
                              const GeoDthEdVtPathPlanningParam_t *param,
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
                              Integer *checkCountOut,
                              Real crashPoseOut[3]) {
  GeoCircleIntrvCheckInput_t checkInput;
  GeoCircleIntrvCheckOutput_t checkOutput;
  GeoDthEdVtCircleIntrvResult_t result;

  geoDthEdVtBuildCircleIntrvCheckInput(input,
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
                                       &checkInput);
  checkOutput = (GeoCircleIntrvCheckOutput_t){0};
  geoCircleIntrvCheckWithScene(&checkInput, &param->SceneQuery, &checkOutput);

  result = (GeoDthEdVtCircleIntrvResult_t){0};
  result.IntrvFlg = checkOutput.IntrvFlg;
  result.IntrvRS = checkOutput.IntrvRS;
  result.AvlLen = checkOutput.SafeLen;
  result.CheckCount = checkOutput.CheckCount;
  result.AvlPos[0] = checkOutput.AvlPosX;
  result.AvlPos[1] = checkOutput.AvlPosY;
  result.AvlPos[2] = checkOutput.AvlPosTheta;
  result.CrashPos[0] = checkOutput.CrshPosX;
  result.CrashPos[1] = checkOutput.CrshPosY;
  result.CrashPos[2] = checkOutput.CrshPosTheta;

  if (checkCountOut != (Integer *)0) {
    *checkCountOut = checkOutput.CheckCount;
  }
  if (crashPoseOut != (Real *)0) {
    crashPoseOut[0] = checkOutput.CrshPosX;
    crashPoseOut[1] = checkOutput.CrshPosY;
    crashPoseOut[2] = checkOutput.CrshPosTheta;
  }
  return result;
}

static Boolean
geoDthEdVtApplyRev2CircleIntrvResults(GeoDthEdVtStageWork_t *work,
                                      const GeoRev2CircleLinkOutput_t *linkOutput,
                                      Real edX,
                                      Real edY,
                                      Real edTheta,
                                      Integer dirS,
                                      Real radius,
                                      const GeoDthEdVtCircleIntrvResult_t *firstIntrv,
                                      const GeoDthEdVtCircleIntrvResult_t *secondIntrv) {
  Integer firstFlg0;
  Integer secondFlg0;
  Boolean appendOk0;
  Boolean ok0;

  ok0 = false;
  firstFlg0 = 2;
  secondFlg0 = 2;
  if ((linkOutput != (const GeoRev2CircleLinkOutput_t *)0) &&
      (firstIntrv != (const GeoDthEdVtCircleIntrvResult_t *)0) &&
      (secondIntrv != (const GeoDthEdVtCircleIntrvResult_t *)0)) {
    firstFlg0 = firstIntrv->IntrvFlg;
    secondFlg0 = secondIntrv->IntrvFlg;
  }

  if ((linkOutput == (const GeoRev2CircleLinkOutput_t *)0) || (linkOutput->GeoOk != 1)) {
    work->PlanCmd = 5;
    if (work->FailRsOut[0] == 0) {
      work->FailRsOut[0] = 7;
    }
  } else {
    if (firstFlg0 == 2) {
      work->PlanCmd = 5;
      work->FailRsOut[0] = 8;
    } else {
      if (firstFlg0 == 0) {
        appendOk0 = geoDthEdVtAppendCircleSegment(work,
                                                  linkOutput->St2X,
                                                  linkOutput->St2Y,
                                                  linkOutput->St2Theta,
                                                  -dirS,
                                                  linkOutput->TrnD1,
                                                  radius,
                                                  firstIntrv->AvlLen,
                                                  linkOutput->Ct1X,
                                                  linkOutput->Ct1Y,
                                                  (Integer *)0);
        if (appendOk0) {
          if (secondFlg0 == 2) {
            work->PlanCmd = 5;
            work->FailRsOut[0] = 8;
          } else {
            if (secondFlg0 == 0) {
              appendOk0 = geoDthEdVtAppendCircleSegment(work,
                                                        linkOutput->St3X,
                                                        linkOutput->St3Y,
                                                        linkOutput->St3Theta,
                                                        dirS,
                                                        linkOutput->TrnD2,
                                                        radius,
                                                        secondIntrv->AvlLen,
                                                        linkOutput->Ct2X,
                                                        linkOutput->Ct2Y,
                                                        (Integer *)0);
              if (appendOk0) {
                appendOk0 = geoDthEdVtAppendFinalStraightSegment(
                    work, edX, edY, edTheta, linkOutput->St3rX, (Integer *)0);
              }
              if (appendOk0) {
                work->PlanCmd = 1;
                ok0 = true;
              }
            } else {
              appendOk0 = geoDthEdVtAppendCircleSegment(work,
                                                        secondIntrv->AvlPos[0],
                                                        secondIntrv->AvlPos[1],
                                                        secondIntrv->AvlPos[2],
                                                        dirS,
                                                        linkOutput->TrnD2,
                                                        radius,
                                                        secondIntrv->AvlLen,
                                                        linkOutput->Ct2X,
                                                        linkOutput->Ct2Y,
                                                        (Integer *)0);
              if (appendOk0) {
                work->PlanCmd = 2;
                ok0 = true;
              }
            }
          }
        }
      } else {
        appendOk0 = geoDthEdVtAppendCircleSegment(work,
                                                  firstIntrv->AvlPos[0],
                                                  firstIntrv->AvlPos[1],
                                                  firstIntrv->AvlPos[2],
                                                  -dirS,
                                                  linkOutput->TrnD1,
                                                  radius,
                                                  firstIntrv->AvlLen,
                                                  linkOutput->Ct1X,
                                                  linkOutput->Ct1Y,
                                                  (Integer *)0);
        if (appendOk0) {
          work->PlanCmd = 2;
          ok0 = true;
        }
      }
      if ((!appendOk0) && (work->PlanCmd != 5)) {
        work->PlanCmd = 5;
        if (work->FailRsOut[0] == 0) {
          work->FailRsOut[0] = 7;
        }
      }
    }
  }
  return ok0;
}

static Boolean
geoDthEdVtTryFirstGroupReverseTwoCircleStageCandidate(GeoDthEdVtStageWork_t *work,
                                                      const GeoDthEdVtPathPlanningInput_t *input,
                                                      const GeoDthEdVtPathPlanningParam_t *param,
                                                      Real stX,
                                                      Real stY,
                                                      Real stTheta,
                                                      Real edX,
                                                      Real edY,
                                                      Real edTheta,
                                                      Integer pathType,
                                                      Integer dirS,
                                                      Real radius) {
  GeoRev2CircleLinkInput_t linkInput;
  GeoRev2CircleLinkOutput_t linkOutput;
  GeoDthEdVtCircleIntrvResult_t firstIntrv;
  GeoDthEdVtCircleIntrvResult_t secondIntrv;
  Boolean ok0;

  geoDthEdVtRecordPlanLoopPathType(work, pathType);
  linkInput = (GeoRev2CircleLinkInput_t){0};
  linkInput.Pt1X = stX;
  linkInput.Pt1Y = stY;
  linkInput.Pt1Theta = stTheta;
  linkInput.Pt2X = edX;
  linkInput.Pt2Y = edY;
  linkInput.Pt2Theta = edTheta;
  linkInput.Dir = dirS;
  linkInput.R1 = radius;
  linkInput.R2 = radius;
  linkOutput = (GeoRev2CircleLinkOutput_t){0};
  geoRev2CircleLink(&linkInput, &linkOutput);

  firstIntrv = (GeoDthEdVtCircleIntrvResult_t){0};
  secondIntrv = (GeoDthEdVtCircleIntrvResult_t){0};
  if (linkOutput.GeoOk == 1) {
    firstIntrv = geoDthEdVtRunCircleIntrvCheck(input,
                                               param,
                                               linkOutput.Ct1X,
                                               linkOutput.Ct1Y,
                                               radius,
                                               stX,
                                               stY,
                                               stTheta,
                                               linkOutput.St2X,
                                               linkOutput.St2Y,
                                               linkOutput.St2Theta,
                                               dirS,
                                               linkOutput.TrnD1,
                                               (Integer *)0,
                                               (Real *)0);
    work->PlnChkCnt = work->PlnChkCnt + firstIntrv.CheckCount;
    if (firstIntrv.IntrvFlg == 0) {
      secondIntrv = geoDthEdVtRunCircleIntrvCheck(input,
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
                                                  (Integer *)0,
                                                  (Real *)0);
      work->PlnChkCnt = work->PlnChkCnt + secondIntrv.CheckCount;
    }
  }

  ok0 = geoDthEdVtApplyRev2CircleIntrvResults(
      work, &linkOutput, edX, edY, edTheta, dirS, radius, &firstIntrv, &secondIntrv);
  return ok0;
}

static Boolean
geoDthEdVtSeedVehicleToEntranceStraightSegment(GeoDthEdVtStageWork_t *work,
                                               const GeoDthEdVtPathPlanningInput_t *input,
                                               Real pt1X,
                                               Real pt1Y,
                                               Integer *stageIdxOut) {
  GeoDthEdVtStageSegment_t segment;
  Real dx0;
  Real dy0;
  Real cosTh0;
  Real sinTh0;
  Real proj0;
  Real lat0;
  Real len0;
  Boolean ok0;

  dx0 = pt1X - input->VehX;
  dy0 = pt1Y - input->VehY;
  cosTh0 = cosf(input->VehTheta);
  sinTh0 = sinf(input->VehTheta);
  proj0 = (dx0 * cosTh0) + (dy0 * sinTh0);
  lat0 = (-dx0 * sinTh0) + (dy0 * cosTh0);
  len0 = fabsf(proj0);

  ok0 = false;
  if ((len0 > GEO_DTH_ED_VT_STRAIGHT_MIN_LEN) && (fabsf(lat0) <= GEO_DTH_ED_VT_STRAIGHT_LAT_EPS)) {
    segment = (GeoDthEdVtStageSegment_t){0};
    segment.GoDir = 1;
    if (proj0 < 0.0f) {
      segment.GoDir = -1;
    }
    segment.TrnDir = 0;
    segment.Crv = 0.0f;
    segment.Tht = 0.0f;
    segment.Len = len0;
    segment.Stx = input->VehX;
    segment.Sty = input->VehY;
    segment.StTh = input->VehTheta;
    segment.CcX = 0.0f;
    segment.CcY = 0.0f;
    ok0 = geoDthEdVtAppendStageSegment(work, &segment, stageIdxOut);
  }
  return ok0;
}

static Boolean geoDthEdVtAdvancePlanCmdState(GeoDthEdVtStageWork_t *work) {
  Boolean keepGoing0;
  Integer tailIdx0;

  keepGoing0 = true;
  if ((work->PlanCmd == 5) || (work->PlanCmd == 1)) {
    keepGoing0 = false;
  } else {
    if (work->PlanCmd == 2) {
      tailIdx0 = GEO_DTH_ED_VT_STAGE_MAX - work->SegNum;
      if ((work->SegNum <= 0) || (work->SegNum > GEO_DTH_ED_VT_STAGE_MAX) || (tailIdx0 < 0) ||
          (tailIdx0 >= GEO_DTH_ED_VT_STAGE_MAX)) {
        if (work->FailRsOut[0] == 0) {
          work->FailRsOut[0] = 9;
        }
        keepGoing0 = false;
      } else if (!(isfinite(work->Len[tailIdx0]) && fabsf(work->Len[tailIdx0]) > 0.01f)) {
        if (work->FailRsOut[0] == 0)
          work->FailRsOut[0] = 9;
        keepGoing0 = false;
      } else {
        for (Integer i = tailIdx0 + 1; i < GEO_DTH_ED_VT_STAGE_MAX; ++i) {
          if (work->GoDir[i] == work->GoDir[tailIdx0] &&
              hypotf(work->Stx[i] - work->Stx[tailIdx0], work->Sty[i] - work->Sty[tailIdx0]) <=
                  0.01f &&
              fabsf(apaCoordWrapToPi(work->StTh[i] - work->StTh[tailIdx0])) <= 0.001f) {
            work->FailRsOut[0] = 9;
            return false;
          }
        }
        work->DirS = work->GoDir[tailIdx0];
        work->CurrentPose[0] = work->Stx[tailIdx0];
        work->CurrentPose[1] = work->Sty[tailIdx0];
        work->CurrentPose[2] = work->StTh[tailIdx0];
      }
    }
    if (keepGoing0) {
      work->CalcCnt = work->CalcCnt + 1;
      if (work->CalcCnt > 30) {
        if (work->FailRsOut[0] == 0) {
          work->FailRsOut[0] = 9;
        }
        keepGoing0 = false;
      }
    }
  }
  return keepGoing0;
}

static void geoDthEdVtClearOutputPath(GeoDthEdVtPathPlanningOutput_t *output) {
  Integer idx;

  idx = 0;
  while (idx < GEO_DTH_ED_VT_SEG_OUT_MAX) {
    output->SegGoDir[idx] = 0;
    output->SegTrnDir[idx] = 0;
    output->SegCrv[idx] = 0.0f;
    output->SegTht[idx] = 0.0f;
    output->SegLen[idx] = 0.0f;
    output->SegStX[idx] = 0.0f;
    output->SegStY[idx] = 0.0f;
    output->SegStTh[idx] = 0.0f;
    output->SegCcX[idx] = 0.0f;
    output->SegCcY[idx] = 0.0f;
    idx = idx + 1;
  }
}

static void geoDthEdVtCopyStageSegment(const GeoDthEdVtStageWork_t *work,
                                       Integer srcIdx,
                                       GeoDthEdVtPathPlanningOutput_t *output,
                                       Integer dstIdx) {
  output->SegGoDir[dstIdx] = work->GoDir[srcIdx];
  output->SegTrnDir[dstIdx] = work->TrnDir[srcIdx];
  output->SegCrv[dstIdx] = work->Crv[srcIdx];
  output->SegTht[dstIdx] = work->Tht[srcIdx];
  output->SegLen[dstIdx] = work->Len[srcIdx];
  output->SegStX[dstIdx] = work->Stx[srcIdx];
  output->SegStY[dstIdx] = work->Sty[srcIdx];
  output->SegStTh[dstIdx] = work->StTh[srcIdx];
  output->SegCcX[dstIdx] = work->CcX[srcIdx];
  output->SegCcY[dstIdx] = work->CcY[srcIdx];
}

static void geoDthEdVtCopyStagePathTypes(const GeoDthEdVtStageWork_t *work,
                                         GeoDthEdVtPathPlanningOutput_t *output);

static Boolean geoDthEdVtAssembleStageOutput(const GeoDthEdVtStageWork_t *work,
                                             GeoDthEdVtPathPlanningOutput_t *output) {
  Integer finalSegNum0;
  Integer tailStart0;
  Integer srcIdx;
  Integer dstIdx;
  Integer idx;
  Boolean countOk0;
  Boolean success0;

  finalSegNum0 = work->SegNum + work->IniSegN;
  tailStart0 = GEO_DTH_ED_VT_STAGE_MAX - work->SegNum;
  countOk0 =
      ((work->SegNum > 0) && (work->IniSegN >= 0) && (work->SegNum <= GEO_DTH_ED_VT_STAGE_MAX) &&
       (tailStart0 >= 0) && (finalSegNum0 <= GEO_DTH_ED_VT_SEG_OUT_MAX));
  success0 = ((work->PlanCmd == 1) && countOk0);

  geoDthEdVtClearOutputPath(output);
  output->PathLen = 0.0f;
  output->StpN = 0;
  output->SegNum = 0;
  if (success0) {
    dstIdx = 0;
    srcIdx = 0;
    while (srcIdx < work->IniSegN) {
      geoDthEdVtCopyStageSegment(work, srcIdx, output, dstIdx);
      srcIdx = srcIdx + 1;
      dstIdx = dstIdx + 1;
    }
    srcIdx = tailStart0;
    while (srcIdx < GEO_DTH_ED_VT_STAGE_MAX) {
      geoDthEdVtCopyStageSegment(work, srcIdx, output, dstIdx);
      srcIdx = srcIdx + 1;
      dstIdx = dstIdx + 1;
    }
    output->SegNum = finalSegNum0;
    output->PlanFlg = 1;
    output->PthPlnFlg = 1;
    idx = 0;
    while (idx < output->SegNum) {
      output->PathLen = output->PathLen + output->SegLen[idx];
      idx = idx + 1;
    }
    output->StpN = geoDthEdVtCountOutputStpN(output);
  } else {
    output->PlanFlg = 0;
    output->PthPlnFlg = 0;
    idx = 0;
    while (idx < GEO_DTH_ED_VT_PATH_LIST_MAX) {
      output->FailRsOut[idx] = work->FailRsOut[idx];
      idx = idx + 1;
    }
    output->FailRs0 = output->FailRsOut[0];
    output->FailRs1 = output->FailRsOut[1];
    if ((finalSegNum0 > GEO_DTH_ED_VT_SEG_OUT_MAX) && (output->FailRs0 == 0)) {
      output->FailRs0 = 9;
      output->FailRsOut[0] = 9;
    }
  }
  geoDthEdVtCopyStagePathTypes(work, output);
  return success0;
}

static Boolean
geoDthEdVtStageCandidateStartsAtVehicle(const GeoDthEdVtPathPlanningInput_t *input,
                                        const GeoDthEdVtPathPlanningOutput_t *output) {
  Boolean startOk0;
  Real dx0;
  Real dy0;
  Real dth0;

  startOk0 = false;
  if ((output->PlanFlg == 1) && (output->SegNum > 0)) {
    dx0 = fabsf(output->SegStX[0] - input->VehX);
    dy0 = fabsf(output->SegStY[0] - input->VehY);
    dth0 = fabsf(output->SegStTh[0] - input->VehTheta);
    if ((dx0 < GEO_DTH_ED_VT_EPS) && (dy0 < GEO_DTH_ED_VT_EPS) && (dth0 < GEO_DTH_ED_VT_EPS)) {
      startOk0 = true;
    }
  }
  return startOk0;
}

static Boolean geoDthEdVtTryStraightStageCandidate(const GeoDthEdVtPathPlanningInput_t *input,
                                                   GeoDthEdVtPathPlanningOutput_t *output) {
  GeoDthEdVtStageWork_t work;
  Boolean ok0;

  geoDthEdVtStageWorkInit(&work);
  ok0 = geoDthEdVtSeedTargetTailSegment(
      &work, output->Pt1X, output->Pt1Y, input->TarX, input->TarY, input->TarTheta, (Integer *)0);
  if (ok0) {
    ok0 = geoDthEdVtSeedVehicleToEntranceStraightSegment(
        &work, input, output->Pt1X, output->Pt1Y, (Integer *)0);
  }
  if (ok0) {
    work.PlanCmd = 1;
    ok0 = geoDthEdVtAssembleStageOutput(&work, output);
  }
  if (ok0 && !geoDthEdVtStageCandidateStartsAtVehicle(input, output)) {
    geoDthEdVtClearOutputPath(output);
    output->PlanFlg = 0;
    output->PthPlnFlg = 0;
    ok0 = false;
  }
  if (ok0) {
    output->PlannerCase = GEO_DTH_ED_VT_PLANNER_CASE_GENERIC_FALLBACK;
    output->FallbackGeomCandidateFlg = 1;
    output->FallbackWinnerKind = GEO_DTH_ED_VT_FALLBACK_WINNER_GEOM;
    output->FallbackDecisionReason = 1;
    output->FallbackGeomStpN = output->StpN;
    output->FallbackGeomPathLen = output->PathLen;
    output->FallbackGeomSegNum = output->SegNum;
    output->FallbackGeomPlnChkCnt = 0;
    output->PlnChkCnt = output->EntrCheckCnt + output->DthSpaceCheckCnt;
  }
  return ok0;
}

static Boolean geoDthEdVtIsFirstGroupPathType(Integer pathType) {
  Boolean firstGroup0;

  firstGroup0 = false;
  if ((pathType == 4) || (pathType == 34) || (pathType == 2) || (pathType == 32) ||
      (pathType == 6) || (pathType == 36) || (pathType == 8) || (pathType == 38) ||
      (pathType == 9) || (pathType == 39) || (pathType == 13) || (pathType == 43) ||
      (pathType == 15) || (pathType == 45)) {
    firstGroup0 = true;
  }
  return firstGroup0;
}

static Boolean geoDthEdVtIsSecondGroupPathType(Integer pathType) {
  Boolean secondGroup0;

  secondGroup0 = false;
  if ((pathType == 1) || (pathType == 31) || (pathType == 3) || (pathType == 33) ||
      (pathType == 5) || (pathType == 35) || (pathType == 7) || (pathType == 37) ||
      (pathType == 10) || (pathType == 40) || (pathType == 11) || (pathType == 41) ||
      (pathType == 12) || (pathType == 42) || (pathType == 14) || (pathType == 44) ||
      (pathType == 16) || (pathType == 46)) {
    secondGroup0 = true;
  }
  return secondGroup0;
}

static Integer geoDthEdVtIsSecondGroupSubType2(const GeoCircle1pt1lineOutput_t *circleOutput,
                                               Real stRelY,
                                               Real stRelTheta) {
  Boolean longArc0;
  Boolean postureSmall0;
  Integer subTyp0;

  postureSmall0 = false;
  subTyp0 = GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_NONE;
  if (circleOutput != (const GeoCircle1pt1lineOutput_t *)0) {
    longArc0 = (circleOutput->ArcLen > GEO_DTH_ED_VT_CIR_LEN_MAX);
    if (fabsf(circleOutput->LnPtRX) > GEO_DTH_ED_VT_CIR_EDX_MAX) {
      longArc0 = true;
    }
    if ((fabsf(stRelY) < GEO_DTH_ED_VT_CIR_EDY_MAX) &&
        (fabsf(stRelTheta) < GEO_DTH_ED_VT_CIR_THETA_MAX)) {
      postureSmall0 = true;
    }
    if (longArc0) {
      if (postureSmall0) {
        subTyp0 = GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_2;
      } else {
        subTyp0 = GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_3;
      }
    } else {
      subTyp0 = GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_1;
    }
  }
  return subTyp0;
}

static Boolean
geoDthEdVtApplySecondGroupCircleLineIntrvResults(GeoDthEdVtStageWork_t *work,
                                                 const GeoCircle1pt1lineOutput_t *circleOutput,
                                                 Real edX,
                                                 Real edY,
                                                 Real edTheta,
                                                 Integer dirS,
                                                 Real radius,
                                                 const GeoDthEdVtCircleIntrvResult_t *intrv) {
  Integer intrvFlg0;
  Boolean appendOk0;
  Boolean ok0;

  ok0 = false;
  appendOk0 = false;
  intrvFlg0 = 2;
  if ((circleOutput != (const GeoCircle1pt1lineOutput_t *)0) &&
      (intrv != (const GeoDthEdVtCircleIntrvResult_t *)0)) {
    intrvFlg0 = intrv->IntrvFlg;
  }

  if ((circleOutput == (const GeoCircle1pt1lineOutput_t *)0) || (radius < GEO_DTH_ED_VT_EPS)) {
    work->PlanCmd = 5;
    if (work->FailRsOut[0] == 0) {
      work->FailRsOut[0] = 7;
    }
  } else if (intrvFlg0 == 2) {
    work->PlanCmd = 5;
    work->FailRsOut[0] = 8;
  } else if (intrvFlg0 == 0) {
    appendOk0 = geoDthEdVtAppendCircleSegment(work,
                                              circleOutput->LnPtX,
                                              circleOutput->LnPtY,
                                              circleOutput->LnPtTheta,
                                              -dirS,
                                              circleOutput->TrnDir,
                                              radius,
                                              intrv->AvlLen,
                                              circleOutput->CptX,
                                              circleOutput->CptY,
                                              (Integer *)0);
    if (appendOk0) {
      appendOk0 = geoDthEdVtAppendFinalStraightSegment(
          work, edX, edY, edTheta, circleOutput->LnPtRX, (Integer *)0);
    }
    if (appendOk0) {
      work->PlanCmd = 1;
      ok0 = true;
    }
  } else {
    appendOk0 = geoDthEdVtAppendCircleSegment(work,
                                              intrv->AvlPos[0],
                                              intrv->AvlPos[1],
                                              intrv->AvlPos[2],
                                              -dirS,
                                              circleOutput->TrnDir,
                                              radius,
                                              intrv->AvlLen,
                                              circleOutput->CptX,
                                              circleOutput->CptY,
                                              (Integer *)0);
    if (appendOk0) {
      work->PlanCmd = 2;
      ok0 = true;
    }
  }

  if ((!appendOk0) && (work->PlanCmd != 5)) {
    work->PlanCmd = 5;
    if (work->FailRsOut[0] == 0) {
      work->FailRsOut[0] = 7;
    }
  }
  return ok0;
}

static Boolean geoDthEdVtTrySecondGroupCircleLineSubType1StageCandidate(
    GeoDthEdVtStageWork_t *work,
    const GeoDthEdVtPathPlanningInput_t *input,
    const GeoDthEdVtPathPlanningParam_t *param,
    Real stX,
    Real stY,
    Real stTheta,
    Real edX,
    Real edY,
    Real edTheta,
    Integer pathType,
    Integer dirS,
    const GeoCircle1pt1lineOutput_t *circleOutput) {
  GeoDthEdVtCircleIntrvResult_t firstIntrv;
  Real radius0;
  Boolean ok0;

  firstIntrv = (GeoDthEdVtCircleIntrvResult_t){0};
  radius0 = 0.0f;
  if (circleOutput != (const GeoCircle1pt1lineOutput_t *)0) {
    radius0 = fabsf(circleOutput->Radius);
  }

  geoDthEdVtRecordPlanLoopPathType(work, pathType);
  if (radius0 >= GEO_DTH_ED_VT_EPS) {
    firstIntrv = geoDthEdVtRunCircleIntrvCheck(input,
                                               param,
                                               circleOutput->CptX,
                                               circleOutput->CptY,
                                               radius0,
                                               stX,
                                               stY,
                                               stTheta,
                                               circleOutput->LnPtX,
                                               circleOutput->LnPtY,
                                               circleOutput->LnPtTheta,
                                               dirS,
                                               circleOutput->TrnDir,
                                               (Integer *)0,
                                               (Real *)0);
    work->PlnChkCnt = work->PlnChkCnt + firstIntrv.CheckCount;
  }

  ok0 = geoDthEdVtApplySecondGroupCircleLineIntrvResults(
      work, circleOutput, edX, edY, edTheta, dirS, radius0, &firstIntrv);
  return ok0;
}

static Boolean geoDthEdVtApplySecondGroupCircle2LineIntrvResults(
    GeoDthEdVtStageWork_t *work,
    const GeoCircle2lineROutput_t *circleOutput,
    Real stX,
    Real stY,
    Real stTheta,
    Real edX,
    Real edY,
    Real edTheta,
    Integer dirS,
    Real radius,
    const GeoDthEdVtCircleIntrvResult_t *lineIntrv,
    const GeoDthEdVtCircleIntrvResult_t *circleIntrv) {
  Integer lineFlg0;
  Integer circleFlg0;
  Integer lineGoDir0;
  Integer nextIdx0;
  Real relDx0;
  Real relDy0;
  Real relX0;
  Real lineLenSq0;
  GeoDthEdVtStageSegment_t segment;
  Boolean appendOk0;
  Boolean ok0;

  ok0 = false;
  appendOk0 = false;
  lineFlg0 = 2;
  circleFlg0 = 2;
  if ((lineIntrv != (const GeoDthEdVtCircleIntrvResult_t *)0) &&
      (circleIntrv != (const GeoDthEdVtCircleIntrvResult_t *)0)) {
    lineFlg0 = lineIntrv->IntrvFlg;
    circleFlg0 = circleIntrv->IntrvFlg;
  }

  if ((circleOutput == (const GeoCircle2lineROutput_t *)0) || (radius < GEO_DTH_ED_VT_EPS)) {
    work->PlanCmd = 5;
    if (work->FailRsOut[0] == 0) {
      work->FailRsOut[0] = 7;
    }
  } else if (lineFlg0 == 2) {
    work->PlanCmd = 5;
    work->FailRsOut[0] = 8;
  } else if (lineFlg0 == 1) {
    work->PlanCmd = 5;
    work->FailRsOut[0] = 14;
  } else {
    nextIdx0 = GEO_DTH_ED_VT_STAGE_MAX - work->SegNum;
    appendOk0 = false;
    if ((work->SegNum <= 0) || (nextIdx0 < 0) || (nextIdx0 >= GEO_DTH_ED_VT_STAGE_MAX)) {
      if (work->FailRsOut[0] == 0) {
        work->FailRsOut[0] = 9;
      }
    } else {
      relDx0 = lineIntrv->AvlPos[0] - stX;
      relDy0 = lineIntrv->AvlPos[1] - stY;
      relX0 = (cosf(stTheta) * relDx0) + (sinf(stTheta) * relDy0);
      lineGoDir0 = 1;
      if (relX0 > 0.0f) {
        lineGoDir0 = -1;
      }
      lineLenSq0 =
          (circleOutput->StPtX - work->Stx[nextIdx0]) *
              (circleOutput->StPtX - work->Stx[nextIdx0]) +
          (circleOutput->StPtY - work->Sty[nextIdx0]) * (circleOutput->StPtY - work->Sty[nextIdx0]);
      segment = (GeoDthEdVtStageSegment_t){0};
      segment.GoDir = lineGoDir0;
      segment.TrnDir = 0;
      segment.Crv = 0.0f;
      segment.Tht = 0.0f;
      segment.Len = sqrtf(lineLenSq0);
      segment.Stx = circleOutput->StPtX;
      segment.Sty = circleOutput->StPtY;
      segment.StTh = circleOutput->StPtTheta;
      segment.CcX = 0.0f;
      segment.CcY = 0.0f;
      appendOk0 = geoDthEdVtAppendStageSegment(work, &segment, (Integer *)0);
    }
    if (appendOk0) {
      if (circleFlg0 == 2) {
        work->PlanCmd = 5;
        work->FailRsOut[0] = 8;
      } else if (circleFlg0 == 0) {
        appendOk0 = geoDthEdVtAppendCircleSegment(work,
                                                  circleOutput->EdPtX,
                                                  circleOutput->EdPtY,
                                                  circleOutput->EdPtTheta,
                                                  -circleOutput->GoDir,
                                                  circleOutput->TrnDir,
                                                  radius,
                                                  circleIntrv->AvlLen,
                                                  circleOutput->CptX,
                                                  circleOutput->CptY,
                                                  (Integer *)0);
        if (appendOk0) {
          appendOk0 = geoDthEdVtAppendFinalStraightSegment(
              work, edX, edY, edTheta, circleOutput->EdPtrX, (Integer *)0);
        }
        if (appendOk0) {
          work->PlanCmd = 1;
          ok0 = true;
        }
      } else {
        appendOk0 = geoDthEdVtAppendCircleSegment(work,
                                                  circleIntrv->AvlPos[0],
                                                  circleIntrv->AvlPos[1],
                                                  circleIntrv->AvlPos[2],
                                                  -dirS,
                                                  circleOutput->TrnDir,
                                                  radius,
                                                  circleIntrv->AvlLen,
                                                  circleOutput->CptX,
                                                  circleOutput->CptY,
                                                  (Integer *)0);
        if (appendOk0) {
          work->PlanCmd = 2;
          ok0 = true;
        }
      }
    }
  }

  if ((!appendOk0) && (work->PlanCmd != 5)) {
    work->PlanCmd = 5;
    if (work->FailRsOut[0] == 0) {
      work->FailRsOut[0] = 7;
    }
  }
  return ok0;
}

static Boolean geoDthEdVtTrySecondGroupCircle2LineSubType3StageCandidate(
    GeoDthEdVtStageWork_t *work,
    const GeoDthEdVtPathPlanningInput_t *input,
    const GeoDthEdVtPathPlanningParam_t *param,
    Real stX,
    Real stY,
    Real stTheta,
    Real edX,
    Real edY,
    Real edTheta,
    Integer pathType,
    Integer dirS,
    Real radius) {
  GeoCircle2lineRInput_t circleInput;
  GeoCircle2lineROutput_t circleOutput;
  GeoDthEdVtCircleIntrvResult_t lineIntrv;
  GeoDthEdVtCircleIntrvResult_t circleIntrv;
  Boolean ok0;

  geoDthEdVtRecordPlanLoopPathType(work, pathType);
  circleInput = (GeoCircle2lineRInput_t){0};
  circleInput.VstX = stX;
  circleInput.VstY = stY;
  circleInput.VstTheta = stTheta;
  circleInput.VtgtX = edX;
  circleInput.VtgtY = edY;
  circleInput.VtgtTheta = edTheta;
  circleInput.Ptyp = pathType;
  circleInput.Dir = dirS;
  circleInput.Radius = radius;
  circleOutput = (GeoCircle2lineROutput_t){0};
  geoCircle2lineR(&circleInput, &circleOutput);

  lineIntrv = geoDthEdVtRunLineIntrvCheck(input,
                                          param,
                                          stX,
                                          stY,
                                          stTheta,
                                          circleOutput.StPtX,
                                          circleOutput.StPtY,
                                          circleOutput.StPtTheta,
                                          2.0f);
  work->PlnChkCnt = work->PlnChkCnt + lineIntrv.CheckCount;
  circleIntrv = (GeoDthEdVtCircleIntrvResult_t){0};
  if (lineIntrv.IntrvFlg == 0) {
    circleIntrv = geoDthEdVtRunCircleIntrvCheck(input,
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
                                                (Integer *)0,
                                                (Real *)0);
    work->PlnChkCnt = work->PlnChkCnt + circleIntrv.CheckCount;
  }

  ok0 = geoDthEdVtApplySecondGroupCircle2LineIntrvResults(work,
                                                          &circleOutput,
                                                          stX,
                                                          stY,
                                                          stTheta,
                                                          edX,
                                                          edY,
                                                          edTheta,
                                                          dirS,
                                                          radius,
                                                          &lineIntrv,
                                                          &circleIntrv);
  return ok0;
}

static Boolean
geoDthEdVtApplySecondGroupTwoCircleIntrvResults(GeoDthEdVtStageWork_t *work,
                                                const Geo2CircleLinkOutput_t *linkOutput,
                                                Real edX,
                                                Real edY,
                                                Real edTheta,
                                                Integer dirS,
                                                Real radius,
                                                const GeoDthEdVtCircleIntrvResult_t *firstIntrv,
                                                const GeoDthEdVtCircleIntrvResult_t *secondIntrv) {
  Integer firstFlg0;
  Integer secondFlg0;
  Boolean appendOk0;
  Boolean ok0;

  ok0 = false;
  firstFlg0 = 2;
  secondFlg0 = 2;
  if ((linkOutput != (const Geo2CircleLinkOutput_t *)0) &&
      (firstIntrv != (const GeoDthEdVtCircleIntrvResult_t *)0) &&
      (secondIntrv != (const GeoDthEdVtCircleIntrvResult_t *)0)) {
    firstFlg0 = firstIntrv->IntrvFlg;
    secondFlg0 = secondIntrv->IntrvFlg;
  }

  if ((linkOutput == (const Geo2CircleLinkOutput_t *)0) || (linkOutput->PlnFlg != 1)) {
    work->PlanCmd = 5;
    if (work->FailRsOut[0] == 0) {
      work->FailRsOut[0] = 7;
    }
  } else {
    if (firstFlg0 == 2) {
      work->PlanCmd = 5;
      work->FailRsOut[0] = 8;
    } else {
      if (firstFlg0 == 0) {
        appendOk0 = geoDthEdVtAppendCircleSegment(work,
                                                  linkOutput->St2X,
                                                  linkOutput->St2Y,
                                                  linkOutput->St2Theta,
                                                  -dirS,
                                                  linkOutput->TrnD1,
                                                  radius,
                                                  firstIntrv->AvlLen,
                                                  linkOutput->Ct1X,
                                                  linkOutput->Ct1Y,
                                                  (Integer *)0);
        if (appendOk0) {
          if (secondFlg0 == 2) {
            work->PlanCmd = 5;
            work->FailRsOut[0] = 8;
          } else {
            if (secondFlg0 == 0) {
              appendOk0 = geoDthEdVtAppendCircleSegment(work,
                                                        linkOutput->St3X,
                                                        linkOutput->St3Y,
                                                        linkOutput->St3Theta,
                                                        -dirS,
                                                        linkOutput->TrnD2,
                                                        radius,
                                                        secondIntrv->AvlLen,
                                                        linkOutput->Ct2X,
                                                        linkOutput->Ct2Y,
                                                        (Integer *)0);
              if (appendOk0) {
                appendOk0 = geoDthEdVtAppendFinalStraightSegment(
                    work, edX, edY, edTheta, linkOutput->St3rX, (Integer *)0);
              }
              if (appendOk0) {
                work->PlanCmd = 1;
                ok0 = true;
              }
            } else {
              appendOk0 = geoDthEdVtAppendCircleSegment(work,
                                                        secondIntrv->AvlPos[0],
                                                        secondIntrv->AvlPos[1],
                                                        secondIntrv->AvlPos[2],
                                                        -dirS,
                                                        linkOutput->TrnD2,
                                                        radius,
                                                        secondIntrv->AvlLen,
                                                        linkOutput->Ct2X,
                                                        linkOutput->Ct2Y,
                                                        (Integer *)0);
              if (appendOk0) {
                work->PlanCmd = 2;
                ok0 = true;
              }
            }
          }
        }
      } else {
        appendOk0 = geoDthEdVtAppendCircleSegment(work,
                                                  firstIntrv->AvlPos[0],
                                                  firstIntrv->AvlPos[1],
                                                  firstIntrv->AvlPos[2],
                                                  -dirS,
                                                  linkOutput->TrnD1,
                                                  radius,
                                                  firstIntrv->AvlLen,
                                                  linkOutput->Ct1X,
                                                  linkOutput->Ct1Y,
                                                  (Integer *)0);
        if (appendOk0) {
          work->PlanCmd = 2;
          ok0 = true;
        }
      }
      if ((!appendOk0) && (work->PlanCmd != 5)) {
        work->PlanCmd = 5;
        if (work->FailRsOut[0] == 0) {
          work->FailRsOut[0] = 7;
        }
      }
    }
  }
  return ok0;
}

static Boolean geoDthEdVtTryTwoCircleStageCandidate(GeoDthEdVtStageWork_t *work,
                                                    const GeoDthEdVtPathPlanningInput_t *input,
                                                    const GeoDthEdVtPathPlanningParam_t *param,
                                                    Real stX,
                                                    Real stY,
                                                    Real stTheta,
                                                    Real edX,
                                                    Real edY,
                                                    Real edTheta,
                                                    Integer pathType,
                                                    Integer dirS,
                                                    Real radius) {
  Geo2CircleLinkInput_t linkInput;
  Geo2CircleLinkOutput_t linkOutput;
  GeoDthEdVtCircleIntrvResult_t firstIntrv;
  GeoDthEdVtCircleIntrvResult_t secondIntrv;
  Boolean ok0;

  geoDthEdVtRecordPlanLoopPathType(work, pathType);
  linkInput = (Geo2CircleLinkInput_t){0};
  linkInput.Pt1X = stX;
  linkInput.Pt1Y = stY;
  linkInput.Pt1Theta = stTheta;
  linkInput.Pt2X = edX;
  linkInput.Pt2Y = edY;
  linkInput.Pt2Theta = edTheta;
  linkInput.Dir = dirS;
  linkInput.R1 = radius;
  linkInput.R2 = radius;
  linkOutput = (Geo2CircleLinkOutput_t){0};
  geo2CircleLink(&linkInput, &linkOutput);

  firstIntrv = (GeoDthEdVtCircleIntrvResult_t){0};
  secondIntrv = (GeoDthEdVtCircleIntrvResult_t){0};
  if (linkOutput.PlnFlg == 1) {
    firstIntrv = geoDthEdVtRunCircleIntrvCheck(input,
                                               param,
                                               linkOutput.Ct1X,
                                               linkOutput.Ct1Y,
                                               radius,
                                               stX,
                                               stY,
                                               stTheta,
                                               linkOutput.St2X,
                                               linkOutput.St2Y,
                                               linkOutput.St2Theta,
                                               dirS,
                                               linkOutput.TrnD1,
                                               (Integer *)0,
                                               (Real *)0);
    work->PlnChkCnt = work->PlnChkCnt + firstIntrv.CheckCount;
    if (firstIntrv.IntrvFlg == 0) {
      secondIntrv = geoDthEdVtRunCircleIntrvCheck(input,
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
                                                  (Integer *)0,
                                                  (Real *)0);
      work->PlnChkCnt = work->PlnChkCnt + secondIntrv.CheckCount;
    }
  }

  ok0 = geoDthEdVtApplySecondGroupTwoCircleIntrvResults(
      work, &linkOutput, edX, edY, edTheta, dirS, radius, &firstIntrv, &secondIntrv);
  return ok0;
}

static Boolean geoDthEdVtTrySecondGroupLowRadiusRev2CircleStageCandidate(
    GeoDthEdVtStageWork_t *work,
    const GeoDthEdVtPathPlanningInput_t *input,
    const GeoDthEdVtPathPlanningParam_t *param,
    Real stX,
    Real stY,
    Real stTheta,
    Real edX,
    Real edY,
    Real edTheta,
    Integer pathType,
    Integer dirS,
    Real radiusMin) {
  GeoRev2CircleLinkInput_t linkInput;
  GeoRev2CircleLinkOutput_t linkOutput;
  GeoDthEdVtCircleIntrvResult_t firstIntrv;
  GeoDthEdVtCircleIntrvResult_t secondIntrv;
  Real radius0;
  Boolean ok0;

  geoDthEdVtRecordPlanLoopPathType(work, pathType);
  radius0 = radiusMin + GEO_DTH_ED_VT_LOW_RADIUS_DELTA;
  linkInput = (GeoRev2CircleLinkInput_t){0};
  linkInput.Pt1X = stX;
  linkInput.Pt1Y = stY;
  linkInput.Pt1Theta = stTheta;
  linkInput.Pt2X = edX;
  linkInput.Pt2Y = edY;
  linkInput.Pt2Theta = edTheta;
  linkInput.Dir = dirS;
  linkInput.R1 = radius0;
  linkInput.R2 = radius0;
  linkOutput = (GeoRev2CircleLinkOutput_t){0};
  geoRev2CircleLink(&linkInput, &linkOutput);

  firstIntrv = (GeoDthEdVtCircleIntrvResult_t){0};
  secondIntrv = (GeoDthEdVtCircleIntrvResult_t){0};
  if (linkOutput.GeoOk == 1) {
    firstIntrv = geoDthEdVtRunCircleIntrvCheck(input,
                                               param,
                                               linkOutput.Ct1X,
                                               linkOutput.Ct1Y,
                                               radius0,
                                               stX,
                                               stY,
                                               stTheta,
                                               linkOutput.St2X,
                                               linkOutput.St2Y,
                                               linkOutput.St2Theta,
                                               dirS,
                                               linkOutput.TrnD1,
                                               (Integer *)0,
                                               (Real *)0);
    work->PlnChkCnt = work->PlnChkCnt + firstIntrv.CheckCount;
    if (firstIntrv.IntrvFlg == 0) {
      secondIntrv = geoDthEdVtRunCircleIntrvCheck(input,
                                                  param,
                                                  linkOutput.Ct2X,
                                                  linkOutput.Ct2Y,
                                                  radius0,
                                                  linkOutput.St2X,
                                                  linkOutput.St2Y,
                                                  linkOutput.St2Theta,
                                                  linkOutput.St3X,
                                                  linkOutput.St3Y,
                                                  linkOutput.St3Theta,
                                                  -dirS,
                                                  linkOutput.TrnD2,
                                                  (Integer *)0,
                                                  (Real *)0);
      work->PlnChkCnt = work->PlnChkCnt + secondIntrv.CheckCount;
    }
  }

  ok0 = geoDthEdVtApplyRev2CircleIntrvResults(
      work, &linkOutput, edX, edY, edTheta, dirS, radius0, &firstIntrv, &secondIntrv);
  return ok0;
}

static Boolean geoDthEdVtTryFirstGroupStageFromLoop(GeoDthEdVtStageWork_t *work,
                                                    const GeoDthEdVtPathPlanningInput_t *input,
                                                    const GeoDthEdVtPathPlanningParam_t *param,
                                                    const GeoPosTypClassPkInInput_t *typeInput,
                                                    const GeoPosTypClassPkInOutput_t *typeOutput,
                                                    Real radiusMin) {
  GeoCircle1pt1lineInput_t circleInput;
  GeoCircle1pt1lineOutput_t circleOutput;
  Real radius0;
  Boolean stageOk0;

  circleInput = (GeoCircle1pt1lineInput_t){0};
  circleInput.VstX = typeInput->StPtX;
  circleInput.VstY = typeInput->StPtY;
  circleInput.VstTheta = typeInput->StPtTheta;
  circleInput.VtgtX = typeInput->EdPtX;
  circleInput.VtgtY = typeInput->EdPtY;
  circleInput.VtgtTheta = typeInput->EdPtTheta;
  circleInput.Dir = -work->DirS;
  circleOutput = (GeoCircle1pt1lineOutput_t){0};
  geoCircle1pt1line(&circleInput, &circleOutput);
  radius0 = fabsf(circleOutput.Radius);
  if (radius0 >= radiusMin) {
    stageOk0 = geoDthEdVtTryTwoCircleStageCandidate(work,
                                                    input,
                                                    param,
                                                    typeInput->StPtX,
                                                    typeInput->StPtY,
                                                    typeInput->StPtTheta,
                                                    typeInput->EdPtX,
                                                    typeInput->EdPtY,
                                                    typeInput->EdPtTheta,
                                                    typeOutput->TarPTyp,
                                                    work->DirS,
                                                    radiusMin);
  } else {
    stageOk0 = geoDthEdVtTryFirstGroupReverseTwoCircleStageCandidate(work,
                                                                     input,
                                                                     param,
                                                                     typeInput->StPtX,
                                                                     typeInput->StPtY,
                                                                     typeInput->StPtTheta,
                                                                     typeInput->EdPtX,
                                                                     typeInput->EdPtY,
                                                                     typeInput->EdPtTheta,
                                                                     typeOutput->TarPTyp,
                                                                     work->DirS,
                                                                     radiusMin);
  }
  return stageOk0;
}

static Boolean
geoDthEdVtTrySecondGroupTwoCircleMainCandidate(const GeoDthEdVtPathPlanningInput_t *input,
                                               const GeoDthEdVtPathPlanningParam_t *param,
                                               GeoDthEdVtPathPlanningOutput_t *output) {
  GeoDthEdVtStageWork_t work;
  GeoPosTypClassPkInInput_t typeInput;
  GeoPosTypClassPkInOutput_t typeOutput;
  GeoCircle1pt1lineInput_t circleInput;
  GeoCircle1pt1lineOutput_t circleOutput;
  Integer tailIdx0;
  Real radiusMin0;
  Real radius0;
  Boolean tailOk0;
  Boolean secondGroup0;
  Integer subType0;
  Boolean loopKeep0;
  Boolean loopDone0;
  Boolean stageOk0;
  Boolean ok0;

  geoDthEdVtStageWorkInit(&work);
  tailOk0 = geoDthEdVtSeedTargetTailSegment(
      &work, output->Pt1X, output->Pt1Y, input->TarX, input->TarY, input->TarTheta, (Integer *)0);
  ok0 = false;
  radiusMin0 = geoDthEdVtResolveRadius(input);
  loopKeep0 = tailOk0;
  loopDone0 = false;
  while (loopKeep0 && (!loopDone0)) {
    tailIdx0 = GEO_DTH_ED_VT_STAGE_MAX - work.SegNum;
    if ((work.SegNum <= 0) || (work.SegNum >= GEO_DTH_ED_VT_STAGE_MAX) || (tailIdx0 < 0) ||
        (tailIdx0 >= GEO_DTH_ED_VT_STAGE_MAX)) {
      if (work.FailRsOut[0] == 0) {
        work.FailRsOut[0] = 9;
      }
      loopKeep0 = false;
    } else {
      typeInput = (GeoPosTypClassPkInInput_t){0};
      typeInput.StPtX = work.Stx[tailIdx0];
      typeInput.StPtY = work.Sty[tailIdx0];
      typeInput.StPtTheta = work.StTh[tailIdx0];
      typeInput.EdPtX = input->VehX;
      typeInput.EdPtY = input->VehY;
      typeInput.EdPtTheta = input->VehTheta;
      typeInput.Dir = work.DirS;
      typeOutput = (GeoPosTypClassPkInOutput_t){0};
      geoPosTypClassPkIn(&typeInput, &typeOutput);
      secondGroup0 = geoDthEdVtIsSecondGroupPathType(typeOutput.TarPTyp);
      if (secondGroup0) {
        circleInput = (GeoCircle1pt1lineInput_t){0};
        circleInput.VstX = typeInput.StPtX;
        circleInput.VstY = typeInput.StPtY;
        circleInput.VstTheta = typeInput.StPtTheta;
        circleInput.VtgtX = typeInput.EdPtX;
        circleInput.VtgtY = typeInput.EdPtY;
        circleInput.VtgtTheta = typeInput.EdPtTheta;
        circleInput.Dir = work.DirS;
        circleOutput = (GeoCircle1pt1lineOutput_t){0};
        geoCircle1pt1line(&circleInput, &circleOutput);
        radius0 = fabsf(circleOutput.Radius);
        if (radius0 < radiusMin0) {
          stageOk0 = geoDthEdVtTrySecondGroupLowRadiusRev2CircleStageCandidate(&work,
                                                                               input,
                                                                               param,
                                                                               typeInput.StPtX,
                                                                               typeInput.StPtY,
                                                                               typeInput.StPtTheta,
                                                                               typeInput.EdPtX,
                                                                               typeInput.EdPtY,
                                                                               typeInput.EdPtTheta,
                                                                               typeOutput.TarPTyp,
                                                                               work.DirS,
                                                                               radiusMin0);
          if (stageOk0) {
            if (work.PlanCmd == 1) {
              loopDone0 = true;
            } else if (work.PlanCmd == 2) {
              loopKeep0 = geoDthEdVtAdvancePlanCmdState(&work);
            } else if (work.PlanCmd == 5) {
              loopKeep0 = false;
            } else {
              work.PlanCmd = 5;
              if (work.FailRsOut[0] == 0) {
                work.FailRsOut[0] = 7;
              }
              loopKeep0 = false;
            }
          } else {
            loopKeep0 = false;
          }
        } else {
          subType0 = geoDthEdVtIsSecondGroupSubType2(
              &circleOutput, typeOutput.PtRelaY, typeOutput.PtRelaTheta);
          if (subType0 == GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_1) {
            stageOk0 = geoDthEdVtTrySecondGroupCircleLineSubType1StageCandidate(&work,
                                                                                input,
                                                                                param,
                                                                                typeInput.StPtX,
                                                                                typeInput.StPtY,
                                                                                typeInput.StPtTheta,
                                                                                typeInput.EdPtX,
                                                                                typeInput.EdPtY,
                                                                                typeInput.EdPtTheta,
                                                                                typeOutput.TarPTyp,
                                                                                work.DirS,
                                                                                &circleOutput);
            if (stageOk0) {
              if (work.PlanCmd == 1) {
                loopDone0 = true;
              } else if (work.PlanCmd == 2) {
                loopKeep0 = geoDthEdVtAdvancePlanCmdState(&work);
              } else if (work.PlanCmd == 5) {
                loopKeep0 = false;
              } else {
                work.PlanCmd = 5;
                if (work.FailRsOut[0] == 0) {
                  work.FailRsOut[0] = 7;
                }
                loopKeep0 = false;
              }
            } else {
              loopKeep0 = false;
            }
          } else if (subType0 == GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_2) {
            stageOk0 = geoDthEdVtTryTwoCircleStageCandidate(&work,
                                                            input,
                                                            param,
                                                            typeInput.StPtX,
                                                            typeInput.StPtY,
                                                            typeInput.StPtTheta,
                                                            typeInput.EdPtX,
                                                            typeInput.EdPtY,
                                                            typeInput.EdPtTheta,
                                                            typeOutput.TarPTyp,
                                                            work.DirS,
                                                            radiusMin0);
            if (stageOk0) {
              if (work.PlanCmd == 1) {
                loopDone0 = true;
              } else if (work.PlanCmd == 2) {
                loopKeep0 = geoDthEdVtAdvancePlanCmdState(&work);
              } else if (work.PlanCmd == 5) {
                loopKeep0 = false;
              } else {
                work.PlanCmd = 5;
                if (work.FailRsOut[0] == 0) {
                  work.FailRsOut[0] = 7;
                }
                loopKeep0 = false;
              }
            } else {
              loopKeep0 = false;
            }
          } else if (subType0 == GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_3) {
            stageOk0 =
                geoDthEdVtTrySecondGroupCircle2LineSubType3StageCandidate(&work,
                                                                          input,
                                                                          param,
                                                                          typeInput.StPtX,
                                                                          typeInput.StPtY,
                                                                          typeInput.StPtTheta,
                                                                          typeInput.EdPtX,
                                                                          typeInput.EdPtY,
                                                                          typeInput.EdPtTheta,
                                                                          typeOutput.TarPTyp,
                                                                          work.DirS,
                                                                          radiusMin0);
            if (stageOk0) {
              if (work.PlanCmd == 1) {
                loopDone0 = true;
              } else if (work.PlanCmd == 2) {
                loopKeep0 = geoDthEdVtAdvancePlanCmdState(&work);
              } else if (work.PlanCmd == 5) {
                loopKeep0 = false;
              } else {
                work.PlanCmd = 5;
                if (work.FailRsOut[0] == 0) {
                  work.FailRsOut[0] = 7;
                }
                loopKeep0 = false;
              }
            } else {
              loopKeep0 = false;
            }
          } else {
            geoDthEdVtRecordPlanLoopPathType(&work, typeOutput.TarPTyp);
            work.PlanCmd = 5;
            if (work.FailRsOut[0] == 0) {
              work.FailRsOut[0] = 7;
            }
            loopKeep0 = false;
          }
        }
      } else if ((work.StpPthTypLN > 0) && geoDthEdVtIsFirstGroupPathType(typeOutput.TarPTyp)) {
        stageOk0 = geoDthEdVtTryFirstGroupStageFromLoop(
            &work, input, param, &typeInput, &typeOutput, radiusMin0);
        if (stageOk0) {
          if (work.PlanCmd == 1) {
            loopDone0 = true;
          } else if (work.PlanCmd == 2) {
            loopKeep0 = geoDthEdVtAdvancePlanCmdState(&work);
          } else if (work.PlanCmd == 5) {
            loopKeep0 = false;
          } else {
            work.PlanCmd = 5;
            if (work.FailRsOut[0] == 0) {
              work.FailRsOut[0] = 7;
            }
            loopKeep0 = false;
          }
        } else {
          loopKeep0 = false;
        }
      } else {
        geoDthEdVtRecordPlanLoopPathType(&work, typeOutput.TarPTyp);
        work.PlanCmd = 5;
        if (work.FailRsOut[0] == 0) {
          work.FailRsOut[0] = 7;
        }
        loopKeep0 = false;
      }
    }
  }
  if (loopDone0 && (work.PlanCmd == 1)) {
    ok0 = geoDthEdVtAssembleStageOutput(&work, output);
  }
  if (ok0 && !geoDthEdVtStageCandidateStartsAtVehicle(input, output)) {
    geoDthEdVtClearOutputPath(output);
    output->PlanFlg = 0;
    output->PthPlnFlg = 0;
    ok0 = false;
  }
  if (ok0) {
    output->PlannerCase = GEO_DTH_ED_VT_PLANNER_CASE_GENERIC_FALLBACK;
    output->FallbackGeomCandidateFlg = 1;
    output->FallbackWinnerKind = GEO_DTH_ED_VT_FALLBACK_WINNER_GEOM;
    output->FallbackDecisionReason = 3;
    output->FallbackGeomStpN = output->StpN;
    output->FallbackGeomPathLen = output->PathLen;
    output->FallbackGeomSegNum = output->SegNum;
    output->FallbackGeomPlnChkCnt = work.PlnChkCnt;
    output->PlnChkCnt = output->EntrCheckCnt + output->DthSpaceCheckCnt + work.PlnChkCnt;
  } else {
    (void)geoDthEdVtAssembleStageOutput(&work, output);
    output->FallbackGeomPlnChkCnt = work.PlnChkCnt;
    output->PlnChkCnt = output->EntrCheckCnt + output->DthSpaceCheckCnt + work.PlnChkCnt;
  }
  return ok0;
}

static void geoDthEdVtRefreshOutputMetrics(GeoDthEdVtPathPlanningOutput_t *output) {
  Integer idx;

  output->PathLen = 0.0f;
  idx = 0;
  while (idx < output->SegNum) {
    output->PathLen = output->PathLen + output->SegLen[idx];
    idx = idx + 1;
  }
  output->StpN = geoDthEdVtCountOutputStpN(output);
}

static void geoDthEdVtCopyStagePathTypes(const GeoDthEdVtStageWork_t *work,
                                         GeoDthEdVtPathPlanningOutput_t *output) {
  Integer idx;
  Integer copyCount0;

  output->StpPthTypLN = work->StpPthTypLN;
  output->StpPthTyp0 = 0;
  output->StpPthTyp1 = 0;
  idx = 0;
  while (idx < GEO_DTH_ED_VT_PATH_LIST_MAX) {
    output->StpPthTypList[idx] = 0;
    idx = idx + 1;
  }

  copyCount0 = work->StpPthTypLN;
  if (copyCount0 > GEO_DTH_ED_VT_PATH_LIST_MAX) {
    copyCount0 = GEO_DTH_ED_VT_PATH_LIST_MAX;
  }
  idx = 0;
  while (idx < copyCount0) {
    output->StpPthTypList[idx] = work->StpPthTypList[idx];
    idx = idx + 1;
  }
  if (copyCount0 > 0) {
    output->StpPthTyp0 = output->StpPthTypList[0];
  }
  if (copyCount0 > 1) {
    output->StpPthTyp1 = output->StpPthTypList[1];
  }
}

static void geoDthEdVtTracePlanLoopPathType(const GeoDthEdVtPathPlanningInput_t *input,
                                            GeoDthEdVtPathPlanningOutput_t *output) {
  GeoDthEdVtStageWork_t work;
  GeoPosTypClassPkInInput_t typeInput;
  GeoPosTypClassPkInOutput_t typeOutput;
  Integer tailIdx0;
  Boolean tailOk0;

  geoDthEdVtStageWorkInit(&work);
  tailOk0 = geoDthEdVtSeedTargetTailSegment(
      &work, output->Pt1X, output->Pt1Y, input->TarX, input->TarY, input->TarTheta, (Integer *)0);
  tailIdx0 = GEO_DTH_ED_VT_STAGE_MAX - work.SegNum;
  if (tailOk0 && (tailIdx0 >= 0) && (tailIdx0 < GEO_DTH_ED_VT_STAGE_MAX)) {
    typeInput = (GeoPosTypClassPkInInput_t){0};
    typeInput.StPtX = work.Stx[tailIdx0];
    typeInput.StPtY = work.Sty[tailIdx0];
    typeInput.StPtTheta = work.StTh[tailIdx0];
    typeInput.EdPtX = input->VehX;
    typeInput.EdPtY = input->VehY;
    typeInput.EdPtTheta = input->VehTheta;
    typeInput.Dir = work.DirS;
    typeOutput = (GeoPosTypClassPkInOutput_t){0};
    geoPosTypClassPkIn(&typeInput, &typeOutput);
    geoDthEdVtRecordPlanLoopPathType(&work, typeOutput.TarPTyp);
    geoDthEdVtCopyStagePathTypes(&work, output);
  }
}

static Boolean geoDthEdVtTrySecondGroupStageFromLoop(GeoDthEdVtStageWork_t *work,
                                                     const GeoDthEdVtPathPlanningInput_t *input,
                                                     const GeoDthEdVtPathPlanningParam_t *param,
                                                     const GeoPosTypClassPkInInput_t *typeInput,
                                                     const GeoPosTypClassPkInOutput_t *typeOutput,
                                                     Real radiusMin) {
  GeoCircle1pt1lineInput_t circleInput;
  GeoCircle1pt1lineOutput_t circleOutput;
  Real radius0;
  Integer subType0;
  Boolean stageOk0;

  stageOk0 = false;
  circleInput = (GeoCircle1pt1lineInput_t){0};
  circleInput.VstX = typeInput->StPtX;
  circleInput.VstY = typeInput->StPtY;
  circleInput.VstTheta = typeInput->StPtTheta;
  circleInput.VtgtX = typeInput->EdPtX;
  circleInput.VtgtY = typeInput->EdPtY;
  circleInput.VtgtTheta = typeInput->EdPtTheta;
  circleInput.Dir = work->DirS;
  circleOutput = (GeoCircle1pt1lineOutput_t){0};
  geoCircle1pt1line(&circleInput, &circleOutput);
  radius0 = fabsf(circleOutput.Radius);
  if (radius0 < radiusMin) {
    stageOk0 = geoDthEdVtTrySecondGroupLowRadiusRev2CircleStageCandidate(work,
                                                                         input,
                                                                         param,
                                                                         typeInput->StPtX,
                                                                         typeInput->StPtY,
                                                                         typeInput->StPtTheta,
                                                                         typeInput->EdPtX,
                                                                         typeInput->EdPtY,
                                                                         typeInput->EdPtTheta,
                                                                         typeOutput->TarPTyp,
                                                                         work->DirS,
                                                                         radiusMin);
  } else {
    subType0 = geoDthEdVtIsSecondGroupSubType2(
        &circleOutput, typeOutput->PtRelaY, typeOutput->PtRelaTheta);
    if (subType0 == GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_1) {
      stageOk0 = geoDthEdVtTrySecondGroupCircleLineSubType1StageCandidate(work,
                                                                          input,
                                                                          param,
                                                                          typeInput->StPtX,
                                                                          typeInput->StPtY,
                                                                          typeInput->StPtTheta,
                                                                          typeInput->EdPtX,
                                                                          typeInput->EdPtY,
                                                                          typeInput->EdPtTheta,
                                                                          typeOutput->TarPTyp,
                                                                          work->DirS,
                                                                          &circleOutput);
    } else if (subType0 == GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_2) {
      stageOk0 = geoDthEdVtTryTwoCircleStageCandidate(work,
                                                      input,
                                                      param,
                                                      typeInput->StPtX,
                                                      typeInput->StPtY,
                                                      typeInput->StPtTheta,
                                                      typeInput->EdPtX,
                                                      typeInput->EdPtY,
                                                      typeInput->EdPtTheta,
                                                      typeOutput->TarPTyp,
                                                      work->DirS,
                                                      radiusMin);
    } else if (subType0 == GEO_DTH_ED_VT_SECOND_GROUP_SUBTYPE_3) {
      stageOk0 = geoDthEdVtTrySecondGroupCircle2LineSubType3StageCandidate(work,
                                                                           input,
                                                                           param,
                                                                           typeInput->StPtX,
                                                                           typeInput->StPtY,
                                                                           typeInput->StPtTheta,
                                                                           typeInput->EdPtX,
                                                                           typeInput->EdPtY,
                                                                           typeInput->EdPtTheta,
                                                                           typeOutput->TarPTyp,
                                                                           work->DirS,
                                                                           radiusMin);
    } else {
      geoDthEdVtRecordPlanLoopPathType(work, typeOutput->TarPTyp);
      work->PlanCmd = 5;
      if (work->FailRsOut[0] == 0) {
        work->FailRsOut[0] = 7;
      }
    }
  }
  return stageOk0;
}

static Boolean
geoDthEdVtTryFirstGroupRev2CircleMainCandidate(const GeoDthEdVtPathPlanningInput_t *input,
                                               const GeoDthEdVtPathPlanningParam_t *param,
                                               GeoDthEdVtPathPlanningOutput_t *output) {
  GeoDthEdVtStageWork_t work;
  GeoPosTypClassPkInInput_t typeInput;
  GeoPosTypClassPkInOutput_t typeOutput;
  GeoCircle1pt1lineInput_t circleInput;
  GeoCircle1pt1lineOutput_t circleOutput;
  Integer tailIdx0;
  Real radiusMin0;
  Real radius0;
  Boolean tailOk0;
  Boolean firstGroup0;
  Boolean loopKeep0;
  Boolean loopDone0;
  Boolean stageOk0;
  Boolean ok0;

  geoDthEdVtStageWorkInit(&work);
  tailOk0 = geoDthEdVtSeedTargetTailSegment(
      &work, output->Pt1X, output->Pt1Y, input->TarX, input->TarY, input->TarTheta, (Integer *)0);
  ok0 = false;
  radiusMin0 = geoDthEdVtResolveRadius(input);
  loopKeep0 = tailOk0;
  loopDone0 = false;
  while (loopKeep0 && (!loopDone0)) {
    tailIdx0 = GEO_DTH_ED_VT_STAGE_MAX - work.SegNum;
    if ((work.SegNum <= 0) || (work.SegNum >= GEO_DTH_ED_VT_STAGE_MAX) || (tailIdx0 < 0) ||
        (tailIdx0 >= GEO_DTH_ED_VT_STAGE_MAX)) {
      if (work.FailRsOut[0] == 0) {
        work.FailRsOut[0] = 9;
      }
      loopKeep0 = false;
    } else {
      typeInput = (GeoPosTypClassPkInInput_t){0};
      typeInput.StPtX = work.Stx[tailIdx0];
      typeInput.StPtY = work.Sty[tailIdx0];
      typeInput.StPtTheta = work.StTh[tailIdx0];
      typeInput.EdPtX = input->VehX;
      typeInput.EdPtY = input->VehY;
      typeInput.EdPtTheta = input->VehTheta;
      typeInput.Dir = work.DirS;
      typeOutput = (GeoPosTypClassPkInOutput_t){0};
      geoPosTypClassPkIn(&typeInput, &typeOutput);
      firstGroup0 = geoDthEdVtIsFirstGroupPathType(typeOutput.TarPTyp);
      if (firstGroup0) {
        circleInput = (GeoCircle1pt1lineInput_t){0};
        circleInput.VstX = typeInput.StPtX;
        circleInput.VstY = typeInput.StPtY;
        circleInput.VstTheta = typeInput.StPtTheta;
        circleInput.VtgtX = typeInput.EdPtX;
        circleInput.VtgtY = typeInput.EdPtY;
        circleInput.VtgtTheta = typeInput.EdPtTheta;
        circleInput.Dir = -work.DirS;
        circleOutput = (GeoCircle1pt1lineOutput_t){0};
        geoCircle1pt1line(&circleInput, &circleOutput);
        radius0 = fabsf(circleOutput.Radius);
        if (radius0 >= radiusMin0) {
          stageOk0 = geoDthEdVtTryTwoCircleStageCandidate(&work,
                                                          input,
                                                          param,
                                                          work.Stx[tailIdx0],
                                                          work.Sty[tailIdx0],
                                                          work.StTh[tailIdx0],
                                                          input->VehX,
                                                          input->VehY,
                                                          input->VehTheta,
                                                          typeOutput.TarPTyp,
                                                          work.DirS,
                                                          radiusMin0);
        } else {
          stageOk0 = geoDthEdVtTryFirstGroupReverseTwoCircleStageCandidate(&work,
                                                                           input,
                                                                           param,
                                                                           work.Stx[tailIdx0],
                                                                           work.Sty[tailIdx0],
                                                                           work.StTh[tailIdx0],
                                                                           input->VehX,
                                                                           input->VehY,
                                                                           input->VehTheta,
                                                                           typeOutput.TarPTyp,
                                                                           work.DirS,
                                                                           radiusMin0);
        }
        if (stageOk0) {
          if (work.PlanCmd == 1) {
            loopDone0 = true;
          } else if (work.PlanCmd == 2) {
            loopKeep0 = geoDthEdVtAdvancePlanCmdState(&work);
          } else if (work.PlanCmd == 5) {
            loopKeep0 = false;
          } else {
            work.PlanCmd = 5;
            if (work.FailRsOut[0] == 0) {
              work.FailRsOut[0] = 7;
            }
            loopKeep0 = false;
          }
        } else {
          loopKeep0 = false;
        }
      } else if ((work.StpPthTypLN > 0) && geoDthEdVtIsSecondGroupPathType(typeOutput.TarPTyp)) {
        stageOk0 = geoDthEdVtTrySecondGroupStageFromLoop(
            &work, input, param, &typeInput, &typeOutput, radiusMin0);
        if (stageOk0) {
          if (work.PlanCmd == 1) {
            loopDone0 = true;
          } else if (work.PlanCmd == 2) {
            loopKeep0 = geoDthEdVtAdvancePlanCmdState(&work);
          } else if (work.PlanCmd == 5) {
            loopKeep0 = false;
          } else {
            work.PlanCmd = 5;
            if (work.FailRsOut[0] == 0) {
              work.FailRsOut[0] = 7;
            }
            loopKeep0 = false;
          }
        } else {
          loopKeep0 = false;
        }
      } else {
        geoDthEdVtRecordPlanLoopPathType(&work, typeOutput.TarPTyp);
        work.PlanCmd = 5;
        if (work.FailRsOut[0] == 0) {
          work.FailRsOut[0] = 7;
        }
        loopKeep0 = false;
      }
    }
  }
  if (loopDone0 && (work.PlanCmd == 1)) {
    ok0 = geoDthEdVtAssembleStageOutput(&work, output);
  }
  if (ok0 && !geoDthEdVtStageCandidateStartsAtVehicle(input, output)) {
    geoDthEdVtClearOutputPath(output);
    output->PlanFlg = 0;
    output->PthPlnFlg = 0;
    ok0 = false;
  }
  if (ok0) {
    output->PlannerCase = GEO_DTH_ED_VT_PLANNER_CASE_GENERIC_FALLBACK;
    output->FallbackGeomCandidateFlg = 1;
    output->FallbackWinnerKind = GEO_DTH_ED_VT_FALLBACK_WINNER_GEOM;
    output->FallbackDecisionReason = 2;
    output->FallbackGeomStpN = output->StpN;
    output->FallbackGeomPathLen = output->PathLen;
    output->FallbackGeomSegNum = output->SegNum;
    output->FallbackGeomPlnChkCnt = work.PlnChkCnt;
    output->PlnChkCnt = output->EntrCheckCnt + output->DthSpaceCheckCnt + work.PlnChkCnt;
  }
  return ok0;
}

static Boolean geoDthEdVtOutputSegmentsCanCompress(const GeoDthEdVtPathPlanningOutput_t *output,
                                                   Integer idx) {
  Integer nextIdx;
  Boolean canCompress;

  nextIdx = idx + 1;
  canCompress = false;
  if (nextIdx < output->SegNum) {
    if ((output->SegGoDir[idx] != output->SegGoDir[nextIdx]) &&
        (output->SegTrnDir[idx] == output->SegTrnDir[nextIdx]) &&
        (fabsf(output->SegCrv[idx] - output->SegCrv[nextIdx]) < GEO_DTH_ED_VT_COMPRESS_CRV_EPS) &&
        (fabsf(output->SegCcX[idx] - output->SegCcX[nextIdx]) < GEO_DTH_ED_VT_COMPRESS_CC_EPS) &&
        (fabsf(output->SegCcY[idx] - output->SegCcY[nextIdx]) < GEO_DTH_ED_VT_COMPRESS_CC_EPS)) {
      canCompress = true;
    }
  }
  return canCompress;
}

static void geoDthEdVtCompressOutputSegments(GeoDthEdVtPathPlanningOutput_t *output) {
  Boolean changed;
  Boolean changedAny;
  Integer idx;

  changed = true;
  changedAny = false;
  while (changed) {
    changed = false;
    idx = 0;
    while ((!changed) && (idx < (output->SegNum - 1))) {
      if (geoDthEdVtOutputSegmentsCanCompress(output, idx)) {
        Integer nextIdx;
        Real firstLen;
        Real secondLen;
        Real mergedTht;
        Real lenDiff;

        nextIdx = idx + 1;
        firstLen = output->SegLen[idx];
        secondLen = output->SegLen[nextIdx];
        mergedTht = output->SegTht[idx] + output->SegTht[nextIdx];
        lenDiff = fabsf(firstLen - secondLen);
        if (lenDiff < GEO_DTH_ED_VT_COMPRESS_LEN_EPS) {
          geoDthEdVtDeleteOutputSegments(output, idx, 2);
        } else {
          if (firstLen > secondLen) {
            output->SegTht[idx] = mergedTht;
            output->SegLen[idx] = firstLen - secondLen;
            geoDthEdVtDeleteOutputSegments(output, nextIdx, 1);
          } else {
            geoDthEdVtDeleteOutputSegments(output, idx, 1);
            output->SegTht[idx] = mergedTht;
            output->SegLen[idx] = secondLen - firstLen;
          }
        }
        changed = true;
        changedAny = true;
      } else {
        idx = idx + 1;
      }
    }
  }
  if (changedAny) {
    geoDthEdVtRefreshOutputMetrics(output);
  }
}

static void geoDthEdVtCopyVtOutput(const GeoVtPathPlanningOutput_t *vtOutput,
                                   GeoDthEdVtPathPlanningOutput_t *output) {
  Integer idx;

  geoDthEdVtClearOutputPath(output);
  output->PlanFlg = vtOutput->PlanFlg;
  output->PthPlnFlg = vtOutput->PlanFlg;
  output->SegNum = vtOutput->SegNum;
  output->PathLen = vtOutput->PathLen;
  output->PlnChkCnt = vtOutput->PlnChkCnt + output->EntrCheckCnt + output->DthSpaceCheckCnt;
  output->StpN = vtOutput->StpN;
  output->StpPthTypLN = vtOutput->StpPthTypLN;
  output->PlannerCase = vtOutput->PlannerCase;
  if (output->PlannerCase == GEO_DTH_ED_VT_PLANNER_CASE_NONE) {
    output->PlannerCase = GEO_DTH_ED_VT_PLANNER_CASE_VT_BRIDGE;
  }
  output->FallbackGeomCandidateFlg = vtOutput->FallbackGeomCandidateFlg;
  output->FallbackHybridCandidateFlg = vtOutput->FallbackHybridCandidateFlg;
  output->FallbackWinnerKind = vtOutput->FallbackWinnerKind;
  output->FallbackDecisionReason = vtOutput->FallbackDecisionReason;
  output->FallbackGeomStpN = vtOutput->FallbackGeomStpN;
  output->FallbackGeomPathLen = vtOutput->FallbackGeomPathLen;
  output->FallbackGeomSegNum = vtOutput->FallbackGeomSegNum;
  output->FallbackGeomPlnChkCnt = vtOutput->FallbackGeomPlnChkCnt;
  output->FallbackHybridStpN = vtOutput->FallbackHybridStpN;
  output->FallbackHybridPathLen = vtOutput->FallbackHybridPathLen;
  output->FallbackHybridSegNum = vtOutput->FallbackHybridSegNum;
  output->FallbackHybridPlnChkCnt = vtOutput->FallbackHybridPlnChkCnt;
  output->StpPthTyp0 = vtOutput->StpPthTyp0;
  output->StpPthTyp1 = vtOutput->StpPthTyp1;
  output->FailRs0 = vtOutput->FailRs0;
  output->FailRs1 = vtOutput->FailRs1;

  idx = 0;
  while (idx < VT_SEG_OUT_MAX) {
    output->SegGoDir[idx] = vtOutput->SegGoDir[idx];
    output->SegTrnDir[idx] = vtOutput->SegTrnDir[idx];
    output->SegCrv[idx] = vtOutput->SegCrv[idx];
    output->SegTht[idx] = vtOutput->SegTht[idx];
    output->SegLen[idx] = vtOutput->SegLen[idx];
    output->SegStX[idx] = vtOutput->SegStX[idx];
    output->SegStY[idx] = vtOutput->SegStY[idx];
    output->SegStTh[idx] = vtOutput->SegStTh[idx];
    output->SegCcX[idx] = vtOutput->SegCcX[idx];
    output->SegCcY[idx] = vtOutput->SegCcY[idx];
    idx = idx + 1;
  }

  idx = 0;
  while (idx < GEO_DTH_ED_VT_PATH_LIST_MAX) {
    output->StpPthTypList[idx] = vtOutput->StpPthTypList[idx];
    output->FailRsOut[idx] = vtOutput->FailRsOut[idx];
    idx = idx + 1;
  }
  geoDthEdVtCompressOutputSegments(output);
}

static void geoDthEdVtHybridBuildSlot(const GeoDthEdVtPathPlanningInput_t *input, ApaSlot_t *slot) {
  Real slotPts[8];
  Integer parkingType0;

  apaSlotSetPoints(slotPts,
                   input->SltPt1X,
                   input->SltPt1Y,
                   input->SltPt2X,
                   input->SltPt2Y,
                   input->SltPt3X,
                   input->SltPt3Y,
                   input->SltPt4X,
                   input->SltPt4Y);
  parkingType0 = geoDthEdVtResolveParkingType(input);
  apaPlannerHybridInitSlot(slot,
                           input->TarX,
                           input->TarY,
                           input->TarTheta,
                           input->SltEntr,
                           slotPts,
                           parkingType0,
                           input->SurdType,
                           APA_PARKING_DIREC_REAR);
}

static void geoDthEdVtHybridInitCollisionContext(const GeoDthEdVtPathPlanningInput_t *input,
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
  while ((idx < input->SlotTrackStopperNum) && (idx < GEO_BOUNDARY_LINE_MAX)) {
    (void)apaPlannerHybridCollisionAddSlotTrackStopper(context,
                                                       input->SlotTrackStopperX1[idx],
                                                       input->SlotTrackStopperY1[idx],
                                                       input->SlotTrackStopperX2[idx],
                                                       input->SlotTrackStopperY2[idx]);
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
}

static void geoDthEdVtHybridCopySegmentsToOutput(const ApaPlannerHybridSegments_t *segments,
                                                 GeoDthEdVtPathPlanningOutput_t *output) {
  Integer idx;

  output->PlanFlg = 1;
  output->PthPlnFlg = 1;
  output->SegNum = segments->SegNum;
  output->PathLen = segments->PathLen;
  output->PlnChkCnt = segments->PlnChkCnt + output->EntrCheckCnt + output->DthSpaceCheckCnt;
  output->StpN = segments->StpN;
  output->StpPthTypLN = segments->SegNum;
  output->PlannerCase = GEO_DTH_ED_VT_PLANNER_CASE_HYBRID_FALLBACK;
  output->FallbackHybridCandidateFlg = 1;
  output->FallbackWinnerKind = GEO_DTH_ED_VT_FALLBACK_WINNER_HYBRID;
  output->FallbackDecisionReason = 1;
  output->FallbackHybridStpN = segments->StpN;
  output->FallbackHybridPathLen = segments->PathLen;
  output->FallbackHybridSegNum = segments->SegNum;
  output->FallbackHybridPlnChkCnt = segments->PlnChkCnt;

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
    output->StpPthTypList[idx] = output->SegTrnDir[idx];
    idx = idx + 1;
  }
  output->StpPthTyp0 = output->StpPthTypList[0];
  if (output->SegNum > 1) {
    output->StpPthTyp1 = output->StpPthTypList[1];
  }
  geoDthEdVtCompressOutputSegments(output);
}

static Boolean geoDthEdVtTryHybridFallback(const GeoDthEdVtPathPlanningInput_t *input,
                                           const GeoDthEdVtPathPlanningParam_t *param,
                                           GeoDthEdVtPathPlanningOutput_t *output) {
  if (!apaPlannerHybridResolveFusion(param->NeedHybridAlgFusion))
    return false;
  ApaPlannerGlobalState_t global = APA_PLANNER_GLOBAL_STATE_INIT;
  ApaPlannerHybridCollisionContext_t collision = {0};
  ApaHybridAStar_t planner = {0};
  ApaSlot_t slot = {0};
  ApaPlannerHybridSegments_t segments;
  const Real start[3] = {input->VehX, input->VehY, input->VehTheta};
  const Real goal[3] = {input->TarX, input->TarY, input->TarTheta};
  apaPlannerHybridInitPlannerGlobal(
      &global, param->VehW, geoDthEdVtResolveRadius(input), param->NeedHybridAlgFusion);
  global.Planner.Veh.LF = param->VehLlF;
  global.Planner.Veh.LB = param->VehLr;
  global.Planner.Veh.LL = param->VehLlF + param->VehLr;
  global.Planner.Veh.Lf = param->VehLlF - global.Planner.Veh.WB;
  geoDthEdVtHybridBuildSlot(input, &slot);
  geoDthEdVtHybridInitCollisionContext(input, &slot, &global, &collision);
  apaHybridAStarConfigure(&planner, &global, start, goal, &slot, true);
  if (!apaPlannerHybridSearch(&planner,
                              &collision,
                              &param->SceneQuery,
                              &param->HybridPolicy,
                              param->Diagnostics,
                              GEO_DTH_ED_VT_SEG_OUT_MAX,
                              &segments,
                              0))
    return false;
  geoDthEdVtHybridCopySegmentsToOutput(&segments, output);
  return true;
}

static void geoDthEdVtFinalizeHybridFailure(const GeoVtPathPlanningOutput_t *vtOutput,
                                            GeoDthEdVtPathPlanningOutput_t *output) {
  output->PlanFlg = 0;
  output->PthPlnFlg = 0;
  output->PlannerCase = GEO_DTH_ED_VT_PLANNER_CASE_NONE;
  if (output->FailRsOut[0] != 0) {
    output->FailRs0 = output->FailRsOut[0];
  } else if (output->StpPthTypLN > 0) {
    output->FailRs0 = 7;
  } else {
    output->FailRs0 = vtOutput->FailRs0;
  }
  output->FailRs1 = vtOutput->FailRs1;
  if (output->FailRsOut[0] == 0) {
    output->FailRsOut[0] = output->FailRs0;
  }
  output->FailRsOut[1] = vtOutput->FailRs1;
  output->PlnChkCnt = vtOutput->PlnChkCnt + output->EntrCheckCnt + output->DthSpaceCheckCnt +
                      output->FallbackGeomPlnChkCnt;
}

static Boolean geoDthEdVtCandidateAllowed(const GeoDthEdVtPathPlanningOutput_t *candidate,
                                          const GeoDthEdVtPathPlanningParam_t *param) {
  return candidate->PlanFlg != 0 && apaPlannerHybridCheckGeometry(candidate->SegNum,
                                                                  GEO_DTH_ED_VT_SEG_OUT_MAX,
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

static Boolean geoDthEdVtCanUseRectangularTemplate(const GeoDthEdVtPathPlanningInput_t *input) {
  const Real points[8] = {input->SltPt1X,
                          input->SltPt1Y,
                          input->SltPt2X,
                          input->SltPt2Y,
                          input->SltPt3X,
                          input->SltPt3Y,
                          input->SltPt4X,
                          input->SltPt4Y};
  for (Integer i = 0; i < 4; ++i) {
    const Integer previous = (i + 3) % 4, next = (i + 1) % 4;
    const Real ax = points[2 * previous] - points[2 * i],
               ay = points[2 * previous + 1] - points[2 * i + 1];
    const Real bx = points[2 * next] - points[2 * i], by = points[2 * next + 1] - points[2 * i + 1];
    const Real scale = hypotf(ax, ay) * hypotf(bx, by);
    if (!isfinite(scale) || scale <= GEO_DTH_ED_VT_EPS || fabsf(ax * bx + ay * by) > 0.02f * scale)
      return false;
  }
  return true;
}

static void geoDthEdVtPlanFresh(const GeoDthEdVtPathPlanningInput_t *input,
                                const GeoDthEdVtPathPlanningParam_t *param,
                                GeoDthEdVtPathPlanningOutput_t *output) {
  GeoVtPathPlanningInput_t vtInput;
  GeoVtPathPlanningParam_t vtParam;
  GeoVtPathPlanningOutput_t vtOutput;
  Boolean precheckBlocked0;
  Boolean validVtOutput0;
  Boolean geomOk0;
  Boolean hybridOk0;
  Integer precheckCollisionType0;
  Integer precheckFailRs0;

  *output = (GeoDthEdVtPathPlanningOutput_t){0};
  geoDthEdVtResolveEntrance(input, output);
  geoDthEdVtRunSpaceProbe(input, param, output);
  precheckCollisionType0 = geoDthEdVtPrecheckCollisionType(input, param);
  precheckBlocked0 = (precheckCollisionType0 != VCC_COLLISION_NONE);

  if (precheckBlocked0) {
    apaPlanningRecordFailure(
        param->Diagnostics, APA_FAULT_COLLISION_AT_TARGET_SLOT, "dth_target_precheck", 5);
    precheckFailRs0 = apaPlannerCollisionFailReason(precheckCollisionType0, 8);
    output->PlanFlg = 0;
    output->PthPlnFlg = 0;
    output->PlannerCase = GEO_DTH_ED_VT_PLANNER_CASE_PRECHECK_BLOCKED;
    output->FailRs0 = precheckFailRs0;
    output->FailRsOut[0] = precheckFailRs0;
    output->PlnChkCnt = output->EntrCheckCnt + output->DthSpaceCheckCnt + 1;

    (void)geoDthEdVtTryHybridFallback(input, param, output);
  } else {
    vtOutput = (GeoVtPathPlanningOutput_t){0};
    if (geoDthEdVtCanUseRectangularTemplate(input)) {
      geoDthEdVtBuildVtInput(input, output, &vtInput);
      geoDthEdVtBuildVtParam(param, &vtParam);
      vtParam.RminClassifiedSlot = vtInput.Rmin;
      geoVtPathPlanning(&vtInput, &vtParam, &vtOutput);
    }
    validVtOutput0 =
        (vtOutput.PlanFlg == 1) && (vtOutput.SegNum > 0) && (vtOutput.PathLen > GEO_DTH_ED_VT_EPS);
    if (validVtOutput0) {
      geoDthEdVtCopyVtOutput(&vtOutput, output);
      validVtOutput0 = geoDthEdVtCandidateAllowed(output, param);
    }
    if (!validVtOutput0) {
      geomOk0 = geoDthEdVtTryStraightStageCandidate(input, output);
      geomOk0 = geomOk0 && geoDthEdVtCandidateAllowed(output, param);
      if (!geomOk0) {
        geomOk0 = geoDthEdVtTryFirstGroupRev2CircleMainCandidate(input, param, output);
        geomOk0 = geomOk0 && geoDthEdVtCandidateAllowed(output, param);
      }
      if (!geomOk0) {
        geomOk0 = geoDthEdVtTrySecondGroupTwoCircleMainCandidate(input, param, output);
        geomOk0 = geomOk0 && geoDthEdVtCandidateAllowed(output, param);
      }
      if (!geomOk0) {
        if (output->StpPthTypLN <= 0) {
          geoDthEdVtTracePlanLoopPathType(input, output);
        }
        hybridOk0 = geoDthEdVtTryHybridFallback(input, param, output);
        if (!hybridOk0) {
          geoDthEdVtFinalizeHybridFailure(&vtOutput, output);
        }
      }
    }
  }
}

/**
 * @brief Oblique-edge VT path planner with VT bridge and Hybrid A* fallback.
 * @en_name geoDthEdVtPathPlanning
 * @cn_name 斜边VT路径规划
 * @type widget_module
 * @config cn_name=车位状态, en_name=SltState, desc=APA车位状态输入, unit=1, type=Integer, range=[0,6], default=1
 * @config cn_name=车宽, en_name=VehW, desc=车辆宽度, unit=m, type=Real, range=[1.0,3.0], default=1.9
 * @config cn_name=前悬到参考点, en_name=VehLlF, desc=前悬到规划参考点长度, unit=m, type=Real, range=[0.1,3.0], default=0.9
 * @config cn_name=最小前向安全长, en_name=VehLfMin, desc=最小前向安全长度, unit=m, type=Real, range=[0.0,3.0], default=2.7
 * @config cn_name=后悬到参考点, en_name=VehLr, desc=后悬到规划参考点长度, unit=m, type=Real, range=[0.1,3.0], default=0.9
 * @config cn_name=回退空间, en_name=FallbackSpace, desc=边界探测失败时的回退空间, unit=m, type=Real, range=[0.5,8.0], default=6.0
 * @config cn_name=Hybrid融合开关, en_name=NeedHybridAlgFusion, desc=是否允许在几何规划失败后进入HybridA*回退, unit=1, type=Integer, range=[0,1], default=1
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
 * @param[IN] Integer PrkType 泊车方向
 * @param[IN] Integer SurdType 周边类型
 * @param[IN] Real AgrOfst 可用空间修正量
 * @param[IN] Integer BoundaryLineNum 边界线数量
 * @param[IN] Real[64] BoundaryLineX1 边界线起点X数组
 * @param[IN] Real[64] BoundaryLineY1 边界线起点Y数组
 * @param[IN] Real[64] BoundaryLineX2 边界线终点X数组
 * @param[IN] Real[64] BoundaryLineY2 边界线终点Y数组
 * @param[IN] Integer VirtualBoundaryLineNum 虚拟边界线数量
 * @param[IN] Real[64] VirtualBoundaryP1X 虚拟边界点1X
 * @param[IN] Real[64] VirtualBoundaryP1Y 虚拟边界点1Y
 * @param[IN] Real[64] VirtualBoundaryP2X 虚拟边界点2X
 * @param[IN] Real[64] VirtualBoundaryP2Y 虚拟边界点2Y
 * @param[IN] Integer SlotTrackStopperNum slot stopper线段数量
 * @param[IN] Real[64] SlotTrackStopperX1 slot stopper端点1X
 * @param[IN] Real[64] SlotTrackStopperY1 slot stopper端点1Y
 * @param[IN] Real[64] SlotTrackStopperX2 slot stopper端点2X
 * @param[IN] Real[64] SlotTrackStopperY2 slot stopper端点2Y
 * @param[IN] Integer BoundarySepPtNum 真实离散点数量
 * @param[IN] Real[128] BoundarySepPtX 真实离散点X
 * @param[IN] Real[128] BoundarySepPtY 真实离散点Y
 * @param[IN] Integer StopperPtNum 挡停点数量
 * @param[IN] Real[128] StopperPtX 挡停点X
 * @param[IN] Real[128] StopperPtY 挡停点Y
 * @param[OUT] Integer PlanFlg 规划成功标志
 * @param[OUT] Integer PthPlnFlg 路径规划成功标志
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
 * @param[OUT] Real Pt1X 解析入口点X
 * @param[OUT] Real Pt1Y 解析入口点Y
 * @param[OUT] Real DngPtX 危险点X
 * @param[OUT] Real DngPtY 危险点Y
 * @param[OUT] Integer EntrDgrPtFlg 入口危险点标志
 * @param[OUT] Integer EntrCheckCnt 入口检查次数
 * @param[OUT] Real DthFtSpc 斜向前方空间
 * @param[OUT] Real DthLtSpc 斜向左侧空间
 * @param[OUT] Real DthRtSpc 斜向右侧空间
 * @param[OUT] Integer DthSpaceCheckCnt 斜向空间检查次数
 * @param[OUT] Integer DthIntrvRS 斜向碰撞检查结果
 * @param[OUT] Real DthCrshPosX 碰撞位姿X
 * @param[OUT] Real DthCrshPosY 碰撞位姿Y
 * @param[OUT] Real DthCrshPosTheta 碰撞位姿航向
 * @param[OUT] Real DthCheckPosCX 检查中心X
 * @param[OUT] Real DthCheckPosCY 检查中心Y
 * @param[OUT] Real DthCheckPosCTheta 检查中心航向
 * @param[OUT] Integer PlnChkCnt 规划检查次数
 * @param[OUT] Integer StpN 换向次数
 * @param[OUT] Integer StpPthTypLN 路径类型序列长度
 * @param[OUT] Integer PlannerCase 规划分支编号
 * @param[OUT] Integer FallbackGeomCandidateFlg 几何候选存在标志
 * @param[OUT] Integer FallbackHybridCandidateFlg Hybrid候选存在标志
 * @param[OUT] Integer FallbackWinnerKind 回退赢家0无1几何2Hybrid
 * @param[OUT] Integer FallbackDecisionReason 回退决策原因
 * @param[OUT] Integer FallbackGeomStpN 几何候选换向次数
 * @param[OUT] Real FallbackGeomPathLen 几何候选路径总长
 * @param[OUT] Integer FallbackGeomSegNum 几何候选段数
 * @param[OUT] Integer FallbackGeomPlnChkCnt 几何候选检查次数
 * @param[OUT] Integer FallbackHybridStpN Hybrid候选换向次数
 * @param[OUT] Real FallbackHybridPathLen Hybrid候选路径总长
 * @param[OUT] Integer FallbackHybridSegNum Hybrid候选段数
 * @param[OUT] Integer FallbackHybridPlnChkCnt Hybrid候选检查次数
 * @param[OUT] Integer StpPthTyp0 路径类型序列首项
 * @param[OUT] Integer StpPthTyp1 路径类型序列次项
 * @param[OUT] Integer[30] StpPthTypList 路径类型序列
 * @param[OUT] Integer FailRs0 失败原因首项
 * @param[OUT] Integer FailRs1 失败原因次项
 * @param[OUT] Integer[30] FailRsOut 失败原因序列
 * @source matlab_rewrite geo_DthEdVT_PathPlanning.m
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 几何路径规划
 * @version 1.0
 * @date 2026-04-24
 * @author aide
 */
void geoDthEdVtPathPlanning(const GeoDthEdVtPathPlanningInput_t *input,
                            const GeoDthEdVtPathPlanningParam_t *param,
                            GeoDthEdVtPathPlanningOutput_t *output) {

  geoDthEdVtPlanFresh(input, param, output);
}
