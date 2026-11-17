#include "apaParkingSearchFallback.h"
#include "apaPlannerHybridParity.h"
#include "apaPathTypes.h"
#include "apaPathInterval.h"
#include "apaVehicleDefaults.h"
#include "apaCountUtils.h"
#include "apaParallelParkoutPlan.h"
#include "apaParkoutUtils.h"
#include "apaCoordUtils.h"
#include "apaGoalTolerance.h"
#include "apaGeoUtils.h"
#include "apaParkingPathPlanningInternal.h"
#include "apaParkingPathSafetyGate.h"
#include "apaParkingPathPlanningTypes.h"
#include "apaSlotTypes.h"
#include "vehicleCollisionCheckTypes.h"
#include <math.h>

#ifndef APA_PARKING_PLANNER_COLLISION_SAMPLE_STEP
#define APA_PARKING_PLANNER_COLLISION_SAMPLE_STEP 0.05f
#endif

#ifndef APA_PARKING_PLANNER_DEFAULT_WHEELBASE
#define APA_PARKING_PLANNER_DEFAULT_WHEELBASE APA_VEHICLE_WHEELBASE_M
#endif

#ifndef APA_PARKING_PLANNER_PARKOUT_RUNTIME_MIN_PATH_LEN
#define APA_PARKING_PLANNER_PARKOUT_RUNTIME_MIN_PATH_LEN 2.0f
#endif

#ifndef APA_PARKING_PLANNER_PARKOUT_FAIL_RS_RETRY_FALLBACK
#define APA_PARKING_PLANNER_PARKOUT_FAIL_RS_RETRY_FALLBACK 9
#endif

#ifndef APA_PARKING_PLANNER_GENERIC_PARKOUT_SAMPLE_STEP
#define APA_PARKING_PLANNER_GENERIC_PARKOUT_SAMPLE_STEP 0.2f
#endif

#ifndef APA_PARKING_PLANNER_GENERIC_PARKOUT_EXIT_CLEARANCE
#define APA_PARKING_PLANNER_GENERIC_PARKOUT_EXIT_CLEARANCE 1.2f
#endif

#ifndef APA_PARKING_PLANNER_REVERSE_PARKOUT_START_SPIKE_DIST
#define APA_PARKING_PLANNER_REVERSE_PARKOUT_START_SPIKE_DIST 2.0f
#endif

#ifndef APA_PARKING_PLANNER_REVERSE_PARKOUT_START_RETURN_DIST
#define APA_PARKING_PLANNER_REVERSE_PARKOUT_START_RETURN_DIST 0.5f
#endif

#ifndef APA_PARKING_PLANNER_GENERIC_PARKOUT_PLANNER_CASE
#define APA_PARKING_PLANNER_GENERIC_PARKOUT_PLANNER_CASE 6
#endif

void apaParkingPathPlanningZeroOutput(ApaParkingPathPlanningOutput_t *output) {
  Integer idx;

  output->SelectedSlotKind = APA_SCENARIO_SLOT_KIND_MANUAL;
  output->PlanFlg = 0;
  output->ErrorType = APA_FAULT_UNKNOWN_ERR;
  output->PlannerCase = 0;
  output->SegNum = 0;
  output->StpN = 0;
  output->PathLen = 0.0f;
  output->PtCount = 0;
  output->StepCount = 0;
  output->St2Idx = 0;
  output->Stp1PtRX = 0.0f;
  output->Stp1PtRY = 0.0f;
  output->Stp1PtRTheta = 0.0f;

  idx = 0;
  while (idx < APA_PARKING_PATH_PT_MAX) {
    output->PthPtX[idx] = 0.0f;
    output->PthPtY[idx] = 0.0f;
    output->PthPtTht[idx] = 0.0f;
    output->PthPtOdo[idx] = 0.0f;
    idx = idx + 1;
  }
}

void apaParkingPathPlanningCopyOutput(const ApaParkingPathPlanningOutput_t *src,
                                      ApaParkingPathPlanningOutput_t *dst) {
  Integer idx;

  dst->SelectedSlotKind = src->SelectedSlotKind;
  dst->PlanFlg = src->PlanFlg;
  dst->ErrorType = src->ErrorType;
  dst->PlannerCase = src->PlannerCase;
  dst->SegNum = src->SegNum;
  dst->StpN = src->StpN;
  dst->PathLen = src->PathLen;
  dst->PtCount = src->PtCount;
  dst->StepCount = src->StepCount;
  dst->St2Idx = src->St2Idx;
  dst->Stp1PtRX = src->Stp1PtRX;
  dst->Stp1PtRY = src->Stp1PtRY;
  dst->Stp1PtRTheta = src->Stp1PtRTheta;

  idx = 0;
  while (idx < APA_PARKING_PATH_PT_MAX) {
    dst->PthPtX[idx] = src->PthPtX[idx];
    dst->PthPtY[idx] = src->PthPtY[idx];
    dst->PthPtTht[idx] = src->PthPtTht[idx];
    dst->PthPtOdo[idx] = src->PthPtOdo[idx];
    idx = idx + 1;
  }
}

Boolean apaParkingPathPlanningSuccessPayloadIsValid(const ApaParkingPathPlanningOutput_t *output,
                                                    Integer segmentMaximum) {
  Integer index0;
  Boolean planValid0;
  Boolean segmentMinimumValid0;
  Boolean segmentMaximumValid0;
  Boolean segmentValid0;
  Boolean pointMinimumValid0;
  Boolean pointMaximumValid0;
  Boolean pointCountValid0;
  Boolean pathLengthFinite0;
  Boolean pathLengthPositive0;
  Boolean pathLengthValid0;
  Boolean payloadValid0;
  Boolean pointLoop0;

  planValid0 = (output->PlanFlg != 0);
  segmentMinimumValid0 = (output->SegNum > 0);
  segmentMaximumValid0 = (output->SegNum <= segmentMaximum);
  segmentValid0 = segmentMinimumValid0 && segmentMaximumValid0;
  pointMinimumValid0 = (output->PtCount > 1);
  pointMaximumValid0 = (output->PtCount <= APA_PARKING_PATH_PT_MAX);
  pointCountValid0 = pointMinimumValid0 && pointMaximumValid0;
  pathLengthFinite0 = (isfinite(output->PathLen) != 0);
  pathLengthPositive0 = (output->PathLen > 0.0f);
  pathLengthValid0 = pathLengthFinite0 && pathLengthPositive0;
  payloadValid0 = planValid0 && segmentValid0;
  payloadValid0 = payloadValid0 && pointCountValid0;
  payloadValid0 = payloadValid0 && pathLengthValid0;

  index0 = 0;
  pointLoop0 = (index0 < output->PtCount) && payloadValid0;
  while (pointLoop0) {
    Boolean xFinite0;
    Boolean yFinite0;
    Boolean thetaFinite0;
    Boolean odoFinite0;
    Boolean pointFinite0;

    xFinite0 = (isfinite(output->PthPtX[index0]) != 0);
    yFinite0 = (isfinite(output->PthPtY[index0]) != 0);
    thetaFinite0 = (isfinite(output->PthPtTht[index0]) != 0);
    odoFinite0 = (isfinite(output->PthPtOdo[index0]) != 0);
    pointFinite0 = xFinite0 && yFinite0;
    pointFinite0 = pointFinite0 && thetaFinite0;
    pointFinite0 = pointFinite0 && odoFinite0;
    if (!pointFinite0) {
      payloadValid0 = false;
    }
    if (payloadValid0 && index0 + 1 < output->PtCount &&
        !apaPathIntervalIsConsistent(output->PthPtX[index0],
                                     output->PthPtY[index0],
                                     output->PthPtTht[index0],
                                     output->PthPtOdo[index0],
                                     output->PthPtX[index0 + 1],
                                     output->PthPtY[index0 + 1],
                                     output->PthPtTht[index0 + 1])) {
      payloadValid0 = false;
    }
    index0 = index0 + 1;
    pointLoop0 = (index0 < output->PtCount) && payloadValid0;
  }
  return payloadValid0;
}

Boolean
apaParkingPathPlanningCandidateIsStructurallyValid(const ApaParkingPathPlanningInput_t *input,
                                                   const ApaParkingPathPlanningOutput_t *output) {
  Integer expectedSlotKind0;
  Integer segmentMaximum0;
  Boolean actionIsParkin0;
  Boolean actionIsParkout0;
  Boolean routeValid0;
  Boolean selectedSlotKindValid0;
  Boolean payloadValid0;
  Boolean stpCountMinimumValid0;
  Boolean stpCountMaximumValid0;
  Boolean stpCountValid0;
  Boolean stepCountMinimumValid0;
  Boolean stepCountMaximumValid0;
  Boolean stepCountValid0;
  Boolean st2MinimumValid0;
  Boolean st2MaximumValid0;
  Boolean st2Valid0;
  Boolean stp1XFinite0;
  Boolean stp1YFinite0;
  Boolean stp1ThetaFinite0;
  Boolean stp1Finite0;
  Boolean stp1UnavailableIsZero0;
  Boolean structuralValid0;

  expectedSlotKind0 = APA_SCENARIO_SLOT_KIND_MANUAL;
  segmentMaximum0 = 0;
  actionIsParkin0 = (input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKIN);
  actionIsParkout0 = (input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKOUT);
  routeValid0 = false;
  if (actionIsParkout0) {
    expectedSlotKind0 = APA_SCENARIO_SLOT_KIND_PARKOUT;
    segmentMaximum0 = APA_PARKING_PATH_SEG_OUT_MAX;
    routeValid0 = true;
  } else if (actionIsParkin0) {
    expectedSlotKind0 = input->TargetSlotKind;
    switch (input->TargetSlotKind) {
    case APA_SCENARIO_SLOT_KIND_VT:
      segmentMaximum0 = VT_SEG_OUT_MAX;
      routeValid0 = true;
      break;
    case APA_SCENARIO_SLOT_KIND_HT:
      segmentMaximum0 = HT_SEG_OUT_MAX;
      routeValid0 = true;
      break;
    case APA_SCENARIO_SLOT_KIND_DTH_ED_VT:
      segmentMaximum0 = GEO_DTH_ED_VT_SEG_OUT_MAX;
      routeValid0 = true;
      break;
    case APA_SCENARIO_SLOT_KIND_VH:
      segmentMaximum0 = VH_SEG_OUT_MAX;
      routeValid0 = true;
      break;
    default:
      break;
    }
  }

  selectedSlotKindValid0 = (output->SelectedSlotKind == expectedSlotKind0);
  payloadValid0 = apaParkingPathPlanningSuccessPayloadIsValid(output, segmentMaximum0);
  stpCountMinimumValid0 = (output->StpN >= 0);
  stpCountMaximumValid0 = (output->StpN <= output->SegNum);
  stpCountValid0 = stpCountMinimumValid0 && stpCountMaximumValid0;
  if (actionIsParkout0) {
    stepCountMinimumValid0 = (output->StepCount == 0);
    stepCountMaximumValid0 = true;
  } else {
    stepCountMinimumValid0 = (output->StepCount > 0);
    stepCountMaximumValid0 = (output->StepCount <= output->SegNum);
  }
  stepCountValid0 = stepCountMinimumValid0 && stepCountMaximumValid0;
  if (actionIsParkout0 || (expectedSlotKind0 == APA_SCENARIO_SLOT_KIND_VH)) {
    st2MinimumValid0 = (output->St2Idx == 0);
    st2MaximumValid0 = true;
  } else {

    st2MinimumValid0 = (output->St2Idx >= 0);
    st2MaximumValid0 = (output->St2Idx <= output->PtCount);
  }
  st2Valid0 = st2MinimumValid0 && st2MaximumValid0;
  stp1XFinite0 = (isfinite(output->Stp1PtRX) != 0);
  stp1YFinite0 = (isfinite(output->Stp1PtRY) != 0);
  stp1ThetaFinite0 = (isfinite(output->Stp1PtRTheta) != 0);
  stp1Finite0 = stp1XFinite0 && stp1YFinite0;
  stp1Finite0 = stp1Finite0 && stp1ThetaFinite0;
  stp1UnavailableIsZero0 = true;
  if (actionIsParkout0 || (expectedSlotKind0 == APA_SCENARIO_SLOT_KIND_HT) ||
      (expectedSlotKind0 == APA_SCENARIO_SLOT_KIND_VH)) {
    stp1UnavailableIsZero0 = (output->Stp1PtRX == 0.0f);
    stp1UnavailableIsZero0 = stp1UnavailableIsZero0 && (output->Stp1PtRY == 0.0f);
    stp1UnavailableIsZero0 = stp1UnavailableIsZero0 && (output->Stp1PtRTheta == 0.0f);
  }
  structuralValid0 = routeValid0 && selectedSlotKindValid0;
  structuralValid0 = structuralValid0 && payloadValid0;
  structuralValid0 = structuralValid0 && stpCountValid0;
  structuralValid0 = structuralValid0 && stepCountValid0;
  structuralValid0 = structuralValid0 && st2Valid0;
  structuralValid0 = structuralValid0 && stp1Finite0;
  structuralValid0 = structuralValid0 && stp1UnavailableIsZero0;
  return structuralValid0;
}

Real apaParkingPathPlanningInterpolateThetaShortest(Real previousTheta,
                                                    Real currentTheta,
                                                    Real ratio) {
  Real thetaDifference0;
  Real interpolatedTheta0;

  thetaDifference0 = apaCoordWrapToPi(currentTheta - previousTheta);
  interpolatedTheta0 = previousTheta + (thetaDifference0 * ratio);
  interpolatedTheta0 = apaCoordWrapToPi(interpolatedTheta0);
  return interpolatedTheta0;
}

void apaParkingPathPlanningFinalizeL1Output(ApaParkingPathPlanningOutput_t *output,
                                            Integer segmentMaximum) {
  Boolean successPayloadValid0;
  Integer selectedSlotKind0;
  Integer plannerCase0;
  Integer errorType0;

  successPayloadValid0 = apaParkingPathPlanningSuccessPayloadIsValid(output, segmentMaximum);
  if (!successPayloadValid0) {
    selectedSlotKind0 = output->SelectedSlotKind;
    plannerCase0 = output->PlannerCase;
    errorType0 = output->ErrorType;
    apaParkingPathPlanningZeroOutput(output);
    output->SelectedSlotKind = selectedSlotKind0;
    output->PlannerCase = plannerCase0;
    output->ErrorType = errorType0;
  } else {
    output->ErrorType = APA_FAULT_NO_ERR;
  }
}

void apaParkingPathPlanningFillVtInput(const ApaParkingPathPlanningInput_t *input,
                                       GeoVtPathPlanningInput_t *vtInput) {
  Integer idx;
  Integer boundaryOutIdx;

  *vtInput = (GeoVtPathPlanningInput_t){0};
  vtInput->VehX = input->VehicleX;
  vtInput->VehY = input->VehicleY;
  vtInput->VehTheta = input->VehicleYaw;
  vtInput->TarX = input->TarX;
  vtInput->TarY = input->TarY;
  vtInput->TarTheta = input->TarTheta;
  vtInput->Rmin = input->Rmin;
  vtInput->EntrPtX = input->EntrPtX;
  vtInput->EntrPtY = input->EntrPtY;
  vtInput->SltPt1X = input->SltPt1X;
  vtInput->SltPt1Y = input->SltPt1Y;
  vtInput->SltPt2X = input->SltPt2X;
  vtInput->SltPt2Y = input->SltPt2Y;
  vtInput->SltPt3X = input->SltPt3X;
  vtInput->SltPt3Y = input->SltPt3Y;
  vtInput->SltPt4X = input->SltPt4X;
  vtInput->SltPt4Y = input->SltPt4Y;
  vtInput->SltEntr = input->SltEntr;
  vtInput->PrkType = input->PrkType;
  vtInput->SurdType = input->SurdType;
  vtInput->AgrOfst = input->AgrOfst;
  vtInput->BoundaryLineNum = 0;
  vtInput->VirtualBoundaryLineNum = input->VirtualBoundaryLineNum;
  vtInput->SlotTrackStopperNum = input->SlotTrackStopperNum;
  vtInput->BoundarySepPtNum = input->BoundarySepPtNum;
  vtInput->StopperPtNum = input->StopperPtNum;

  idx = 0;
  boundaryOutIdx = 0;
  while (idx < APA_SCENARIO_BOUNDARY_LINE_MAX) {
    if (idx < input->BoundaryLineNum) {
      vtInput->BoundaryLineX1[boundaryOutIdx] = input->BoundaryLineX1[idx];
      vtInput->BoundaryLineY1[boundaryOutIdx] = input->BoundaryLineY1[idx];
      vtInput->BoundaryLineX2[boundaryOutIdx] = input->BoundaryLineX2[idx];
      vtInput->BoundaryLineY2[boundaryOutIdx] = input->BoundaryLineY2[idx];
      boundaryOutIdx = boundaryOutIdx + 1;
    }
    vtInput->VirtualBoundaryP1X[idx] = input->VirtualBoundaryP1X[idx];
    vtInput->VirtualBoundaryP1Y[idx] = input->VirtualBoundaryP1Y[idx];
    vtInput->VirtualBoundaryP2X[idx] = input->VirtualBoundaryP2X[idx];
    vtInput->VirtualBoundaryP2Y[idx] = input->VirtualBoundaryP2Y[idx];
    vtInput->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    vtInput->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    vtInput->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    vtInput->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    idx = idx + 1;
  }
  vtInput->BoundaryLineNum = boundaryOutIdx;

  idx = 0;
  while (idx < APA_SCENARIO_BOUNDARY_POINT_MAX) {
    vtInput->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    vtInput->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    vtInput->StopperPtX[idx] = input->StopperPtX[idx];
    vtInput->StopperPtY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }
}

void apaParkingPathPlanningFillHtInput(const ApaParkingPathPlanningInput_t *input,
                                       GeoHtPathPlanningInput_t *htInput) {
  Integer idx;
  Integer boundaryOutIdx;

  *htInput = (GeoHtPathPlanningInput_t){0};
  htInput->VehX = input->VehicleX;
  htInput->VehY = input->VehicleY;
  htInput->VehTheta = input->VehicleYaw;
  htInput->TarX = input->TarX;
  htInput->TarY = input->TarY;
  htInput->TarTheta = input->TarTheta;
  htInput->Rmin = input->Rmin;
  htInput->SltPt1X = input->SltPt1X;
  htInput->SltPt1Y = input->SltPt1Y;
  htInput->SltPt2X = input->SltPt2X;
  htInput->SltPt2Y = input->SltPt2Y;
  htInput->SltPt3X = input->SltPt3X;
  htInput->SltPt3Y = input->SltPt3Y;
  htInput->SltPt4X = input->SltPt4X;
  htInput->SltPt4Y = input->SltPt4Y;
  htInput->SltEntr = input->SltEntr;
  htInput->SlotType = 1;
  htInput->PrkType = input->PrkType;
  htInput->SurdType = input->SurdType;
  htInput->BoundaryLineNum = 0;
  htInput->VirtualBoundaryLineNum = input->VirtualBoundaryLineNum;
  htInput->SlotTrackStopperNum = input->SlotTrackStopperNum;
  htInput->BoundarySepPtNum = input->BoundarySepPtNum;
  htInput->StopperPtNum = input->StopperPtNum;

  idx = 0;
  boundaryOutIdx = 0;
  while (idx < APA_SCENARIO_BOUNDARY_LINE_MAX) {
    if (idx < input->BoundaryLineNum) {
      htInput->BoundaryLineX1[boundaryOutIdx] = input->BoundaryLineX1[idx];
      htInput->BoundaryLineY1[boundaryOutIdx] = input->BoundaryLineY1[idx];
      htInput->BoundaryLineX2[boundaryOutIdx] = input->BoundaryLineX2[idx];
      htInput->BoundaryLineY2[boundaryOutIdx] = input->BoundaryLineY2[idx];
      boundaryOutIdx = boundaryOutIdx + 1;
    }
    htInput->VirtualBoundaryP1X[idx] = input->VirtualBoundaryP1X[idx];
    htInput->VirtualBoundaryP1Y[idx] = input->VirtualBoundaryP1Y[idx];
    htInput->VirtualBoundaryP2X[idx] = input->VirtualBoundaryP2X[idx];
    htInput->VirtualBoundaryP2Y[idx] = input->VirtualBoundaryP2Y[idx];
    htInput->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    htInput->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    htInput->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    htInput->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    idx = idx + 1;
  }
  htInput->BoundaryLineNum = boundaryOutIdx;

  idx = 0;
  while (idx < APA_SCENARIO_BOUNDARY_POINT_MAX) {
    htInput->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    htInput->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    htInput->StopperPtX[idx] = input->StopperPtX[idx];
    htInput->StopperPtY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }
}

void apaParkingPathPlanningFillDthInput(const ApaParkingPathPlanningInput_t *input,
                                        GeoDthEdVtPathPlanningInput_t *dthInput) {
  Integer idx;
  Integer boundaryOutIdx;

  *dthInput = (GeoDthEdVtPathPlanningInput_t){0};
  dthInput->VehX = input->VehicleX;
  dthInput->VehY = input->VehicleY;
  dthInput->VehTheta = input->VehicleYaw;
  dthInput->TarX = input->TarX;
  dthInput->TarY = input->TarY;
  dthInput->TarTheta = input->TarTheta;
  dthInput->Rmin = input->Rmin;
  dthInput->EntrPtX = input->EntrPtX;
  dthInput->EntrPtY = input->EntrPtY;
  dthInput->SltPt1X = input->SltPt1X;
  dthInput->SltPt1Y = input->SltPt1Y;
  dthInput->SltPt2X = input->SltPt2X;
  dthInput->SltPt2Y = input->SltPt2Y;
  dthInput->SltPt3X = input->SltPt3X;
  dthInput->SltPt3Y = input->SltPt3Y;
  dthInput->SltPt4X = input->SltPt4X;
  dthInput->SltPt4Y = input->SltPt4Y;
  dthInput->SltEntr = input->SltEntr;
  dthInput->PrkType = input->PrkType;
  dthInput->SurdType = input->SurdType;
  dthInput->AgrOfst = input->AgrOfst;
  dthInput->BoundaryLineNum = 0;
  dthInput->VirtualBoundaryLineNum = input->VirtualBoundaryLineNum;
  dthInput->SlotTrackStopperNum = input->SlotTrackStopperNum;
  dthInput->BoundarySepPtNum = input->BoundarySepPtNum;
  dthInput->StopperPtNum = input->StopperPtNum;

  idx = 0;
  boundaryOutIdx = 0;
  while (idx < APA_SCENARIO_BOUNDARY_LINE_MAX) {
    if (idx < input->BoundaryLineNum) {
      dthInput->BoundaryLineX1[boundaryOutIdx] = input->BoundaryLineX1[idx];
      dthInput->BoundaryLineY1[boundaryOutIdx] = input->BoundaryLineY1[idx];
      dthInput->BoundaryLineX2[boundaryOutIdx] = input->BoundaryLineX2[idx];
      dthInput->BoundaryLineY2[boundaryOutIdx] = input->BoundaryLineY2[idx];
      boundaryOutIdx = boundaryOutIdx + 1;
    }
    dthInput->VirtualBoundaryP1X[idx] = input->VirtualBoundaryP1X[idx];
    dthInput->VirtualBoundaryP1Y[idx] = input->VirtualBoundaryP1Y[idx];
    dthInput->VirtualBoundaryP2X[idx] = input->VirtualBoundaryP2X[idx];
    dthInput->VirtualBoundaryP2Y[idx] = input->VirtualBoundaryP2Y[idx];
    dthInput->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    dthInput->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    dthInput->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    dthInput->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    idx = idx + 1;
  }
  dthInput->BoundaryLineNum = boundaryOutIdx;

  idx = 0;
  while (idx < APA_SCENARIO_BOUNDARY_POINT_MAX) {
    dthInput->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    dthInput->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    dthInput->StopperPtX[idx] = input->StopperPtX[idx];
    dthInput->StopperPtY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }
}

void apaParkingPathPlanningFillVhInput(const ApaParkingPathPlanningInput_t *input,
                                       GeoVhPathPlanningInput_t *vhInput) {
  Integer idx;
  Integer boundaryOutIdx;

  *vhInput = (GeoVhPathPlanningInput_t){0};
  vhInput->VehX = input->VehicleX;
  vhInput->VehY = input->VehicleY;
  vhInput->VehTheta = input->VehicleYaw;
  vhInput->TarX = input->TarX;
  vhInput->TarY = input->TarY;
  vhInput->TarTheta = input->TarTheta;
  vhInput->Rmin = input->Rmin;
  vhInput->EntrPtX = input->EntrPtX;
  vhInput->EntrPtY = input->EntrPtY;
  vhInput->SltPt1X = input->SltPt1X;
  vhInput->SltPt1Y = input->SltPt1Y;
  vhInput->SltPt2X = input->SltPt2X;
  vhInput->SltPt2Y = input->SltPt2Y;
  vhInput->SltPt3X = input->SltPt3X;
  vhInput->SltPt3Y = input->SltPt3Y;
  vhInput->SltPt4X = input->SltPt4X;
  vhInput->SltPt4Y = input->SltPt4Y;
  vhInput->SltEntr = input->SltEntr;
  vhInput->BoundaryLineNum = 0;
  vhInput->VirtualBoundaryLineNum = input->VirtualBoundaryLineNum;
  vhInput->SlotTrackStopperNum = input->SlotTrackStopperNum;
  vhInput->BoundarySepPtNum = input->BoundarySepPtNum;
  vhInput->StopperPtNum = input->StopperPtNum;

  idx = 0;
  boundaryOutIdx = 0;
  while (idx < APA_SCENARIO_BOUNDARY_LINE_MAX) {
    if (idx < input->BoundaryLineNum) {
      vhInput->BoundaryLineX1[boundaryOutIdx] = input->BoundaryLineX1[idx];
      vhInput->BoundaryLineY1[boundaryOutIdx] = input->BoundaryLineY1[idx];
      vhInput->BoundaryLineX2[boundaryOutIdx] = input->BoundaryLineX2[idx];
      vhInput->BoundaryLineY2[boundaryOutIdx] = input->BoundaryLineY2[idx];
      boundaryOutIdx = boundaryOutIdx + 1;
    }
    vhInput->VirtualBoundaryP1X[idx] = input->VirtualBoundaryP1X[idx];
    vhInput->VirtualBoundaryP1Y[idx] = input->VirtualBoundaryP1Y[idx];
    vhInput->VirtualBoundaryP2X[idx] = input->VirtualBoundaryP2X[idx];
    vhInput->VirtualBoundaryP2Y[idx] = input->VirtualBoundaryP2Y[idx];
    vhInput->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    vhInput->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    vhInput->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    vhInput->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    idx = idx + 1;
  }
  vhInput->BoundaryLineNum = boundaryOutIdx;

  idx = 0;
  while (idx < APA_SCENARIO_BOUNDARY_POINT_MAX) {
    vhInput->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    vhInput->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    vhInput->StopperPtX[idx] = input->StopperPtX[idx];
    vhInput->StopperPtY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }
}

void apaParkingPathPlanningFillVtParam(const ApaParkingPathPlanningParam_t *param,
                                       GeoVtPathPlanningParam_t *vtParam) {
  *vtParam = (GeoVtPathPlanningParam_t){0};
  vtParam->SltState = param->SltState;
  vtParam->VehW = param->VehW;
  vtParam->VehLlF = param->VehLlF;
  vtParam->VehLfMin = param->VehLfMin;
  vtParam->VehLr = param->VehLr;
  vtParam->FallbackSpace = param->VtFallbackSpace;
  vtParam->NeedHybridAlgFusion = param->NeedHybridAlgFusion;
  vtParam->Diagnostics = param->Diagnostics;
  vtParam->HybridPolicy = param->HybridPolicy;
  vtParam->SceneQuery = param->SceneQuery;
}

void apaParkingPathPlanningFillHtParam(const ApaParkingPathPlanningParam_t *param,
                                       GeoHtPathPlanningParam_t *htParam) {
  *htParam = (GeoHtPathPlanningParam_t){0};
  htParam->SltState = param->SltState;
  htParam->VehW = param->VehW;
  htParam->VehLlF = param->VehLlF;
  htParam->VehLfMin = param->VehLfMin;
  htParam->VehLr = param->VehLr;
  htParam->RfoutMin = param->HtRfoutMin;
  htParam->NeedHybridAlgFusion = param->NeedHybridAlgFusion;
  htParam->SceneQuery = param->SceneQuery;
  htParam->Diagnostics = param->Diagnostics;
  htParam->HybridPolicy = param->HybridPolicy;
}

void apaParkingPathPlanningFillDthParam(const ApaParkingPathPlanningParam_t *param,
                                        GeoDthEdVtPathPlanningParam_t *dthParam) {
  *dthParam = (GeoDthEdVtPathPlanningParam_t){0};
  dthParam->SltState = param->SltState;
  dthParam->VehW = param->VehW;
  dthParam->VehLlF = param->VehLlF;
  dthParam->VehLfMin = param->VehLfMin;
  dthParam->VehLr = param->VehLr;
  dthParam->FallbackSpace = param->DthFallbackSpace;
  dthParam->NeedHybridAlgFusion = param->NeedHybridAlgFusion;
  dthParam->SceneQuery = param->SceneQuery;
  dthParam->Diagnostics = param->Diagnostics;
  dthParam->HybridPolicy = param->HybridPolicy;
}

void apaParkingPathPlanningFillVhParam(const ApaParkingPathPlanningParam_t *param,
                                       GeoVhPathPlanningParam_t *vhParam) {
  *vhParam = (GeoVhPathPlanningParam_t){0};
  vhParam->SltState = param->SltState;
  vhParam->VehW = param->VehW;
  vhParam->VehLlF = param->VehLlF;
  vhParam->VehLr = param->VehLr;
  vhParam->NeedHybridAlgFusion = param->NeedHybridAlgFusion;
  vhParam->SceneQuery = param->SceneQuery;
  vhParam->Diagnostics = param->Diagnostics;
  vhParam->HybridPolicy = param->HybridPolicy;
}

void apaParkingPathPlanningFillVtDiscretInput(const ApaParkingPathPlanningInput_t *input,
                                              const GeoVtPathPlanningOutput_t *plannerOutput,
                                              GeoPthDiscretVtInput_t *discretInput) {
  Integer idx;
  Boolean segmentCountValid0;

  *discretInput = (GeoPthDiscretVtInput_t){0};
  discretInput->VehPosX = input->VehicleX;
  discretInput->VehPosY = input->VehicleY;
  discretInput->VehPosTheta = input->VehicleYaw;
  discretInput->TarPtX = input->TarX;
  discretInput->TarPtY = input->TarY;
  discretInput->TarPtTheta = input->TarTheta;
  discretInput->PathLen = plannerOutput->PathLen;
  segmentCountValid0 = (plannerOutput->SegNum > 0);
  if (segmentCountValid0) {
    segmentCountValid0 = (plannerOutput->SegNum <= VT_SEG_OUT_MAX);
  }
  if (segmentCountValid0) {
    discretInput->SegNum = plannerOutput->SegNum;
  }
  discretInput->ShortCutSw = input->ShortCutSw;
  discretInput->ShortCutThd = input->ShortCutThd;

  idx = 0;
  while (idx < VT_SEG_OUT_MAX) {
    discretInput->SegGoDir[idx] = plannerOutput->SegGoDir[idx];
    discretInput->SegTrnDir[idx] = plannerOutput->SegTrnDir[idx];
    discretInput->SegCrv[idx] = plannerOutput->SegCrv[idx];
    discretInput->SegTht[idx] = plannerOutput->SegTht[idx];
    discretInput->SegLen[idx] = plannerOutput->SegLen[idx];
    discretInput->SegStX[idx] = plannerOutput->SegStX[idx];
    discretInput->SegStY[idx] = plannerOutput->SegStY[idx];
    discretInput->SegStTh[idx] = plannerOutput->SegStTh[idx];
    discretInput->SegCcX[idx] = plannerOutput->SegCcX[idx];
    discretInput->SegCcY[idx] = plannerOutput->SegCcY[idx];
    idx = idx + 1;
  }
}

void apaParkingPathPlanningFillHtDiscretInput(const ApaParkingPathPlanningInput_t *input,
                                              const GeoHtPathPlanningOutput_t *plannerOutput,
                                              GeoPthDiscretHtInput_t *discretInput) {
  Integer idx;
  Boolean segmentCountValid0;

  *discretInput = (GeoPthDiscretHtInput_t){0};
  discretInput->VehPosX = input->VehicleX;
  discretInput->VehPosY = input->VehicleY;
  discretInput->VehPosTheta = input->VehicleYaw;
  discretInput->TarPtX = input->TarX;
  discretInput->TarPtY = input->TarY;
  discretInput->TarPtTheta = input->TarTheta;
  discretInput->PathLen = plannerOutput->PathLen;
  segmentCountValid0 = (plannerOutput->SegNum > 0);
  if (segmentCountValid0) {
    segmentCountValid0 = (plannerOutput->SegNum <= HT_SEG_OUT_MAX);
  }
  if (segmentCountValid0) {
    discretInput->SegNum = plannerOutput->SegNum;

    const Integer last = plannerOutput->SegNum - 1;
    const Real yaw = plannerOutput->SegStTh[last];
    const Real travel = (Real)plannerOutput->SegGoDir[last] * plannerOutput->SegLen[last];
    const Real curvature = plannerOutput->SegCrv[last];
    const Real endYaw = yaw + curvature * travel;
    discretInput->TarPtX = plannerOutput->SegStX[last];
    discretInput->TarPtY = plannerOutput->SegStY[last];
    if (fabsf(curvature) > 1.0e-6f) {
      discretInput->TarPtX += (sinf(endYaw) - sinf(yaw)) / curvature;
      discretInput->TarPtY -= (cosf(endYaw) - cosf(yaw)) / curvature;
    } else {
      discretInput->TarPtX += travel * cosf(yaw);
      discretInput->TarPtY += travel * sinf(yaw);
    }
    discretInput->TarPtTheta = apaCoordWrapToPi(endYaw);
  }
  discretInput->ShortCutSw = input->ShortCutSw;
  discretInput->ShortCutThd = input->ShortCutThd;

  idx = 0;
  while (idx < HT_SEG_OUT_MAX) {
    discretInput->SegGoDir[idx] = plannerOutput->SegGoDir[idx];
    discretInput->SegTrnDir[idx] = plannerOutput->SegTrnDir[idx];
    discretInput->SegCrv[idx] = plannerOutput->SegCrv[idx];
    discretInput->SegTht[idx] = plannerOutput->SegTht[idx];
    discretInput->SegLen[idx] = plannerOutput->SegLen[idx];
    discretInput->SegStX[idx] = plannerOutput->SegStX[idx];
    discretInput->SegStY[idx] = plannerOutput->SegStY[idx];
    discretInput->SegStTh[idx] = plannerOutput->SegStTh[idx];
    discretInput->SegCcX[idx] = plannerOutput->SegCcX[idx];
    discretInput->SegCcY[idx] = plannerOutput->SegCcY[idx];
    idx = idx + 1;
  }
}

void apaParkingPathPlanningFillDthDiscretInput(const ApaParkingPathPlanningInput_t *input,
                                               const GeoDthEdVtPathPlanningOutput_t *plannerOutput,
                                               GeoPthDiscretVtInput_t *discretInput) {
  Integer idx;
  Boolean segmentCountValid0;

  *discretInput = (GeoPthDiscretVtInput_t){0};
  discretInput->VehPosX = input->VehicleX;
  discretInput->VehPosY = input->VehicleY;
  discretInput->VehPosTheta = input->VehicleYaw;
  discretInput->TarPtX = input->TarX;
  discretInput->TarPtY = input->TarY;
  discretInput->TarPtTheta = input->TarTheta;
  discretInput->PathLen = plannerOutput->PathLen;
  segmentCountValid0 = (plannerOutput->SegNum > 0);
  if (segmentCountValid0) {
    segmentCountValid0 = (plannerOutput->SegNum <= GEO_DTH_ED_VT_SEG_OUT_MAX);
  }
  if (segmentCountValid0) {
    discretInput->SegNum = plannerOutput->SegNum;
  }
  discretInput->ShortCutSw = input->ShortCutSw;
  discretInput->ShortCutThd = input->ShortCutThd;

  idx = 0;
  while (idx < GEO_DTH_ED_VT_SEG_OUT_MAX) {
    discretInput->SegGoDir[idx] = plannerOutput->SegGoDir[idx];
    discretInput->SegTrnDir[idx] = plannerOutput->SegTrnDir[idx];
    discretInput->SegCrv[idx] = plannerOutput->SegCrv[idx];
    discretInput->SegTht[idx] = plannerOutput->SegTht[idx];
    discretInput->SegLen[idx] = plannerOutput->SegLen[idx];
    discretInput->SegStX[idx] = plannerOutput->SegStX[idx];
    discretInput->SegStY[idx] = plannerOutput->SegStY[idx];
    discretInput->SegStTh[idx] = plannerOutput->SegStTh[idx];
    discretInput->SegCcX[idx] = plannerOutput->SegCcX[idx];
    discretInput->SegCcY[idx] = plannerOutput->SegCcY[idx];
    idx = idx + 1;
  }
}

void apaParkingPathPlanningFillVhDiscretInput(const ApaParkingPathPlanningInput_t *input,
                                              const GeoVhPathPlanningOutput_t *plannerOutput,
                                              GeoPthDiscretVhInput_t *discretInput) {
  Integer idx;
  Boolean segmentCountValid0;

  *discretInput = (GeoPthDiscretVhInput_t){0};
  discretInput->VehPosX = input->VehicleX;
  discretInput->VehPosY = input->VehicleY;
  discretInput->VehPosTheta = input->VehicleYaw;
  discretInput->TarPtX = input->TarX;
  discretInput->TarPtY = input->TarY;
  discretInput->TarPtTheta = input->TarTheta;
  discretInput->PathLen = plannerOutput->PathLen;
  segmentCountValid0 = (plannerOutput->SegNum > 0);
  if (segmentCountValid0) {
    segmentCountValid0 = (plannerOutput->SegNum <= VH_SEG_OUT_MAX);
  }
  if (segmentCountValid0) {
    discretInput->SegNum = plannerOutput->SegNum;
  }

  idx = 0;
  while (idx < VH_SEG_OUT_MAX) {
    discretInput->SegGoDir[idx] = plannerOutput->SegGoDir[idx];
    discretInput->SegTrnDir[idx] = plannerOutput->SegTrnDir[idx];
    discretInput->SegCrv[idx] = plannerOutput->SegCrv[idx];
    discretInput->SegTht[idx] = plannerOutput->SegTht[idx];
    discretInput->SegLen[idx] = plannerOutput->SegLen[idx];
    discretInput->SegStX[idx] = plannerOutput->SegStX[idx];
    discretInput->SegStY[idx] = plannerOutput->SegStY[idx];
    discretInput->SegStTh[idx] = plannerOutput->SegStTh[idx];
    discretInput->SegCcX[idx] = plannerOutput->SegCcX[idx];
    discretInput->SegCcY[idx] = plannerOutput->SegCcY[idx];
    idx = idx + 1;
  }
}

Real apaParkingPathPlanningCalcParkoutPathLen(const ApaPath_t *path) {
  Integer idx;
  Real pathLen0;

  pathLen0 = 0.0f;
  idx = 0;
  while ((idx + 1 < path->PathN) && (idx < APA_PATH_MAX_POINTS)) {
    pathLen0 = pathLen0 + fabsf(path->OdoStep[idx]);
    idx = idx + 1;
  }
  return pathLen0;
}

Boolean apaParkingPathPlanningShouldRetryParkoutSearch(const ApaComplexPath_t *parkoutPath) {
  Boolean shouldRetry0;
  Real pathLen0;

  shouldRetry0 = false;
  if (parkoutPath->Path.ValidFlag && (parkoutPath->Path.PathSource == APA_PATH_SOURCE_GEO)) {
    pathLen0 = apaParkingPathPlanningCalcParkoutPathLen(&parkoutPath->Path);
    if (pathLen0 <= APA_PARKING_PLANNER_PARKOUT_RUNTIME_MIN_PATH_LEN) {
      shouldRetry0 = true;
    }
  }
  return shouldRetry0;
}

Boolean
apaParkingPathPlanningShouldRetryParkoutSearchAfterFailure(const ApaParallelParkoutDiag_t *diag) {
  Boolean shouldRetry0;

  shouldRetry0 = false;
  if (diag->PkOutFailRs == APA_PARKING_PLANNER_PARKOUT_FAIL_RS_RETRY_FALLBACK) {
    shouldRetry0 = true;
  }
  return shouldRetry0;
}

void apaParkingPathPlanningCopyVtPath(const GeoPthDiscretVtOutput_t *discretOutput,
                                      ApaParkingPathPlanningOutput_t *output) {
  Integer idx;

  output->PtCount = discretOutput->PtCount;
  output->StepCount = discretOutput->StepCount;
  output->St2Idx = discretOutput->St2Idx;
  output->Stp1PtRX = discretOutput->Stp1PtRX;
  output->Stp1PtRY = discretOutput->Stp1PtRY;
  output->Stp1PtRTheta = discretOutput->Stp1PtRTheta;

  idx = 0;
  while (idx < APA_PARKING_PATH_PT_MAX) {
    output->PthPtX[idx] = discretOutput->PthPtX[idx];
    output->PthPtY[idx] = discretOutput->PthPtY[idx];
    output->PthPtTht[idx] = discretOutput->PthPtTht[idx];
    output->PthPtOdo[idx] = discretOutput->PthPtOdo[idx];
    idx = idx + 1;
  }
}

void apaParkingPathPlanningSwapInputStartTargetPose(ApaParkingPathPlanningInput_t *input) {
  Real vehicleX0;
  Real vehicleY0;
  Real vehicleYaw0;

  vehicleX0 = input->VehicleX;
  vehicleY0 = input->VehicleY;
  vehicleYaw0 = input->VehicleYaw;
  input->VehicleX = input->TarX;
  input->VehicleY = input->TarY;
  input->VehicleYaw = input->TarTheta;
  input->TarX = vehicleX0;
  input->TarY = vehicleY0;
  input->TarTheta = vehicleYaw0;
}

Boolean apaParkingPathPlanningShouldTryReverseVtPath(const ApaParkingPathPlanningInput_t *input) {
  const Real x[4] = {input->SltPt1X, input->SltPt2X, input->SltPt3X, input->SltPt4X};
  const Real y[4] = {input->SltPt1Y, input->SltPt2Y, input->SltPt3Y, input->SltPt4Y};
  const Real margin = 0.6f;
  Real centerX = 0.0f, centerY = 0.0f;
  Integer i;
  if (input->TargetSlotKind != APA_SCENARIO_SLOT_KIND_VT || input->SltEntr < 1 ||
      input->SltEntr > 4)
    return false;
  for (i = 0; i < 4; ++i) {
    centerX += x[i] * 0.25f;
    centerY += y[i] * 0.25f;
  }

  for (i = 0; i < 4; ++i) {
    const Integer next = (i + 1) % 4;
    const Real dx = x[next] - x[i], dy = y[next] - y[i];
    const Real length = hypotf(dx, dy);
    const Real centerSide = dx * (centerY - y[i]) - dy * (centerX - x[i]);
    Real inward, vehicleDistance, targetDistance;
    if (!isfinite(length) || length < 0.01f || !isfinite(centerSide) || fabsf(centerSide) < 1.0e-6f)
      return false;
    inward = (centerSide > 0.0f ? 1.0f : -1.0f) / length;
    vehicleDistance = inward * (dx * (input->VehicleY - y[i]) - dy * (input->VehicleX - x[i]));
    if (!isfinite(vehicleDistance) || vehicleDistance < -margin)
      return false;
    if (i == input->SltEntr - 1) {
      targetDistance = inward * (dx * (input->TarY - y[i]) - dy * (input->TarX - x[i]));
      if (!isfinite(targetDistance) || targetDistance >= -margin)
        return false;
    }
  }
  return true;
}

void apaParkingPathPlanningRefreshOutputOdo(ApaParkingPathPlanningOutput_t *output) {
  Integer ptCount0;
  Integer idx;
  Real pathLen0;

  ptCount0 = apaClampCount(output->PtCount, APA_PARKING_PATH_PT_MAX);
  pathLen0 = 0.0f;
  idx = 0;
  while (idx < ptCount0) {
    Boolean hasNext0;

    hasNext0 = (idx < (ptCount0 - 1));
    if (hasNext0) {
      Integer nextIdx0;
      Real currX0;
      Real currY0;
      Real currTheta0;
      Real nextX0;
      Real nextY0;
      Real dx0;
      Real dy0;
      Real distSq0;
      Real dist0;
      Real cosTheta0;
      Real sinTheta0;
      Real dot0;
      Real odo0;

      nextIdx0 = idx + 1;
      currX0 = output->PthPtX[idx];
      currY0 = output->PthPtY[idx];
      currTheta0 = output->PthPtTht[idx];
      nextX0 = output->PthPtX[nextIdx0];
      nextY0 = output->PthPtY[nextIdx0];
      dx0 = nextX0 - currX0;
      dy0 = nextY0 - currY0;
      distSq0 = (dx0 * dx0) + (dy0 * dy0);
      dist0 = sqrtf(distSq0);
      cosTheta0 = cosf(currTheta0);
      sinTheta0 = sinf(currTheta0);
      dot0 = (cosTheta0 * dx0) + (sinTheta0 * dy0);
      if (dot0 < 0.0f) {
        odo0 = 0.0f - dist0;
      } else {
        odo0 = dist0;
      }
      output->PthPtOdo[idx] = odo0;
      pathLen0 = pathLen0 + fabsf(odo0);
    } else {
      output->PthPtOdo[idx] = 0.0f;
    }
    idx = idx + 1;
  }

  output->PathLen = pathLen0;
}

void apaParkingPathPlanningReverseOutputPath(ApaParkingPathPlanningOutput_t *output) {
  Integer ptCount0;
  Integer leftIdx0;
  Integer rightIdx0;
  Boolean swapLoop0;

  ptCount0 = apaClampCount(output->PtCount, APA_PARKING_PATH_PT_MAX);
  leftIdx0 = 0;
  rightIdx0 = ptCount0 - 1;
  swapLoop0 = (leftIdx0 < rightIdx0);
  while (swapLoop0) {
    Real tempX0;
    Real tempY0;
    Real tempTheta0;

    tempX0 = output->PthPtX[leftIdx0];
    tempY0 = output->PthPtY[leftIdx0];
    tempTheta0 = output->PthPtTht[leftIdx0];
    output->PthPtX[leftIdx0] = output->PthPtX[rightIdx0];
    output->PthPtY[leftIdx0] = output->PthPtY[rightIdx0];
    output->PthPtTht[leftIdx0] = output->PthPtTht[rightIdx0];
    output->PthPtX[rightIdx0] = tempX0;
    output->PthPtY[rightIdx0] = tempY0;
    output->PthPtTht[rightIdx0] = tempTheta0;
    leftIdx0 = leftIdx0 + 1;
    rightIdx0 = rightIdx0 - 1;
    swapLoop0 = (leftIdx0 < rightIdx0);
  }

  apaParkingPathPlanningRefreshOutputOdo(output);
  output->St2Idx = 0;
  output->Stp1PtRX = 0.0f;
  output->Stp1PtRY = 0.0f;
  output->Stp1PtRTheta = 0.0f;
}

void apaParkingPathPlanningRefreshVtStp1Metadata(const ApaParkingPathPlanningInput_t *input,
                                                 ApaParkingPathPlanningOutput_t *output) {
  Integer pointCount0;
  Integer pointIndex0;
  Real stp1PtX0;
  Real stp1PtY0;
  Real stp1PtTheta0;
  Real deltaX0;
  Real deltaY0;
  Real cosVehicle0;
  Real sinVehicle0;
  Boolean foundForwardPoint0;

  pointCount0 = apaClampCount(output->PtCount, APA_PARKING_PATH_PT_MAX);
  pointIndex0 = 0;
  stp1PtX0 = 0.0f;
  stp1PtY0 = 0.0f;
  stp1PtTheta0 = 0.0f;
  foundForwardPoint0 = false;
  while ((pointIndex0 < pointCount0) && (!foundForwardPoint0)) {
    if (output->PthPtOdo[pointIndex0] > 0.01f) {
      stp1PtX0 = output->PthPtX[pointIndex0];
      stp1PtY0 = output->PthPtY[pointIndex0];
      stp1PtTheta0 = output->PthPtTht[pointIndex0];
      foundForwardPoint0 = true;
    } else {
      pointIndex0 = pointIndex0 + 1;
    }
  }

  deltaX0 = stp1PtX0 - input->VehicleX;
  deltaY0 = stp1PtY0 - input->VehicleY;
  cosVehicle0 = cosf(input->VehicleYaw);
  sinVehicle0 = sinf(input->VehicleYaw);
  output->Stp1PtRX = (cosVehicle0 * deltaX0) + (sinVehicle0 * deltaY0);
  output->Stp1PtRY = (cosVehicle0 * deltaY0) - (sinVehicle0 * deltaX0);
  output->Stp1PtRTheta = apaCoordWrapToPi(stp1PtTheta0 - input->VehicleYaw);
}

void apaParkingPathPlanningCopyHtPath(const GeoPthDiscretHtOutput_t *discretOutput,
                                      ApaParkingPathPlanningOutput_t *output) {
  Integer idx;

  output->PtCount = discretOutput->PtCount;
  output->StepCount = discretOutput->StepCount;
  output->St2Idx = discretOutput->St2Idx;
  output->Stp1PtRX = 0.0f;
  output->Stp1PtRY = 0.0f;
  output->Stp1PtRTheta = 0.0f;

  idx = 0;
  while (idx < APA_PARKING_PATH_PT_MAX) {
    output->PthPtX[idx] = discretOutput->PthPtX[idx];
    output->PthPtY[idx] = discretOutput->PthPtY[idx];
    output->PthPtTht[idx] = discretOutput->PthPtTht[idx];
    output->PthPtOdo[idx] = discretOutput->PthPtOdo[idx];
    idx = idx + 1;
  }

  if (output->PtCount >= APA_PARKING_PATH_PT_MAX) {
    output->PlanFlg = 0;
    output->ErrorType = APA_FAULT_PATH_N_EXCEED_TO_ADD;
  } else if (output->PtCount > 1) {
    ApaPath_t path = {0};
    path.PathN = output->PtCount;
    for (idx = 0; idx < path.PathN; ++idx) {
      path.OdoStep[idx] = output->PthPtOdo[idx];
    }
    if (apaPathCalcSegNum(&path) > 26) {
      output->PlanFlg = 0;
      output->ErrorType = APA_FAULT_SEG_NUM_OVERMANY;
    }
  }
}

void apaParkingPathPlanningCopyVhPath(const GeoPthDiscretVhOutput_t *discretOutput,
                                      ApaParkingPathPlanningOutput_t *output) {
  Integer idx;

  output->PtCount = discretOutput->PtCount;
  output->StepCount = discretOutput->StepCount;
  output->St2Idx = 0;
  output->Stp1PtRX = 0.0f;
  output->Stp1PtRY = 0.0f;
  output->Stp1PtRTheta = 0.0f;

  idx = 0;
  while (idx < APA_PARKING_PATH_PT_MAX) {
    output->PthPtX[idx] = discretOutput->PthPtX[idx];
    output->PthPtY[idx] = discretOutput->PthPtY[idx];
    output->PthPtTht[idx] = discretOutput->PthPtTht[idx];
    output->PthPtOdo[idx] = discretOutput->PthPtOdo[idx];
    idx = idx + 1;
  }
}

void apaParkingPathPlanningCopyParkoutPath(const ApaComplexPath_t *parkoutPath,
                                           ApaParkingPathPlanningOutput_t *output) {
  Integer idx;
  Integer pointCount0;
  Real pathLen0;
  Real currOdo0;

  pointCount0 = parkoutPath->Path.PathN;
  if (pointCount0 > APA_PARKING_PATH_PT_MAX) {
    pointCount0 = APA_PARKING_PATH_PT_MAX;
  }
  output->PtCount = pointCount0;
  output->StepCount = 0;
  output->St2Idx = 0;
  output->Stp1PtRX = 0.0f;
  output->Stp1PtRY = 0.0f;
  output->Stp1PtRTheta = 0.0f;
  output->SegNum = parkoutPath->Path.SegNum;
  output->StpN = 0;
  if (parkoutPath->Path.SegNum > 0) {
    output->StpN = parkoutPath->Path.SegNum - 1;
  }

  pathLen0 = 0.0f;

  idx = 0;
  while (idx < pointCount0) {
    currOdo0 = parkoutPath->Path.OdoStep[idx];
    output->PthPtX[idx] = parkoutPath->Path.X[idx];
    output->PthPtY[idx] = parkoutPath->Path.Y[idx];
    output->PthPtTht[idx] = parkoutPath->Path.Tht[idx];
    output->PthPtOdo[idx] = currOdo0;
    pathLen0 = pathLen0 + fabsf(currOdo0);
    idx = idx + 1;
  }
  output->PathLen = pathLen0;
}

static void apaParkingPathPlanningAddVccBoundaryLine(
    VehicleCollisionCheckInput_t *vccInput, Real x1, Real y1, Real x2, Real y2) {
  Integer idx;

  idx = vccInput->BoundaryLineNum;
  if (idx < VCC_MAX_BOUNDARY_LINES) {
    vccInput->BoundaryLineX1[idx] = x1;
    vccInput->BoundaryLineY1[idx] = y1;
    vccInput->BoundaryLineX2[idx] = x2;
    vccInput->BoundaryLineY2[idx] = y2;
    vccInput->BoundaryLineNum = idx + 1;
  }
}

void apaParkingPathPlanningBuildPathVccInput(const ApaParkingPathPlanningInput_t *input,
                                             const ApaParkingPathPlanningParam_t *param,
                                             VehicleCollisionCheckInput_t *vccInput) {
  Integer idx;
  Integer lineCount;
  Integer pointCount;

  *vccInput = (VehicleCollisionCheckInput_t){0};
  vccInput->CheckType = VCC_CHECK_TYPE_PLANNING;

  vccInput->CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  vccInput->CollisionDemand = VCC_COLLISION_DEMAND_SIMPLIFY;
  vccInput->VehLF = param->VehLlF;
  vccInput->VehLB = param->VehLr;
  vccInput->VehW = param->VehW;
  vccInput->VehLL = param->VehLlF + param->VehLr;
  vccInput->VehWB = APA_PARKING_PLANNER_DEFAULT_WHEELBASE;
  vccInput->SlotClass = input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_HT
                            ? VCC_SLOT_CLASS_PARALLEL
                            : input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_DTH_ED_VT
                                  ? VCC_SLOT_CLASS_DIAGONAL
                                  : VCC_SLOT_CLASS_VERTICAL;
  vccInput->ParkingOutFlag = input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKOUT;
  vccInput->TarPosValid = true;
  vccInput->TarPosX = input->TarX;
  vccInput->TarPosY = input->TarY;
  vccInput->TarPosTheta = input->TarTheta;
  vehicleCollisionCheckLoadDefaultMargins(vccInput);
  vccInput->SlotPtsValid = true;
  vccInput->SlotPts[0] = input->SltPt1X;
  vccInput->SlotPts[1] = input->SltPt1Y;
  vccInput->SlotPts[2] = input->SltPt2X;
  vccInput->SlotPts[3] = input->SltPt2Y;
  vccInput->SlotPts[4] = input->SltPt3X;
  vccInput->SlotPts[5] = input->SltPt3Y;
  vccInput->SlotPts[6] = input->SltPt4X;
  vccInput->SlotPts[7] = input->SltPt4Y;
  apaSlotOrderFromEntrance(input->SltEntr,
                           &vccInput->SlotPts[0],
                           &vccInput->SlotPts[1],
                           &vccInput->SlotPts[2],
                           &vccInput->SlotPts[3],
                           &vccInput->SlotPts[4],
                           &vccInput->SlotPts[5],
                           &vccInput->SlotPts[6],
                           &vccInput->SlotPts[7]);

  if (input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_PARKOUT) {
    const Real *points = vccInput->SlotPts;
    const Real entrance = hypotf(points[2] - points[0], points[3] - points[1]);
    const Real depth = hypotf(points[4] - points[2], points[5] - points[3]);
    vccInput->SlotClass = entrance > depth ? VCC_SLOT_CLASS_PARALLEL : VCC_SLOT_CLASS_VERTICAL;
  }

  lineCount = apaClampCount(input->BoundaryLineNum, VCC_MAX_BOUNDARY_LINES);
  idx = 0;
  while (idx < lineCount) {

    apaParkingPathPlanningAddVccBoundaryLine(vccInput,
                                             input->BoundaryLineX1[idx],
                                             input->BoundaryLineY1[idx],
                                             input->BoundaryLineX2[idx],
                                             input->BoundaryLineY2[idx]);
    idx = idx + 1;
  }

  lineCount = apaClampCount(input->VirtualBoundaryLineNum, VCC_MAX_VIRTUAL_LINES);
  vccInput->VirtualBoundaryLineNum = lineCount;
  idx = 0;
  while (idx < lineCount) {
    vccInput->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    vccInput->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    vccInput->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    vccInput->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }

  lineCount = apaClampCount(input->SlotTrackStopperNum, VCC_MAX_TRACK_STOPPER);
  vccInput->SlotTrackStopperNum = lineCount;
  idx = 0;
  while (idx < lineCount) {
    vccInput->SlotTrackStopperX1[idx] = input->SlotTrackStopperX1[idx];
    vccInput->SlotTrackStopperY1[idx] = input->SlotTrackStopperY1[idx];
    vccInput->SlotTrackStopperX2[idx] = input->SlotTrackStopperX2[idx];
    vccInput->SlotTrackStopperY2[idx] = input->SlotTrackStopperY2[idx];
    idx = idx + 1;
  }

  pointCount = apaClampCount(input->BoundarySepPtNum, VCC_MAX_SEPARATE_PTS);
  vccInput->BoundarySepPtsNum = pointCount;
  idx = 0;
  while (idx < pointCount) {
    vccInput->BoundarySepPtsX[idx] = input->BoundarySepPtX[idx];
    vccInput->BoundarySepPtsY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  pointCount = apaClampCount(input->StopperPtNum, VCC_MAX_SEPARATE_PTS);
  vccInput->SeparateStopperNum = pointCount;
  idx = 0;
  while (idx < pointCount) {
    vccInput->SeparateStopperX[idx] = input->StopperPtX[idx];
    vccInput->SeparateStopperY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }
}

static Boolean
apaParkingPathPlanningPoseIsCollisionFree(const VehicleCollisionCheckInput_t *vccTemplate,
                                          const ApaPlanningSceneQuery_t *scene,
                                          Real poseX,
                                          Real poseY,
                                          Real poseTheta) {
  VehicleCollisionCheckInput_t vccInput;
  VehicleCollisionCheckOutput_t vccOutput;

  vccInput = *vccTemplate;
  vccInput.VehPosX = poseX;
  vccInput.VehPosY = poseY;
  vccInput.VehPosTheta = poseTheta;
  vccOutput = (VehicleCollisionCheckOutput_t){0};
  apaPlanningSceneCollisionCheck(&vccInput, scene, &vccOutput);
  return !vccOutput.IsCollision;
}

void apaParkingPathPlanningBuildTargetSlotConstraint(const ApaParkingPathPlanningInput_t *input,
                                                     const ApaParkingPathPlanningParam_t *param,
                                                     ApaTargetSlotConstraint_t *constraint) {
  *constraint = (ApaTargetSlotConstraint_t){0};

  constraint->Enabled = true;
  apaSlotSetPoints(constraint->SlotPts,
                   input->SltPt1X,
                   input->SltPt1Y,
                   input->SltPt2X,
                   input->SltPt2Y,
                   input->SltPt3X,
                   input->SltPt3Y,
                   input->SltPt4X,
                   input->SltPt4Y);
  constraint->EnterEdge = input->SltEntr;
  constraint->VehFront = param->VehLlF;
  constraint->VehRear = param->VehLr;
  constraint->VehWidth = param->VehW;
  constraint->Margin = 0.0f;
}

static Boolean
apaParkingPathPlanningPoseTouchesTargetSlotFrame(const ApaParkingPathPlanningInput_t *input,
                                                 const ApaParkingPathPlanningParam_t *param,
                                                 Real poseX,
                                                 Real poseY,
                                                 Real poseTheta) {
  ApaTargetSlotConstraint_t constraint;
  Integer collisionEdge;
  Boolean clear0;
  apaParkingPathPlanningBuildTargetSlotConstraint(input, param, &constraint);
  clear0 = apaTargetSlotConstraintPoseIsClear(&constraint, poseX, poseY, poseTheta, &collisionEdge);
  return !clear0;
}

Boolean apaParkingPathPlanningOutputPathIsSafe(const ApaParkingPathPlanningInput_t *input,
                                               const ApaParkingPathPlanningParam_t *param,
                                               const ApaParkingPathPlanningOutput_t *output,
                                               Boolean checkEndPose) {
  VehicleCollisionCheckInput_t vccTemplate;
  Integer ptLimit;
  Integer idx;
  Boolean pathSafe0;

  pathSafe0 = true;
  if (output->PlanFlg != 0) {
    ptLimit = apaClampCount(output->PtCount, APA_PARKING_PATH_PT_MAX);
    pathSafe0 = (ptLimit > 1);
    if (pathSafe0) {
      if (checkEndPose) {
        Real endX;
        Real endY;
        Real endTheta;
        Real endDx;
        Real endDy;
        Real endDist;
        Real endThetaErr;
        Boolean endDistInvalid0;
        Boolean endThetaInvalid0;

        endX = output->PthPtX[ptLimit - 1];
        endY = output->PthPtY[ptLimit - 1];
        endTheta = output->PthPtTht[ptLimit - 1];
        endDx = endX - input->TarX;
        endDy = endY - input->TarY;
        endDist = sqrtf((endDx * endDx) + (endDy * endDy));
        endThetaErr = apaCoordWrapToHalfOpenPi(endTheta - input->TarTheta);
        endDistInvalid0 = !isfinite(endDist) ||
                          !apaGoalToleranceValid(input->GoalPositionTolerance) ||
                          endDist > apaGoalPositionTolerance(input->GoalPositionTolerance);
        endThetaInvalid0 = !isfinite(endThetaErr) ||
                           !apaGoalToleranceValid(input->GoalYawTolerance) ||
                           fabsf(endThetaErr) > apaGoalYawTolerance(input->GoalYawTolerance);
        if (endDistInvalid0) {
          apaPlanningRecordFailure(
              param->Diagnostics, APA_FAULT_PATH_NOT_MATCH_TARPOS, "candidate_goal_position", 4);
          pathSafe0 = false;
        }
        if (endThetaInvalid0) {
          apaPlanningRecordFailure(
              param->Diagnostics, APA_FAULT_PATH_NOT_MATCH_TARPOS, "candidate_goal_heading", 4);
          pathSafe0 = false;
        }
      }
    }
    if (pathSafe0) {
      Boolean pathLoop0;

      apaParkingPathPlanningBuildPathVccInput(input, param, &vccTemplate);
      idx = 0;
      pathLoop0 = (idx < ptLimit);
      while (pathLoop0) {
        if (apaParkingPathPlanningPoseTouchesTargetSlotFrame(
                input, param, output->PthPtX[idx], output->PthPtY[idx], output->PthPtTht[idx])) {
          pathSafe0 = false;
        }
        if (pathSafe0) {
          pathSafe0 = apaParkingPathPlanningPoseIsCollisionFree(&vccTemplate,
                                                                &param->SceneQuery,
                                                                output->PthPtX[idx],
                                                                output->PthPtY[idx],
                                                                output->PthPtTht[idx]);
        }
        if (pathSafe0) {
          if (idx > 0) {
            Real prevX;
            Real prevY;
            Real prevTheta;
            Real currX;
            Real currY;
            Real currTheta;
            Real deltaX;
            Real deltaY;
            Real distance;
            Real cornerRadius;
            Real cornerTravel;
            Integer sampleCount;
            Integer sampleIdx;
            Boolean sampleLoop0;

            prevX = output->PthPtX[idx - 1];
            prevY = output->PthPtY[idx - 1];
            prevTheta = output->PthPtTht[idx - 1];
            currX = output->PthPtX[idx];
            currY = output->PthPtY[idx];
            currTheta = output->PthPtTht[idx];
            deltaX = currX - prevX;
            deltaY = currY - prevY;
            distance = sqrtf((deltaX * deltaX) + (deltaY * deltaY));

            cornerRadius =
                hypotf(fmaxf(vccTemplate.VehLF, vccTemplate.VehLB), vccTemplate.VehW * 0.5f);
            cornerTravel = distance + cornerRadius * fabsf(apaCoordWrapToPi(currTheta - prevTheta));
            sampleCount = (Integer)ceilf(cornerTravel / APA_PARKING_PLANNER_COLLISION_SAMPLE_STEP);
            sampleIdx = 1;
            sampleLoop0 = (sampleIdx < sampleCount);
            while (sampleLoop0) {
              Real ratio;
              Real sampleX;
              Real sampleY;
              Real sampleTheta;

              ratio = ((Real)sampleIdx) / ((Real)sampleCount);
              sampleX = prevX + ((currX - prevX) * ratio);
              sampleY = prevY + ((currY - prevY) * ratio);
              sampleTheta =
                  apaParkingPathPlanningInterpolateThetaShortest(prevTheta, currTheta, ratio);
              if (apaParkingPathPlanningPoseTouchesTargetSlotFrame(
                      input, param, sampleX, sampleY, sampleTheta)) {
                pathSafe0 = false;
              }
              if (pathSafe0) {
                pathSafe0 = apaParkingPathPlanningPoseIsCollisionFree(
                    &vccTemplate, &param->SceneQuery, sampleX, sampleY, sampleTheta);
              }
              sampleIdx = sampleIdx + 1;
              sampleLoop0 = (sampleIdx < sampleCount);
              if (!pathSafe0) {
                sampleLoop0 = false;
              }
            }
          }
        }
        idx = idx + 1;
        pathLoop0 = (idx < ptLimit);
        if (!pathSafe0) {
          pathLoop0 = false;
        }
      }
    }
  }

  if (!pathSafe0)
    apaPlanningRecordFailure(
        param->Diagnostics, APA_FAULT_COLLISION_RISK_AT_NEW_OBS, "candidate_path_collision", 3);
  return pathSafe0;
}

void apaParkingPathPlanningRejectUnsafeOutput(ApaParkingPathPlanningOutput_t *output) {
  Integer selectedSlotKind;
  Integer plannerCase;

  selectedSlotKind = output->SelectedSlotKind;
  plannerCase = output->PlannerCase;
  apaParkingPathPlanningZeroOutput(output);
  output->SelectedSlotKind = selectedSlotKind;
  output->PlannerCase = plannerCase;
  output->PlanFlg = 0;
  output->ErrorType = APA_FAULT_UNKNOWN_ERR;
}

Boolean apaParkingPathPlanningBuildHybridCandidate(const ApaPlannerHybridSegments_t *segments,
                                                   const ApaParkingHybridTrial_t *trial,
                                                   ApaParkingPathPlanningOutput_t *candidate) {
  const Integer maximum = trial->Input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_HT
                              ? HT_SEG_OUT_MAX
                              : trial->Input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_DTH_ED_VT
                                    ? GEO_DTH_ED_VT_SEG_OUT_MAX
                                    : VT_SEG_OUT_MAX;
  if (segments->SegNum <= 0 || segments->SegNum > maximum)
    return false;

  ApaParkingPathPlanningInput_t resolvedInput = *trial->Input;
  ApaParkingHybridTrial_t resolvedTrial = *trial;
  const Integer last = segments->SegNum - 1;
  Real endX, endY, endYaw;
  apaPlannerHybridEvalPrimitivePose(segments->PathGoDir[last],
                                    segments->PathCrv[last],
                                    segments->PathLenArr[last],
                                    segments->PathStx[last],
                                    segments->PathSty[last],
                                    segments->PathStTh[last],
                                    &endX,
                                    &endY,
                                    &endYaw);
  const Real error = hypotf(endX - resolvedInput.TarX, endY - resolvedInput.TarY);
  const Real yawError = fabsf(apaCoordWrapToPi(endYaw - resolvedInput.TarTheta));
  if (!isfinite(error) || !isfinite(yawError) ||
      error > apaGoalPositionTolerance(resolvedInput.GoalPositionTolerance) ||
      yawError > apaGoalYawTolerance(resolvedInput.GoalYawTolerance))
    return false;
  resolvedInput.TarX = endX;
  resolvedInput.TarY = endY;
  resolvedInput.TarTheta = endYaw;
  resolvedTrial.Input = &resolvedInput;
  trial = &resolvedTrial;
  apaParkingPathPlanningZeroOutput(candidate);
  candidate->SelectedSlotKind = trial->Input->TargetSlotKind;
  candidate->PlanFlg = 1;
  candidate->PlannerCase = segments->PlannerCase;
  candidate->SegNum = segments->SegNum;
  candidate->StpN = segments->StpN;
  candidate->PathLen = segments->PathLen;
#define APA_COPY_HYBRID_GEOMETRY(geometry)                                                         \
  do {                                                                                             \
    (geometry).PlanFlg = 1;                                                                        \
    (geometry).SegNum = segments->SegNum;                                                          \
    (geometry).StpN = segments->StpN;                                                              \
    (geometry).PathLen = segments->PathLen;                                                        \
    for (Integer i = 0; i < segments->SegNum; ++i) {                                               \
      (geometry).SegGoDir[i] = segments->PathGoDir[i];                                             \
      (geometry).SegTrnDir[i] = segments->PathTrnDir[i];                                           \
      (geometry).SegCrv[i] = segments->PathCrv[i];                                                 \
      (geometry).SegTht[i] = segments->PathTht[i];                                                 \
      (geometry).SegLen[i] = segments->PathLenArr[i];                                              \
      (geometry).SegStX[i] = segments->PathStx[i];                                                 \
      (geometry).SegStY[i] = segments->PathSty[i];                                                 \
      (geometry).SegStTh[i] = segments->PathStTh[i];                                               \
      (geometry).SegCcX[i] = segments->PathCC[2 * i];                                              \
      (geometry).SegCcY[i] = segments->PathCC[2 * i + 1];                                          \
    }                                                                                              \
  } while (0)
  if (trial->Input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_VT ||
      trial->Input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_PARKOUT) {
    GeoVtPathPlanningOutput_t geometry = {0};
    GeoPthDiscretVtInput_t input;
    GeoPthDiscretVtOutput_t output = {0};
    APA_COPY_HYBRID_GEOMETRY(geometry);
    apaParkingPathPlanningFillVtDiscretInput(trial->Input, &geometry, &input);
    geoPthDiscretVt(&input, &output);
    apaParkingPathPlanningCopyVtPath(&output, candidate);
  } else if (trial->Input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_HT) {
    GeoHtPathPlanningOutput_t geometry = {0};
    GeoPthDiscretHtInput_t input;
    GeoPthDiscretHtOutput_t output = {0};
    APA_COPY_HYBRID_GEOMETRY(geometry);
    apaParkingPathPlanningFillHtDiscretInput(trial->Input, &geometry, &input);
    if (trial->Input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKOUT)
      geoPthDiscretHt(&input, &output);
    else
      geoPthDiscretHtPkIn(&input, &output);
    apaParkingPathPlanningCopyHtPath(&output, candidate);
  } else if (trial->Input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_DTH_ED_VT) {
    GeoDthEdVtPathPlanningOutput_t geometry = {0};
    GeoPthDiscretVtInput_t input;
    GeoPthDiscretVtOutput_t output = {0};
    APA_COPY_HYBRID_GEOMETRY(geometry);
    apaParkingPathPlanningFillDthDiscretInput(trial->Input, &geometry, &input);
    geoPthDiscretVt(&input, &output);
    apaParkingPathPlanningCopyVtPath(&output, candidate);
  } else if (trial->Input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_VH) {
    GeoVhPathPlanningOutput_t geometry = {0};
    GeoPthDiscretVhInput_t input;
    GeoPthDiscretVhOutput_t output = {0};
    APA_COPY_HYBRID_GEOMETRY(geometry);
    apaParkingPathPlanningFillVhDiscretInput(trial->Input, &geometry, &input);
    geoPthDiscretVh(&input, &output);
    apaParkingPathPlanningCopyVhPath(&output, candidate);
  } else
    return false;
#undef APA_COPY_HYBRID_GEOMETRY
  /* Discretizers retain their slot-family metadata. Normalize the action at
   * this shared boundary, before structural and execution acceptance. */
  if (trial->Input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKOUT) {
    candidate->SelectedSlotKind = APA_SCENARIO_SLOT_KIND_PARKOUT;
    candidate->StepCount = 0;
    candidate->St2Idx = 0;
    candidate->Stp1PtRX = 0.0f;
    candidate->Stp1PtRY = 0.0f;
    candidate->Stp1PtRTheta = 0.0f;
  }
  return candidate->PlanFlg != 0 && candidate->PtCount > 1 &&
         candidate->PtCount <= APA_PARKING_PATH_PT_MAX;
}

Boolean apaParkingPathPlanningAcceptHybridSegments(const ApaPlannerHybridSegments_t *segments,
                                                   void *userData) {
  const ApaParkingHybridTrial_t *trial = userData;
  ApaParkingPathSafetyGateInput_t gate = {0};
  ApaParkingPathSafetyGateOutput_t checked = {0};
  gate.PlannerInput = *trial->Input;
  gate.PlannerParam = *trial->Param;
  if (!apaParkingPathPlanningBuildHybridCandidate(segments, trial, &gate.CandidateOutput)) {
    const Integer error = gate.CandidateOutput.ErrorType;
    apaPlanningRecordFailure(trial->Param->Diagnostics,
                             error == APA_FAULT_PATH_N_EXCEED_TO_ADD ||
                                     error == APA_FAULT_SEG_NUM_OVERMANY
                                 ? error
                                 : APA_FAULT_PATH_STATUS_ERR,
                             "candidate_discretization",
                             4);
    return false;
  }
  apaParkingPathSafetyGate(&gate, &checked);
  return checked.ValidatedOutput.PlanFlg != 0;
}

Integer apaParkingPathPlanningCompareHybridSegments(const ApaPlannerHybridSegments_t *origin,
                                                    const ApaPlannerHybridSegments_t *candidate,
                                                    void *userData) {
  const ApaParkingHybridTrial_t *trial = userData;
  ApaParkingPathPlanningOutput_t first, second;
  if (!apaParkingPathPlanningBuildHybridCandidate(origin, trial, &first) ||
      !apaParkingPathPlanningBuildHybridCandidate(candidate, trial, &second))
    return APA_PATH_COMP_UNKNOWN;
  return apaParkingQualityCompare(
             apaParkingManeuverQuality(&second,
                                       apaParkingManeuverCost(&second, trial->Input, trial->Param)),
             apaParkingManeuverQuality(
                 &first, apaParkingManeuverCost(&first, trial->Input, trial->Param))) < 0
             ? APA_PATH_COMP_NEW_PATH_BETTER
             : APA_PATH_COMP_ORIGIN_PATH_BETTER;
}
