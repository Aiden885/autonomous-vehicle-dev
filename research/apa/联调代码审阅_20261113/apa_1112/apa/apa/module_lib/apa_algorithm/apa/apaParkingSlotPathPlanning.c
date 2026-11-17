/* MATLAB sources: geo_VT_PathPlanning.m */

#include "apaParkingSlotPathPlanning.h"
#include "apaVehicleDefaults.h"
#include "apaParkingPathPlanning.h"
#include "apaParkingPathPlanningInternal.h"
#include "apaParkingSlotTargetResolve.h"
#include "apaCoordUtils.h"
#include "apaGoalTolerance.h"
#include "apaSlotTypes.h"
#include "checkParkingTypeTypes.h"
#include <math.h>

static Boolean apaParkingSlotRequestLinesAreValid(
    Integer count, const Real x1[64], const Real y1[64], const Real x2[64], const Real y2[64]) {
  Integer index;
  Boolean valid0;
  Boolean loop0;
  valid0 = (count >= 0) && (count <= APA_SCENARIO_BOUNDARY_LINE_MAX);
  index = 0;
  loop0 = valid0 && (index < count);
  while (loop0) {
    valid0 =
        isfinite(x1[index]) && isfinite(y1[index]) && isfinite(x2[index]) && isfinite(y2[index]);
    index = index + 1;
    loop0 = valid0 && (index < count);
  }
  return valid0;
}

static Boolean
apaParkingSlotRequestPointsAreValid(Integer count, const Real x[128], const Real y[128]) {
  Integer index;
  Boolean valid0;
  Boolean loop0;
  valid0 = (count >= 0) && (count <= APA_SCENARIO_BOUNDARY_POINT_MAX);
  index = 0;
  loop0 = valid0 && (index < count);
  while (loop0) {
    valid0 = isfinite(x[index]) && isfinite(y[index]);
    index = index + 1;
    loop0 = valid0 && (index < count);
  }
  return valid0;
}

static Boolean apaParkingSlotRequestEnvironmentIsValid(const ApaParkingPathPlanningInput_t *input) {
  Boolean lines0;
  Boolean virtualLines0;
  Boolean stopperLines0;
  Boolean points0;
  Boolean stopperPoints0;
  Boolean valid0;
  lines0 = apaParkingSlotRequestLinesAreValid(input->BoundaryLineNum,
                                              input->BoundaryLineX1,
                                              input->BoundaryLineY1,
                                              input->BoundaryLineX2,
                                              input->BoundaryLineY2);
  virtualLines0 = apaParkingSlotRequestLinesAreValid(input->VirtualBoundaryLineNum,
                                                     input->VirtualBoundaryP1X,
                                                     input->VirtualBoundaryP1Y,
                                                     input->VirtualBoundaryP2X,
                                                     input->VirtualBoundaryP2Y);
  stopperLines0 = apaParkingSlotRequestLinesAreValid(input->SlotTrackStopperNum,
                                                     input->SlotTrackStopperX1,
                                                     input->SlotTrackStopperY1,
                                                     input->SlotTrackStopperX2,
                                                     input->SlotTrackStopperY2);
  points0 = apaParkingSlotRequestPointsAreValid(
      input->BoundarySepPtNum, input->BoundarySepPtX, input->BoundarySepPtY);
  stopperPoints0 = apaParkingSlotRequestPointsAreValid(
      input->StopperPtNum, input->StopperPtX, input->StopperPtY);
  valid0 = lines0 && virtualLines0 && stopperLines0 && points0 && stopperPoints0;
  return valid0;
}

static Boolean
apaParkingSlotRequestPlannerFieldsAreValid(const ApaParkingPathPlanningInput_t *input) {
  Boolean sideValid0;
  Boolean surroundingValid0;
  Boolean shortcutValid0;
  Boolean tuningValid0;
  Boolean valid0;
  sideValid0 = (input->PrkType >= 0) && (input->PrkType <= 3);
  surroundingValid0 = (input->SurdType >= 0) && (input->SurdType <= 4);
  shortcutValid0 = (input->ShortCutSw == 0) || (input->ShortCutSw == 1);
  tuningValid0 = isfinite(input->AgrOfst) && isfinite(input->ShortCutThd) &&
                 (input->ShortCutThd >= 0.0f) &&
                 apaGoalToleranceValid(input->GoalPositionTolerance) &&
                 apaGoalToleranceValid(input->GoalYawTolerance);
  valid0 = sideValid0 && surroundingValid0 && shortcutValid0 && tuningValid0;
  return valid0;
}

static Boolean
apaParkingSlotRequestPlannerParamIsValid(const ApaParkingPathPlanningParam_t *param) {
  Real values[4];
  Integer index;
  Boolean scalarsValid0;
  Boolean fusionValid0;
  Boolean statusValid0;
  Boolean valid0;
  values[0] = param->VehLfMin;
  values[1] = param->VtFallbackSpace;
  values[2] = param->DthFallbackSpace;
  values[3] = param->HtRfoutMin;
  index = 0;
  scalarsValid0 = true;
  while (index < 4) {
    Boolean scalarValid0;
    scalarValid0 = isfinite(values[index]) && (values[index] >= 0.0f);
    scalarsValid0 = scalarsValid0 && scalarValid0;
    index = index + 1;
  }
  fusionValid0 = (param->NeedHybridAlgFusion == 0) || (param->NeedHybridAlgFusion == 1);
  statusValid0 = (param->SltState >= APA_SLOT_STATUS_INVALID) &&
                 (param->SltState <= APA_SLOT_STATUS_TARGET_SLOT_REPLAN);
  valid0 = scalarsValid0 && fusionValid0 && statusValid0;
  return valid0;
}

static Boolean apaParkingSlotRequestIsValid(const ApaParkingPathPlanningFromSlotInput_t *input,
                                            const ApaParkingPathPlanningFromSlotParam_t *param) {
  Boolean poseValid0;
  Boolean unitValid0;
  Boolean offsetValid0;
  Boolean radiusValid0;
  Boolean enumValid0;
  Boolean environmentValid0;
  Boolean paramValid0;
  Boolean valid0;
  poseValid0 = isfinite(input->PlannerInput.VehicleX) && isfinite(input->PlannerInput.VehicleY) &&
               isfinite(input->PlannerInput.VehicleYaw);
  unitValid0 = (input->AngleUnit == APA_SLOT_REQUEST_ANGLE_RAD) ||
               (input->AngleUnit == APA_SLOT_REQUEST_ANGLE_DEG);
  offsetValid0 =
      isfinite(param->VehicleReferenceOffset) && (fabsf(param->VehicleReferenceOffset) <= 1000.0f);
  radiusValid0 = isfinite(input->PlannerInput.Rmin) && (input->PlannerInput.Rmin > 0.0f);
  enumValid0 = apaParkingSlotRequestPlannerFieldsAreValid(&input->PlannerInput);
  environmentValid0 = apaParkingSlotRequestEnvironmentIsValid(&input->PlannerInput);
  paramValid0 = apaParkingSlotRequestPlannerParamIsValid(&param->PlannerParam);
  valid0 = poseValid0 && unitValid0 && offsetValid0 && radiusValid0 && enumValid0 &&
           environmentValid0 && paramValid0;
  return valid0;
}

static void apaParkingSlotRequestFillTargetInput(const ApaParkingPathPlanningFromSlotInput_t *input,
                                                 const ApaParkingPathPlanningFromSlotParam_t *param,
                                                 ApaParkingSlotTargetResolveInput_t *targetInput) {
  *targetInput = (ApaParkingSlotTargetResolveInput_t){0};
  targetInput->SlotKind = input->PlannerInput.TargetSlotKind;
  targetInput->EnterEdge = input->PlannerInput.SltEntr;
  apaSlotSetPoints(targetInput->SlotPts,
                   input->PlannerInput.SltPt1X,
                   input->PlannerInput.SltPt1Y,
                   input->PlannerInput.SltPt2X,
                   input->PlannerInput.SltPt2Y,
                   input->PlannerInput.SltPt3X,
                   input->PlannerInput.SltPt3Y,
                   input->PlannerInput.SltPt4X,
                   input->PlannerInput.SltPt4Y);
  targetInput->ParkingAction = input->PlannerInput.ParkingAction;
  targetInput->RequestedTargetX = input->PlannerInput.TarX;
  targetInput->RequestedTargetY = input->PlannerInput.TarY;
  targetInput->RequestedTargetTheta = input->PlannerInput.TarTheta;
  targetInput->VehFront = param->PlannerParam.VehLlF;
  targetInput->VehRear = param->PlannerParam.VehLr;
  targetInput->VehWidth = param->PlannerParam.VehW;
}

static void apaParkingSlotRequestNormalizePose(const ApaParkingPathPlanningFromSlotInput_t *input,
                                               const ApaParkingPathPlanningFromSlotParam_t *param,
                                               ApaParkingPathPlanningInput_t *resolved) {
  Real theta0;
  Real theta1;
  Real cosTheta0;
  Real sinTheta0;
  *resolved = input->PlannerInput;
  resolved->Rmin = fmaxf(input->PlannerInput.Rmin, APA_VEHICLE_MIN_TURN_RADIUS_M);
  if (input->AngleUnit == APA_SLOT_REQUEST_ANGLE_DEG) {
    theta0 = input->PlannerInput.VehicleYaw * (APA_PI / 180.0f);
    resolved->TarTheta = input->PlannerInput.TarTheta * (APA_PI / 180.0f);
  } else {
    theta0 = input->PlannerInput.VehicleYaw;
  }
  theta1 = apaCoordWrapToPi(theta0);
  cosTheta0 = cosf(theta1);
  sinTheta0 = sinf(theta1);
  resolved->VehicleYaw = theta1;
  resolved->VehicleX = input->PlannerInput.VehicleX - param->VehicleReferenceOffset * cosTheta0;
  resolved->VehicleY = input->PlannerInput.VehicleY - param->VehicleReferenceOffset * sinTheta0;
  resolved->TarTheta = apaCoordWrapToPi(resolved->TarTheta);
}

static void apaParkingSlotRequestResolveSide(ApaParkingPathPlanningInput_t *resolved) {
  Boolean sideSpecified0;
  sideSpecified0 =
      (resolved->PrkType == APA_PARKING_TYPE_LEFT) || (resolved->PrkType == APA_PARKING_TYPE_RIGHT);
  if (!sideSpecified0) {
    CheckParkingTypeInput_t sideInput;
    CheckParkingTypeOutput_t sideOutput;
    sideInput.VehX = resolved->VehicleX;
    sideInput.VehY = resolved->VehicleY;
    sideInput.VehTheta = resolved->VehicleYaw;
    sideInput.TarX = resolved->TarX;
    sideInput.TarY = resolved->TarY;
    sideInput.TarTheta = resolved->TarTheta;
    sideInput.SltPt1X = resolved->SltPt1X;
    sideInput.SltPt1Y = resolved->SltPt1Y;
    sideInput.SltPt2X = resolved->SltPt2X;
    sideInput.SltPt2Y = resolved->SltPt2Y;
    sideInput.SltPt3X = resolved->SltPt3X;
    sideInput.SltPt3Y = resolved->SltPt3Y;
    sideInput.SltPt4X = resolved->SltPt4X;
    sideInput.SltPt4Y = resolved->SltPt4Y;
    sideInput.EnterEdge = resolved->SltEntr;
    sideOutput = (CheckParkingTypeOutput_t){0};
    checkParkingType(&sideInput, &sideOutput);
    resolved->PrkType = sideOutput.ParkingType;
  }
}

static void apaParkingSlotRequestResolveAndPlan(const ApaParkingPathPlanningFromSlotInput_t *input,
                                                const ApaParkingPathPlanningFromSlotParam_t *param,
                                                ApaParkingPathPlanningFromSlotOutput_t *output) {
  ApaParkingSlotTargetResolveInput_t targetInput;
  ApaParkingPathPlanningFromSlotInput_t normalizedRequest = *input;
  apaParkingSlotRequestNormalizePose(input, param, &output->ResolvedInput);
  normalizedRequest.PlannerInput = output->ResolvedInput;
  apaParkingSlotRequestFillTargetInput(&normalizedRequest, param, &targetInput);
  apaParkingSlotTargetResolve(&targetInput, &output->Target);
  if (output->Target.Valid) {
    output->ResolvedInput.EntrPtX = output->Target.EntryX;
    output->ResolvedInput.EntrPtY = output->Target.EntryY;
    apaParkingSlotRequestResolveSide(&output->ResolvedInput);
    apaParkingPathPlanning(&output->ResolvedInput, &param->PlannerParam, &output->Path);
  } else if (output->Target.Status == APA_SLOT_TARGET_UNSUPPORTED) {
    output->Path.ErrorType = APA_FAULT_PLANNER_NOT_ACCEPT_THE_SLOT;
  } else if ((output->Target.Status == APA_SLOT_TARGET_INVALID_GEOMETRY) ||
             (output->Target.Status == APA_SLOT_TARGET_INVALID_ENTRY)) {
    output->Path.ErrorType = APA_FAULT_SLOT_INVALID;
  }
}

void apaParkingPathPlanningFromSlotDefaultParam(ApaParkingPathPlanningFromSlotParam_t *param) {
  *param = (ApaParkingPathPlanningFromSlotParam_t){0};
  param->PlannerParam.SltState = APA_SLOT_STATUS_TARGET_SLOT;
  param->PlannerParam.VehW = APA_VEHICLE_WIDTH_M;
  param->PlannerParam.VehLlF = APA_VEHICLE_REAR_TO_FRONT_M;
  param->PlannerParam.VehLfMin = 0.4f;
  param->PlannerParam.VehLr = APA_VEHICLE_REAR_TO_BACK_M;
  param->PlannerParam.VtFallbackSpace = 4.1f;
  param->PlannerParam.DthFallbackSpace = 6.0f;

  param->PlannerParam.HtRfoutMin = 0.0f;
  param->PlannerParam.NeedHybridAlgFusion = 1;
}

void apaParkingPathPlanningFromSlot(const ApaParkingPathPlanningFromSlotInput_t *input,
                                    const ApaParkingPathPlanningFromSlotParam_t *param,
                                    ApaParkingPathPlanningFromSlotOutput_t *output) {
  *output = (ApaParkingPathPlanningFromSlotOutput_t){0};
  output->Target.Status = APA_SLOT_TARGET_INVALID_REQUEST;
  output->Path.ErrorType = APA_FAULT_INVALID_APA_STATE;
  if ((input->PlannerInput.ParkingAction != APA_SCENARIO_PARKING_ACTION_PARKIN) &&
      (input->PlannerInput.ParkingAction != APA_SCENARIO_PARKING_ACTION_PARKOUT)) {
    output->Target.Status = APA_SLOT_TARGET_UNSUPPORTED;
    output->Path.ErrorType = APA_FAULT_PLANNER_NOT_ACCEPT_THE_SLOT;
  } else if (apaParkingSlotRequestIsValid(input, param)) {
    apaParkingSlotRequestResolveAndPlan(input, param, output);
  }
}
