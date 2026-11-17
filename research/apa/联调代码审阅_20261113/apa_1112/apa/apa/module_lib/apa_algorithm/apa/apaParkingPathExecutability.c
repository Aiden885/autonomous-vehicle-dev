#include "apaParkingPathExecutability.h"
#include "apaMocoParkingControl.h"
#include "apaParkingPathPlanningInternal.h"
#include "apaTargetSlotConstraint.h"
#include "apaCoordUtils.h"
#include "apaGoalTolerance.h"
#include "apaEnums.h"
#include <math.h>

void apaParkingPathExecutabilityDefaultParam(ApaParkingPathExecutabilityParam_t *param) {
  *param = (ApaParkingPathExecutabilityParam_t){0};
  apaMocoParkingControlDefaultParam(&param->Control);
  param->Plant.CycleTime = param->Control.Longi.CycleTime;
  param->Plant.WheelBase = param->Control.Lat.Wheelbase;
  param->Plant.ForwardSpeedLimit = param->Control.Longi.SpeedUpperLimit;
  param->Plant.ReverseSpeedLimit = -param->Control.Longi.SpeedLowerLimit;
  param->Plant.AccelerationUpperLimit = param->Control.Longi.AccelerationUpperLimit;
  param->Plant.AccelerationLowerLimit = param->Control.Longi.AccelerationLowerLimit;
  param->Plant.StraightSteerThreshold = 0.0001f;
  param->LateralAllowance = 0.04f;
  param->YawAllowance = 0.04f;
  param->StopAllowance = 0.03f;
  param->PositionLimit = 0.15f;
  param->YawLimit = 0.07f;
  param->LateralLimit = 0.12f;
  param->MaxSteps = 6500;
}

static Boolean
apaParkingPathExecutabilityAllowancesAreValid(const ApaParkingPathExecutabilityParam_t *param) {
  Boolean allowances0;
  allowances0 = isfinite(param->LateralAllowance) && isfinite(param->YawAllowance) &&
                isfinite(param->StopAllowance) && (param->LateralAllowance >= 0.0f) &&
                (param->YawAllowance >= 0.0f) && (param->YawAllowance <= 0.2f) &&
                (param->StopAllowance >= 0.0f);
  return allowances0;
}

static Boolean
apaParkingPathExecutabilityLimitsAreValid(const ApaParkingPathExecutabilityParam_t *param) {
  Boolean limits0;
  limits0 = isfinite(param->PositionLimit) && isfinite(param->YawLimit) &&
            isfinite(param->LateralLimit) && (param->PositionLimit > 0.0f) &&
            (param->YawLimit > 0.0f) && (param->LateralLimit > 0.0f) && (param->MaxSteps > 0) &&
            (param->MaxSteps <= 6500);
  return limits0;
}

static Boolean
apaParkingPathExecutabilityPeriodsAreValid(const ApaParkingPathExecutabilityParam_t *param) {
  Boolean periods0;
  periods0 = isfinite(param->Plant.CycleTime) && (param->Plant.CycleTime > 0.0f) &&
             (param->Plant.CycleTime <= 0.05f) &&
             (param->Plant.CycleTime == param->Control.Longi.CycleTime) &&
             isfinite(param->Plant.WheelBase) && (param->Plant.WheelBase > 0.0f) &&
             (param->Plant.WheelBase == param->Control.Lat.Wheelbase);
  return periods0;
}

static Boolean
apaParkingPathExecutabilityBrakesAreValid(const ApaParkingPathExecutabilityParam_t *param) {
  Boolean brakes0;
  brakes0 = isfinite(param->Control.ReactionTime) && isfinite(param->Control.StopBuffer) &&
            (param->Control.ReactionTime >= 0.0f) && (param->Control.StopBuffer >= 0.0f) &&
            isfinite(param->Control.Longi.StopDeceleration) &&
            isfinite(param->Control.Longi.JerkUpperLimit) &&
            isfinite(param->Control.Longi.JerkLowerLimit) &&
            isfinite(param->Control.Longi.StandstillSpeedThreshold) &&
            (param->Control.Longi.StopDeceleration > 0.0f) &&
            (param->Control.Longi.JerkUpperLimit > 0.0f) &&
            (param->Control.Longi.JerkLowerLimit < 0.0f) &&
            (param->Control.Longi.StandstillSpeedThreshold >= 0.0f) &&
            (param->Control.Longi.StandstillSpeedThreshold <= param->Control.Chart.StandstillSpeed);
  return brakes0;
}

Boolean apaParkingPathExecutabilityParamIsValid(const ApaParkingPathExecutabilityParam_t *param) {
  Boolean allowances0;
  Boolean limits0;
  Boolean periods0;
  Boolean brakes0;
  Boolean valid0;
  allowances0 = apaParkingPathExecutabilityAllowancesAreValid(param);
  limits0 = apaParkingPathExecutabilityLimitsAreValid(param);
  periods0 = apaParkingPathExecutabilityPeriodsAreValid(param);
  brakes0 = apaParkingPathExecutabilityBrakesAreValid(param);
  valid0 = allowances0 && limits0 && periods0 && brakes0;
  return valid0;
}

void apaParkingPathExecutabilityEnvelope(const ApaParkingPathPlanningParam_t *vehicle,
                                         const ApaParkingPathExecutabilityParam_t *param,
                                         ApaParkingPathPlanningParam_t *envelope) {
  Real sine0;
  Real length0;
  Real lateral0;
  Real longitudinal0;
  sine0 = sinf(param->YawAllowance);
  length0 = fmaxf(vehicle->VehLlF, vehicle->VehLr);
  lateral0 = param->LateralAllowance + (length0 * sine0);
  longitudinal0 = param->StopAllowance + (0.5f * vehicle->VehW * sine0);
  *envelope = *vehicle;
  envelope->VehLlF = vehicle->VehLlF + longitudinal0;
  envelope->VehLr = vehicle->VehLr + longitudinal0;
  envelope->VehW = vehicle->VehW + (2.0f * lateral0);
}

Boolean apaParkingPathExecutabilityMotionIsSafe(const ApaParkingPathPlanningInput_t *scene,
                                                const ApaParkingPathPlanningParam_t *vehicle,
                                                const Real previousPose[3],
                                                const Real nextPose[3]) {
  ApaParkingPathPlanningOutput_t motion;
  ApaTargetSlotConstraint_t constraint;
  Boolean clear0;
  Boolean loop0;
  Integer index;
  Real yawDelta0;
  motion = (ApaParkingPathPlanningOutput_t){0};
  motion.PlanFlg = 1;
  motion.PtCount = 2;
  motion.PthPtX[0] = previousPose[0];
  motion.PthPtY[0] = previousPose[1];
  motion.PthPtTht[0] = previousPose[2];
  motion.PthPtX[1] = nextPose[0];
  motion.PthPtY[1] = nextPose[1];
  motion.PthPtTht[1] = nextPose[2];
  clear0 = apaParkingPathPlanningOutputPathIsSafe(scene, vehicle, &motion, false);
  apaParkingPathPlanningBuildTargetSlotConstraint(scene, vehicle, &constraint);
  constraint.Enabled = true;
  yawDelta0 = nextPose[2] - previousPose[2];
  yawDelta0 = apaCoordWrapToPi(yawDelta0);
  index = 0;
  loop0 = clear0 && (index < 3);
  while (loop0) {
    Integer edge;
    Real fraction0;
    Real x0;
    Real y0;
    Real yaw0;
    fraction0 = ((Real)index) * 0.5f;
    x0 = previousPose[0] + fraction0 * (nextPose[0] - previousPose[0]);
    y0 = previousPose[1] + fraction0 * (nextPose[1] - previousPose[1]);
    yaw0 = previousPose[2] + fraction0 * yawDelta0;
    clear0 = apaTargetSlotConstraintPoseIsClear(&constraint, x0, y0, yaw0, &edge);
    index = index + 1;
    loop0 = clear0 && (index < 3);
  }
  return clear0;
}

static void apaParkingPathExecutabilityInitialize(const ApaParkingPathExecutabilityInput_t *input,
                                                  ApaMocoParkingControlInput_t *control) {
  Integer index;
  Boolean signFound0;
  *control = (ApaMocoParkingControlInput_t){0};
  control->Chart.VehicleX = input->Scene.VehicleX;
  control->Chart.VehicleY = input->Scene.VehicleY;
  control->Chart.VehicleYaw = input->Scene.VehicleYaw;
  control->Chart.GoalX = input->Scene.TarX;
  control->Chart.GoalY = input->Scene.TarY;
  control->Chart.GoalYaw = input->Scene.TarTheta;
  control->Chart.PtCount = input->Path.PtCount;
  control->Chart.PathUpdateIdx = 1;
  control->Chart.TargetSlotKind = input->Scene.TargetSlotKind;
  control->Chart.ParkingAction = input->Scene.ParkingAction;
  control->Chart.GearPos = APA_GEAR_POS_D;
  signFound0 = false;
  index = 0;
  while (index < input->Path.PtCount) {
    control->Chart.PthPtX[index] = input->Path.PthPtX[index];
    control->Chart.PthPtY[index] = input->Path.PthPtY[index];
    control->Chart.PthPtTht[index] = input->Path.PthPtTht[index];
    control->Chart.PthPtOdo[index] = input->Path.PthPtOdo[index];
    if (!signFound0) {
      if (fabsf(input->Path.PthPtOdo[index]) > 0.0001f) {
        signFound0 = true;
        if (input->Path.PthPtOdo[index] < 0.0f) {
          control->Chart.GearPos = APA_GEAR_POS_R;
        }
      }
    }
    index = index + 1;
  }
}

static Boolean
apaParkingPathExecutabilityCycleIsFinite(const ApaMocoParkingControlOutput_t *control,
                                         const VehicleDynamicStepOutput_t *plant) {
  Boolean finite0;
  finite0 = isfinite(plant->VehicleX) && isfinite(plant->VehicleY) && isfinite(plant->VehicleYaw) &&
            isfinite(plant->VehicleSpeed) && isfinite(plant->VehicleAcceleration) &&
            isfinite(control->Longi.TargetAcceleration) && isfinite(control->Lat.SteerAngle) &&
            isfinite(control->Path.CrossTrackError) && isfinite(control->Path.HeadingError) &&
            isfinite(control->StopDistance) && (plant->ModelValid != 0);
  return finite0;
}

static void apaParkingPathExecutabilityRun(const ApaParkingPathExecutabilityInput_t *input,
                                           const ApaParkingPathExecutabilityParam_t *param,
                                           ApaParkingPathExecutabilityOutput_t *output) {
  ApaMocoParkingControlInput_t controlInput;
  ApaMocoParkingControlParam_t controlParam;
  ApaMocoParkingControlState_t controlState;
  ApaMocoParkingControlOutput_t controlOutput;
  VehicleDynamicStepInput_t plantInput;
  VehicleDynamicStepState_t plantState;
  VehicleDynamicStepOutput_t plantOutput;
  Real previousPose[3];
  Real nextPose[3];
  Boolean loop0;
  Boolean finite0;
  Boolean safe0;
  Boolean stopped0;
  Boolean done0;
  Real dx0;
  Real dy0;
  Real yaw0;
  apaParkingPathExecutabilityInitialize(input, &controlInput);
  controlParam = param->Control;
  controlParam.Chart.GoalPositionTolerance =
      fminf(controlParam.Chart.GoalPositionTolerance,
            apaGoalPositionTolerance(input->Scene.GoalPositionTolerance));
  controlParam.Chart.GoalYawTolerance = fminf(controlParam.Chart.GoalYawTolerance,
                                              apaGoalYawTolerance(input->Scene.GoalYawTolerance));
  controlState = (ApaMocoParkingControlState_t){0};
  plantState = (VehicleDynamicStepState_t){0};
  loop0 = true;
  while (loop0) {
    apaMocoParkingControl(&controlInput, &controlParam, &controlState, &controlOutput);
    plantInput = (VehicleDynamicStepInput_t){0};
    plantInput.VehicleX = controlInput.Chart.VehicleX;
    plantInput.VehicleY = controlInput.Chart.VehicleY;
    plantInput.VehicleYaw = controlInput.Chart.VehicleYaw;
    plantInput.GearPos = controlInput.Chart.GearPos;
    if (controlOutput.Longi.GearChangeAllowed != 0) {
      plantInput.GearPos = controlOutput.Chart.TargetGearReq;
    }
    plantInput.AccelerationCmd = controlOutput.Longi.TargetAcceleration;
    plantInput.SteerAngle = controlOutput.Lat.SteerAngle;
    vehicleDynamicStep(&plantInput, &param->Plant, &plantState, &plantOutput);
    previousPose[0] = controlInput.Chart.VehicleX;
    previousPose[1] = controlInput.Chart.VehicleY;
    previousPose[2] = controlInput.Chart.VehicleYaw;
    nextPose[0] = plantOutput.VehicleX;
    nextPose[1] = plantOutput.VehicleY;
    nextPose[2] = plantOutput.VehicleYaw;
    output->StepCount = output->StepCount + 1;
    finite0 = apaParkingPathExecutabilityCycleIsFinite(&controlOutput, &plantOutput);
    safe0 = false;
    if (finite0) {
      safe0 = apaParkingPathExecutabilityMotionIsSafe(
          &input->Scene, &input->Vehicle, previousPose, nextPose);
    }
    if (safe0) {
      output->FinalPose[0] = nextPose[0];
      output->FinalPose[1] = nextPose[1];
      output->FinalPose[2] = nextPose[2];
      output->MaxLateralError =
          fmaxf(output->MaxLateralError, fabsf(controlOutput.Path.CrossTrackError));
      controlInput.Chart.VehicleX = nextPose[0];
      controlInput.Chart.VehicleY = nextPose[1];
      controlInput.Chart.VehicleYaw = nextPose[2];
      controlInput.Chart.GearPos = plantInput.GearPos;
      controlInput.VehicleSpeed = plantOutput.VehicleSpeed;
      controlInput.VehicleAcceleration = plantOutput.VehicleAcceleration;
    }
    stopped0 = (fabsf(plantOutput.VehicleSpeed) <= 0.01f) &&
               (fabsf(controlOutput.Longi.TargetAcceleration) <= 0.001f);
    done0 = (controlOutput.Chart.CtrlPhase == 6) && stopped0;
    output->Completed = done0 && safe0;
    loop0 = safe0 && !done0 && (output->StepCount < param->MaxSteps);
  }
  dx0 = output->FinalPose[0] - input->Scene.TarX;
  dy0 = output->FinalPose[1] - input->Scene.TarY;
  output->FinalPositionError = sqrtf(dx0 * dx0 + dy0 * dy0);
  yaw0 = output->FinalPose[2] - input->Scene.TarTheta;
  yaw0 = apaCoordWrapToPi(yaw0);
  output->FinalYawError = fabsf(yaw0);
  output->Passed = output->Completed && (output->FinalPositionError <= param->PositionLimit) &&
                   (output->FinalYawError <= param->YawLimit) &&
                   (output->MaxLateralError <= param->LateralLimit);
}

static Boolean
apaParkingPathExecutabilityInputIsValid(const ApaParkingPathExecutabilityInput_t *input) {
  Boolean valid0;
  Boolean loop0;
  Integer index;
  valid0 = isfinite(input->Scene.VehicleX) && isfinite(input->Scene.VehicleY) &&
           isfinite(input->Scene.TarX) && isfinite(input->Scene.TarY) &&
           isfinite(input->Scene.TarTheta) &&
           apaGoalToleranceValid(input->Scene.GoalPositionTolerance) &&
           apaGoalToleranceValid(input->Scene.GoalYawTolerance) &&
           isfinite(input->Scene.VehicleYaw) && isfinite(input->Vehicle.VehLlF) &&
           isfinite(input->Vehicle.VehLr) && isfinite(input->Vehicle.VehW) &&
           (input->Vehicle.VehLlF > 0.0f) && (input->Vehicle.VehLr > 0.0f) &&
           (input->Vehicle.VehW > 0.0f);
  index = 0;
  loop0 = valid0 && (index < input->Path.PtCount);
  while (loop0) {
    valid0 = isfinite(input->Path.PthPtX[index]) && isfinite(input->Path.PthPtY[index]) &&
             isfinite(input->Path.PthPtTht[index]) && isfinite(input->Path.PthPtOdo[index]);
    index = index + 1;
    loop0 = valid0 && (index < input->Path.PtCount);
  }
  return valid0;
}

/**
 * @brief Publish a candidate only after finite, safe, complete closed-loop execution.
 * @en_name apaParkingPathExecutability
 * @cn_name 泊车候选执行检查
 * @type module
 * @param[IN] ApaParkingPathPlanningInput_t Scene 原场景与真实后轴起点
 * @param[IN] ApaParkingPathPlanningParam_t Vehicle 真实车型
 * @param[IN] ApaParkingPathPlanningOutput_t Path 原候选路径
 * @param[OUT] Boolean Passed 执行通过
 * @param[OUT] Boolean Completed 完成且实际停稳
 * @param[OUT] Integer StepCount 周期数
 * @param[OUT] Real FinalPositionError 终点位置误差
 * @param[OUT] Real FinalYawError 终点航向误差
 * @param[OUT] Real MaxLateralError 全程横向误差
 * @param[OUT] Real[3] FinalPose 最后安全后轴位姿
 * @granularity composite
 * @tag_level1 规划
 * @tag_level2 执行验收
 * @source README.md
 * @version 1.0
 * @date 2026-10-01
 * @author aide
 */
void apaParkingPathExecutability(const ApaParkingPathExecutabilityInput_t *input,
                                 const ApaParkingPathExecutabilityParam_t *param,
                                 ApaParkingPathExecutabilityOutput_t *output) {
  Boolean valid0;
  *output = (ApaParkingPathExecutabilityOutput_t){0};
  output->FinalPose[0] = input->Scene.VehicleX;
  output->FinalPose[1] = input->Scene.VehicleY;
  output->FinalPose[2] = input->Scene.VehicleYaw;
  valid0 = apaParkingPathExecutabilityParamIsValid(param);
  valid0 = valid0 && (input->Path.PlanFlg != 0) && (input->Path.PtCount >= 2) &&
           (input->Path.PtCount <= PATH_PT_MAX);
  if (valid0) {
    valid0 = apaParkingPathExecutabilityInputIsValid(input);
  }
  if (valid0) {
    apaParkingPathExecutabilityRun(input, param, output);
  }
}
