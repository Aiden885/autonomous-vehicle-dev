#include "apaCoordUtils.h"
#include "apaEnums.h"
#include "vehicleDynamicStepTypes.h"
#include <math.h>

static Real
vehicleDynamicStepClamp(Real value, Real lowerLimit, Real upperLimit, Integer *limited) {
  Real result0;

  if (value > upperLimit) {
    result0 = upperLimit;
    *limited = 1;
  } else if (value < lowerLimit) {
    result0 = lowerLimit;
    *limited = 1;
  } else {
    result0 = value;
  }

  return result0;
}

static Boolean vehicleDynamicStepAccelerationInputIsValid(const VehicleDynamicStepInput_t *input) {
  Boolean finite0;
  Boolean gearLowerValid0;
  Boolean gearUpperValid0;
  Boolean gearValid0;
  Boolean valid0;

  finite0 = isfinite(input->VehicleX) && isfinite(input->VehicleY) && isfinite(input->VehicleYaw) &&
            isfinite(input->SteerAngle) && isfinite(input->AccelerationCmd);
  gearLowerValid0 = (input->GearPos >= APA_GEAR_POS_N);
  gearUpperValid0 = (input->GearPos <= APA_GEAR_POS_P);
  gearValid0 = gearLowerValid0 && gearUpperValid0;
  valid0 = finite0 && gearValid0;

  return valid0;
}

static Boolean vehicleDynamicStepAccelerationParamIsFinite(const VehicleDynamicStepParam_t *param) {
  Boolean finite0;

  finite0 = isfinite(param->WheelBase) && isfinite(param->StraightSteerThreshold) &&
            isfinite(param->CycleTime) && isfinite(param->ForwardSpeedLimit) &&
            isfinite(param->ReverseSpeedLimit) && isfinite(param->AccelerationUpperLimit) &&
            isfinite(param->AccelerationLowerLimit);

  return finite0;
}

static Boolean
vehicleDynamicStepAccelerationParamRangeIsValid(const VehicleDynamicStepParam_t *param) {
  Boolean kinematicValid0;
  Boolean speedLimitValid0;
  Boolean accelerationOrderValid0;
  Boolean accelerationZeroInside0;
  Boolean valid0;

  kinematicValid0 = (param->WheelBase > 0.0f) && (param->StraightSteerThreshold > 0.0f) &&
                    (param->CycleTime > 0.0f);
  speedLimitValid0 = (param->ForwardSpeedLimit >= 0.0f) && (param->ReverseSpeedLimit >= 0.0f);
  accelerationOrderValid0 = (param->AccelerationUpperLimit >= param->AccelerationLowerLimit);
  accelerationZeroInside0 =
      (param->AccelerationLowerLimit <= 0.0f) && (param->AccelerationUpperLimit >= 0.0f);
  valid0 =
      kinematicValid0 && speedLimitValid0 && accelerationOrderValid0 && accelerationZeroInside0;

  return valid0;
}

static Boolean
vehicleDynamicStepAccelerationContextIsValid(const VehicleDynamicStepInput_t *input,
                                             const VehicleDynamicStepParam_t *param,
                                             const VehicleDynamicStepState_t *state) {
  Boolean inputValid0;
  Boolean paramFinite0;
  Boolean paramRangeValid0;
  Boolean stateValid0;
  Boolean valid0;

  inputValid0 = vehicleDynamicStepAccelerationInputIsValid(input);
  paramFinite0 = vehicleDynamicStepAccelerationParamIsFinite(param);
  paramRangeValid0 = vehicleDynamicStepAccelerationParamRangeIsValid(param);
  stateValid0 = isfinite(state->VehicleSpeed) && isfinite(state->VehicleAcceleration);
  valid0 = inputValid0 && paramFinite0 && paramRangeValid0 && stateValid0;

  return valid0;
}

static Real vehicleDynamicStepInitialSpeed(Real vehicleSpeed, Integer gearPos, Integer *limited) {
  Boolean driveConflict0;
  Boolean reverseConflict0;
  Boolean noDriveGear0;
  Boolean resetSpeed0;
  Real initialSpeed0;

  driveConflict0 = (gearPos == APA_GEAR_POS_D) && (vehicleSpeed < 0.0f);
  reverseConflict0 = (gearPos == APA_GEAR_POS_R) && (vehicleSpeed > 0.0f);
  noDriveGear0 = (gearPos != APA_GEAR_POS_D) && (gearPos != APA_GEAR_POS_R);
  resetSpeed0 = driveConflict0 || reverseConflict0 || noDriveGear0;
  if (resetSpeed0) {
    initialSpeed0 = 0.0f;
    *limited = 1;
  } else {
    initialSpeed0 = vehicleSpeed;
  }

  return initialSpeed0;
}

static Real vehicleDynamicStepNextSpeed(Real initialSpeed,
                                        Real accelerationStep,
                                        Integer gearPos,
                                        const VehicleDynamicStepParam_t *param,
                                        Integer *limited) {
  Real candidateSpeed0;
  Real lowerLimit0;
  Real upperLimit0;
  Real nextSpeed0;

  candidateSpeed0 = initialSpeed + accelerationStep;
  lowerLimit0 = 0.0f;
  upperLimit0 = 0.0f;
  if (gearPos == APA_GEAR_POS_D) {
    upperLimit0 = param->ForwardSpeedLimit;
  } else if (gearPos == APA_GEAR_POS_R) {
    lowerLimit0 = -param->ReverseSpeedLimit;
  } else {
    *limited = 1;
  }
  nextSpeed0 = vehicleDynamicStepClamp(candidateSpeed0, lowerLimit0, upperLimit0, limited);

  return nextSpeed0;
}

static void vehicleDynamicStepCalculateLongitudinal(const VehicleDynamicStepInput_t *input,
                                                    const VehicleDynamicStepParam_t *param,
                                                    const VehicleDynamicStepState_t *state,
                                                    Boolean modelValid,
                                                    VehicleDynamicStepOutput_t *output) {
  Integer limited0;
  Real boundedAcceleration0;
  Real accelerationStep0;
  Real initialSpeed0;
  Real nextSpeed0;
  Real speedSum0;
  Real speedDelta0;

  limited0 = 0;
  if (modelValid) {
    boundedAcceleration0 = vehicleDynamicStepClamp(input->AccelerationCmd,
                                                   param->AccelerationLowerLimit,
                                                   param->AccelerationUpperLimit,
                                                   &limited0);
    accelerationStep0 = boundedAcceleration0 * param->CycleTime;
    initialSpeed0 = vehicleDynamicStepInitialSpeed(state->VehicleSpeed, input->GearPos, &limited0);
    nextSpeed0 = vehicleDynamicStepNextSpeed(
        initialSpeed0, accelerationStep0, input->GearPos, param, &limited0);
    speedSum0 = initialSpeed0 + nextSpeed0;
    speedDelta0 = nextSpeed0 - initialSpeed0;
    output->TravelDistance = 0.5f * speedSum0 * param->CycleTime;
    output->VehicleAcceleration = speedDelta0 / param->CycleTime;
    output->VehicleSpeed = nextSpeed0;
    output->DirectionSign = 0;
    if (input->GearPos == APA_GEAR_POS_D) {
      output->DirectionSign = 1;
    } else if (input->GearPos == APA_GEAR_POS_R) {
      output->DirectionSign = -1;
    }
  } else {
    output->TravelDistance = 0.0f;
    output->VehicleAcceleration = 0.0f;
    output->VehicleSpeed = 0.0f;
    output->DirectionSign = 0;
    limited0 = 1;
  }
  output->SpeedLimited = limited0;
  output->ModelValid = (Integer)modelValid;
}

/**
 * @brief 将有符号加速度和前轮转角积分为车辆速度、位移和位姿
 * @en_name vehicleDynamicStep
 * @cn_name 车辆动力学单步更新
 * @type widget_module
 * @config cn_name=轴距, en_name=WheelBase, desc=前后轴中心距离, unit=m, type=Real, range=[0.1,10.0], default=3.00465
 * @config cn_name=小转角近似阈值, en_name=StraightSteerThreshold, desc=半步航向增量的级数近似阈值, unit=rad, type=Real, range=[0.000001,0.1], default=0.0001
 * @config cn_name=仿真周期, en_name=CycleTime, desc=车辆模型离散仿真周期, unit=s, type=Real, range=[0.001,1.0], default=0.02
 * @config cn_name=前进速度上限, en_name=ForwardSpeedLimit, desc=D档正速度上限, unit=m/s, type=Real, range=[0,20], default=2.0
 * @config cn_name=倒车速度上限, en_name=ReverseSpeedLimit, desc=R档负速度绝对值上限, unit=m/s, type=Real, range=[0,20], default=2.0
 * @config cn_name=加速度上限, en_name=AccelerationUpperLimit, desc=车体前向加速度上限, unit=m/s2, type=Real, range=[0,10], default=1.2
 * @config cn_name=加速度下限, en_name=AccelerationLowerLimit, desc=车体前向加速度下限, unit=m/s2, type=Real, range=[-10,0], default=-1.5
 * @param[IN] Real VehicleX 当前车辆X位置
 * @param[IN] Real VehicleY 当前车辆Y位置
 * @param[IN] Real VehicleYaw 当前车辆航向角
 * @param[IN] Real AccelerationCmd 车体前向有符号加速度指令
 * @param[IN] Real SteerAngle 前轮转角指令
 * @param[IN] Integer GearPos 实际档位
 * @param[OUT] Real VehicleX 更新后的车辆X位置
 * @param[OUT] Real VehicleY 更新后的车辆Y位置
 * @param[OUT] Real VehicleYaw 更新后的车辆航向角
 * @param[OUT] Real VehicleSpeed 车体前向有符号实际速度
 * @param[OUT] Real VehicleAcceleration 车体前向有符号实际加速度
 * @param[OUT] Real TravelDistance 本周期有符号行驶距离
 * @param[OUT] Integer DirectionSign 实际档位方向正负号
 * @param[OUT] Integer SpeedLimited 纵向约束介入标志
 * @param[OUT] Integer ModelValid 当前车辆模型上下文有效标志
 * @var Real VehicleSpeed 有符号车速
 * @var Real VehicleAcceleration 有符号实际加速度
 * @granularity atomic
 * @tag_level1 车辆模型
 * @tag_level2 闭环仿真
 * @source matlab_rewrite VehicleDynamic.m + engineering_spec docs/CARLA_ACCELERATION_CLOSED_LOOP_INTERFACE.md
 * @formula \f[v_{k+1}=sat_{gear}(v_k+sat(a_k)T_s),\quad s_k=(v_k+v_{k+1})T_s/2\f]
 * @version 1.3
 * @date 2026-11-07
 * @author aide
 */
void vehicleDynamicStep(const VehicleDynamicStepInput_t *input,
                        const VehicleDynamicStepParam_t *param,
                        VehicleDynamicStepState_t *state,
                        VehicleDynamicStepOutput_t *output) {
  const Boolean valid = vehicleDynamicStepAccelerationContextIsValid(input, param, state);
  vehicleDynamicStepCalculateLongitudinal(input, param, state, valid, output);
  double x = isfinite(input->VehicleX) ? input->VehicleX : 0.0;
  double y = isfinite(input->VehicleY) ? input->VehicleY : 0.0;
  double yaw = isfinite(input->VehicleYaw) ? input->VehicleYaw : 0.0;
  if (valid) {

    const double distance = output->TravelDistance;
    const double halfTurn = 0.5 * distance * tan((double)input->SteerAngle) / param->WheelBase;
    const double scale = fabs(halfTurn) < param->StraightSteerThreshold
                             ? 1.0 - halfTurn * halfTurn / 6.0
                             : sin(halfTurn) / halfTurn;
    x += distance * scale * cos(yaw + halfTurn);
    y += distance * scale * sin(yaw + halfTurn);
    yaw += 2.0 * halfTurn;
  }
  output->VehicleX = (Real)x;
  output->VehicleY = (Real)y;
  output->VehicleYaw = apaCoordWrapToPi((Real)yaw);
  state->VehicleSpeed = output->VehicleSpeed;
  state->VehicleAcceleration = output->VehicleAcceleration;
}
