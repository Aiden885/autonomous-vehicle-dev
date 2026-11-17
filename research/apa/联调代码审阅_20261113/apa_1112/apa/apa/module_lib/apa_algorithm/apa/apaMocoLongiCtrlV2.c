#include "apaEnums.h"
#include "apaMathUtils.h"
#include "apaMocoLongiCtrlV2Types.h"
#include "apaMocoExecution.h"
#include "apaMocoActuator.h"
#include <math.h>

#define APA_MOCO_LONGI_V2_PHASE_CRUISE 1
#define APA_MOCO_LONGI_V2_PHASE_DECEL 2
#define APA_MOCO_LONGI_V2_PHASE_TERMINAL_CREEP 3
#define APA_MOCO_LONGI_V2_PHASE_STOP_HOLD 4

static Real apaMocoLongiCtrlV2BaseSpeed(const ApaMocoLongiCtrlV2Input_t *input,
                                        const ApaMocoLongiCtrlV2Param_t *param) {
  Real speed0;

  if (input->ApaGoDir < 0) {
    speed0 = param->ReverseRefSpeed;
  } else {
    speed0 = param->ForwardRefSpeed;
  }

  return speed0;
}

static Real apaMocoLongiCtrlV2MinTrackingSpeed(Integer parkingCate,
                                               const ApaMocoLongiCtrlV2Param_t *param) {
  Real speed0;

  if (parkingCate == 2) {
    speed0 = param->ParallelMinTrackingSpeed;
  } else if (parkingCate == 3) {
    speed0 = param->DiagonalMinTrackingSpeed;
  } else if (parkingCate == 4) {
    speed0 = param->HeadInMinTrackingSpeed;
  } else {
    speed0 = param->VerticalMinTrackingSpeed;
  }

  return speed0;
}

static Real apaMocoLongiCtrlV2TerminalSpeed(Integer parkingCate,
                                            const ApaMocoLongiCtrlV2Param_t *param) {
  Real speed0;

  if (parkingCate == 2) {
    speed0 = param->ParallelTerminalKeepSpeed;
  } else if (parkingCate == 4) {
    speed0 = param->HeadInMinTrackingSpeed;
  } else {
    speed0 = param->TerminalKeepSpeed;
  }

  return speed0;
}

static Boolean apaMocoLongiCtrlV2IsCusp(const ApaMocoLongiCtrlV2Input_t *input) {
  return input->ApaStpNow > 0 && input->ApaStpNow < input->ApaStpNum;
}

static Real apaMocoLongiCtrlV2ApproachSpeed(const ApaMocoLongiCtrlV2Input_t *input,
                                            const ApaMocoLongiCtrlV2Param_t *param) {
  return apaMocoLongiCtrlV2IsCusp(input)
             ? param->CuspApproachSpeed
             : apaMocoLongiCtrlV2TerminalSpeed(input->ParkingCate, param);
}

static Boolean apaMocoLongiCtrlV2AccelParamIsFinite(const ApaMocoLongiCtrlV2Param_t *param) {
  Boolean finite0;

  finite0 = isfinite(param->CycleTime) && isfinite(param->KpSpeed) && isfinite(param->KiSpeed) &&
            isfinite(param->AccelerationFeedbackGain) && isfinite(param->IntegralUpperLimit) &&
            isfinite(param->IntegralLowerLimit) && isfinite(param->AccelerationUpperLimit) &&
            isfinite(param->AccelerationLowerLimit) && isfinite(param->JerkUpperLimit) &&
            isfinite(param->JerkLowerLimit) && isfinite(param->StopDeceleration) &&
            isfinite(param->CuspApproachSpeed) && isfinite(param->ShortSegmentLength) &&
            isfinite(param->ShortSegmentSpeed) && isfinite(param->TerminalDistThd) &&
            isfinite(param->ParallelTerminalDistThd) && isfinite(param->SlowDistThd) &&
            isfinite(param->StandstillSpeedThreshold);

  return finite0;
}

static Boolean apaMocoLongiCtrlV2AccelParamRangeIsValid(const ApaMocoLongiCtrlV2Param_t *param) {
  Boolean condLimits0;
  Boolean condZeroInside0;
  Boolean condGain0;
  Boolean valid0;

  condLimits0 =
      (param->CycleTime > 0.0f) && (param->IntegralUpperLimit >= param->IntegralLowerLimit) &&
      (param->AccelerationUpperLimit >= param->AccelerationLowerLimit) &&
      (param->JerkUpperLimit >= param->JerkLowerLimit) && (param->StopDeceleration >= 0.0f) &&
      (param->CuspApproachSpeed > param->StandstillSpeedThreshold) &&
      (param->CuspApproachSpeed <= 0.30f) && (param->ShortSegmentLength > 0.0f) &&
      (param->ShortSegmentLength <= 5.0f) &&
      (param->ShortSegmentSpeed >= param->CuspApproachSpeed) &&
      (param->ShortSegmentSpeed <= 0.60f) && (param->TerminalDistThd >= 0.0f) &&
      (param->ParallelTerminalDistThd >= 0.0f) && (param->SlowDistThd > 0.0f) &&
      (param->StandstillSpeedThreshold >= 0.0f);
  condZeroInside0 = (param->IntegralLowerLimit <= 0.0f) && (param->IntegralUpperLimit >= 0.0f) &&
                    (param->AccelerationLowerLimit <= 0.0f) &&
                    (param->AccelerationUpperLimit >= 0.0f) && (param->JerkLowerLimit <= 0.0f) &&
                    (param->JerkUpperLimit >= 0.0f);
  condGain0 = (param->KpSpeed >= 0.0f) && (param->KiSpeed >= 0.0f) &&
              (param->AccelerationFeedbackGain >= 0.0f);
  valid0 = condLimits0 && condZeroInside0 && condGain0;

  return valid0;
}

static Boolean apaMocoLongiCtrlV2AccelContextIsValid(const ApaMocoLongiCtrlV2Input_t *input,
                                                     const ApaMocoLongiCtrlV2Param_t *param,
                                                     const ApaMocoLongiCtrlV2State_t *state) {
  Boolean inputValid0;
  Boolean paramFinite0;
  Boolean paramRangeValid0;
  Boolean stateValid0;
  Boolean valid0;

  inputValid0 = isfinite(input->VehicleSpeed) && isfinite(input->VehicleAcceleration) &&
                isfinite(input->BrakeDistance) && input->BrakeDistance >= 0.0f &&
                isfinite(input->SegmentLength) && input->SegmentLength >= 0.0f;
  paramFinite0 = apaMocoLongiCtrlV2AccelParamIsFinite(param);
  paramRangeValid0 = apaMocoLongiCtrlV2AccelParamRangeIsValid(param);
  stateValid0 = isfinite(state->IntegralError) && isfinite(state->LastAcceleration);
  valid0 = inputValid0 && paramFinite0 && paramRangeValid0 && stateValid0;

  return valid0;
}

static Integer apaMocoLongiCtrlV2DirectionSign(Integer gearPos) {
  Integer directionSign0;

  directionSign0 = 0;
  if (gearPos == APA_GEAR_POS_D) {
    directionSign0 = 1;
  } else if (gearPos == APA_GEAR_POS_R) {
    directionSign0 = -1;
  }

  return directionSign0;
}

static Boolean apaMocoLongiCtrlV2DirectionIsConsistent(Real targetSpeed,
                                                       Integer targetGearReq,
                                                       Real standstillThreshold) {
  Boolean consistent0;

  consistent0 = false;
  if (targetGearReq == APA_GEAR_POS_D) {
    consistent0 = (targetSpeed >= -standstillThreshold);
  } else if (targetGearReq == APA_GEAR_POS_R) {
    consistent0 = (targetSpeed <= standstillThreshold);
  }

  return consistent0;
}

static Boolean apaMocoLongiCtrlV2AccelRunIsAllowed(const ApaMocoLongiCtrlV2Input_t *input,
                                                   const ApaMocoLongiCtrlV2Param_t *param,
                                                   Real targetSpeed,
                                                   Integer stopReq,
                                                   Boolean contextValid,
                                                   Integer directionSign) {
  Boolean phaseRun0;
  Boolean stopClear0;
  Boolean gearMatched0;
  Boolean directionConsistent0;
  Boolean allowed0;

  phaseRun0 = apaMocoMotionPhase(input->CtrlPhase) || input->CtrlPhase == APA_MOCO_STOP_WAIT;
  stopClear0 = (stopReq == 0);
  gearMatched0 = (input->GearPos == input->TargetGearReq);
  directionConsistent0 = apaMocoLongiCtrlV2DirectionIsConsistent(
      targetSpeed, input->TargetGearReq, param->StandstillSpeedThreshold);
  allowed0 = contextValid && phaseRun0 && stopClear0 && gearMatched0 && directionConsistent0 &&
             (directionSign != 0);

  return allowed0;
}

static Real apaMocoLongiCtrlV2NormalizeTargetSpeed(Real targetSpeed, Real standstillThreshold) {
  Real normalizedSpeed0;

  normalizedSpeed0 = targetSpeed;
  if (fabsf(targetSpeed) <= standstillThreshold) {
    normalizedSpeed0 = 0.0f;
  }

  return normalizedSpeed0;
}

static Real apaMocoLongiCtrlV2RunAcceleration(const ApaMocoLongiCtrlV2Input_t *input,
                                              const ApaMocoLongiCtrlV2Param_t *param,
                                              const ApaMocoLongiCtrlV2State_t *state,
                                              Real targetSpeed,
                                              Real *integralError) {
  Real speedError0;
  Real integralCandidate0;
  Real proportionalTerm0;
  Real integralTerm0;
  Real feedbackTerm0;
  Real requestedAcceleration0;

  speedError0 = targetSpeed - input->VehicleSpeed;
  integralCandidate0 = state->IntegralError + (speedError0 * param->CycleTime);
  *integralError =
      apaMathClamp(integralCandidate0, param->IntegralLowerLimit, param->IntegralUpperLimit);
  proportionalTerm0 = param->KpSpeed * speedError0;
  integralTerm0 = param->KiSpeed * (*integralError);
  feedbackTerm0 = param->AccelerationFeedbackGain * input->VehicleAcceleration;
  requestedAcceleration0 = proportionalTerm0 + integralTerm0 - feedbackTerm0;
  if ((requestedAcceleration0 > param->AccelerationUpperLimit && speedError0 > 0.0f) ||
      (requestedAcceleration0 < param->AccelerationLowerLimit && speedError0 < 0.0f)) {
    *integralError = state->IntegralError;
    requestedAcceleration0 = proportionalTerm0 + param->KiSpeed * (*integralError) - feedbackTerm0;
  }

  return requestedAcceleration0;
}

static Real apaMocoLongiCtrlV2StopAcceleration(Real vehicleSpeed,
                                               const ApaMocoLongiCtrlV2Param_t *param) {
  Real acceleration0;

  acceleration0 = 0.0f;
  if (vehicleSpeed > param->StandstillSpeedThreshold) {
    acceleration0 = -param->StopDeceleration;
  } else if (vehicleSpeed < -param->StandstillSpeedThreshold) {
    acceleration0 = param->StopDeceleration;
  }

  return acceleration0;
}

static Real apaMocoLongiCtrlV2LimitAcceleration(Real requestedAcceleration,
                                                const ApaMocoLongiCtrlV2Param_t *param,
                                                const ApaMocoLongiCtrlV2State_t *state,
                                                Integer *saturated) {
  Real jerkLower0;
  Real jerkUpper0;
  Real jerkLimited0;
  Real accelerationLimited0;

  jerkLower0 = state->LastAcceleration + (param->JerkLowerLimit * param->CycleTime);
  jerkUpper0 = state->LastAcceleration + (param->JerkUpperLimit * param->CycleTime);
  jerkLimited0 = apaMathClamp(requestedAcceleration, jerkLower0, jerkUpper0);
  accelerationLimited0 =
      apaMathClamp(jerkLimited0, param->AccelerationLowerLimit, param->AccelerationUpperLimit);
  if ((jerkLimited0 != requestedAcceleration) || (accelerationLimited0 != jerkLimited0)) {
    *saturated = 1;
  }

  return accelerationLimited0;
}

static Real apaMocoLongiCtrlV2GuardStopAcceleration(Real vehicleSpeed,
                                                    Real acceleration,
                                                    Real standstillThreshold,
                                                    Integer *saturated) {
  Boolean condStandstill0;
  Boolean condWrongDirection0;
  Real guardedAcceleration0;

  condStandstill0 = (fabsf(vehicleSpeed) <= standstillThreshold);
  condWrongDirection0 = ((vehicleSpeed > standstillThreshold) && (acceleration > 0.0f)) ||
                        ((vehicleSpeed < -standstillThreshold) && (acceleration < 0.0f));
  guardedAcceleration0 = acceleration;
  if (condStandstill0) {
    guardedAcceleration0 = 0.0f;
  } else if (condWrongDirection0) {
    guardedAcceleration0 = 0.0f;
    *saturated = 1;
  }

  return guardedAcceleration0;
}

static void apaMocoLongiCtrlV2ApplyAccelerationControl(const ApaMocoLongiCtrlV2Input_t *input,
                                                       const ApaMocoLongiCtrlV2Param_t *param,
                                                       ApaMocoLongiCtrlV2State_t *state,
                                                       ApaMocoLongiCtrlV2Output_t *output) {
  Boolean contextValid0;
  Boolean runAllowed0;
  Boolean standstill0;
  Integer directionSign0;
  Integer saturated0;
  Real targetSpeed0;
  Real integralError0;
  Real requestedAcceleration0;
  Real unlimitedAcceleration0;
  Real targetAcceleration0;

  output->TargetAcceleration = 0.0f;
  output->DirectionSign = 0;
  output->TargetGearReq = APA_GEAR_POS_N;
  output->ControlEnabled = 0;
  output->GearChangeAllowed = 0;
  output->AccelerationSaturated = 0;
  contextValid0 = apaMocoLongiCtrlV2AccelContextIsValid(input, param, state);
  directionSign0 = apaMocoLongiCtrlV2DirectionSign(input->TargetGearReq);
  runAllowed0 = apaMocoLongiCtrlV2AccelRunIsAllowed(
      input, param, output->TargetSpeed, output->StopReq, contextValid0, directionSign0);
  standstill0 = contextValid0 && (fabsf(input->VehicleSpeed) <= param->StandstillSpeedThreshold);
  if (contextValid0 && (directionSign0 != 0)) {
    output->TargetGearReq = input->TargetGearReq;
    output->DirectionSign = directionSign0;
  }
  if (standstill0 && (directionSign0 != 0)) {
    output->GearChangeAllowed = 1;
  }
  targetSpeed0 = 0.0f;
  integralError0 = 0.0f;
  requestedAcceleration0 = 0.0f;
  if (runAllowed0) {
    targetSpeed0 = apaMocoLongiCtrlV2NormalizeTargetSpeed(output->TargetSpeed,
                                                          param->StandstillSpeedThreshold);
    requestedAcceleration0 =
        apaMocoLongiCtrlV2RunAcceleration(input, param, state, targetSpeed0, &integralError0);
    if (directionSign0 * input->VehicleSpeed < APA_MOCO_LAUNCH_SPEED_MPS) {
      requestedAcceleration0 = directionSign0 * fminf(directionSign0 * requestedAcceleration0,
                                                      APA_MOCO_LAUNCH_ACCEL_MPS2);
      integralError0 = 0.0f;
    }
    if (directionSign0 * input->VehicleSpeed > fabsf(targetSpeed0) + 0.03f) {
      requestedAcceleration0 =
          directionSign0 * fminf(0.0f, directionSign0 * requestedAcceleration0);
      integralError0 = 0.0f;
    }
    if (input->CtrlPhase == APA_MOCO_STOP_WAIT) {
      requestedAcceleration0 = apaMocoLongiCtrlV2StopAcceleration(input->VehicleSpeed, param);
      integralError0 = 0.0f;
    }
    output->ControlEnabled = 1;
  } else if (contextValid0) {
    requestedAcceleration0 = apaMocoLongiCtrlV2StopAcceleration(input->VehicleSpeed, param);
    output->StopReq = 1;
  }

  unlimitedAcceleration0 = requestedAcceleration0;
  saturated0 = 0;
  if (runAllowed0) {

    if (directionSign0 * requestedAcceleration0 < -param->StopDeceleration) {
      requestedAcceleration0 = -directionSign0 * param->StopDeceleration;
      saturated0 = 1;
    }
  }
  if (contextValid0) {
    targetAcceleration0 =
        apaMocoLongiCtrlV2LimitAcceleration(requestedAcceleration0, param, state, &saturated0);

    if (directionSign0 != 0 && directionSign0 * requestedAcceleration0 <= 0.0f &&
        directionSign0 * targetAcceleration0 > 0.0f) {
      targetAcceleration0 = 0.0f;
      saturated0 = 1;
    }
    if (!runAllowed0 || input->CtrlPhase == APA_MOCO_STOP_WAIT) {
      targetAcceleration0 = apaMocoLongiCtrlV2GuardStopAcceleration(
          input->VehicleSpeed, targetAcceleration0, param->StandstillSpeedThreshold, &saturated0);
    }
  } else {
    targetAcceleration0 = 0.0f;
    saturated0 = 1;
    output->StopReq = 1;
  }
  output->TargetSpeed = targetSpeed0;
  output->TargetAcceleration = targetAcceleration0;
  output->AccelerationSaturated = saturated0;
  if (runAllowed0 && saturated0 &&
      (unlimitedAcceleration0 - targetAcceleration0) * (integralError0 - state->IntegralError) >
          0.0f) {

    integralError0 = state->IntegralError;
  }
  state->IntegralError = integralError0;
  state->LastAcceleration = targetAcceleration0;
}

/**
 * @brief 调度有符号目标速度，并进行速度到加速度的纵向闭环控制
 * @en_name apaMocoLongiCtrlV2
 * @cn_name MoCo v2纵向控制
 * @type widget_module
 * @config cn_name=前进参考速度, en_name=ForwardRefSpeed,
 * desc=前进段基础速度, unit=m/s, type=Real, range=[0,5], default=0.556
 * @config cn_name=倒车参考速度, en_name=ReverseRefSpeed,
 * desc=倒车段基础速度, unit=m/s, type=Real, range=[-5,0], default=-0.444
 * @config cn_name=终端保持速度, en_name=TerminalKeepSpeed,
 * desc=非平行车位终端低速, unit=m/s, type=Real, range=[0,0.5], default=0.08
 * @config cn_name=平行终端保持速度, en_name=ParallelTerminalKeepSpeed,
 * desc=平行车位终端低速, unit=m/s, type=Real, range=[0,0.5], default=0.04
 * @config cn_name=换向接近速度, en_name=CuspApproachSpeed,
 * desc=中间换向点接近速度且不超过该段基础速度, unit=m/s, type=Real, range=(0.02,0.3], default=0.10
 * @config cn_name=短段长度, en_name=ShortSegmentLength,
 * desc=使用短换向段行驶速度的完整弧长上限, unit=m, type=Real, range=(0,5], default=1.0
 * @config cn_name=短段行驶速度, en_name=ShortSegmentSpeed,
 * desc=短换向段的行驶速度上限且不低于换向接近速度, unit=m/s, type=Real, range=(0.02,0.6], default=0.10
 * @config cn_name=终端距离阈值, en_name=TerminalDistThd,
 * desc=终端低速距离阈值, unit=m, type=Real, range=[0,2], default=0.18
 * @config cn_name=平行终端距离阈值, en_name=ParallelTerminalDistThd,
 * desc=平行车位终端低速距离阈值, unit=m, type=Real, range=[0,2], default=0.12
 * @config cn_name=终端航向阈值, en_name=TerminalYawTol,
 * desc=终端航向误差阈值, unit=rad, type=Real, range=[0,1], default=0.035
 * @config cn_name=停车距离阈值, en_name=StopDistThd,
 * desc=保留的停车距离接口，停车由控制状态触发, unit=m, type=Real, range=[0,1], default=0.04
 * @config cn_name=降速距离阈值, en_name=SlowDistThd,
 * desc=每段段末降速距离阈值, unit=m, type=Real, range=[0.1,5], default=1.5
 * @config cn_name=控制周期, en_name=CycleTime,
 * desc=加速度闭环离散控制周期, unit=s, type=Real, range=[0.001,1], default=0.02
 * @config cn_name=速度比例增益, en_name=KpSpeed,
 * desc=速度误差比例增益, unit=1/s, type=Real, range=[0,20], default=1.5
 * @config cn_name=速度积分增益, en_name=KiSpeed,
 * desc=速度误差积分增益，执行端积分补偿时为0, unit=1/s2, type=Real, range=[0,20], default=0
 * @config cn_name=加速度反馈增益, en_name=AccelerationFeedbackGain,
 * desc=实际加速度反馈阻尼增益，执行端闭环时为0, unit=1, type=Real, range=[0,5], default=0
 * @config cn_name=车身误差降速起点, en_name=TrackingErrorSoft,
 * desc=车身横偏与航向角点误差开始降速的阈值, unit=m, type=Real, range=[0,0.2], default=0.02
 * @config cn_name=车身误差降速终点, en_name=TrackingErrorHard,
 * desc=达到纠偏巡航速度的误差阈值且大于起点, unit=m, type=Real, range=[0,0.5], default=0.08
 * @config cn_name=纠偏巡航速度, en_name=TrackingMinSpeed,
 * desc=误差降速下限，段末减速及停车仍独立生效, unit=m/s, type=Real, range=[0,1], default=0.30
 * @config cn_name=车身角点半径, en_name=TrackingCornerRadius,
 * desc=后轴到最远车身角点的距离，由车辆尺寸初始化, unit=m, type=Real, range=[0,10], default=0
 * @config cn_name=积分上限, en_name=IntegralUpperLimit,
 * desc=速度误差积分上限, unit=m, type=Real, range=[0,20], default=3
 * @config cn_name=积分下限, en_name=IntegralLowerLimit,
 * desc=速度误差积分下限, unit=m, type=Real, range=[-20,0], default=-3
 * @config cn_name=加速度上限, en_name=AccelerationUpperLimit,
 * desc=车体前向有符号加速度上限, unit=m/s2, type=Real, range=[0,10], default=1.2
 * @config cn_name=加速度下限, en_name=AccelerationLowerLimit,
 * desc=车体前向有符号加速度下限, unit=m/s2, type=Real, range=[-10,0], default=-1.5
 * @config cn_name=加加速度上限, en_name=JerkUpperLimit,
 * desc=目标加速度上升斜率限制, unit=m/s3, type=Real, range=[0,20], default=3
 * @config cn_name=加加速度下限, en_name=JerkLowerLimit,
 * desc=目标加速度下降斜率限制, unit=m/s3, type=Real, range=[-20,0], default=-4
 * @config cn_name=停车减速度, en_name=StopDeceleration,
 * desc=停车时与当前运动方向相反的减速度幅值, unit=m/s2, type=Real, range=[0,10], default=1.2
 * @config cn_name=静止速度阈值, en_name=StandstillSpeedThreshold,
 * desc=目标速度死区与换挡许可阈值, unit=m/s, type=Real, range=[0,1], default=0.01
 * @param[IN] Integer CtrlPhase 控制阶段
 * @param[IN] Integer ApaStpNow 当前段1-based
 * @param[IN] Integer ApaStpNum 总段数
 * @param[IN] Integer ApaGoDir 当前段方向
 * @param[IN] Real Dist2Go 段内路径剩余距离
 * @param[IN] Real BrakeDistance 实测运动及响应延迟对应的停车距离
 * @param[IN] Real SegmentLength 当前活动段完整弧长
 * @param[IN] Real LongitudinalError 预瞄点纵向误差
 * @param[IN] Real GoalDistance 段终点欧氏距离
 * @param[IN] Real TerminalYawError 终点航向误差
 * @param[IN] Real HeadingError 参考航向误差
 * @param[IN] Real LateralError 横向误差
 * @param[IN] Real CrossTrackError 投影点横偏，不包含预瞄曲线的名义横向坐标
 * @param[IN] Integer TerminalMode 终端模式
 * @param[IN] Integer ParkingCate 泊车类型
 * @param[IN] Integer DecelerationReq 未使用
 * @param[IN] Real VehicleSpeed 车体前向有符号实际速度
 * @param[IN] Real VehicleAcceleration 车体前向有符号实际加速度
 * @param[IN] Integer TargetGearReq 目标档位请求
 * @param[IN] Integer GearPos 实际档位
 * @param[OUT] Real TargetSpeed 目标速度
 * @param[OUT] Real Dist2GoCmd 纵向距离命令
 * @param[OUT] Integer LongiBlockFlag 阻塞标志
 * @param[OUT] Integer StopReq 停车请求
 * @param[OUT] Integer LongiPhase 纵向相位
 * @param[OUT] Real SpeedScale 基础速度缩放
 * @param[OUT] Integer TerminalCreepActive 终端低速标志
 * @param[OUT] Integer StopReason 停车原因
 * @param[OUT] Real TargetAcceleration 车体前向有符号目标加速度
 * @param[OUT] Integer DirectionSign 目标档位方向正负号
 * @param[OUT] Integer TargetGearReq 经验证的目标档位请求
 * @param[OUT] Integer ControlEnabled 加速度闭环驱动使能
 * @param[OUT] Integer GearChangeAllowed 静止换挡许可
 * @param[OUT] Integer AccelerationSaturated 加速度或加加速度限幅标志
 * @var Real LastSpeed 上周期速度
 * @var Integer TerminalLatch 最后一段终端捕获锁存
 * @var Real IntegralError 速度误差积分
 * @var Real LastAcceleration 上周期目标加速度
 * @granularity atomic
 * @tag_level1 泊车控制
 * @tag_level2 MoCo
 * @source slx_stateflow LongiCtrl_Pre1 LongiCtrl_mod1 StrSpdUp + engineering_spec docs/CARLA_ACCELERATION_CLOSED_LOOP_INTERFACE.md
 * @formula \f[a_k=sat_{a,j}(K_p(v_k^*-v_k)+K_i\int(v^*-v)dt-K_a a_k^{act})\f]
 * @version 1.2
 * @date 2026-09-01
 * @author aide
 */
void apaMocoLongiCtrlV2(const ApaMocoLongiCtrlV2Input_t *input,
                        const ApaMocoLongiCtrlV2Param_t *param,
                        ApaMocoLongiCtrlV2State_t *state,
                        ApaMocoLongiCtrlV2Output_t *output) {
  Real cruise = apaMocoLongiCtrlV2BaseSpeed(input, param);
  Real base = cruise;
  Real scale = 1.0f;
  Boolean cusp = apaMocoLongiCtrlV2IsCusp(input);
  Boolean movingPhase = apaMocoMotionPhase(input->CtrlPhase);
  if (cusp && input->SegmentLength > 0.0f &&
      input->SegmentLength <= param->ShortSegmentLength + 1.0e-4f) {
    base = (input->ApaGoDir < 0 ? -1.0f : 1.0f) * fminf(fabsf(base), param->ShortSegmentSpeed);
  }
  *output = (ApaMocoLongiCtrlV2Output_t){0};
  output->Dist2GoCmd = input->Dist2Go;
  output->LongiPhase = APA_MOCO_LONGI_V2_PHASE_STOP_HOLD;
  output->StopReq = 1;
  if (movingPhase) {
    Real slow = fmaxf(param->SlowDistThd, 0.01f);
    Real creepDistance =
        input->ParkingCate == 2 ? param->ParallelTerminalDistThd : param->TerminalDistThd;
    if (cusp && input->SegmentLength > 0.0f)
      creepDistance = fminf(creepDistance, fmaxf(0.08f, 0.25f * input->SegmentLength));
    if (cusp && input->SegmentLength > 0.0f)
      slow = fminf(slow, fmaxf(creepDistance + 0.01f, 0.5f * input->SegmentLength));
    Real minimum = apaMocoLongiCtrlV2IsCusp(input)
                       ? param->CuspApproachSpeed
                       : apaMocoLongiCtrlV2MinTrackingSpeed(input->ParkingCate, param);

    Real approach = fminf(fabsf(base), apaMocoLongiCtrlV2ApproachSpeed(input, param));
    Real fraction = apaMathClamp(
        (input->Dist2Go - creepDistance) / fmaxf(slow - creepDistance, 0.01f), 0.0f, 1.0f);
    Real distanceSpeed = approach + (fabsf(base) - approach) * fraction;
    scale = distanceSpeed / fmaxf(fabsf(base), 0.001f);

    if (isfinite(input->CrossTrackError) && isfinite(input->HeadingError) &&
        param->TrackingErrorHard > param->TrackingErrorSoft && param->TrackingErrorSoft >= 0.0f &&
        isfinite(param->TrackingMinSpeed) && param->TrackingMinSpeed >= 0.0f &&
        isfinite(param->TrackingCornerRadius) && param->TrackingCornerRadius > 0.0f) {
      const Real bodyError = fabsf(input->CrossTrackError) +
                             param->TrackingCornerRadius * fabsf(sinf(input->HeadingError));
      const Real severity = apaMathClamp((bodyError - param->TrackingErrorSoft) /
                                             (param->TrackingErrorHard - param->TrackingErrorSoft),
                                         0.0f,
                                         1.0f);
      const Real trackingFloor = fminf(fabsf(cruise), fmaxf(minimum, param->TrackingMinSpeed));
      const Real trackingCap = fabsf(cruise) + (trackingFloor - fabsf(cruise)) * severity;
      scale = fminf(scale, trackingCap / fmaxf(fabsf(base), 0.001f));
    }
    output->TargetSpeed = base * fminf(scale, 1.0f);
    output->LongiPhase = input->CtrlPhase == APA_MOCO_TRACK ? APA_MOCO_LONGI_V2_PHASE_CRUISE
                                                            : APA_MOCO_LONGI_V2_PHASE_DECEL;
    output->TerminalCreepActive = input->Dist2Go <= creepDistance;
    output->StopReq = 0;
  } else if (input->CtrlPhase == APA_MOCO_STOP_WAIT) {

    output->StopReq = fabsf(input->VehicleSpeed) <= param->StandstillSpeedThreshold;
    output->StopReason = 1;
  } else if (input->CtrlPhase == APA_MOCO_DONE)
    output->StopReason = 2;
  output->TargetSpeed =
      apaMathClamp(output->TargetSpeed, param->SpeedLowerLimit, param->SpeedUpperLimit);
  output->SpeedScale = fabsf(cruise) > 0.001f ? fabsf(output->TargetSpeed / cruise) : 0.0f;
  state->LastSpeed = output->TargetSpeed;
  state->TerminalLatch = output->TerminalCreepActive;
  apaMocoLongiCtrlV2ApplyAccelerationControl(input, param, state, output);
}
