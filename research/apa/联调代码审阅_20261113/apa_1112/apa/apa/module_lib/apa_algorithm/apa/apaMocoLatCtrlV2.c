#include "apaMathUtils.h"
#include "apaMocoExecution.h"
#include "apaMocoLatCtrlV2Types.h"
#include <math.h>

#ifndef APA_MOCO_LAT_V2_EPS
#define APA_MOCO_LAT_V2_EPS 1.0e-6f
#endif

static Real
apaMocoLatCtrlV2RateLimit(Real value, Real lastValue, Real rateLimit, Integer *saturated) {
  Real value0;
  Real delta0;

  value0 = value;
  if (rateLimit > APA_MOCO_LAT_V2_EPS) {
    delta0 = value0 - lastValue;
    if (delta0 > rateLimit) {
      value0 = lastValue + rateLimit;
      *saturated = 1;
    } else if (delta0 < -rateLimit) {
      value0 = lastValue - rateLimit;
      *saturated = 1;
    }
  }

  return value0;
}

static Real apaMocoLatCtrlV2Lookahead(const ApaMocoLatCtrlV2Input_t *input,
                                      const ApaMocoLatCtrlV2Param_t *param) {
  return fmaxf(fmaxf(0.4f, 0.2f * param->Wheelbase), input->FeedbackDistance);
}

static Real apaMocoLatCtrlV2BaseCurvature(const ApaMocoLatCtrlV2Input_t *input,
                                          const ApaMocoLatCtrlV2Param_t *param,
                                          Real lookahead,
                                          Integer *allowEy) {
  Real error = input->CrossTrackError;
  *allowEy = fabsf(error) >= param->AllowEyThreshold;
  if (!*allowEy)
    error = 0.0f;
  return input->RefCurvature + fmaxf(param->Kpp, 0.0f) * error / (lookahead * lookahead);
}

static Real apaMocoLatCtrlV2SlipAngle(const ApaMocoLatCtrlV2Input_t *input) {
  const Real direction = input->ApaGoDir < 0 ? -1.0f : 1.0f;
  if (!isfinite(input->VehicleSpeed) || !isfinite(input->VehicleLateralSpeed) ||
      input->VehicleSpeed * direction <= 0.10f) {
    return 0.0f;
  }
  const Real speed = hypotf(input->VehicleSpeed, input->VehicleLateralSpeed);
  if (speed <= 0.15f)
    return 0.0f;
  const Real weight = apaMathClamp((speed - 0.15f) / 0.15f, 0.0f, 1.0f);
  const Real rawSlip = atanf(input->VehicleLateralSpeed / input->VehicleSpeed);
  const Real deadbanded = copysignf(fmaxf(0.0f, fabsf(rawSlip) - 0.003f), rawSlip);
  return apaMathClamp(weight * deadbanded, -0.08f, 0.08f);
}

static Real apaMocoLatCtrlV2HeadingCurvature(const ApaMocoLatCtrlV2Input_t *input,
                                             const ApaMocoLatCtrlV2Param_t *param,
                                             Real lookahead,
                                             Real lateralWeight) {
  Real error = input->HeadingError - lateralWeight * apaMocoLatCtrlV2SlipAngle(input);
  const Real deadband = fmaxf(param->HeadingDeadband, 0.0f);
  if (fabsf(error) < deadband)
    error = 0.0f;
  Real direction = input->ApaGoDir < 0 ? -1.0f : 1.0f;

  const Real damping = param->HeadingDampingRatio > 0.0f ? param->HeadingDampingRatio : 1.0f;
  Real correction = damping * 2.0f * sqrtf(fmaxf(param->Kpp, 0.0f)) * sinf(error) / lookahead;

  if (input->TerminalMode && input->Dist2Go > 0.0f &&
      input->TerminalYawError > param->TerminalYawTol) {
    Real geomError = input->HeadingError;
    if (fabsf(geomError) < deadband)
      geomError = 0.0f;
    const Real required = fabsf(geomError) / fmaxf(input->Dist2Go, 0.02f);
    correction = copysignf(fmaxf(fabsf(correction), required),
                           fabsf(geomError) > APA_MOCO_LAT_V2_EPS ? geomError : error);
  }
  return direction * correction;
}

static Real apaMocoLatCtrlV2IntegralCandidate(const ApaMocoLatCtrlV2Input_t *input,
                                              const ApaMocoLatCtrlV2Param_t *param,
                                              const ApaMocoLatCtrlV2State_t *state,
                                              Real *previous) {
  const Integer direction = input->ApaGoDir < 0 ? -1 : 1;
  const Real limit = param->IntegralCurvatureLimit;
  *previous = 0.0f;
  if (!isfinite(limit) || !isfinite(param->IntegralGain) || limit <= 0.0f ||
      param->IntegralGain <= 0.0f)
    return 0.0f;

  if (input->CtrlPhase != APA_MOCO_NOACT && input->CtrlPhase != APA_MOCO_INITIAL &&
      input->CtrlPhase != APA_MOCO_DONE && input->CtrlPhase != APA_MOCO_FAULT &&
      state->LastGoDir == direction && isfinite(state->IntegralCurvature) &&
      fabsf(input->RefCurvature - state->LastRefCurvature) <= 0.04f) {
    *previous = apaMathClamp(state->IntegralCurvature, -limit, limit);
  }
  if (!apaMocoMotionPhase(input->CtrlPhase) || input->PathPointNum < 2 ||
      !isfinite(input->CycleTime) || input->CycleTime <= 0.0f || input->CycleTime > 0.1f ||
      !isfinite(input->VehicleSpeed) || !isfinite(input->CrossTrackError) ||
      !isfinite(input->HeadingError) || !isfinite(input->Dist2Go) ||
      !isfinite(input->TargetSpeed) ||
      input->VehicleSpeed * (Real)direction <= fmaxf(0.03f, param->StaticSpeedThd) ||
      fabsf(input->TargetSpeed) < param->StaticSpeedThd || fabsf(input->CrossTrackError) < 0.001f ||
      fabsf(input->CrossTrackError) > 0.15f || fabsf(input->HeadingError) > 0.20f ||
      fabsf(input->Dist2Go) <= 0.20f)
    return *previous;

  const Real distance = fminf(fabsf(input->VehicleSpeed) * input->CycleTime, 0.02f);
  return apaMathClamp(
      *previous + param->IntegralGain * input->CrossTrackError * distance, -limit, limit);
}

/**
 * @brief 曲率前馈与低速空间误差反馈，保留原转角/静止守卫
 * @en_name apaMocoLatCtrlV2
 * @cn_name MoCo v2横向控制
 * @type widget_module
 * @config cn_name=轴距, en_name=Wheelbase,
 * desc=车辆轴距, unit=m, type=Real, range=[1,5], default=3.00465
 * @config cn_name=横向增益, en_name=Kpp,
 * desc=空间横向误差反馈增益, unit=1, type=Real, range=[0,5], default=0.9
 * @config cn_name=转向传动比, en_name=SteeringRatio,
 * desc=保留的转向传动比标定, unit=1, type=Real, range=[1,40], default=20
 * @config cn_name=转角间隙限幅, en_name=SteerGapMax,
 * desc=单步转角最大偏离, unit=rad, type=Real, range=[0,2], default=1.6
 * @config cn_name=转角速率限制, en_name=SteerRateLimit,
 * desc=每步最大转角变化，零表示仿真理想转向, unit=rad, type=Real, range=[0,0.5], default=0
 * @config cn_name=转角上限, en_name=SteerUpperLimit,
 * desc=前轮转角上限, unit=rad, type=Real, range=[0,1.57], default=0.8
 * @config cn_name=转角下限, en_name=SteerLowerLimit,
 * desc=前轮转角下限, unit=rad, type=Real, range=[-1.57,0], default=-0.8
 * @config cn_name=终端航向阈值, en_name=TerminalYawTol,
 * desc=末段航向捕获阈值，组合控制与到位容差共用, unit=rad, type=Real, range=[0,1], default=0.035
 * @config cn_name=横向允许阈值, en_name=AllowEyThreshold,
 * desc=低速空间横向反馈误差死区, unit=m, type=Real, range=[0,1], default=0.002
 * @config cn_name=静止速度阈值, en_name=StaticSpeedThd,
 * desc=非运行相位转角保持速度阈值, unit=m/s, type=Real, range=[0,1], default=0.03
 * @config cn_name=短轨迹点数, en_name=ShortPathPointCount,
 * desc=不参与转角回正判定, unit=1, type=Integer, range=[0,20], default=3
 * @config cn_name=空间积分增益, en_name=IntegralGain,
 * desc=按实际行驶距离累计持续横偏，零关闭, unit=1/m3, type=Real, range=[0,1], default=0.25
 * @config cn_name=积分曲率上限, en_name=IntegralCurvatureLimit,
 * desc=积分补偿限幅，零关闭, unit=1/m, type=Real, range=[0,0.2], default=0.06
 * @config cn_name=航向阻尼倍率, en_name=HeadingDampingRatio,
 * desc=空间航向反馈阻尼倍率，1保留原等效值, unit=1, type=Real, range=[0.1,3], default=1
 * @config cn_name=航向死区, en_name=HeadingDeadband,
 * desc=航向反馈死区，默认0.05度, unit=rad, type=Real, range=[0,0.1], default=0.0008726646
 * @param[IN] Real RefCurvature 参考曲率
 * @param[IN] Real TargetSpeed 目标速度
 * @param[IN] Real Dist2Go 当前段有符号剩余里程
 * @param[IN] Real LongitudinalError 预瞄点车体纵向坐标
 * @param[IN] Real LateralError 预瞄点车体横向坐标
 * @param[IN] Real FeedbackDistance 独立于段末取点的反馈调度距离
 * @param[IN] Real CrossTrackError 当前最近点横向跟踪误差
 * @param[IN] Real HeadingError 参考航向误差
 * @param[IN] Real GoalDistance 终点距离
 * @param[IN] Real TerminalYawError 终点航向误差
 * @param[IN] Integer CtrlPhase 控制阶段
 * @param[IN] Integer ApaGoDir 当前段方向
 * @param[IN] Integer TerminalMode 终端模式
 * @param[IN] Integer ParkingCate 泊车类型
 * @param[IN] Integer PathPointNum 局部路径点数
 * @param[IN] Real VehicleSpeed 实测有符号车速
 * @param[IN] Real VehicleLateralSpeed 实测后轴横向速度(左正)
 * @param[IN] Real CycleTime 有效仿真控制周期
 * @param[OUT] Real SteerAngle 前轮转角
 * @param[OUT] Real StrPP 曲率前馈和横向反馈转角项
 * @param[OUT] Real StrH 航向阻尼附加转角项
 * @param[OUT] Real DisErr 横向误差调试量
 * @param[OUT] Real HeadErr 航向误差调试量
 * @param[OUT] Integer DecelerationReq 降速请求
 * @param[OUT] Integer SteerSaturated 转角限幅标志
 * @param[OUT] Integer AllowEy 横向误差允许门控
 * @param[OUT] Integer StrZeroFlag 完成或故障相位转角归零
 * @param[OUT] Real IntegralCurvature 有界空间积分曲率补偿
 * @var Real LastSteerAngle 上周期前轮转角
 * @var Real IntegralCurvature 累计曲率补偿
 * @var Integer LastGoDir 上周期行驶方向
 * @var Real LastRefCurvature 上周期参考曲率
 * @granularity atomic
 * @tag_level1 泊车控制
 * @tag_level2 MoCo
 * @source low_speed_bicycle_feedback + slx_stateflow steering guards
 * @version 1.2
 * @date 2026-11-03
 * @author aide
 */
void apaMocoLatCtrlV2(const ApaMocoLatCtrlV2Input_t *input,
                      const ApaMocoLatCtrlV2Param_t *param,
                      ApaMocoLatCtrlV2State_t *state,
                      ApaMocoLatCtrlV2Output_t *output) {
  Real steerPp0;
  Real steerH0;
  Real steerRaw0;
  Real steerGapHigh0;
  Real steerGapLow0;
  Real steerGap0;
  Real steerClamp0;
  Real steerRate0;
  Integer allowEy0;
  Integer strZeroFlag0;
  Integer saturated0;
  Boolean condStaticSpeed0;
  Boolean condHoldSteer0;
  Boolean condGapLimit0;
  Boolean condDone0;

  Real lookahead0 = apaMocoLatCtrlV2Lookahead(input, param);
  Real baseCurvature0 = apaMocoLatCtrlV2BaseCurvature(input, param, lookahead0, &allowEy0);

  Real lateralWeight0 = 1.0f;
  Real feedbackWeight0 = 1.0f;
  if (input->TerminalMode) {
    Real ratio0 = apaMathClamp(input->Dist2Go / lookahead0, 0.0f, 1.0f);
    lateralWeight0 = ratio0 * ratio0 * (3.0f - 2.0f * ratio0);
    feedbackWeight0 = fmaxf(0.15f, lateralWeight0);
    baseCurvature0 = input->RefCurvature + feedbackWeight0 * (baseCurvature0 - input->RefCurvature);
  }
  Real headingCurvature0 =
      apaMocoLatCtrlV2HeadingCurvature(input, param, lookahead0, lateralWeight0);
  Real previousIntegral0;
  Real integral0 = apaMocoLatCtrlV2IntegralCandidate(input, param, state, &previousIntegral0);

  condStaticSpeed0 = fabsf(input->TargetSpeed) < param->StaticSpeedThd;
  condDone0 = input->CtrlPhase == APA_MOCO_DONE || input->CtrlPhase == APA_MOCO_FAULT;

  strZeroFlag0 = condDone0;
  condHoldSteer0 = condStaticSpeed0 && (!apaMocoMotionPhase(input->CtrlPhase)) && (!condDone0) &&
                   !(input->CtrlPhase == APA_MOCO_GEAR_CHECK &&
                     fabsf(input->VehicleSpeed) <= param->StaticSpeedThd);

  condGapLimit0 = (param->SteerGapMax > APA_MOCO_LAT_V2_EPS);
  for (Integer attempt0 = 0; attempt0 < 2; ++attempt0) {
    steerPp0 = atanf(param->Wheelbase * (baseCurvature0 + feedbackWeight0 * integral0));
    steerRaw0 = atanf(param->Wheelbase *
                      (baseCurvature0 + feedbackWeight0 * integral0 + headingCurvature0));
    steerH0 = steerRaw0 - steerPp0;
    if (strZeroFlag0 != 0) {
      steerRaw0 = 0.0f;
    } else if (condHoldSteer0) {
      steerRaw0 = state->LastSteerAngle;
    }
    saturated0 = 0;
    if (condGapLimit0) {
      steerGapHigh0 = state->LastSteerAngle + param->SteerGapMax;
      steerGapLow0 = state->LastSteerAngle - param->SteerGapMax;
      steerGap0 = apaMathClamp(steerRaw0, steerGapLow0, steerGapHigh0);
      if (fabsf(steerGap0 - steerRaw0) > APA_MOCO_LAT_V2_EPS)
        saturated0 = 1;
    } else {
      steerGap0 = steerRaw0;
    }
    steerClamp0 = apaMathClamp(steerGap0, param->SteerLowerLimit, param->SteerUpperLimit);
    if (fabsf(steerClamp0 - steerGap0) > APA_MOCO_LAT_V2_EPS) {
      saturated0 = 1;
    }
    steerRate0 = apaMocoLatCtrlV2RateLimit(
        steerClamp0, state->LastSteerAngle, param->SteerRateLimit, &saturated0);
    if (attempt0 == 0 && saturated0 &&
        (steerRaw0 - steerRate0) * (integral0 - previousIntegral0) > 0.0f) {
      integral0 = previousIntegral0;
      continue;
    }
    break;
  }
  state->LastSteerAngle = steerRate0;
  state->SteerTarget = apaMathClamp(steerRaw0, param->SteerLowerLimit, param->SteerUpperLimit);
  state->StrZeroFlagLast = strZeroFlag0;
  state->IntegralCurvature = integral0;
  state->LastGoDir = input->ApaGoDir < 0 ? -1 : 1;
  state->LastRefCurvature = input->RefCurvature;

  output->SteerAngle = steerRate0;
  output->StrPP = steerPp0;
  output->StrH = steerH0;
  output->DisErr = input->CrossTrackError;
  output->HeadErr = input->HeadingError;
  output->DecelerationReq = 0;
  if (fabsf(steerRate0) > 0.65f) {
    output->DecelerationReq = 1;
  }
  output->SteerSaturated = saturated0;
  output->AllowEy = allowEy0;
  output->StrZeroFlag = strZeroFlag0;
  output->IntegralCurvature = integral0;
}
