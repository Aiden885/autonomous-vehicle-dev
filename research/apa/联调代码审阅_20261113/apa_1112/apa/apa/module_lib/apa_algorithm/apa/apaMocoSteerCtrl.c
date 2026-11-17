#include "apaMocoSteerCtrlTypes.h"
#include <math.h>

#ifndef APA_MOCO_STEER_CTRL_EPSILON
#define APA_MOCO_STEER_CTRL_EPSILON 1e-6f
#endif

#ifndef APA_MOCO_STEER_CTRL_YAW_SPEED_TOL
#define APA_MOCO_STEER_CTRL_YAW_SPEED_TOL 0.12f
#endif

#ifndef APA_MOCO_STEER_CTRL_YAW_SPEED_MIN_SCALE
#define APA_MOCO_STEER_CTRL_YAW_SPEED_MIN_SCALE 0.8f
#endif

#ifndef APA_MOCO_STEER_CTRL_STATIC_SPEED
#define APA_MOCO_STEER_CTRL_STATIC_SPEED 0.3f
#endif

#ifndef APA_MOCO_STEER_CTRL_TURN_SLOW_STEER
#define APA_MOCO_STEER_CTRL_TURN_SLOW_STEER 0.75f
#endif

static Real apaMocoSteerCtrlCalcSpeed(Real refX,
                                      Real refY,
                                      Real vehX,
                                      Real vehY,
                                      Real refSpeed,
                                      Real headingError,
                                      Real kdist,
                                      Real speedUpper,
                                      Real speedLower) {
  Real dx0;
  Real dy0;
  Real dist0;
  Real speedScale0;
  Real speedScale1;
  Real speedScale2;
  Real headingScale0;
  Real headingScale1;
  Real speedScale3;
  Real speedRaw0;
  Real speedRaw1;
  Real speedRaw2;
  Real headingScale2;
  Boolean condScaleHi0;
  Boolean condScaleLo0;
  Boolean condHeadingScaleHi0;
  Boolean condHeadingCorrection0;
  Boolean condHeadingScaleLo0;
  Boolean condHeadingScaleFloor0;
  Boolean condHeadingDominates0;
  Boolean condSpdHi0;
  Boolean condSpdLo0;

  dx0 = refX - vehX;
  dy0 = refY - vehY;
  dist0 = sqrtf(dx0 * dx0 + dy0 * dy0);

  speedScale0 = kdist * dist0;
  condScaleHi0 = (speedScale0 > 1.0f);
  if (condScaleHi0) {
    speedScale1 = 1.0f;
  } else {
    speedScale1 = speedScale0;
  }
  condScaleLo0 = (speedScale1 < 0.0f);
  if (condScaleLo0) {
    speedScale2 = 0.0f;
  } else {
    speedScale2 = speedScale1;
  }

  headingScale0 = fabsf(headingError);
  condHeadingScaleHi0 = (headingScale0 > 1.0f);
  if (condHeadingScaleHi0) {
    headingScale1 = 1.0f;
  } else {
    headingScale1 = headingScale0;
  }
  condHeadingCorrection0 = (headingScale1 > APA_MOCO_STEER_CTRL_YAW_SPEED_TOL);
  condHeadingScaleLo0 = (headingScale1 < APA_MOCO_STEER_CTRL_YAW_SPEED_MIN_SCALE);
  condHeadingScaleFloor0 = condHeadingCorrection0 && condHeadingScaleLo0;
  if (condHeadingScaleFloor0) {
    headingScale2 = APA_MOCO_STEER_CTRL_YAW_SPEED_MIN_SCALE;
  } else {
    headingScale2 = headingScale1;
  }

  condHeadingDominates0 = (headingScale2 > speedScale2);
  if (condHeadingDominates0) {
    speedScale3 = headingScale2;
  } else {
    speedScale3 = speedScale2;
  }

  speedRaw0 = refSpeed * speedScale3;
  condSpdHi0 = (speedRaw0 > speedUpper);
  if (condSpdHi0) {
    speedRaw1 = speedUpper;
  } else {
    speedRaw1 = speedRaw0;
  }
  condSpdLo0 = (speedRaw1 < speedLower);
  if (condSpdLo0) {
    speedRaw2 = speedLower;
  } else {
    speedRaw2 = speedRaw1;
  }

  return speedRaw2;
}

/**
 * @brief MoCo前馈转向控制器 — 基于路径曲率前馈 + 横向/航向偏差反馈
 * @en_name apaMocoSteerCtrl
 * @cn_name MoCo前馈转向控制器
 * @type widget_module
 * @config cn_name=轴距, en_name=Wheelbase,
 * desc=前后轴距, unit=m,
 * type=Real, range=[1,5], default=2.7
 * @config cn_name=横向增益, en_name=Klat,
 * desc=横向偏差反馈增益, unit=1,
 * type=Real, range=[0,5], default=1.0
 * @config cn_name=航向增益, en_name=Kheading,
 * desc=航向偏差反馈增益, unit=1,
 * type=Real, range=[0,3], default=0.5
 * @config cn_name=最小速度, en_name=VMin,
 * desc=Stanley横向增益分母最小速度(防除零), unit=m/s,
 * type=Real, range=[0.01,2], default=0.3
 * @config cn_name=转角间隙限幅, en_name=SteerGapMax,
 * desc=单步转角最大偏离上限(仿MATLAB k_AngGapMax), unit=rad,
 * type=Real, range=[0,1], default=0.3
 * @config cn_name=转角速率限制, en_name=SteerRateLimit,
 * desc=每步最大转角变化量(仿MATLAB k_AngStepMax/StrDelta), unit=rad,
 * type=Real, range=[0,0.2], default=0.02
 * @config cn_name=转向角上限, en_name=SteerUpperLimit,
 * desc=前轮转向角最大值, unit=rad,
 * type=Real, range=[0,1.57], default=0.55
 * @config cn_name=转向角下限, en_name=SteerLowerLimit,
 * desc=前轮转向角最小值, unit=rad,
 * type=Real, range=[-1.57,0], default=-0.55
 * @config cn_name=距离增益, en_name=Kdist,
 * desc=距离到速度缩放增益, unit=1,
 * type=Real, range=[0,10], default=2.0
 * @config cn_name=速度上限, en_name=SpeedUpperLimit,
 * desc=速度指令最大值, unit=m/s,
 * type=Real, range=[0,30], default=2.0
 * @config cn_name=速度下限, en_name=SpeedLowerLimit,
 * desc=速度指令最小值(倒车), unit=m/s,
 * type=Real, range=[-10,0], default=-2.0
 * @param[IN] Real VehicleX 车辆当前位置X坐标
 * @param[IN] Real VehicleY 车辆当前位置Y坐标
 * @param[IN] Real VehicleYaw 车辆当前航向角
 * @param[IN] Real RefX 参考轨迹X坐标
 * @param[IN] Real RefY 参考轨迹Y坐标
 * @param[IN] Real RefYaw 参考轨迹航向角
 * @param[IN] Real RefSpeed 参考目标速度
 * @param[IN] Real RefCurvature 参考点曲率
 * @param[IN] Real LateralError 横向偏差
 * @param[IN] Real HeadingError 航向偏差
 * @param[OUT] Real SteerAngle 转向角指令
 * @param[OUT] Real Speed 速度指令
 * @param[OUT] Integer SteerSaturated 转向角是否触发饱和
 * @var Real LastSteerAngle 上周期转向角(用于速率限制)
 * @granularity atomic
 * @tag_level1 泊车控制
 * @tag_level2 运动控制
 * @source demo_new phase-1 Native MoCo controller projection
 * @version 1.0
 * @date 2026-03-14
 * @author aide
 *
 */
void apaMocoSteerCtrl(const ApaMocoSteerCtrlInput_t *input,
                      const ApaMocoSteerCtrlParam_t *param,
                      ApaMocoSteerCtrlState_t *state,
                      ApaMocoSteerCtrlOutput_t *output) {
  Real wb0;
  Real refCurvature0;
  Real latError0;
  Real headingError0;
  Real refSpeed0;
  Real steerFF0;
  Real steerLat0;
  Real steerHeading0;
  Real steerRaw0;
  Real steerAngle0;
  Real steerAngle1;
  Real steerAngle2;
  Real lastSteer0;
  Real steerDelta0;
  Boolean condSteerHigh0;
  Boolean condSteerLow0;
  Boolean condTurnSlow0;
  Integer steerSaturated0;

  wb0 = param->Wheelbase;
  refCurvature0 = input->RefCurvature;
  latError0 = input->LateralError;
  headingError0 = input->HeadingError;
  refSpeed0 = input->RefSpeed;

  steerFF0 = atanf(wb0 * refCurvature0);

  {
    Real vDenom0;

    vDenom0 = fabsf(refSpeed0);
    if (vDenom0 < param->VMin) {
      vDenom0 = param->VMin;
    }

    steerLat0 = atanf(param->Klat * latError0 / vDenom0);

    steerHeading0 = param->Kheading * headingError0;
  }

  if (refSpeed0 < 0.0f) {
    steerHeading0 = -steerHeading0;
  }

  steerRaw0 = steerFF0 + steerLat0 + steerHeading0;

  lastSteer0 = state->LastSteerAngle;
  steerSaturated0 = 0;
  if (param->SteerGapMax > APA_MOCO_STEER_CTRL_EPSILON) {
    if (steerRaw0 > lastSteer0 + param->SteerGapMax) {
      steerRaw0 = lastSteer0 + param->SteerGapMax;
      steerSaturated0 = 1;
    } else if (steerRaw0 < lastSteer0 - param->SteerGapMax) {
      steerRaw0 = lastSteer0 - param->SteerGapMax;
      steerSaturated0 = 1;
    }
  }

  steerAngle0 = steerRaw0;

  condSteerHigh0 = (steerAngle0 > param->SteerUpperLimit);
  if (condSteerHigh0) {
    steerAngle1 = param->SteerUpperLimit;
  } else {
    steerAngle1 = steerAngle0;
  }
  condSteerLow0 = (steerAngle1 < param->SteerLowerLimit);
  if (condSteerLow0) {
    steerAngle2 = param->SteerLowerLimit;
  } else {
    steerAngle2 = steerAngle1;
  }
  if (condSteerHigh0 || condSteerLow0) {
    steerSaturated0 = 1;
  }

  if (param->SteerRateLimit > APA_MOCO_STEER_CTRL_EPSILON) {
    steerDelta0 = steerAngle2 - lastSteer0;
    if (steerDelta0 > param->SteerRateLimit) {
      steerAngle2 = lastSteer0 + param->SteerRateLimit;
      steerSaturated0 = 1;
    } else if (steerDelta0 < -param->SteerRateLimit) {
      steerAngle2 = lastSteer0 - param->SteerRateLimit;
      steerSaturated0 = 1;
    }
  }
  state->LastSteerAngle = steerAngle2;

  output->SteerAngle = steerAngle2;
  output->SteerSaturated = steerSaturated0;
  {
    Real speedCmd0;
    Real absSteer0;
    speedCmd0 = apaMocoSteerCtrlCalcSpeed(input->RefX,
                                          input->RefY,
                                          input->VehicleX,
                                          input->VehicleY,
                                          refSpeed0,
                                          headingError0,
                                          param->Kdist,
                                          param->SpeedUpperLimit,
                                          param->SpeedLowerLimit);
    absSteer0 = fabsf(steerAngle2);
    condTurnSlow0 = (absSteer0 > APA_MOCO_STEER_CTRL_TURN_SLOW_STEER);
    if (condTurnSlow0) {
      if (speedCmd0 > APA_MOCO_STEER_CTRL_STATIC_SPEED) {
        speedCmd0 = APA_MOCO_STEER_CTRL_STATIC_SPEED;
      } else if (speedCmd0 < -APA_MOCO_STEER_CTRL_STATIC_SPEED) {
        speedCmd0 = -APA_MOCO_STEER_CTRL_STATIC_SPEED;
      }
    }
    output->Speed = speedCmd0;
  }
}
