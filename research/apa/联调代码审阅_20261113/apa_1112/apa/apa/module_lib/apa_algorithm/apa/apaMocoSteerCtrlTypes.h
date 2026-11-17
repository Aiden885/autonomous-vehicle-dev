#ifndef APA_MOCO_STEER_CTRL_TYPES_H
#define APA_MOCO_STEER_CTRL_TYPES_H

#include "baseTypes.h"

/**
 * @brief MoCo前馈转向控制器输入
 * @en_name ApaMocoSteerCtrlInput
 * @cn_name MoCo前馈转向控制器输入
 * @tag STRUCT
 * @field name=VehicleX, type=Real, unit=m, desc=车辆X
 * @field name=VehicleY, type=Real, unit=m, desc=车辆Y
 * @field name=VehicleYaw, type=Real, unit=rad, desc=车辆航向
 * @field name=RefX, type=Real, unit=m, desc=参考点X
 * @field name=RefY, type=Real, unit=m, desc=参考点Y
 * @field name=RefYaw, type=Real, unit=rad, desc=参考点航向
 * @field name=RefSpeed, type=Real, unit=m/s, desc=参考速度(正=前进,负=倒车)
 * @field name=RefCurvature, type=Real, unit=1/m, desc=参考点曲率
 * @field name=LateralError, type=Real, unit=m, desc=横向偏差(右正左负)
 * @field name=HeadingError, type=Real, unit=rad, desc=航向偏差
 * @version 1.0
 * @date 2026-03-14
 * @author aide
 */
typedef struct {
  Real VehicleX;
  Real VehicleY;
  Real VehicleYaw;
  Real RefX;
  Real RefY;
  Real RefYaw;
  Real RefSpeed;
  Real RefCurvature;
  Real LateralError;
  Real HeadingError;
} ApaMocoSteerCtrlInput_t;

/**
 * @brief MoCo前馈转向控制器参数
 * @en_name ApaMocoSteerCtrlParam
 * @cn_name MoCo前馈转向控制器参数
 * @tag STRUCT
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
 * type=Real, range=[0,0.5], default=0.08
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
 * @version 1.0
 * @date 2026-03-14
 * @author aide
 */
typedef struct {
  Real Wheelbase;
  Real Klat;
  Real Kheading;
  Real VMin;
  Real SteerGapMax;
  Real SteerRateLimit;
  Real SteerUpperLimit;
  Real SteerLowerLimit;
  Real Kdist;
  Real SpeedUpperLimit;
  Real SpeedLowerLimit;
} ApaMocoSteerCtrlParam_t;

/**
 * @brief MoCo前馈转向控制器状态
 * @en_name ApaMocoSteerCtrlState
 * @cn_name MoCo前馈转向控制器状态
 * @tag STRUCT
 * @var Real LastSteerAngle 上周期转向角(用于速率限制)
 * @version 1.0
 * @date 2026-03-14
 * @author aide
 */
typedef struct {
  Real LastSteerAngle;
} ApaMocoSteerCtrlState_t;

/**
 * @brief MoCo前馈转向控制器输出
 * @en_name ApaMocoSteerCtrlOutput
 * @cn_name MoCo前馈转向控制器输出
 * @tag STRUCT
 * @field name=SteerAngle, type=Real, unit=rad, desc=前轮转向角指令
 * @field name=Speed, type=Real, unit=m/s, desc=速度指令
 * @field name=SteerSaturated, type=Integer, unit=1, desc=转向角是否饱和
 * @version 1.0
 * @date 2026-03-14
 * @author aide
 */
typedef struct {
  Real SteerAngle;
  Real Speed;
  Integer SteerSaturated;
} ApaMocoSteerCtrlOutput_t;

void apaMocoSteerCtrl(const ApaMocoSteerCtrlInput_t *input,
                      const ApaMocoSteerCtrlParam_t *param,
                      ApaMocoSteerCtrlState_t *state,
                      ApaMocoSteerCtrlOutput_t *output);

#endif
