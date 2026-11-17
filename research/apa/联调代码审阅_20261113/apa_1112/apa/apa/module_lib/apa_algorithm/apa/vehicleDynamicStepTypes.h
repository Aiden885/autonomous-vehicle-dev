#ifndef VEHICLE_DYNAMIC_STEP_TYPES_H
#define VEHICLE_DYNAMIC_STEP_TYPES_H

#include "apaMacros.h"
#include "baseTypes.h"

/* PI constants provided by apaMacros.h: APA_PI, APA_TWO_PI, APA_HALF_PI */

/**
 * @brief 加速度转向车辆模型输入
 * @en_name VehicleDynamicStepInput
 * @cn_name 加速度转向车辆模型输入
 * @tag STRUCT
 * @field name=VehicleX, type=Real, unit=m, desc=当前车辆X位置
 * @field name=VehicleY, type=Real, unit=m, desc=当前车辆Y位置
 * @field name=VehicleYaw, type=Real, unit=rad, desc=当前车辆航向角
 * @field name=AccelerationCmd, type=Real, unit=m/s2, desc=车体前向有符号加速度指令
 * @field name=SteerAngle, type=Real, unit=rad, desc=当前前轮转角
 * @field name=GearPos, type=Integer, unit=1, desc=实际档位(0N或未知1D2R3P)
 * @version 1.1
 * @date 2026-09-01
 * @author aide
 */
typedef struct {
  Real VehicleX;
  Real VehicleY;
  Real VehicleYaw;
  Real AccelerationCmd;
  Real SteerAngle;
  Integer GearPos;
} VehicleDynamicStepInput_t;

/**
 * @brief 加速度转向车辆模型参数
 * @en_name VehicleDynamicStepParam
 * @cn_name 加速度转向车辆模型参数
 * @tag STRUCT
 * @field name=WheelBase, type=Real, unit=m, desc=车辆轴距
 * @field name=StraightSteerThreshold, type=Real, unit=rad,
 * desc=半步航向增量的级数近似阈值
 * @field name=CycleTime, type=Real, unit=s, desc=车辆模型离散仿真周期
 * @field name=ForwardSpeedLimit, type=Real, unit=m/s, desc=D档正速度上限
 * @field name=ReverseSpeedLimit, type=Real, unit=m/s, desc=R档负速度绝对值上限
 * @field name=AccelerationUpperLimit, type=Real, unit=m/s2, desc=车体前向加速度上限
 * @field name=AccelerationLowerLimit, type=Real, unit=m/s2, desc=车体前向加速度下限
 * @version 1.1
 * @date 2026-09-01
 * @author aide
 */
typedef struct {
  Real WheelBase;
  Real StraightSteerThreshold;
  Real CycleTime;
  Real ForwardSpeedLimit;
  Real ReverseSpeedLimit;
  Real AccelerationUpperLimit;
  Real AccelerationLowerLimit;
} VehicleDynamicStepParam_t;

/**
 * @brief 加速度转向车辆模型状态
 * @en_name VehicleDynamicStepState
 * @cn_name 加速度转向车辆模型状态
 * @tag STRUCT
 * @field name=VehicleSpeed, type=Real, unit=m/s, desc=车体前向有符号车速
 * @field name=VehicleAcceleration, type=Real, unit=m/s2, desc=车体前向有符号实际加速度
 * @version 1.1
 * @date 2026-09-01
 * @author aide
 */
typedef struct {
  Real VehicleSpeed;
  Real VehicleAcceleration;
} VehicleDynamicStepState_t;

/**
 * @brief 加速度转向车辆模型输出
 * @en_name VehicleDynamicStepOutput
 * @cn_name 加速度转向车辆模型输出
 * @tag STRUCT
 * @field name=VehicleX, type=Real, unit=m, desc=下一时刻车辆X位置
 * @field name=VehicleY, type=Real, unit=m, desc=下一时刻车辆Y位置
 * @field name=VehicleYaw, type=Real, unit=rad, desc=下一时刻车辆航向角
 * @field name=VehicleSpeed, type=Real, unit=m/s, desc=车体前向有符号实际速度
 * @field name=VehicleAcceleration, type=Real, unit=m/s2, desc=车体前向有符号实际加速度
 * @field name=TravelDistance, type=Real, unit=m, desc=本周期有符号行驶距离
 * @field name=DirectionSign, type=Integer, unit=1, desc=档位方向(+1前进-1倒车0无驱动方向)
 * @field name=SpeedLimited, type=Integer, unit=1, desc=纵向速度或加速度约束介入标志
 * @field name=ModelValid, type=Integer, unit=1, desc=当前车辆模型上下文有效标志
 * @version 1.1
 * @date 2026-09-01
 * @author aide
 */
typedef struct {
  Real VehicleX;
  Real VehicleY;
  Real VehicleYaw;
  Real VehicleSpeed;
  Real VehicleAcceleration;
  Real TravelDistance;
  Integer DirectionSign;
  Integer SpeedLimited;
  Integer ModelValid;
} VehicleDynamicStepOutput_t;

void vehicleDynamicStep(const VehicleDynamicStepInput_t *input,
                        const VehicleDynamicStepParam_t *param,
                        VehicleDynamicStepState_t *state,
                        VehicleDynamicStepOutput_t *output);

#endif
