#ifndef APA_PARKING_PATH_EXECUTABILITY_TYPES_H
#define APA_PARKING_PATH_EXECUTABILITY_TYPES_H
#include "apaMocoParkingControlTypes.h"
#include "apaParkingPathPlanningTypes.h"
#include "vehicleDynamicStepTypes.h"

/**
 * @brief Immutable scene, actual vehicle and candidate path.
 * @en_name ApaParkingPathExecutabilityInput
 * @cn_name 路径执行验收输入
 * @tag STRUCT
 * @field name=Scene, type=ApaParkingPathPlanningInput_t, unit=1, desc=真实后轴起点地图与目标
 * @field name=Vehicle, type=ApaParkingPathPlanningParam_t, unit=1, desc=实际车型不使用规划膨胀包络
 * @field name=Path, type=ApaParkingPathPlanningOutput_t, unit=1, desc=完整候选公开路径
 */
typedef struct {
  ApaParkingPathPlanningInput_t Scene;
  ApaParkingPathPlanningParam_t Vehicle;
  ApaParkingPathPlanningOutput_t Path;
} ApaParkingPathExecutabilityInput_t;

/**
 * @brief Explicit design allowances, control calibration and execution limits.
 * @en_name ApaParkingPathExecutabilityParam
 * @cn_name 路径执行验收参数
 * @tag STRUCT
 * @field name=Control, type=ApaMocoParkingControlParam_t, unit=1, desc=与实际回放相同控制标定
 * @field name=Plant, type=VehicleDynamicStepParam_t, unit=1, desc=理想车辆模型标定
 * @field name=LateralAllowance, type=Real, unit=m, desc=规划设计横向余量
 * @field name=YawAllowance, type=Real, unit=rad, desc=规划设计航向余量
 * @field name=StopAllowance, type=Real, unit=m, desc=规划设计纵向停车余量
 * @field name=PositionLimit, type=Real, unit=m, desc=终点位置误差上限
 * @field name=YawLimit, type=Real, unit=rad, desc=终点航向误差上限
 * @field name=LateralLimit, type=Real, unit=m, desc=全程横向误差上限
 * @field name=MaxSteps, type=Integer, unit=1, desc=候选闭环最多6500周期
 */
typedef struct {
  ApaMocoParkingControlParam_t Control;
  VehicleDynamicStepParam_t Plant;
  Real LateralAllowance;
  Real YawAllowance;
  Real StopAllowance;
  Real PositionLimit;
  Real YawLimit;
  Real LateralLimit;
  Integer MaxSteps;
} ApaParkingPathExecutabilityParam_t;

/**
 * @brief Complete execution decision and failure location for a single candidate.
 * @en_name ApaParkingPathExecutabilityOutput
 * @cn_name 路径执行验收输出
 * @tag STRUCT
 * @field name=Passed, type=Boolean, unit=1, desc=完成停稳误差与安全均通过
 * @field name=Completed, type=Boolean, unit=1, desc=控制完成且实际静止
 * @field name=StepCount, type=Integer, unit=1, desc=执行周期数
 * @field name=FinalPositionError, type=Real, unit=m, desc=相对原候选末点的位置误差
 * @field name=FinalYawError, type=Real, unit=rad, desc=相对原候选末点的航向误差
 * @field name=MaxLateralError, type=Real, unit=m, desc=全周期横向最大误差
 * @field name=FinalPose, type=Array<Real, [3]>, unit=1, desc=最后安全后轴位姿XY米航向弧度
 */
typedef struct {
  Boolean Passed;
  Boolean Completed;
  Integer StepCount;
  Real FinalPositionError;
  Real FinalYawError;
  Real MaxLateralError;
  Real FinalPose[3];
} ApaParkingPathExecutabilityOutput_t;
#endif
