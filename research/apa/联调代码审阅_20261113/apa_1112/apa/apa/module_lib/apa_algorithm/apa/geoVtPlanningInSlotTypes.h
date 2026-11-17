/**
 * @file geoVtPlanningInSlotTypes.h
 * @brief 槽内 VT 规划输入输出类型定义
 * @date 2026-04-13
 */
#ifndef GEO_VT_PLANNING_IN_SLOT_TYPES_H
#define GEO_VT_PLANNING_IN_SLOT_TYPES_H

#include "baseTypes.h"
#include "vehicleCollisionCheckTypes.h"

/**
 * @brief 槽内 VT 规划输入
 * @en_name GeoVtPlanningInSlotInput
 * @cn_name 槽内VT规划输入
 * @tag STRUCT
 * @field name=StartX, type=Real, unit=m, desc=起点X
 * @field name=StartY, type=Real, unit=m, desc=起点Y
 * @field name=StartTheta, type=Real, unit=rad, desc=起点航向
 * @field name=GoalX, type=Real, unit=m, desc=目标X
 * @field name=GoalY, type=Real, unit=m, desc=目标Y
 * @field name=GoalTheta, type=Real, unit=rad, desc=目标航向
 * @field name=R1, type=Real, unit=m, desc=第一段圆弧半径
 * @field name=R2, type=Real, unit=m, desc=第二段圆弧半径
 * @field name=SeparateStopperNum, type=Integer, unit=1, desc=离散障碍物数量
 * @field name=SeparateStopperX, type=Array<Real, [128]>, unit=m, desc=离散障碍物X数组
 * @field name=SeparateStopperY, type=Array<Real, [128]>, unit=m, desc=离散障碍物Y数组
 * @field name=SlotPointsX, type=Array<Real, [4]>, unit=m, desc=车位四个角点X(顺序: 左外,左内,右内,右外)
 * @field name=SlotPointsY, type=Array<Real, [4]>, unit=m, desc=车位四个角点Y
 * @field name=VehLF, type=Real, unit=m, desc=车辆前悬+轴距
 * @field name=VehLB, type=Real, unit=m, desc=车辆后悬
 * @field name=VehW, type=Real, unit=m, desc=车宽
 * @field name=UseVccTemplate, type=Boolean, unit=1, desc=是否复用完整碰撞输入模板
 * @field name=VccTemplate, type=VehicleCollisionCheckInput_t, unit=1, desc=完整障碍物碰撞输入模板
 */
typedef struct {
  Real StartX;
  Real StartY;
  Real StartTheta;
  Real GoalX;
  Real GoalY;
  Real GoalTheta;
  Real R1;
  Real R2;
  Integer SeparateStopperNum;
  Real SeparateStopperX[VCC_MAX_SEPARATE_PTS];
  Real SeparateStopperY[VCC_MAX_SEPARATE_PTS];
  Real SlotPointsX[4];
  Real SlotPointsY[4];
  Real VehLF;
  Real VehLB;
  Real VehW;
  Boolean UseVccTemplate;
  VehicleCollisionCheckInput_t VccTemplate;
} GeoVtPlanningInSlotInput_t;

/**
 * @brief 槽内 VT 规划输出
 * @en_name GeoVtPlanningInSlotOutput
 * @cn_name 槽内VT规划输出
 * @tag STRUCT
 * @field name=Success, type=Boolean, unit=1, desc=规划是否成功
 * @field name=AnchorsX, type=Array<Real, [3]>, unit=m, desc=定点X数组(3)
 * @field name=AnchorsY, type=Array<Real, [3]>, unit=m, desc=定点Y数组(3)
 * @field name=AnchorsTheta, type=Array<Real, [3]>, unit=rad, desc=定点航向数组(3)
 * @field name=CentersX, type=Array<Real, [3]>, unit=m, desc=圆心X数组(3)
 * @field name=CentersY, type=Array<Real, [3]>, unit=m, desc=圆心Y数组(3)
 * @field name=Curves, type=Array<Real, [3]>, unit=1/m, desc=曲率数组(3)
 * @field name=GoDirs, type=Array<Integer, [3]>, unit=1, desc=行驶方向数组(3)
 * @field name=TurnDirs, type=Array<Integer, [3]>, unit=1, desc=转向方向数组(3)
 * @field name=DThetas, type=Array<Real, [3]>, unit=rad, desc=角度增量数组(3)
 * @field name=Lengths, type=Array<Real, [3]>, unit=m, desc=段长数组(3)
 */
typedef struct {
  Boolean Success;
  Real AnchorsX[3];
  Real AnchorsY[3];
  Real AnchorsTheta[3];
  Real CentersX[3];
  Real CentersY[3];
  Real Curves[3];
  Integer GoDirs[3];
  Integer TurnDirs[3];
  Real DThetas[3];
  Real Lengths[3];
} GeoVtPlanningInSlotOutput_t;

void geoVtPlanningInSlot(const GeoVtPlanningInSlotInput_t *input,
                         GeoVtPlanningInSlotOutput_t *output);

#endif
