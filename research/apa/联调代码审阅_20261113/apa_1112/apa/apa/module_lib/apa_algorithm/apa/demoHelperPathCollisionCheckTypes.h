#ifndef DEMO_HELPER_PATH_COLLISION_CHECK_TYPES_H
#define DEMO_HELPER_PATH_COLLISION_CHECK_TYPES_H

#include "baseTypes.h"

#ifndef DEMO_PATH_POINT_COUNT
#define DEMO_PATH_POINT_COUNT 100
#endif

#ifndef DEMO_HELPER_PATH_COLLISION_EPSILON
#define DEMO_HELPER_PATH_COLLISION_EPSILON 1e-6f
#endif

/**
 * @brief 演示路径碰撞检测输入
 * @en_name DemoHelperPathCollisionCheckInput
 * @cn_name 演示路径碰撞检测输入
 * @tag STRUCT
 * @field name=PtCount, type=Integer, unit=1, desc=离散路径点数量
 * @field name=PthPtX, type=Array<Real, [100]>, unit=m, desc=离散路径点X数组
 * @field name=PthPtY, type=Array<Real, [100]>, unit=m, desc=离散路径点Y数组
 * @field name=PthPtTht, type=Array<Real, [100]>, unit=rad, desc=离散路径点航向数组
 * @field name=GoalYaw, type=Real, unit=rad, desc=车位方向航向角
 * @field name=GoalX, type=Real, unit=m, desc=目标车辆X位置
 * @field name=GoalY, type=Real, unit=m, desc=目标车辆Y位置
 * @field name=SlotP1X, type=Real, unit=m, desc=车位入口角点1X
 * @field name=SlotP1Y, type=Real, unit=m, desc=车位入口角点1Y
 * @field name=SlotP2X, type=Real, unit=m, desc=车位入口角点2X
 * @field name=SlotP2Y, type=Real, unit=m, desc=车位入口角点2Y
 * @field name=SlotDepth, type=Real, unit=m, desc=车位深度
 * @version 1.0
 * @date 2026-03-10
 * @author aide
 */
typedef struct {
  Integer PtCount;
  Real PthPtX[DEMO_PATH_POINT_COUNT];
  Real PthPtY[DEMO_PATH_POINT_COUNT];
  Real PthPtTht[DEMO_PATH_POINT_COUNT];
  Real GoalYaw;
  Real GoalX;
  Real GoalY;
  Real SlotP1X;
  Real SlotP1Y;
  Real SlotP2X;
  Real SlotP2Y;
  Real SlotDepth;
} DemoHelperPathCollisionCheckInput_t;

/**
 * @brief 演示路径碰撞检测参数
 * @en_name DemoHelperPathCollisionCheckParam
 * @cn_name 演示路径碰撞检测参数
 * @tag STRUCT
 * @field name=VehicleLength, type=Real, unit=m, desc=用于检测的车辆长度
 * @field name=VehicleWidth, type=Real, unit=m, desc=用于检测的车辆宽度
 * @field name=VehicleFrontOffset, type=Real, unit=m, desc=前包络检测中心到车辆参考点的前向偏置
 * @field name=SafetyMargin, type=Real, unit=m, desc=路径最小安全间隙阈值
 * @version 1.0
 * @date 2026-03-10
 * @author aide
 */
typedef struct {
  Real VehicleLength;
  Real VehicleWidth;
  Real VehicleFrontOffset;
  Real SafetyMargin;
} DemoHelperPathCollisionCheckParam_t;

/**
 * @brief 演示路径碰撞检测状态
 * @en_name DemoHelperPathCollisionCheckState
 * @cn_name 演示路径碰撞检测状态
 * @tag STRUCT
 * @field name=Reserved, type=Integer, unit=1, desc=无状态占位字段
 * @version 1.0
 * @date 2026-03-10
 * @author aide
 */
typedef struct {
  Integer Reserved;
} DemoHelperPathCollisionCheckState_t;

/**
 * @brief 演示路径碰撞检测输出
 * @en_name DemoHelperPathCollisionCheckOutput
 * @cn_name 演示路径碰撞检测输出
 * @tag STRUCT
 * @field name=PathCollisionFlg, type=Integer, unit=1, desc=路径碰撞标志0安全1碰撞
 * @field name=PathMinClearance, type=Real, unit=m, desc=路径最小边界间隙
 * @field name=RiskPoseX, type=Real, unit=m, desc=风险路径点X
 * @field name=RiskPoseY, type=Real, unit=m, desc=风险路径点Y
 * @version 1.0
 * @date 2026-03-10
 * @author aide
 */
typedef struct {
  Integer PathCollisionFlg;
  Real PathMinClearance;
  Real RiskPoseX;
  Real RiskPoseY;
} DemoHelperPathCollisionCheckOutput_t;

void demoHelperPathCollisionCheck(const DemoHelperPathCollisionCheckInput_t *input,
                                  const DemoHelperPathCollisionCheckParam_t *param,
                                  DemoHelperPathCollisionCheckState_t *state,
                                  DemoHelperPathCollisionCheckOutput_t *output);

#endif
