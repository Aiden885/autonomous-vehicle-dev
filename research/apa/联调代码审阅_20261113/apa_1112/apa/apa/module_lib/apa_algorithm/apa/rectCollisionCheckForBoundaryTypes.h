#ifndef RECT_COLLISION_CHECK_FOR_BOUNDARY_TYPES_H
#define RECT_COLLISION_CHECK_FOR_BOUNDARY_TYPES_H

#include "baseTypes.h"
#include "collisionCheckSatTypes.h"

/** @brief 边界线段最大数量 */
#ifndef MAX_BOUNDARY_LINES
#define MAX_BOUNDARY_LINES 64
#endif

/** @brief 离散障碍物点最大数量 */
#ifndef MAX_SEPARATE_PTS
#define MAX_SEPARATE_PTS 32
#endif

/**
 * @brief 矩形vs边界碰撞检测输入
 * @en_name RectCollisionCheckForBoundaryInput
 * @cn_name 矩形边界碰撞检测输入
 * @tag STRUCT
 * @field name=RectX1, type=Real, unit=m, desc=矩形角点1X(前左)
 * @field name=RectY1, type=Real, unit=m, desc=矩形角点1Y
 * @field name=RectX2, type=Real, unit=m, desc=矩形角点2X(前右)
 * @field name=RectY2, type=Real, unit=m, desc=矩形角点2Y
 * @field name=RectX3, type=Real, unit=m, desc=矩形角点3X(后右)
 * @field name=RectY3, type=Real, unit=m, desc=矩形角点3Y
 * @field name=RectX4, type=Real, unit=m, desc=矩形角点4X(后左)
 * @field name=RectY4, type=Real, unit=m, desc=矩形角点4Y
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=边界线段数
 * @field name=BoundaryLineX1, type=Array<Real, [64]>, unit=m, desc=边界线段起点X数组
 * @field name=BoundaryLineY1, type=Array<Real, [64]>, unit=m, desc=边界线段起点Y数组
 * @field name=BoundaryLineX2, type=Array<Real, [64]>, unit=m, desc=边界线段终点X数组
 * @field name=BoundaryLineY2, type=Array<Real, [64]>, unit=m, desc=边界线段终点Y数组
 * @field name=SeparatePtsNum, type=Integer, unit=1, desc=离散障碍物点数
 * @field name=SeparatePtsX, type=Array<Real, [32]>, unit=m, desc=离散点X数组
 * @field name=SeparatePtsY, type=Array<Real, [32]>, unit=m, desc=离散点Y数组
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real RectX1; // 前左X
  Real RectY1;
  Real RectX2; // 前右X
  Real RectY2;
  Real RectX3; // 后右X
  Real RectY3;
  Real RectX4; // 后左X
  Real RectY4;
  Integer BoundaryLineNum;
  Real BoundaryLineX1[MAX_BOUNDARY_LINES];
  Real BoundaryLineY1[MAX_BOUNDARY_LINES];
  Real BoundaryLineX2[MAX_BOUNDARY_LINES];
  Real BoundaryLineY2[MAX_BOUNDARY_LINES];
  Integer SeparatePtsNum;
  Real SeparatePtsX[MAX_SEPARATE_PTS];
  Real SeparatePtsY[MAX_SEPARATE_PTS];
} RectCollisionCheckForBoundaryInput_t;

/**
 * @brief 矩形vs边界碰撞检测输出
 * @en_name RectCollisionCheckForBoundaryOutput
 * @cn_name 矩形边界碰撞检测输出
 * @tag STRUCT
 * @field name=IsCollision, type=Boolean, unit=1, desc=是否碰撞
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Boolean IsCollision; // 是否碰撞
} RectCollisionCheckForBoundaryOutput_t;

void rectCollisionCheckForBoundary(const RectCollisionCheckForBoundaryInput_t *input,
                                   RectCollisionCheckForBoundaryOutput_t *output);

#endif
