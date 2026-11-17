#ifndef COLLISION_CHECK_SAT_TYPES_H
#define COLLISION_CHECK_SAT_TYPES_H

#include "baseTypes.h"

/**
 * @brief SAT矩形vs线段碰撞检测输入
 * @en_name CollisionCheckSatInput
 * @cn_name SAT碰撞检测输入
 * @tag STRUCT
 * @field name=RectX1, type=Real, unit=m, desc=矩形角点1 X
 * @field name=RectY1, type=Real, unit=m, desc=矩形角点1 Y
 * @field name=RectX2, type=Real, unit=m, desc=矩形角点2 X
 * @field name=RectY2, type=Real, unit=m, desc=矩形角点2 Y
 * @field name=RectX3, type=Real, unit=m, desc=矩形角点3 X
 * @field name=RectY3, type=Real, unit=m, desc=矩形角点3 Y
 * @field name=RectX4, type=Real, unit=m, desc=矩形角点4 X
 * @field name=RectY4, type=Real, unit=m, desc=矩形角点4 Y
 * @field name=LineX1, type=Real, unit=m, desc=线段起点 X
 * @field name=LineY1, type=Real, unit=m, desc=线段起点 Y
 * @field name=LineX2, type=Real, unit=m, desc=线段终点 X
 * @field name=LineY2, type=Real, unit=m, desc=线段终点 Y
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real RectX1; // 矩形角点1 X
  Real RectY1; // 矩形角点1 Y
  Real RectX2; // 矩形角点2 X
  Real RectY2; // 矩形角点2 Y
  Real RectX3; // 矩形角点3 X
  Real RectY3; // 矩形角点3 Y
  Real RectX4; // 矩形角点4 X
  Real RectY4; // 矩形角点4 Y
  Real LineX1; // 线段起点 X
  Real LineY1; // 线段起点 Y
  Real LineX2; // 线段终点 X
  Real LineY2; // 线段终点 Y
} CollisionCheckSatInput_t;

/**
 * @brief SAT矩形vs线段碰撞检测输出
 * @en_name CollisionCheckSatOutput
 * @cn_name SAT碰撞检测输出
 * @tag STRUCT
 * @field name=IsColliding, type=Boolean, unit=1, desc=是否碰撞
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Boolean IsColliding; // 是否碰撞
} CollisionCheckSatOutput_t;

void collisionCheckSat(const CollisionCheckSatInput_t *input, CollisionCheckSatOutput_t *output);

#endif
