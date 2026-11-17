#ifndef IS_LINE_INTERSECT_WITH_RECT_TYPES_H
#define IS_LINE_INTERSECT_WITH_RECT_TYPES_H

#include "baseTypes.h"

/**
 * @brief 线段与矩形相交检测输入
 * @en_name IsLineIntersectWithRectInput
 * @cn_name 线段与矩形相交检测输入
 * @tag STRUCT
 * @field name=LineX1, type=Real, unit=m, desc=线段起点X
 * @field name=LineY1, type=Real, unit=m, desc=线段起点Y
 * @field name=LineX2, type=Real, unit=m, desc=线段终点X
 * @field name=LineY2, type=Real, unit=m, desc=线段终点Y
 * @field name=RectX1, type=Real, unit=m, desc=矩形角点1 X
 * @field name=RectY1, type=Real, unit=m, desc=矩形角点1 Y
 * @field name=RectX2, type=Real, unit=m, desc=矩形角点2 X
 * @field name=RectY2, type=Real, unit=m, desc=矩形角点2 Y
 * @field name=RectX3, type=Real, unit=m, desc=矩形角点3 X
 * @field name=RectY3, type=Real, unit=m, desc=矩形角点3 Y
 * @field name=RectX4, type=Real, unit=m, desc=矩形角点4 X
 * @field name=RectY4, type=Real, unit=m, desc=矩形角点4 Y
 * @field name=UseSAT, type=Boolean, unit=1, desc=是否使用SAT碰撞检测
 * @version 1.0
 * @date 2026-04-28
 * @author aide
 */
typedef struct {
  Real LineX1;
  Real LineY1;
  Real LineX2;
  Real LineY2;
  Real RectX1;
  Real RectY1;
  Real RectX2;
  Real RectY2;
  Real RectX3;
  Real RectY3;
  Real RectX4;
  Real RectY4;
  Boolean UseSAT;
} IsLineIntersectWithRectInput_t;

/**
 * @brief 线段与矩形相交检测输出
 * @en_name IsLineIntersectWithRectOutput
 * @cn_name 线段与矩形相交检测输出
 * @tag STRUCT
 * @field name=IsCollision, type=Boolean, unit=1, desc=是否碰撞
 * @version 1.0
 * @date 2026-04-28
 * @author aide
 */
typedef struct {
  Boolean IsCollision;
} IsLineIntersectWithRectOutput_t;

void isLineIntersectWithRect(const IsLineIntersectWithRectInput_t *input,
                             IsLineIntersectWithRectOutput_t *output);

#endif
