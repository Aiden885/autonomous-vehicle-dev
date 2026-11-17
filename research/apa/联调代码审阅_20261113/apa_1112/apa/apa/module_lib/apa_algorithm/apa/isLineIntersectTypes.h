#ifndef IS_LINE_INTERSECT_TYPES_H
#define IS_LINE_INTERSECT_TYPES_H

#include "baseTypes.h"

/** @brief 浮点比较精度 */
#ifndef APA_FLOAT_EPSILON
#define APA_FLOAT_EPSILON 1e-6f
#endif

/**
 * @brief 线段相交检测输入
 * @en_name IsLineIntersectInput
 * @cn_name 线段相交检测输入
 * @tag STRUCT
 * @field name=Ax, type=Real, unit=m, desc=线段A起点X
 * @field name=Ay, type=Real, unit=m, desc=线段A起点Y
 * @field name=Bx, type=Real, unit=m, desc=线段A终点X
 * @field name=By, type=Real, unit=m, desc=线段A终点Y
 * @field name=Cx, type=Real, unit=m, desc=线段B起点X
 * @field name=Cy, type=Real, unit=m, desc=线段B起点Y
 * @field name=Dx, type=Real, unit=m, desc=线段B终点X
 * @field name=Dy, type=Real, unit=m, desc=线段B终点Y
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real Ax; // 线段A起点X
  Real Ay; // 线段A起点Y
  Real Bx; // 线段A终点X
  Real By; // 线段A终点Y
  Real Cx; // 线段B起点X
  Real Cy; // 线段B起点Y
  Real Dx; // 线段B终点X
  Real Dy; // 线段B终点Y
} IsLineIntersectInput_t;

/**
 * @brief 线段相交检测输出
 * @en_name IsLineIntersectOutput
 * @cn_name 线段相交检测输出
 * @tag STRUCT
 * @field name=IsIntersect, type=Boolean, unit=1, desc=是否相交
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Boolean IsIntersect; // 是否相交
} IsLineIntersectOutput_t;

void isLineIntersect(const IsLineIntersectInput_t *input, IsLineIntersectOutput_t *output);

#endif
