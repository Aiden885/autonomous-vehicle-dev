#ifndef IS_PT_IN_POLYGON_TYPES_H
#define IS_PT_IN_POLYGON_TYPES_H

#include "baseTypes.h"

/** @brief 多边形最大顶点数 */
#define MAX_POLYGON_VERTICES 16

/** @brief 浮点比较精度 */
#ifndef APA_FLOAT_EPSILON
#define APA_FLOAT_EPSILON 1e-6f
#endif

/**
 * @brief 点在多边形内检测输入
 * @en_name IsPtInPolygonInput
 * @cn_name 点在多边形内检测输入
 * @tag STRUCT
 * @field name=PtX, type=Real, unit=m, desc=待检测点X
 * @field name=PtY, type=Real, unit=m, desc=待检测点Y
 * @field name=PolyX, type=Array<Real, [16]>, unit=m, desc=多边形顶点X数组
 * @field name=PolyY, type=Array<Real, [16]>, unit=m, desc=多边形顶点Y数组
 * @field name=PolyN, type=Integer, unit=1, desc=多边形顶点数(闭合,首尾相同)
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real PtX;                         // 待检测点X
  Real PtY;                         // 待检测点Y
  Real PolyX[MAX_POLYGON_VERTICES]; // 多边形顶点X数组
  Real PolyY[MAX_POLYGON_VERTICES]; // 多边形顶点Y数组
  Integer PolyN;                    // 多边形顶点数(闭合,首尾相同)
} IsPtInPolygonInput_t;

/**
 * @brief 点在多边形内检测输出
 * @en_name IsPtInPolygonOutput
 * @cn_name 点在多边形内检测输出
 * @tag STRUCT
 * @field name=IsInside, type=Boolean, unit=1, desc=点是否在多边形内
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Boolean IsInside; // 点是否在多边形内
} IsPtInPolygonOutput_t;

void isPtInPolygon(const IsPtInPolygonInput_t *input, IsPtInPolygonOutput_t *output);

#endif
