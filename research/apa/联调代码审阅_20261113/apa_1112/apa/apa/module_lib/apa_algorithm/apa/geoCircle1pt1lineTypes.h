#ifndef GEO_CIRCLE_1PT1LINE_TYPES_H
#define GEO_CIRCLE_1PT1LINE_TYPES_H

#include "baseTypes.h"
#include <math.h>

/** @brief 圆周率 */
#ifndef APA_PI
#define APA_PI 3.14159265f
#endif
/** @brief 两倍圆周率 */
#ifndef APA_TWO_PI
#define APA_TWO_PI 6.2831853f
#endif
/** @brief 半圆周率 */
#ifndef APA_HALF_PI
#define APA_HALF_PI 1.57079632f
#endif
/** @brief 除零保护精度 */
#ifndef GEO_EPSILON
#define GEO_EPSILON 1e-6f
#endif

/**
 * @brief 退化半径占位值
 *
 * 当起点航向几乎与目标线平行时，圆弧退化为直线，MATLAB
 * geo_circle_1pt1line.m 返回 Inf 半径。保持该显式失败语义，避免有限
 * 占位数继续生成看似有效的圆心、切点或弧长。
 */
#ifndef GEO_CIRCLE_1PT1LINE_DEGEN_RADIUS
#define GEO_CIRCLE_1PT1LINE_DEGEN_RADIUS INFINITY
#endif

/**
 * @brief 过一点切直线圆弧输入
 * @en_name GeoCircle1pt1lineInput
 * @cn_name 过一点切直线圆弧输入
 * @tag STRUCT
 * @field name=VstX, type=Real, unit=m, desc=起点X坐标
 * @field name=VstY, type=Real, unit=m, desc=起点Y坐标
 * @field name=VstTheta, type=Real, unit=rad, desc=起点航向角
 * @field name=VtgtX, type=Real, unit=m, desc=目标点X坐标
 * @field name=VtgtY, type=Real, unit=m, desc=目标点Y坐标
 * @field name=VtgtTheta, type=Real, unit=rad, desc=目标点航向角
 * @field name=Dir, type=Integer, unit=1, desc=行驶方向1前进2后退
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real VstX;      // 起点X坐标
  Real VstY;      // 起点Y坐标
  Real VstTheta;  // 起点航向角
  Real VtgtX;     // 目标点X坐标
  Real VtgtY;     // 目标点Y坐标
  Real VtgtTheta; // 目标点航向角
  Integer Dir;    // 行驶方向: 1前进, 2后退
} GeoCircle1pt1lineInput_t;

/**
 * @brief 过一点切直线圆弧输出
 * @en_name GeoCircle1pt1lineOutput
 * @cn_name 过一点切直线圆弧输出
 * @tag STRUCT
 * @field name=CptX, type=Real, unit=m, desc=圆心X坐标
 * @field name=CptY, type=Real, unit=m, desc=圆心Y坐标
 * @field name=Radius, type=Real, unit=m, desc=圆弧半径
 * @field name=LnPtX, type=Real, unit=m, desc=圆弧与直线切点X
 * @field name=LnPtY, type=Real, unit=m, desc=圆弧与直线切点Y
 * @field name=LnPtTheta, type=Real, unit=rad, desc=切点航向角
 * @field name=LnPtRX, type=Real, unit=m, desc=目标坐标系切点X
 * @field name=LnPtRY, type=Real, unit=m, desc=目标坐标系切点Y
 * @field name=LnPtRTheta, type=Real, unit=rad, desc=目标坐标系切点航向角
 * @field name=TrnDir, type=Integer, unit=1, desc=圆弧方向1左转负1右转
 * @field name=ArcLen, type=Real, unit=m, desc=圆弧长度
 * @field name=ArcAngle, type=Real, unit=rad, desc=圆弧对应圆心角
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real CptX;       // 圆心X坐标
  Real CptY;       // 圆心Y坐标
  Real Radius;     // 圆弧半径
  Real LnPtX;      // 圆弧与直线切点X
  Real LnPtY;      // 圆弧与直线切点Y
  Real LnPtTheta;  // 切点航向角
  Real LnPtRX;     // 目标坐标系切点X
  Real LnPtRY;     // 目标坐标系切点Y
  Real LnPtRTheta; // 目标坐标系切点航向角
  Integer TrnDir;  // 圆弧方向: 1左转, -1右转
  Real ArcLen;     // 圆弧长度
  Real ArcAngle;   // 圆弧对应圆心角
} GeoCircle1pt1lineOutput_t;

void geoCircle1pt1line(const GeoCircle1pt1lineInput_t *input, GeoCircle1pt1lineOutput_t *output);

#endif
