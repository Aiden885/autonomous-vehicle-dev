#ifndef GEO_CIRCLE_2LINE_R_TYPES_H
#define GEO_CIRCLE_2LINE_R_TYPES_H

#include "baseTypes.h"

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
 * @brief 指定半径双切线圆弧输入
 * @en_name GeoCircle2lineRInput
 * @cn_name 指定半径双切线圆弧输入
 * @tag STRUCT
 * @field name=VstX, type=Real, unit=m, desc=起点X
 * @field name=VstY, type=Real, unit=m, desc=起点Y
 * @field name=VstTheta, type=Real, unit=rad, desc=起点航向角
 * @field name=VtgtX, type=Real, unit=m, desc=目标点X
 * @field name=VtgtY, type=Real, unit=m, desc=目标点Y
 * @field name=VtgtTheta, type=Real, unit=rad, desc=目标点航向角
 * @field name=Ptyp, type=Integer, unit=1, desc=停车位类型编号
 * @field name=Dir, type=Integer, unit=1, desc=行驶方向1前进2后退
 * @field name=Radius, type=Real, unit=m, desc=指定半径
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real VstX;
  Real VstY;
  Real VstTheta;
  Real VtgtX;
  Real VtgtY;
  Real VtgtTheta;
  Integer Ptyp;
  Integer Dir;
  Real Radius;
} GeoCircle2lineRInput_t;

/**
 * @brief 指定半径双切线圆弧输出
 * @en_name GeoCircle2lineROutput
 * @cn_name 指定半径双切线圆弧输出
 * @tag STRUCT
 * @field name=CptX, type=Real, unit=m, desc=圆心X
 * @field name=CptY, type=Real, unit=m, desc=圆心Y
 * @field name=StPtX, type=Real, unit=m, desc=圆弧起点X
 * @field name=StPtY, type=Real, unit=m, desc=圆弧起点Y
 * @field name=StPtTheta, type=Real, unit=rad, desc=圆弧起点航向角
 * @field name=EdPtX, type=Real, unit=m, desc=圆弧终点X
 * @field name=EdPtY, type=Real, unit=m, desc=圆弧终点Y
 * @field name=EdPtTheta, type=Real, unit=rad, desc=圆弧终点航向角
 * @field name=EdPtrX, type=Real, unit=m, desc=终点在目标位姿坐标系X
 * @field name=EdPtrY, type=Real, unit=m, desc=终点在目标位姿坐标系Y
 * @field name=EdPtrTheta, type=Real, unit=rad, desc=终点在目标位姿坐标系航向
 * @field name=TrnDir, type=Integer, unit=1, desc=转弯方向1左转负1右转
 * @field name=GoDir, type=Integer, unit=1, desc=行进方向1同向负1反向
 * @field name=ArcLen, type=Real, unit=m, desc=圆弧长度
 * @field name=ArcAngle, type=Real, unit=rad, desc=圆弧对应圆心角
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real CptX;
  Real CptY;
  Real StPtX;
  Real StPtY;
  Real StPtTheta;
  Real EdPtX;
  Real EdPtY;
  Real EdPtTheta;
  Real EdPtrX;
  Real EdPtrY;
  Real EdPtrTheta;
  Integer TrnDir;
  Integer GoDir;
  Real ArcLen;
  Real ArcAngle;
} GeoCircle2lineROutput_t;

void geoCircle2lineR(const GeoCircle2lineRInput_t *input, GeoCircle2lineROutput_t *output);

#endif
