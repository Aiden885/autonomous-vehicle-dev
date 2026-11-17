/**
 * @file geoCircleCheckTypes.h
 * @brief 几何点圆弧连接检查类型定义
 * @date 2026-04-14
 */
#ifndef GEO_CIRCLE_CHECK_TYPES_H
#define GEO_CIRCLE_CHECK_TYPES_H

#include "baseTypes.h"

/**
 * @brief 圆弧连接检查输入
 * @en_name GeoCircleCheckInput
 * @cn_name 圆弧连接检查输入
 * @tag STRUCT
 * @field name=Pt1X, type=Real, unit=m, desc=测试点1X
 * @field name=Pt1Y, type=Real, unit=m, desc=测试点1Y
 * @field name=Pt1Theta, type=Real, unit=rad, desc=测试点1朝向
 * @field name=Pt2X, type=Real, unit=m, desc=测试点2X
 * @field name=Pt2Y, type=Real, unit=m, desc=测试点2Y
 * @field name=Pt2Theta, type=Real, unit=rad, desc=测试点2朝向
 */
typedef struct {
  Real Pt1X;
  Real Pt1Y;
  Real Pt1Theta;
  Real Pt2X;
  Real Pt2Y;
  Real Pt2Theta;
} GeoCircleCheckInput_t;

/**
 * @brief 圆弧连接检查输出
 * @en_name GeoCircleCheckOutput
 * @cn_name 圆弧连接检查输出
 * @tag STRUCT
 * @field name=PosRelaX, type=Real, unit=m, desc=相对坐标X
 * @field name=PosRelaY, type=Real, unit=m, desc=相对坐标Y
 * @field name=PosRelaTheta, type=Real, unit=rad, desc=相对航向
 * @field name=COk, type=Boolean, unit=1, desc=连接许可标志
 */
typedef struct {
  Real PosRelaX;
  Real PosRelaY;
  Real PosRelaTheta;
  Boolean COk;
} GeoCircleCheckOutput_t;

void geoCircleCheck(const GeoCircleCheckInput_t *input, GeoCircleCheckOutput_t *output);

#endif
