#ifndef GEO_LINE_CROSS_PT_TYPES_H
#define GEO_LINE_CROSS_PT_TYPES_H

#include "baseTypes.h"

/**
 * @brief 两直线交点输入
 * @en_name GeoLineCrossPtInput
 * @cn_name 两直线交点输入
 * @tag STRUCT
 * @field name=Ln1X0, type=Real, unit=m, desc=直线1起点X
 * @field name=Ln1Y0, type=Real, unit=m, desc=直线1起点Y
 * @field name=Ln1X1, type=Real, unit=m, desc=直线1终点X
 * @field name=Ln1Y1, type=Real, unit=m, desc=直线1终点Y
 * @field name=Ln2X0, type=Real, unit=m, desc=直线2起点X
 * @field name=Ln2Y0, type=Real, unit=m, desc=直线2起点Y
 * @field name=Ln2X1, type=Real, unit=m, desc=直线2终点X
 * @field name=Ln2Y1, type=Real, unit=m, desc=直线2终点Y
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real Ln1X0; // 直线1起点X
  Real Ln1Y0; // 直线1起点Y
  Real Ln1X1; // 直线1终点X
  Real Ln1Y1; // 直线1终点Y
  Real Ln2X0; // 直线2起点X
  Real Ln2Y0; // 直线2起点Y
  Real Ln2X1; // 直线2终点X
  Real Ln2Y1; // 直线2终点Y
} GeoLineCrossPtInput_t;

/**
 * @brief 两直线交点输出
 * @en_name GeoLineCrossPtOutput
 * @cn_name 两直线交点输出
 * @tag STRUCT
 * @field name=CrossX, type=Real, unit=m, desc=交点X坐标
 * @field name=CrossY, type=Real, unit=m, desc=交点Y坐标
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real CrossX; // 交点X坐标
  Real CrossY; // 交点Y坐标
} GeoLineCrossPtOutput_t;

void geoLineCrossPt(const GeoLineCrossPtInput_t *input, GeoLineCrossPtOutput_t *output);

#endif
