#ifndef GEO_2_CIRCLE_LINK_VT_TYPES_H
#define GEO_2_CIRCLE_LINK_VT_TYPES_H

#include "baseTypes.h"

/** @brief 圆周率 */
#ifndef APA_PI
#define APA_PI 3.14159265f
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
 * @brief VT双圆弧连接输入
 * @en_name Geo2CircleLinkVTInput
 * @cn_name VT双圆弧连接输入
 * @tag STRUCT
 * @field name=Pt1X, type=Real, unit=m, desc=起点X
 * @field name=Pt1Y, type=Real, unit=m, desc=起点Y
 * @field name=Pt1Theta, type=Real, unit=rad, desc=起点航向角
 * @field name=Pt2X, type=Real, unit=m, desc=终点X
 * @field name=Pt2Y, type=Real, unit=m, desc=终点Y
 * @field name=Pt2Theta, type=Real, unit=rad, desc=终点航向角
 * @field name=Dir, type=Integer, unit=1, desc=方向负1左侧模式正1右侧模式
 * @field name=R1, type=Real, unit=m, desc=第一段圆弧半径
 * @field name=R2, type=Real, unit=m, desc=第二段圆弧半径
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real Pt1X;
  Real Pt1Y;
  Real Pt1Theta;
  Real Pt2X;
  Real Pt2Y;
  Real Pt2Theta;
  Integer Dir;
  Real R1;
  Real R2;
} Geo2CircleLinkVTInput_t;

/**
 * @brief VT双圆弧连接输出
 * @en_name Geo2CircleLinkVTOutput
 * @cn_name VT双圆弧连接输出
 * @tag STRUCT
 * @field name=PosRelaX, type=Real, unit=m, desc=起点在终点坐标系X
 * @field name=PosRelaY, type=Real, unit=m, desc=起点在终点坐标系Y
 * @field name=PosRelaTheta, type=Real, unit=rad, desc=起点在终点坐标系航向
 * @field name=Ct1X, type=Real, unit=m, desc=圆心1 X
 * @field name=Ct1Y, type=Real, unit=m, desc=圆心1 Y
 * @field name=Ct2X, type=Real, unit=m, desc=圆心2 X
 * @field name=Ct2Y, type=Real, unit=m, desc=圆心2 Y
 * @field name=St2X, type=Real, unit=m, desc=衔接点X
 * @field name=St2Y, type=Real, unit=m, desc=衔接点Y
 * @field name=St2Theta, type=Real, unit=rad, desc=衔接点航向
 * @field name=St3X, type=Real, unit=m, desc=终端点X
 * @field name=St3Y, type=Real, unit=m, desc=终端点Y
 * @field name=St3Theta, type=Real, unit=rad, desc=终端点航向
 * @field name=St3RelX, type=Real, unit=m, desc=终端点相对坐标X
 * @field name=St3RelY, type=Real, unit=m, desc=终端点相对坐标Y
 * @field name=St3RelTheta, type=Real, unit=rad, desc=终端点相对坐标航向
 * @field name=Tht1, type=Real, unit=rad, desc=第一段圆弧角
 * @field name=Tht2, type=Real, unit=rad, desc=第二段圆弧角
 * @field name=TrnD1, type=Integer, unit=1, desc=第一段转向
 * @field name=TrnD2, type=Integer, unit=1, desc=第二段转向
 * @field name=Len1, type=Real, unit=m, desc=第一段弧长
 * @field name=Len2, type=Real, unit=m, desc=第二段弧长
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real PosRelaX;
  Real PosRelaY;
  Real PosRelaTheta;
  Real Ct1X;
  Real Ct1Y;
  Real Ct2X;
  Real Ct2Y;
  Real St2X;
  Real St2Y;
  Real St2Theta;
  Real St3X;
  Real St3Y;
  Real St3Theta;
  Real St3RelX;
  Real St3RelY;
  Real St3RelTheta;
  Real Tht1;
  Real Tht2;
  Integer TrnD1;
  Integer TrnD2;
  Real Len1;
  Real Len2;
} Geo2CircleLinkVTOutput_t;

void geo2CircleLinkVT(const Geo2CircleLinkVTInput_t *input, Geo2CircleLinkVTOutput_t *output);

#endif
