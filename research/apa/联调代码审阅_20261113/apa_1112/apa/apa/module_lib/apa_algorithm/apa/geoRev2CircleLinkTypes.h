#ifndef GEO_REV_2_CIRCLE_LINK_TYPES_H
#define GEO_REV_2_CIRCLE_LINK_TYPES_H

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
 * @brief 换向双圆弧连接输入
 * @en_name GeoRev2CircleLinkInput
 * @cn_name 换向双圆弧连接输入
 * @tag STRUCT
 * @field name=Pt1X, type=Real, unit=m, desc=起点X
 * @field name=Pt1Y, type=Real, unit=m, desc=起点Y
 * @field name=Pt1Theta, type=Real, unit=rad, desc=起点航向角
 * @field name=Pt2X, type=Real, unit=m, desc=终点X
 * @field name=Pt2Y, type=Real, unit=m, desc=终点Y
 * @field name=Pt2Theta, type=Real, unit=rad, desc=终点航向角
 * @field name=Dir, type=Integer, unit=1, desc=行驶方向1前进负1后退
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
} GeoRev2CircleLinkInput_t;

/**
 * @brief 换向双圆弧连接输出
 * @en_name GeoRev2CircleLinkOutput
 * @cn_name 换向双圆弧连接输出
 * @tag STRUCT
 * @field name=PosRelaX, type=Real, unit=m, desc=Pt1在Pt2坐标系X
 * @field name=PosRelaY, type=Real, unit=m, desc=Pt1在Pt2坐标系Y
 * @field name=PosRelaTheta, type=Real, unit=rad, desc=Pt1在Pt2坐标系航向
 * @field name=Ct1X, type=Real, unit=m, desc=圆心1 X
 * @field name=Ct1Y, type=Real, unit=m, desc=圆心1 Y
 * @field name=Ct2X, type=Real, unit=m, desc=圆心2 X
 * @field name=Ct2Y, type=Real, unit=m, desc=圆心2 Y
 * @field name=St2X, type=Real, unit=m, desc=衔接点X
 * @field name=St2Y, type=Real, unit=m, desc=衔接点Y
 * @field name=St2Theta, type=Real, unit=rad, desc=衔接点航向角
 * @field name=St3X, type=Real, unit=m, desc=终端点X
 * @field name=St3Y, type=Real, unit=m, desc=终端点Y
 * @field name=St3Theta, type=Real, unit=rad, desc=终端点航向角
 * @field name=St3rX, type=Real, unit=m, desc=终端点在Pt2坐标系X
 * @field name=St3rY, type=Real, unit=m, desc=终端点在Pt2坐标系Y
 * @field name=St3rTheta, type=Real, unit=rad, desc=终端点在Pt2坐标系航向
 * @field name=Tht1, type=Real, unit=rad, desc=第一段圆弧角
 * @field name=Tht2, type=Real, unit=rad, desc=第二段圆弧角
 * @field name=TrnD1, type=Integer, unit=1, desc=第一段转弯方向
 * @field name=TrnD2, type=Integer, unit=1, desc=第二段转弯方向
 * @field name=Len1, type=Real, unit=m, desc=第一段弧长
 * @field name=Len2, type=Real, unit=m, desc=第二段弧长
 * @field name=GeoOk, type=Integer, unit=1, desc=几何可行标志1可行0不可行
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
  Real St3rX;
  Real St3rY;
  Real St3rTheta;
  Real Tht1;
  Real Tht2;
  Integer TrnD1;
  Integer TrnD2;
  Real Len1;
  Real Len2;
  Integer GeoOk;
} GeoRev2CircleLinkOutput_t;

void geoRev2CircleLink(const GeoRev2CircleLinkInput_t *input, GeoRev2CircleLinkOutput_t *output);

#endif
