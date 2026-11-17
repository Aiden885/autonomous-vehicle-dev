#ifndef GEO_VT_1R_FIX_TYPES_H
#define GEO_VT_1R_FIX_TYPES_H

#include "baseTypes.h"
#include "vehicleCollisionCheckTypes.h"
#include "geoCircleIntrvCheckTypes.h"

/** @brief 圆周率 */
#ifndef APA_PI
#define APA_PI 3.14159265f
#endif
/** @brief 两倍圆周率 */
#ifndef APA_TWO_PI
#define APA_TWO_PI 6.2831853f
#endif
/** @brief 除零保护精度 */
#ifndef GEO_EPSILON
#define GEO_EPSILON 1e-6f
#endif

/**
 * @brief VT单半径修正输入
 * @en_name GeoVt1RFixInput
 * @cn_name VT单半径修正输入
 * @tag STRUCT
 * @field name=P0X, type=Real, unit=m, desc=车端起点X
 * @field name=P0Y, type=Real, unit=m, desc=车端起点Y
 * @field name=P0Theta, type=Real, unit=rad, desc=车端起点航向
 * @field name=P1X, type=Real, unit=m, desc=弧线起点X
 * @field name=P1Y, type=Real, unit=m, desc=弧线起点Y
 * @field name=P1Theta, type=Real, unit=rad, desc=弧线起点航向
 * @field name=CenterX, type=Real, unit=m, desc=圆心X
 * @field name=CenterY, type=Real, unit=m, desc=圆心Y
 * @field name=TurnDir, type=Integer, unit=1, desc=转弯方向1左转负1右转
 * @field name=Curve, type=Real, unit=1/m, desc=曲率
 * @field name=PendX, type=Real, unit=m, desc=终点X
 * @field name=PendY, type=Real, unit=m, desc=终点Y
 * @field name=PendTheta, type=Real, unit=rad, desc=终点航向
 * @field name=Rd1, type=Real, unit=m, desc=双圆连接段1半径
 * @field name=Rd2, type=Real, unit=m, desc=双圆连接段2半径
 * @field name=UseFullSearch, type=Boolean, unit=1, desc=是否启用候选scale搜索
 * @field name=VccTemplate, type=VehicleCollisionCheckInput_t, unit=1, desc=快速碰撞检查模板
 * @field name=IntrvTemplate, type=GeoCircleIntrvCheckInput_t, unit=1, desc=圆弧干涉检查模板
 * @version 2.0
 * @date 2026-05-01
 * @author aide
 */
typedef struct {
  Real P0X;
  Real P0Y;
  Real P0Theta;
  Real P1X;
  Real P1Y;
  Real P1Theta;
  Real CenterX;
  Real CenterY;
  Integer TurnDir;
  Real Curve;
  Real PendX;
  Real PendY;
  Real PendTheta;
  Real Rd1;
  Real Rd2;
  Boolean UseFullSearch;
  VehicleCollisionCheckInput_t VccTemplate;
  GeoCircleIntrvCheckInput_t IntrvTemplate;
} GeoVt1RFixInput_t;

/**
 * @brief VT单半径修正输出
 * @en_name GeoVt1RFixOutput
 * @cn_name VT单半径修正输出
 * @tag STRUCT
 * @field name=Success, type=Integer, unit=1, desc=成功标志1成功0失败
 * @field name=FixedRadius, type=Real, unit=m, desc=修正后半径
 * @field name=DeltaTheta, type=Real, unit=rad, desc=p0/p1夹角
 * @field name=ArcLen, type=Real, unit=m, desc=p0->p1弧长
 * @field name=AnchorX, type=Array<Real, [4]>, unit=m, desc=4段锚点X
 * @field name=AnchorY, type=Array<Real, [4]>, unit=m, desc=4段锚点Y
 * @field name=AnchorTheta, type=Array<Real, [4]>, unit=rad, desc=4段锚点航向
 * @field name=SegCenterX, type=Array<Real, [4]>, unit=m, desc=4段圆心X
 * @field name=SegCenterY, type=Array<Real, [4]>, unit=m, desc=4段圆心Y
 * @field name=SegCurve, type=Array<Real, [4]>, unit=1/m, desc=4段曲率
 * @field name=SegGoDir, type=Array<Integer, [4]>, unit=1, desc=4段行驶方向
 * @field name=SegTurnDir, type=Array<Integer, [4]>, unit=1, desc=4段转弯方向
 * @field name=SegDTheta, type=Array<Real, [4]>, unit=rad, desc=4段角度变化
 * @field name=SegLength, type=Array<Real, [4]>, unit=m, desc=4段长度
 * @version 2.0
 * @date 2026-05-01
 * @author aide
 */
typedef struct {
  Integer Success;
  Real FixedRadius;
  Real DeltaTheta;
  Real ArcLen;
  Real AnchorX[4];
  Real AnchorY[4];
  Real AnchorTheta[4];
  Real SegCenterX[4];
  Real SegCenterY[4];
  Real SegCurve[4];
  Integer SegGoDir[4];
  Integer SegTurnDir[4];
  Real SegDTheta[4];
  Real SegLength[4];
} GeoVt1RFixOutput_t;

void geoVt1RFix(const GeoVt1RFixInput_t *input, GeoVt1RFixOutput_t *output);

#endif
