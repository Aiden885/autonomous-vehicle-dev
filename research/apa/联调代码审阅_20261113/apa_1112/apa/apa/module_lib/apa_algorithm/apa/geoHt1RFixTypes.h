#ifndef GEO_HT_1R_FIX_TYPES_H
#define GEO_HT_1R_FIX_TYPES_H

#include "baseTypes.h"
#include "geo2CircleLinkHTTypes.h"
#include "geoCircleIntrvCheckTypes.h"
#include "geoDivideTypes.h"
#include "apaMod2piTypes.h"

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

/** @brief 1RFix 最大分段数 */
#ifndef HT_1R_SEG_MAX
#define HT_1R_SEG_MAX 5
#endif

/**
 * @brief HT单弯修正输入
 * @en_name GeoHt1RFixInput
 * @cn_name HT单弯修正输入
 * @tag STRUCT
 * @field name=P0X, type=Real, unit=m, desc=起点X
 * @field name=P0Y, type=Real, unit=m, desc=起点Y
 * @field name=P0Theta, type=Real, unit=rad, desc=起点航向
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
 * @field name=RdLs1, type=Real, unit=m, desc=双圆连接半径1
 * @field name=RdLs2, type=Real, unit=m, desc=双圆连接半径2
 * @field name=VehWidth, type=Real, unit=m, desc=车宽
 * @field name=VehFrontOffset, type=Real, unit=m, desc=车辆前包络偏置
 * @field name=VehRearOffset, type=Real, unit=m, desc=车辆后包络偏置
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=障碍边界线段数
 * @field name=SceneQuery, type=ApaPlanningSceneQuery_t, unit=1, desc=同一规划进程内的完整实体场景查询
 * @field name=SlotP1X, type=Real, unit=m, desc=车位角点1世界X坐标
 * @field name=SlotP1Y, type=Real, unit=m, desc=车位角点1世界Y坐标
 * @field name=SlotP2X, type=Real, unit=m, desc=车位角点2世界X坐标
 * @field name=SlotP2Y, type=Real, unit=m, desc=车位角点2世界Y坐标
 * @field name=SlotP3X, type=Real, unit=m, desc=车位角点3世界X坐标
 * @field name=SlotP3Y, type=Real, unit=m, desc=车位角点3世界Y坐标
 * @field name=SlotP4X, type=Real, unit=m, desc=车位角点4世界X坐标
 * @field name=SlotP4Y, type=Real, unit=m, desc=车位角点4世界Y坐标
 * @field name=BoundaryLineX1, type=Array<Real, [64]>, unit=m, desc=物理边界线段起点世界X坐标
 * @field name=BoundaryLineY1, type=Array<Real, [64]>, unit=m, desc=物理边界线段起点世界Y坐标
 * @field name=BoundaryLineX2, type=Array<Real, [64]>, unit=m, desc=物理边界线段终点世界X坐标
 * @field name=BoundaryLineY2, type=Array<Real, [64]>, unit=m, desc=物理边界线段终点世界Y坐标
 * @field name=VirtualBoundaryLineNum, type=Integer, unit=1, desc=虚拟约束线段数量
 * @field name=VirtualBoundaryLineX1, type=Array<Real, [64]>, unit=m, desc=虚拟约束线段起点世界X坐标
 * @field name=VirtualBoundaryLineY1, type=Array<Real, [64]>, unit=m, desc=虚拟约束线段起点世界Y坐标
 * @field name=VirtualBoundaryLineX2, type=Array<Real, [64]>, unit=m, desc=虚拟约束线段终点世界X坐标
 * @field name=VirtualBoundaryLineY2, type=Array<Real, [64]>, unit=m, desc=虚拟约束线段终点世界Y坐标
 * @field name=BoundarySepPtNum, type=Integer, unit=1, desc=物理边界离散点数量
 * @field name=BoundarySepPtX, type=Array<Real, [128]>, unit=m, desc=物理边界离散点世界X坐标
 * @field name=BoundarySepPtY, type=Array<Real, [128]>, unit=m, desc=物理边界离散点世界Y坐标
 * @field name=StopperPtNum, type=Integer, unit=1, desc=独立轮挡点数量
 * @field name=StopperPtX, type=Array<Real, [128]>, unit=m, desc=独立轮挡点世界X坐标
 * @field name=StopperPtY, type=Array<Real, [128]>, unit=m, desc=独立轮挡点世界Y坐标
 * @version 1.0
 * @date 2026-03-15
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
  Real RdLs1;
  Real RdLs2;
  Real VehWidth;
  Real VehFrontOffset;
  Real VehRearOffset;
  Real SlotP1X;
  Real SlotP1Y;
  Real SlotP2X;
  Real SlotP2Y;
  Real SlotP3X;
  Real SlotP3Y;
  Real SlotP4X;
  Real SlotP4Y;
  Integer BoundaryLineNum;
  Real BoundaryLineX1[GEO_BOUNDARY_LINE_MAX];
  Real BoundaryLineY1[GEO_BOUNDARY_LINE_MAX];
  Real BoundaryLineX2[GEO_BOUNDARY_LINE_MAX];
  Real BoundaryLineY2[GEO_BOUNDARY_LINE_MAX];
  Integer VirtualBoundaryLineNum;
  Real VirtualBoundaryLineX1[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineY1[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineX2[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineY2[VCC_MAX_VIRTUAL_LINES];
  Integer BoundarySepPtNum;
  Real BoundarySepPtX[VCC_MAX_SEPARATE_PTS];
  Real BoundarySepPtY[VCC_MAX_SEPARATE_PTS];
  Integer StopperPtNum;
  Real StopperPtX[VCC_MAX_SEPARATE_PTS];
  Real StopperPtY[VCC_MAX_SEPARATE_PTS];
  ApaPlanningSceneQuery_t SceneQuery;
} GeoHt1RFixInput_t;

/**
 * @brief HT单弯修正输出 — 5段路径结果
 * @en_name GeoHt1RFixOutput
 * @cn_name HT单弯修正输出
 * @tag STRUCT
 * @field name=Success, type=Integer, unit=1, desc=成功标志
 * @field name=SegNum, type=Integer, unit=1, desc=有效段数
 * @field name=AnchorX, type=Array<Real, [5]>, unit=m, desc=锚点X
 * @field name=AnchorY, type=Array<Real, [5]>, unit=m, desc=锚点Y
 * @field name=AnchorTheta, type=Array<Real, [5]>, unit=rad, desc=锚点航向
 * @field name=CtrX, type=Array<Real, [5]>, unit=m, desc=圆心X
 * @field name=CtrY, type=Array<Real, [5]>, unit=m, desc=圆心Y
 * @field name=Curves, type=Array<Real, [5]>, unit=1/m, desc=各段曲率
 * @field name=GoDirs, type=Array<Integer, [5]>, unit=1, desc=各段行驶方向
 * @field name=TurnDirs, type=Array<Integer, [5]>, unit=1, desc=各段转弯方向
 * @field name=DThetas, type=Array<Real, [5]>, unit=rad, desc=各段转角
 * @field name=Lengths, type=Array<Real, [5]>, unit=m, desc=各段弧长
 * @version 1.0
 * @date 2026-03-15
 * @author aide
 */
typedef struct {
  Integer Success;
  Integer SegNum;
  Real AnchorX[HT_1R_SEG_MAX];
  Real AnchorY[HT_1R_SEG_MAX];
  Real AnchorTheta[HT_1R_SEG_MAX];
  Real CtrX[HT_1R_SEG_MAX];
  Real CtrY[HT_1R_SEG_MAX];
  Real Curves[HT_1R_SEG_MAX];
  Integer GoDirs[HT_1R_SEG_MAX];
  Integer TurnDirs[HT_1R_SEG_MAX];
  Real DThetas[HT_1R_SEG_MAX];
  Real Lengths[HT_1R_SEG_MAX];
} GeoHt1RFixOutput_t;

void geoHt1RFix(const GeoHt1RFixInput_t *input, GeoHt1RFixOutput_t *output);

#endif
