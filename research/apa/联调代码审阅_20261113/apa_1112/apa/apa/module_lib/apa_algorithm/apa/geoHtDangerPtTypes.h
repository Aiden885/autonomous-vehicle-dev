#ifndef GEO_HT_DANGER_PT_TYPES_H
#define GEO_HT_DANGER_PT_TYPES_H

#include "baseTypes.h"

#ifndef APA_PI
#define APA_PI 3.14159265f
#endif

/** @brief 危险点检测最大线段数量 */
#ifndef DGR_LINE_MAX
#define DGR_LINE_MAX 64
#endif

/** @brief 危险点检测最大离散点数量 */
#ifndef DGR_PT_MAX
#define DGR_PT_MAX 128
#endif

/**
 * @brief HT危险点检测输入
 * @en_name GeoHtDangerPtInput
 * @cn_name HT危险点检测输入
 * @tag STRUCT
 * @field name=SltEntrPt1X, type=Real, unit=m, desc=入口边点1X
 * @field name=SltEntrPt1Y, type=Real, unit=m, desc=入口边点1Y
 * @field name=SltEntrPt2X, type=Real, unit=m, desc=入口边点2X
 * @field name=SltEntrPt2Y, type=Real, unit=m, desc=入口边点2Y
 * @field name=TgtX, type=Real, unit=m, desc=目标位姿X
 * @field name=TgtY, type=Real, unit=m, desc=目标位姿Y
 * @field name=TgtTheta, type=Real, unit=rad, desc=目标位姿航向
 * @field name=VehX, type=Real, unit=m, desc=车辆位姿X
 * @field name=VehY, type=Real, unit=m, desc=车辆位姿Y
 * @field name=VehTheta, type=Real, unit=rad, desc=车辆位姿航向
 * @field name=Pt1X, type=Real, unit=m, desc=前后揉库参考点X
 * @field name=Pt1Y, type=Real, unit=m, desc=前后揉库参考点Y
 * @field name=Pt1Theta, type=Real, unit=rad, desc=前后揉库参考点航向
 * @field name=Radius, type=Real, unit=m, desc=转弯半径
 * @field name=PrkType, type=Integer, unit=1, desc=泊车方向1左2右
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=真实边界线数量
 * @field name=BoundaryP1X, type=Array<Real, [64]>, unit=m, desc=真实边界点1X
 * @field name=BoundaryP1Y, type=Array<Real, [64]>, unit=m, desc=真实边界点1Y
 * @field name=BoundaryP2X, type=Array<Real, [64]>, unit=m, desc=真实边界点2X
 * @field name=BoundaryP2Y, type=Array<Real, [64]>, unit=m, desc=真实边界点2Y
 * @field name=VirtualBoundaryLineNum, type=Integer, unit=1, desc=虚拟边界线数量
 * @field name=VirtualBoundaryP1X, type=Array<Real, [64]>, unit=m, desc=虚拟边界点1X
 * @field name=VirtualBoundaryP1Y, type=Array<Real, [64]>, unit=m, desc=虚拟边界点1Y
 * @field name=VirtualBoundaryP2X, type=Array<Real, [64]>, unit=m, desc=虚拟边界点2X
 * @field name=VirtualBoundaryP2Y, type=Array<Real, [64]>, unit=m, desc=虚拟边界点2Y
 * @field name=BoundarySepPtNum, type=Integer, unit=1, desc=真实离散点数量
 * @field name=BoundarySepPtX, type=Array<Real, [128]>, unit=m, desc=真实离散点X
 * @field name=BoundarySepPtY, type=Array<Real, [128]>, unit=m, desc=真实离散点Y
 * @field name=StopperPtNum, type=Integer, unit=1, desc=挡停点数量
 * @field name=StopperPtX, type=Array<Real, [128]>, unit=m, desc=挡停点X
 * @field name=StopperPtY, type=Array<Real, [128]>, unit=m, desc=挡停点Y
 * @field name=LLf, type=Real, unit=m, desc=前悬距离
 * @field name=VehW, type=Real, unit=m, desc=车辆宽度
 * @version 1.0
 * @date 2026-03-15
 * @author aide
 */
typedef struct {
  Real SltEntrPt1X;
  Real SltEntrPt1Y;
  Real SltEntrPt2X;
  Real SltEntrPt2Y;
  Real TgtX;
  Real TgtY;
  Real TgtTheta;
  Real VehX;
  Real VehY;
  Real VehTheta;
  Real Pt1X;
  Real Pt1Y;
  Real Pt1Theta;
  Real Radius;
  Integer PrkType;
  Integer BoundaryLineNum;
  Real BoundaryP1X[DGR_LINE_MAX];
  Real BoundaryP1Y[DGR_LINE_MAX];
  Real BoundaryP2X[DGR_LINE_MAX];
  Real BoundaryP2Y[DGR_LINE_MAX];
  Integer VirtualBoundaryLineNum;
  Real VirtualBoundaryP1X[DGR_LINE_MAX];
  Real VirtualBoundaryP1Y[DGR_LINE_MAX];
  Real VirtualBoundaryP2X[DGR_LINE_MAX];
  Real VirtualBoundaryP2Y[DGR_LINE_MAX];
  Integer BoundarySepPtNum;
  Real BoundarySepPtX[DGR_PT_MAX];
  Real BoundarySepPtY[DGR_PT_MAX];
  Integer StopperPtNum;
  Real StopperPtX[DGR_PT_MAX];
  Real StopperPtY[DGR_PT_MAX];
  Real LLf;
  Real VehW;
} GeoHtDangerPtInput_t;

/**
 * @brief HT危险点检测输出
 * @en_name GeoHtDangerPtOutput
 * @cn_name HT危险点检测输出
 * @tag STRUCT
 * @field name=PtEx, type=Integer, unit=1, desc=最终危险点标志0无1真实2虚拟
 * @field name=DangerPtX, type=Real, unit=m, desc=最终危险点X
 * @field name=DangerPtY, type=Real, unit=m, desc=最终危险点Y
 * @field name=DgrPtRX, type=Real, unit=m, desc=最终危险点目标坐标系X
 * @field name=DgrPtRY, type=Real, unit=m, desc=最终危险点目标坐标系Y
 * @field name=PtExReal, type=Integer, unit=1, desc=真实边界危险点标志
 * @field name=DangerPtRealX, type=Real, unit=m, desc=真实边界危险点X
 * @field name=DangerPtRealY, type=Real, unit=m, desc=真实边界危险点Y
 * @field name=PtExVirtual, type=Integer, unit=1, desc=虚拟边界危险点标志
 * @field name=DangerPtVirtualX, type=Real, unit=m, desc=虚拟边界危险点X
 * @field name=DangerPtVirtualY, type=Real, unit=m, desc=虚拟边界危险点Y
 * @field name=CheckCount, type=Integer, unit=1, desc=有效检查次数
 * @version 1.0
 * @date 2026-03-15
 * @author aide
 */
typedef struct {
  Integer PtEx;
  Real DangerPtX;
  Real DangerPtY;
  Real DgrPtRX;
  Real DgrPtRY;
  Integer PtExReal;
  Real DangerPtRealX;
  Real DangerPtRealY;
  Integer PtExVirtual;
  Real DangerPtVirtualX;
  Real DangerPtVirtualY;
  Integer CheckCount;
} GeoHtDangerPtOutput_t;

void geoHtDangerPt(const GeoHtDangerPtInput_t *input, GeoHtDangerPtOutput_t *output);

#endif
