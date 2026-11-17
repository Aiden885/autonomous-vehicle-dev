#ifndef GEO_VH_ENTR_PT_DNG_PT_TYPES_H
#define GEO_VH_ENTR_PT_DNG_PT_TYPES_H

#include "baseTypes.h"
#include "geoAxisTransTypes.h"
#include "geoDivideTypes.h"
#include "geoLineCrossPtTypes.h"

/** @brief 入口危险点输入最大线段数量 */
#ifndef DGR_LINE_MAX
#define DGR_LINE_MAX 64
#endif

/** @brief 入口危险点输入最大离散点数量 */
#ifndef DGR_PT_MAX
#define DGR_PT_MAX 128
#endif

/**
 * @brief VH入口点及危险点检测输入
 * @en_name GeoVhEntrPtDngPtInput
 * @cn_name VH入口点及危险点检测输入
 * @tag STRUCT
 * @field name=SltPt1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SltPt1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SltPt2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SltPt2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SltPt3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SltPt3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SltPt4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SltPt4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=SltEntr, type=Integer, unit=1, desc=车位入口边编号
 * @field name=VehX, type=Real, unit=m, desc=车辆X
 * @field name=VehY, type=Real, unit=m, desc=车辆Y
 * @field name=VehTheta, type=Real, unit=rad, desc=车辆航向
 * @field name=TgtX, type=Real, unit=m, desc=目标X
 * @field name=TgtY, type=Real, unit=m, desc=目标Y
 * @field name=TgtTheta, type=Real, unit=rad, desc=目标航向
 * @field name=Radius, type=Real, unit=m, desc=规划转弯半径
 * @field name=PrkType, type=Integer, unit=1, desc=泊车方向1左2右
 * @field name=SurdType, type=Integer, unit=1, desc=周边类型0无障碍1左开2右开3双侧受限
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
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real SltPt1X;
  Real SltPt1Y;
  Real SltPt2X;
  Real SltPt2Y;
  Real SltPt3X;
  Real SltPt3Y;
  Real SltPt4X;
  Real SltPt4Y;
  Integer SltEntr;
  Real VehX;
  Real VehY;
  Real VehTheta;
  Real TgtX;
  Real TgtY;
  Real TgtTheta;
  Real Radius;
  Integer PrkType;
  Integer SurdType;
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
} GeoVhEntrPtDngPtInput_t;

/**
 * @brief VH入口点及危险点检测输出
 * @en_name GeoVhEntrPtDngPtOutput
 * @cn_name VH入口点及危险点检测输出
 * @tag STRUCT
 * @field name=X1, type=Real, unit=m, desc=修正后的入口点X
 * @field name=Y1, type=Real, unit=m, desc=修正后的入口点Y
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real X1;
  Real Y1;
} GeoVhEntrPtDngPtOutput_t;

void geoVhEntrPtDngPt(const GeoVhEntrPtDngPtInput_t *input, GeoVhEntrPtDngPtOutput_t *output);

#endif
