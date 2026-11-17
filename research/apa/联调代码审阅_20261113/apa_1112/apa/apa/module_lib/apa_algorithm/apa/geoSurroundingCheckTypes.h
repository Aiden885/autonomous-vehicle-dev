#ifndef GEO_SURROUNDING_CHECK_TYPES_H
#define GEO_SURROUNDING_CHECK_TYPES_H

#include "baseTypes.h"
#include "checkParkingTypeTypes.h"
#include "vehicleCollisionCheckTypes.h"

/** @brief 障碍物最大边界线数量 */
#ifndef SURD_LINE_MAX
#define SURD_LINE_MAX 16
#endif

/** @brief 障碍物最大点数量 */
#ifndef SURD_PT_MAX
#define SURD_PT_MAX 32
#endif

/**
 * @brief 环绕检测输入
 * @en_name GeoSurroundingCheckInput
 * @cn_name 环绕检测输入
 * @tag STRUCT
 * @field name=TarPtX, type=Real, unit=m, desc=目标点X坐标
 * @field name=TarPtY, type=Real, unit=m, desc=目标点Y坐标
 * @field name=TarPtTheta, type=Real, unit=rad, desc=目标点航向角
 * @field name=SltPt1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SltPt1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SltPt2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SltPt2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SltPt3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SltPt3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SltPt4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SltPt4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=SltEntr, type=Integer, unit=1, desc=车位入口边
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=真实边界线数量
 * @field name=BoundaryP1X, type=Array<Real, [16]>, unit=m, desc=真实边界点1X
 * @field name=BoundaryP1Y, type=Array<Real, [16]>, unit=m, desc=真实边界点1Y
 * @field name=BoundaryP2X, type=Array<Real, [16]>, unit=m, desc=真实边界点2X
 * @field name=BoundaryP2Y, type=Array<Real, [16]>, unit=m, desc=真实边界点2Y
 * @field name=BoundarySepPtNum, type=Integer, unit=1, desc=真实离散点数量
 * @field name=BoundarySepPtX, type=Array<Real, [32]>, unit=m, desc=真实离散点X
 * @field name=BoundarySepPtY, type=Array<Real, [32]>, unit=m, desc=真实离散点Y
 * @field name=StopperPtNum, type=Integer, unit=1, desc=挡停点数量
 * @field name=StopperPtX, type=Array<Real, [32]>, unit=m, desc=挡停点X
 * @field name=StopperPtY, type=Array<Real, [32]>, unit=m, desc=挡停点Y
 * @field name=SlotTrackStopperNum, type=Integer, unit=1, desc=slot stopper线段数量
 * @field name=SlotTrackStopperX1, type=Array<Real, [64]>, unit=m, desc=slot stopper端点1X
 * @field name=SlotTrackStopperY1, type=Array<Real, [64]>, unit=m, desc=slot stopper端点1Y
 * @field name=SlotTrackStopperX2, type=Array<Real, [64]>, unit=m, desc=slot stopper端点2X
 * @field name=SlotTrackStopperY2, type=Array<Real, [64]>, unit=m, desc=slot stopper端点2Y
 * @field name=VirtualBoundaryLineNum, type=Integer, unit=1, desc=虚拟边界线数量
 * @field name=VirtualBoundaryLineX1, type=Array<Real, [64]>, unit=m, desc=虚拟边界起点X
 * @field name=VirtualBoundaryLineY1, type=Array<Real, [64]>, unit=m, desc=虚拟边界起点Y
 * @field name=VirtualBoundaryLineX2, type=Array<Real, [64]>, unit=m, desc=虚拟边界终点X
 * @field name=VirtualBoundaryLineY2, type=Array<Real, [64]>, unit=m, desc=虚拟边界终点Y
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real TarPtX;
  Real TarPtY;
  Real TarPtTheta;
  Real SltPt1X;
  Real SltPt1Y;
  Real SltPt2X;
  Real SltPt2Y;
  Real SltPt3X;
  Real SltPt3Y;
  Real SltPt4X;
  Real SltPt4Y;
  Integer SltEntr;
  Integer BoundaryLineNum;
  Real BoundaryP1X[SURD_LINE_MAX];
  Real BoundaryP1Y[SURD_LINE_MAX];
  Real BoundaryP2X[SURD_LINE_MAX];
  Real BoundaryP2Y[SURD_LINE_MAX];
  Integer BoundarySepPtNum;
  Real BoundarySepPtX[SURD_PT_MAX];
  Real BoundarySepPtY[SURD_PT_MAX];
  Integer StopperPtNum;
  Real StopperPtX[SURD_PT_MAX];
  Real StopperPtY[SURD_PT_MAX];
  Integer SlotTrackStopperNum;
  Real SlotTrackStopperX1[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperY1[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperX2[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperY2[VCC_MAX_TRACK_STOPPER];
  Integer VirtualBoundaryLineNum;
  Real VirtualBoundaryLineX1[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineY1[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineX2[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineY2[VCC_MAX_VIRTUAL_LINES];
} GeoSurroundingCheckInput_t;

/**
 * @brief 环绕检测输出
 * @en_name GeoSurroundingCheckOutput
 * @cn_name 环绕检测输出
 * @tag STRUCT
 * @field name=SurroundingType, type=Integer, unit=1, desc=环绕环境类型
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Integer SurroundingType;
} GeoSurroundingCheckOutput_t;

void geoSurroundingCheck(const GeoSurroundingCheckInput_t *input,
                         GeoSurroundingCheckOutput_t *output);

#endif
