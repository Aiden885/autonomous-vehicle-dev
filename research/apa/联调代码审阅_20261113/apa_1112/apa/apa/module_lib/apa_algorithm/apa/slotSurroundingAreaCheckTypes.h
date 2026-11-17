#ifndef SLOT_SURROUNDING_AREA_CHECK_TYPES_H
#define SLOT_SURROUNDING_AREA_CHECK_TYPES_H

#include "baseTypes.h"
#include "geoSurroundingCheckTypes.h"
#include "vehicleCollisionCheckTypes.h"

/**
 * @brief 车位周围区域检查输入
 * @en_name SlotSurroundingAreaCheckInput
 * @cn_name 车位周围区域检查输入
 * @tag STRUCT
 * @field name=TarPtX, type=Real, unit=m, desc=目标点X
 * @field name=TarPtY, type=Real, unit=m, desc=目标点Y
 * @field name=TarPtTheta, type=Real, unit=rad, desc=目标点航向角
 * @field name=SltPt1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SltPt1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SltPt2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SltPt2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SltPt3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SltPt3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SltPt4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SltPt4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=SltEntr, type=Integer, unit=1, desc=入口边
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=真实边界线数量
 * @field name=BoundaryP1X, type=Array<Real, [16]>, unit=m, desc=真实边界点1X
 * @field name=BoundaryP1Y, type=Array<Real, [16]>, unit=m, desc=真实边界点1Y
 * @field name=BoundaryP2X, type=Array<Real, [16]>, unit=m, desc=真实边界点2X
 * @field name=BoundaryP2Y, type=Array<Real, [16]>, unit=m, desc=真实边界点2Y
 * @field name=BoundarySepPtNum, type=Integer, unit=1, desc=真实离散点数量
 * @field name=BoundarySepPtX, type=Array<Real, [32]>, unit=m, desc=真实离散点X
 * @field name=BoundarySepPtY, type=Array<Real, [32]>, unit=m, desc=真实离散点Y
 * @field name=SeparateStopperNum, type=Integer, unit=1, desc=离散挡停点数量
 * @field name=SeparateStopperX, type=Array<Real, [128]>, unit=m, desc=离散挡停点X
 * @field name=SeparateStopperY, type=Array<Real, [128]>, unit=m, desc=离散挡停点Y
 * @field name=SlotTrackStopperNum, type=Integer, unit=1, desc=车位跟踪挡停点数量
 * @field name=SlotTrackStopperX1, type=Array<Real, [64]>, unit=m, desc=跟踪挡停点端点1X
 * @field name=SlotTrackStopperY1, type=Array<Real, [64]>, unit=m, desc=跟踪挡停点端点1Y
 * @field name=SlotTrackStopperX2, type=Array<Real, [64]>, unit=m, desc=跟踪挡停点端点2X
 * @field name=SlotTrackStopperY2, type=Array<Real, [64]>, unit=m, desc=跟踪挡停点端点2Y
 * @field name=VirtualBoundaryLineNum, type=Integer, unit=1, desc=虚拟边界线数量
 * @field name=VirtualBoundaryLineX1, type=Array<Real, [64]>, unit=m, desc=虚拟边界起点X
 * @field name=VirtualBoundaryLineY1, type=Array<Real, [64]>, unit=m, desc=虚拟边界起点Y
 * @field name=VirtualBoundaryLineX2, type=Array<Real, [64]>, unit=m, desc=虚拟边界终点X
 * @field name=VirtualBoundaryLineY2, type=Array<Real, [64]>, unit=m, desc=虚拟边界终点Y
 * @version 1.0
 * @date 2026-04-19
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
  Integer SeparateStopperNum;
  Real SeparateStopperX[VCC_MAX_SEPARATE_PTS];
  Real SeparateStopperY[VCC_MAX_SEPARATE_PTS];
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
} SlotSurroundingAreaCheckInput_t;

/**
 * @brief 车位周围区域检查输出
 * @en_name SlotSurroundingAreaCheckOutput
 * @cn_name 车位周围区域检查输出
 * @tag STRUCT
 * @field name=SurroundingType, type=Integer, unit=1, desc=周边环境类型
 * @field name=ExistStopperInSlot, type=Integer, unit=1, desc=车位内是否存在挡停点
 * @version 1.0
 * @date 2026-04-19
 * @author aide
 */
typedef struct {
  Integer SurroundingType;
  Integer ExistStopperInSlot;
} SlotSurroundingAreaCheckOutput_t;

void slotSurroundingAreaCheck(const SlotSurroundingAreaCheckInput_t *input,
                              SlotSurroundingAreaCheckOutput_t *output);

#endif
