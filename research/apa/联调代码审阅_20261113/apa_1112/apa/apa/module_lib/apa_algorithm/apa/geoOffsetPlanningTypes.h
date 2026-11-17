#ifndef GEO_OFFSET_PLANNING_TYPES_H
#define GEO_OFFSET_PLANNING_TYPES_H

#include "baseTypes.h"

/** @brief 障碍物最大边界线数量 */
#ifndef OFS_LINE_MAX
#define OFS_LINE_MAX 16
#endif

/** @brief 障碍物最大点数量 */
#ifndef OFS_PT_MAX
#define OFS_PT_MAX 32
#endif

/** @brief 偏移锚点最大数量 */
#ifndef OFS_ANCHOR_MAX
#define OFS_ANCHOR_MAX 20
#endif

/**
 * @brief 偏移量修正规划输入
 * @en_name GeoOffsetPlanningInput
 * @cn_name 偏移量修正规划输入
 * @tag STRUCT
 * @field name=VehPosX, type=Real, unit=m, desc=车辆位置X
 * @field name=VehPosY, type=Real, unit=m, desc=车辆位置Y
 * @field name=VehPosTheta, type=Real, unit=rad, desc=车辆位置航向角
 * @field name=OffsetDirection, type=Integer, unit=1, desc=偏移方向：1左偏移，-1右偏移
 * @field name=AtLeastOffsetDistance, type=Real, unit=m, desc=至少偏移距离
 * @field name=SlotPoseX, type=Real, unit=m, desc=车位位置X
 * @field name=SlotPoseY, type=Real, unit=m, desc=车位位置Y
 * @field name=SlotPoseTheta, type=Real, unit=rad, desc=车位位置航向角
 * @field name=SltPt1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SltPt1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SltPt2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SltPt2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SltPt3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SltPt3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SltPt4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SltPt4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=Radius, type=Real, unit=m, desc=规划的转弯半径
 * @field name=ExistWheelStop, type=Integer, unit=1, desc=是否存在轮挡标志
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=真实边界线数量
 * @field name=BoundaryP1X, type=Array<Real, [16]>, unit=m, desc=真实边界点1X
 * @field name=BoundaryP1Y, type=Array<Real, [16]>, unit=m, desc=真实边界点1Y
 * @field name=BoundaryP2X, type=Array<Real, [16]>, unit=m, desc=真实边界点2X
 * @field name=BoundaryP2Y, type=Array<Real, [16]>, unit=m, desc=真实边界点2Y
 * @field name=VirtualBoundaryLineNum, type=Integer, unit=1, desc=虚拟边界线数量
 * @field name=VirtualBoundaryP1X, type=Array<Real, [16]>, unit=m, desc=虚拟边界点1X
 * @field name=VirtualBoundaryP1Y, type=Array<Real, [16]>, unit=m, desc=虚拟边界点1Y
 * @field name=VirtualBoundaryP2X, type=Array<Real, [16]>, unit=m, desc=虚拟边界点2X
 * @field name=VirtualBoundaryP2Y, type=Array<Real, [16]>, unit=m, desc=虚拟边界点2Y
 * @field name=BoundarySepPtNum, type=Integer, unit=1, desc=真实离散点数量
 * @field name=BoundarySepPtX, type=Array<Real, [32]>, unit=m, desc=真实离散点X
 * @field name=BoundarySepPtY, type=Array<Real, [32]>, unit=m, desc=真实离散点Y
 * @field name=StopperPtNum, type=Integer, unit=1, desc=挡停点数量
 * @field name=StopperPtX, type=Array<Real, [32]>, unit=m, desc=挡停点X
 * @field name=StopperPtY, type=Array<Real, [32]>, unit=m, desc=挡停点Y
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real VehPosX;
  Real VehPosY;
  Real VehPosTheta;
  Integer OffsetDirection;
  Real AtLeastOffsetDistance;
  Real SlotPoseX;
  Real SlotPoseY;
  Real SlotPoseTheta;
  Real SltPt1X;
  Real SltPt1Y;
  Real SltPt2X;
  Real SltPt2Y;
  Real SltPt3X;
  Real SltPt3Y;
  Real SltPt4X;
  Real SltPt4Y;
  Real Radius;
  Integer ExistWheelStop;
  Integer BoundaryLineNum;
  Real BoundaryP1X[OFS_LINE_MAX];
  Real BoundaryP1Y[OFS_LINE_MAX];
  Real BoundaryP2X[OFS_LINE_MAX];
  Real BoundaryP2Y[OFS_LINE_MAX];
  Integer VirtualBoundaryLineNum;
  Real VirtualBoundaryP1X[OFS_LINE_MAX];
  Real VirtualBoundaryP1Y[OFS_LINE_MAX];
  Real VirtualBoundaryP2X[OFS_LINE_MAX];
  Real VirtualBoundaryP2Y[OFS_LINE_MAX];
  Integer BoundarySepPtNum;
  Real BoundarySepPtX[OFS_PT_MAX];
  Real BoundarySepPtY[OFS_PT_MAX];
  Integer StopperPtNum;
  Real StopperPtX[OFS_PT_MAX];
  Real StopperPtY[OFS_PT_MAX];
} GeoOffsetPlanningInput_t;

/**
 * @brief 偏移量修正规划输出
 * @en_name GeoOffsetPlanningOutput
 * @cn_name 偏移量修正规划输出
 * @tag STRUCT
 * @field name=AnchorsNum, type=Integer, unit=1, desc=规划出的锚点总数
 * @field name=AnchorX, type=Array<Real, [20]>, unit=m, desc=各锚点横坐标X
 * @field name=AnchorY, type=Array<Real, [20]>, unit=m, desc=各锚点纵坐标Y
 * @field name=AnchorTheta, type=Array<Real, [20]>, unit=rad, desc=各锚点航向角
 * @field name=AnchorTurnDir, type=Array<Integer, [20]>, unit=1, desc=转弯方向左1右-1
 * @field name=AnchorGoDir, type=Array<Integer, [20]>, unit=1, desc=行驶方向前进1后退-1
 * @field name=AnchorDTheta, type=Array<Real, [20]>, unit=rad, desc=变化角度
 * @field name=AnchorCurv, type=Array<Real, [20]>, unit=1/m, desc=曲率左正右负
 * @field name=AnchorCcX, type=Array<Real, [20]>, unit=m, desc=圆心横坐标X
 * @field name=AnchorCcY, type=Array<Real, [20]>, unit=m, desc=圆心纵坐标Y
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Integer AnchorsNum;
  Real AnchorX[OFS_ANCHOR_MAX];
  Real AnchorY[OFS_ANCHOR_MAX];
  Real AnchorTheta[OFS_ANCHOR_MAX];
  Integer AnchorTurnDir[OFS_ANCHOR_MAX];
  Integer AnchorGoDir[OFS_ANCHOR_MAX];
  Real AnchorDTheta[OFS_ANCHOR_MAX];
  Real AnchorCurv[OFS_ANCHOR_MAX];
  Real AnchorCcX[OFS_ANCHOR_MAX];
  Real AnchorCcY[OFS_ANCHOR_MAX];
} GeoOffsetPlanningOutput_t;

void geoOffsetPlanning(const GeoOffsetPlanningInput_t *input, GeoOffsetPlanningOutput_t *output);

#endif
