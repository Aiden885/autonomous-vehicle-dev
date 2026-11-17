#ifndef GEO_LINE_INTRV_CHECK_TYPES_H
#define GEO_LINE_INTRV_CHECK_TYPES_H

#include "baseTypes.h"
#include "vehicleCollisionCheckTypes.h"
#include "apaPlanningScene.h"

/** @brief 除零保护精度 */
#ifndef GEO_EPSILON
#define GEO_EPSILON 1e-6f
#endif

/**
 * @brief 直线区间碰撞检测输入
 * @en_name GeoLineIntrvCheckInput
 * @cn_name 直线区间碰撞检测输入
 * @tag STRUCT
 * @field name=Pt1X, type=Real, unit=m, desc=直线起点X
 * @field name=Pt1Y, type=Real, unit=m, desc=直线起点Y
 * @field name=Pt1Theta, type=Real, unit=rad, desc=直线起点航向
 * @field name=Pt2X, type=Real, unit=m, desc=直线终点X
 * @field name=Pt2Y, type=Real, unit=m, desc=直线终点Y
 * @field name=Pt2Theta, type=Real, unit=rad, desc=直线终点航向
 * @field name=CheckStep, type=Real, unit=m, desc=检测步长
 * @field name=GoalX, type=Real, unit=m, desc=目标点X
 * @field name=GoalY, type=Real, unit=m, desc=目标点Y
 * @field name=GoalYaw, type=Real, unit=rad, desc=目标航向
 * @field name=SlotP1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SlotP1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SlotP2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SlotP2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SlotP3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SlotP3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SlotP4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SlotP4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=VehWidth, type=Real, unit=m, desc=车宽
 * @field name=VehFrontOffset, type=Real, unit=m, desc=车辆前包络偏置
 * @field name=VehRearOffset, type=Real, unit=m, desc=车辆后包络偏置
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=障碍边界线段数
 * @field name=VirtualBoundaryLineNum, type=Integer, unit=1, desc=虚拟边界线数量
 * @field name=VirtualBoundaryLineX1, type=Array<Real, [128]>, unit=m, desc=虚拟边界线起点X
 * @field name=VirtualBoundaryLineY1, type=Array<Real, [128]>, unit=m, desc=虚拟边界线起点Y
 * @field name=VirtualBoundaryLineX2, type=Array<Real, [128]>, unit=m, desc=虚拟边界线终点X
 * @field name=VirtualBoundaryLineY2, type=Array<Real, [128]>, unit=m, desc=虚拟边界线终点Y
 * @field name=UsePlanningCheck, type=Boolean, unit=1, desc=是否启用泊出规划碰撞检查模式
 * @field name=ParkingOutFlag, type=Boolean, unit=1, desc=是否使用泊出碰撞契约
 * @field name=DirOut, type=Integer, unit=1, desc=泊出方向1左侧2右侧3前方
 * @field name=SlotClass, type=Integer, unit=1, desc=车位类别0垂直1平行
 * @field name=BoundarySepPtNum, type=Integer, unit=1, desc=边界离散点数
 * @field name=BoundarySepPtX, type=Array<Real, [128]>, unit=m, desc=边界离散点X
 * @field name=BoundarySepPtY, type=Array<Real, [128]>, unit=m, desc=边界离散点Y
 * @field name=SeparateStopperNum, type=Integer, unit=1, desc=离散stopper数
 * @field name=SeparateStopperX, type=Array<Real, [128]>, unit=m, desc=离散stopper X
 * @field name=SeparateStopperY, type=Array<Real, [128]>, unit=m, desc=离散stopper Y
 * @field name=SlotTrackStopperNum, type=Integer, unit=1, desc=slot stopper线段数量
 * @field name=SlotTrackStopperX1, type=Array<Real, [64]>, unit=m, desc=slot stopper起点X
 * @field name=SlotTrackStopperY1, type=Array<Real, [64]>, unit=m, desc=slot stopper起点Y
 * @field name=SlotTrackStopperX2, type=Array<Real, [64]>, unit=m, desc=slot stopper终点X
 * @field name=SlotTrackStopperY2, type=Array<Real, [64]>, unit=m, desc=slot stopper终点Y
 * @field name=DisableSlotBoundaryFallback, type=Boolean, unit=1, desc=空实体边界时不从车位角点构造障碍
 * @version 1.5
 * @date 2026-05-05
 * @author aide
 */

#define GEO_BOUNDARY_LINE_MAX 64

typedef struct {
  Real Pt1X;
  Real Pt1Y;
  Real Pt1Theta;
  Real Pt2X;
  Real Pt2Y;
  Real Pt2Theta;
  Real CheckStep;
  Real GoalX;
  Real GoalY;
  Real GoalYaw;
  Real SlotP1X;
  Real SlotP1Y;
  Real SlotP2X;
  Real SlotP2Y;
  Real SlotP3X;
  Real SlotP3Y;
  Real SlotP4X;
  Real SlotP4Y;
  Real VehWidth;
  Real VehFrontOffset;
  Real VehRearOffset;
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
  Boolean UsePlanningCheck;
  Boolean ParkingOutFlag;
  Integer DirOut;
  Integer SlotClass;
  Integer BoundarySepPtNum;
  Real BoundarySepPtX[VCC_MAX_SEPARATE_PTS];
  Real BoundarySepPtY[VCC_MAX_SEPARATE_PTS];
  Integer SeparateStopperNum;
  Real SeparateStopperX[VCC_MAX_SEPARATE_PTS];
  Real SeparateStopperY[VCC_MAX_SEPARATE_PTS];
  Integer SlotTrackStopperNum;
  Real SlotTrackStopperX1[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperY1[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperX2[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperY2[VCC_MAX_TRACK_STOPPER];
  Boolean DisableSlotBoundaryFallback;
} GeoLineIntrvCheckInput_t;

/**
 * @brief 直线区间碰撞检测输出
 * @en_name GeoLineIntrvCheckOutput
 * @cn_name 直线区间碰撞检测输出
 * @tag STRUCT
 * @field name=IntrvFlg, type=Integer, unit=1,
 * desc=碰撞标志0无碰撞1碰撞2起点碰撞
 * @field name=IntrvRS, type=Integer, unit=1, desc=碰撞类型
 * @field name=SafeLen, type=Real, unit=m, desc=安全长度
 * @field name=CheckCount, type=Integer, unit=1, desc=检测次数
 * @field name=AvlPosX, type=Real, unit=m, desc=安全点X
 * @field name=AvlPosY, type=Real, unit=m, desc=安全点Y
 * @field name=AvlPosTheta, type=Real, unit=rad, desc=安全点航向
 * @field name=CrshPosX, type=Real, unit=m, desc=碰撞点X
 * @field name=CrshPosY, type=Real, unit=m, desc=碰撞点Y
 * @field name=CrshPosTheta, type=Real, unit=rad, desc=碰撞点航向
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Integer IntrvFlg;
  Integer IntrvRS;
  Real SafeLen;
  Integer CheckCount;
  Real AvlPosX;
  Real AvlPosY;
  Real AvlPosTheta;
  Real CrshPosX;
  Real CrshPosY;
  Real CrshPosTheta;
} GeoLineIntrvCheckOutput_t;

void geoLineIntrvCheck(const GeoLineIntrvCheckInput_t *input, GeoLineIntrvCheckOutput_t *output);

void geoLineIntrvCheckWithScene(const GeoLineIntrvCheckInput_t *input,
                                const ApaPlanningSceneQuery_t *scene,
                                GeoLineIntrvCheckOutput_t *output);

void geoLineIntrvCheckPkOt(const GeoLineIntrvCheckInput_t *input,
                           GeoLineIntrvCheckOutput_t *output);

#endif
