#ifndef GEO_CIRCLE_INTRV_CHECK_TYPES_H
#define GEO_CIRCLE_INTRV_CHECK_TYPES_H

#include "baseTypes.h"
#include "vehicleCollisionCheckTypes.h"
#include "apaPlanningScene.h"

/** @brief 圆周率 */
#ifndef APA_PI
#define APA_PI 3.14159265f
#endif
/** @brief 两倍圆周率 */
#ifndef APA_TWO_PI
#define APA_TWO_PI 6.2831853f
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
 * @brief 圆弧区间碰撞检测输入
 * @en_name GeoCircleIntrvCheckInput
 * @cn_name 圆弧区间碰撞检测输入
 * @tag STRUCT
 * @field name=CptX, type=Real, unit=m, desc=圆心X
 * @field name=CptY, type=Real, unit=m, desc=圆心Y
 * @field name=Radius, type=Real, unit=m, desc=半径
 * @field name=Pt1X, type=Real, unit=m, desc=弧线起点X
 * @field name=Pt1Y, type=Real, unit=m, desc=弧线起点Y
 * @field name=Pt1Theta, type=Real, unit=rad, desc=弧线起点航向
 * @field name=Pt2X, type=Real, unit=m, desc=弧线终点X
 * @field name=Pt2Y, type=Real, unit=m, desc=弧线终点Y
 * @field name=Pt2Theta, type=Real, unit=rad, desc=弧线终点航向
 * @field name=GoDir, type=Integer, unit=1, desc=行驶方向1前进负1后退
 * @field name=TrnDir, type=Integer, unit=1, desc=转弯方向1左转负1右转
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
 * @field name=SlotClass, type=Integer, unit=1, desc=车位类别0垂直1平行
 * @field name=VehWidth, type=Real, unit=m, desc=车宽
 * @field name=VehFrontOffset, type=Real, unit=m, desc=车辆前包络偏置
 * @field name=VehRearOffset, type=Real, unit=m, desc=车辆后包络偏置
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=障碍边界线段数
 * @field name=VirtualBoundaryLineNum, type=Integer, unit=1, desc=虚拟边界线数量
 * @field name=BoundarySepPtNum, type=Integer, unit=1, desc=真实离散点数量
 * @field name=StopperPtNum, type=Integer, unit=1, desc=挡停点数量
 * @field name=SlotTrackStopperNum, type=Integer, unit=1, desc=slot stopper线段数量
 * @field name=DisableSlotBoundaryFallback, type=Boolean, unit=1, desc=空真实边界时禁止把车位三边合成为障碍
 * @field name=UseCollisionContract, type=Boolean, unit=1, desc=是否复用调用方碰撞模型和需求
 * @field name=CollisionModel, type=Integer, unit=1, desc=碰撞模型
 * @field name=CollisionDemand, type=Integer, unit=1, desc=碰撞需求
 * @field name=UsePlanningCheck, type=Boolean, unit=1, desc=是否启用泊出规划碰撞检查模式
 * @field name=ParkingOutFlag, type=Boolean, unit=1, desc=是否使用泊出碰撞契约
 * @field name=DirOut, type=Integer, unit=1, desc=泊出方向1左侧2右侧3前方
 * @version 1.2
 * @date 2026-03-14
 * @author aide
 */

#ifndef GEO_BOUNDARY_LINE_MAX
#define GEO_BOUNDARY_LINE_MAX 64
#endif

typedef struct {
  Real CptX;
  Real CptY;
  Real Radius;
  Real Pt1X;
  Real Pt1Y;
  Real Pt1Theta;
  Real Pt2X;
  Real Pt2Y;
  Real Pt2Theta;
  Integer GoDir;
  Integer TrnDir;
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
  Integer SlotClass;
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
  Integer BoundarySepPtNum;
  Real BoundarySepPtX[VCC_MAX_SEPARATE_PTS];
  Real BoundarySepPtY[VCC_MAX_SEPARATE_PTS];
  Integer StopperPtNum;
  Real StopperPtX[VCC_MAX_SEPARATE_PTS];
  Real StopperPtY[VCC_MAX_SEPARATE_PTS];
  Integer SlotTrackStopperNum;
  Real SlotTrackStopperX1[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperY1[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperX2[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperY2[VCC_MAX_TRACK_STOPPER];
  Boolean DisableSlotBoundaryFallback;
  Boolean UseCollisionContract;
  Integer CollisionModel;
  Integer CollisionDemand;
  Boolean UsePlanningCheck;
  Boolean ParkingOutFlag;
  Integer DirOut;
} GeoCircleIntrvCheckInput_t;

/**
 * @brief 圆弧区间碰撞检测输出
 * @en_name GeoCircleIntrvCheckOutput
 * @cn_name 圆弧区间碰撞检测输出
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
} GeoCircleIntrvCheckOutput_t;

void geoCircleIntrvCheck(const GeoCircleIntrvCheckInput_t *input,
                         GeoCircleIntrvCheckOutput_t *output);

void geoCircleIntrvCheckWithScene(const GeoCircleIntrvCheckInput_t *input,
                                  const ApaPlanningSceneQuery_t *scene,
                                  GeoCircleIntrvCheckOutput_t *output);

void geoCircleIntrvCheckPkOt(const GeoCircleIntrvCheckInput_t *input,
                             GeoCircleIntrvCheckOutput_t *output);

#endif
