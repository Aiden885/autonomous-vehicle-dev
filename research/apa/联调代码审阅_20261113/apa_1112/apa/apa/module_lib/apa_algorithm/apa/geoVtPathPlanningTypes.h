#ifndef GEO_VT_PATH_PLANNING_TYPES_H
#define GEO_VT_PATH_PLANNING_TYPES_H

#include "baseTypes.h"
#include "apaHybridPlanningTypes.h"
#include "apaPlanningScene.h"
#include "geoCircle1pt1lineTypes.h"
#include "geoCircle2lineRTypes.h"
#include "geoCircleIntrvCheckTypes.h"
#include "geoLineIntrvCheckTypes.h"
#include "geoPosTypClassPkInTypes.h"
#include "geoRev2CircleLinkTypes.h"
#include "geoStp0trnmdfTypes.h"
#include "geoStp1trnmdfTypes.h"
#include "geoStPtSpcVtTypes.h"
#include "geoVT1stpMdfPlanningTypes.h"

/**
 * @brief VT路径规划输入
 * @en_name GeoVtPathPlanningInput
 * @cn_name VT路径规划输入
 * @tag STRUCT
 * @field name=VehX, type=Real, unit=m, desc=车辆X
 * @field name=VehY, type=Real, unit=m, desc=车辆Y
 * @field name=VehTheta, type=Real, unit=rad, desc=车辆航向
 * @field name=TarX, type=Real, unit=m, desc=目标X
 * @field name=TarY, type=Real, unit=m, desc=目标Y
 * @field name=TarTheta, type=Real, unit=rad, desc=目标航向
 * @field name=Rmin, type=Real, unit=m, desc=最小转弯半径
 * @field name=EntrPtX, type=Real, unit=m, desc=入口点X
 * @field name=EntrPtY, type=Real, unit=m, desc=入口点Y
 * @field name=SltPt1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SltPt1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SltPt2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SltPt2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SltPt3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SltPt3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SltPt4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SltPt4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=SltEntr, type=Integer, unit=1, desc=车位入口边编号
 * @field name=PrkType, type=Integer, unit=1, desc=泊车方向1左2右
 * @field name=SurdType, type=Integer, unit=1, desc=周边类型0无障碍1左开2右开3双侧受限
 * @field name=AgrOfst, type=Real, unit=m, desc=可用空间修正量
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=障碍边界线段数
 * @field name=VirtualBoundaryLineNum, type=Integer, unit=1, desc=虚拟边界线段数
 * @field name=VirtualBoundaryP1X, type=Array<Real, [64]>, unit=m, desc=虚拟边界线起点X
 * @field name=VirtualBoundaryP1Y, type=Array<Real, [64]>, unit=m, desc=虚拟边界线起点Y
 * @field name=VirtualBoundaryP2X, type=Array<Real, [64]>, unit=m, desc=虚拟边界线终点X
 * @field name=VirtualBoundaryP2Y, type=Array<Real, [64]>, unit=m, desc=虚拟边界线终点Y
 * @field name=SlotTrackStopperNum, type=Integer, unit=1, desc=车位限位器线段数
 * @field name=SlotTrackStopperX1, type=Array<Real, [64]>, unit=m, desc=车位限位器起点X
 * @field name=SlotTrackStopperY1, type=Array<Real, [64]>, unit=m, desc=车位限位器起点Y
 * @field name=SlotTrackStopperX2, type=Array<Real, [64]>, unit=m, desc=车位限位器终点X
 * @field name=SlotTrackStopperY2, type=Array<Real, [64]>, unit=m, desc=车位限位器终点Y
 * @field name=BoundarySepPtNum, type=Integer, unit=1, desc=真实边界离散点数量
 * @field name=BoundarySepPtX, type=Array<Real, [128]>, unit=m, desc=真实边界离散点X
 * @field name=BoundarySepPtY, type=Array<Real, [128]>, unit=m, desc=真实边界离散点Y
 * @field name=StopperPtNum, type=Integer, unit=1, desc=挡停点数量
 * @field name=StopperPtX, type=Array<Real, [128]>, unit=m, desc=挡停点X
 * @field name=StopperPtY, type=Array<Real, [128]>, unit=m, desc=挡停点Y
 * @version 1.1
 * @date 2026-03-14
 * @author aide
 */

#ifndef GEO_BOUNDARY_LINE_MAX
#define GEO_BOUNDARY_LINE_MAX 64
#endif

#ifndef GEO_BOUNDARY_POINT_MAX
#define GEO_BOUNDARY_POINT_MAX 128
#endif

typedef struct {
  Real VehX;
  Real VehY;
  Real VehTheta;
  Real TarX;
  Real TarY;
  Real TarTheta;
  Real Rmin;
  Real EntrPtX;
  Real EntrPtY;
  Real SltPt1X;
  Real SltPt1Y;
  Real SltPt2X;
  Real SltPt2Y;
  Real SltPt3X;
  Real SltPt3Y;
  Real SltPt4X;
  Real SltPt4Y;
  Integer SltEntr;
  Integer PrkType;
  Integer SurdType;
  Real AgrOfst;
  Integer BoundaryLineNum;
  Real BoundaryLineX1[GEO_BOUNDARY_LINE_MAX];
  Real BoundaryLineY1[GEO_BOUNDARY_LINE_MAX];
  Real BoundaryLineX2[GEO_BOUNDARY_LINE_MAX];
  Real BoundaryLineY2[GEO_BOUNDARY_LINE_MAX];
  Integer VirtualBoundaryLineNum;
  Real VirtualBoundaryP1X[GEO_BOUNDARY_LINE_MAX];
  Real VirtualBoundaryP1Y[GEO_BOUNDARY_LINE_MAX];
  Real VirtualBoundaryP2X[GEO_BOUNDARY_LINE_MAX];
  Real VirtualBoundaryP2Y[GEO_BOUNDARY_LINE_MAX];
  Integer SlotTrackStopperNum;
  Real SlotTrackStopperX1[GEO_BOUNDARY_LINE_MAX];
  Real SlotTrackStopperY1[GEO_BOUNDARY_LINE_MAX];
  Real SlotTrackStopperX2[GEO_BOUNDARY_LINE_MAX];
  Real SlotTrackStopperY2[GEO_BOUNDARY_LINE_MAX];
  Integer BoundarySepPtNum;
  Real BoundarySepPtX[GEO_BOUNDARY_POINT_MAX];
  Real BoundarySepPtY[GEO_BOUNDARY_POINT_MAX];
  Integer StopperPtNum;
  Real StopperPtX[GEO_BOUNDARY_POINT_MAX];
  Real StopperPtY[GEO_BOUNDARY_POINT_MAX];
} GeoVtPathPlanningInput_t;

/**
 * @brief VT路径规划参数
 * @en_name GeoVtPathPlanningParam
 * @cn_name VT路径规划参数
 * @tag STRUCT
 * @field name=SltState, type=Integer, unit=1, desc=车位状态
 * @field name=RminClassifiedSlot, type=Real, unit=m, desc=成功/分类车位最小半径
 * @field name=VehW, type=Real, unit=m, desc=车宽
 * @field name=VehLlF, type=Real, unit=m, desc=前悬到参考点长度
 * @field name=VehLfMin, type=Real, unit=m, desc=最小前向安全长度
 * @field name=VehLr, type=Real, unit=m, desc=后悬到参考点长度
 * @field name=FallbackSpace, type=Real, unit=m, desc=边界探测失败时的回退空间
 * @field name=NeedHybridAlgFusion, type=Integer, unit=1, desc=是否允许进入HybridA*融合回退
 * @field name=SceneQuery, type=ApaPlanningSceneQuery_t, unit=1, desc=完整场景查询回调及进程内上下文
 * @field name=Diagnostics, type=ApaPlanningDiagnostics_t*, unit=1, desc=本次规划诊断输出指针，仅进程内使用
 * @field name=HybridPolicy, type=ApaPlannerHybridPolicy_t, unit=1, desc=本次搜索的候选比较和准入回调策略
 * @version 1.1
 * @date 2026-03-14
 * @author aide
 */
typedef struct {
  Integer SltState;
  Real RminClassifiedSlot;
  Real VehW;
  Real VehLlF;
  Real VehLfMin;
  Real VehLr;
  Real FallbackSpace;
  Integer NeedHybridAlgFusion;
  ApaPlanningSceneQuery_t SceneQuery; /* Optional, owned by the calling worker. */
  ApaPlanningDiagnostics_t *Diagnostics;
  ApaPlannerHybridPolicy_t HybridPolicy;
} GeoVtPathPlanningParam_t;

/**
 * @brief VT路径规划输出
 * @en_name GeoVtPathPlanningOutput
 * @cn_name VT路径规划输出
 * @tag STRUCT
 * @field name=PlanFlg, type=Integer, unit=1, desc=规划成功标志
 * @field name=SegNum, type=Integer, unit=1, desc=路径段数
 * @field name=PathLen, type=Real, unit=m, desc=路径总长
 * @field name=SegGoDir, type=Array<Integer, [9]>, unit=1, desc=各段行驶方向
 * @field name=SegTrnDir, type=Array<Integer, [9]>, unit=1, desc=各段转弯方向
 * @field name=SegCrv, type=Array<Real, [9]>, unit=1/m, desc=各段曲率
 * @field name=SegTht, type=Array<Real, [9]>, unit=rad, desc=各段转角
 * @field name=SegLen, type=Array<Real, [9]>, unit=m, desc=各段长度
 * @field name=SegStX, type=Array<Real, [9]>, unit=m, desc=各段起点X
 * @field name=SegStY, type=Array<Real, [9]>, unit=m, desc=各段起点Y
 * @field name=SegStTh, type=Array<Real, [9]>, unit=rad, desc=各段起点航向
 * @field name=SegCcX, type=Array<Real, [9]>, unit=m, desc=各段圆心X
 * @field name=SegCcY, type=Array<Real, [9]>, unit=m, desc=各段圆心Y
 * @field name=StartPosTyp, type=Integer, unit=1, desc=起点相对位置类型
 * @field name=StartRelaX, type=Real, unit=m, desc=起点相对X
 * @field name=StartRelaY, type=Real, unit=m, desc=起点相对Y
 * @field name=StartRelaTheta, type=Real, unit=rad, desc=起点相对航向
 * @field name=ProbeLeftSpace, type=Real, unit=m, desc=左侧探测空间
 * @field name=ProbeRightSpace, type=Real, unit=m, desc=右侧探测空间
 * @field name=ResolvedEntrPtX, type=Real, unit=m, desc=最终使用的入口点X
 * @field name=ResolvedEntrPtY, type=Real, unit=m, desc=最终使用的入口点Y
 * @field name=EntrDgrPtFlg, type=Integer, unit=1, desc=入口危险点标志0无1真实2虚拟
 * @field name=EntrCheckCnt, type=Integer, unit=1, desc=入口点检测检查次数
 * @field name=PlnChkCnt, type=Integer, unit=1, desc=规划检查次数
 * @field name=StpN, type=Integer, unit=1, desc=换向次数
 * @field name=PlannerCase, type=Integer, unit=1, desc=规划分支编号
 * @field name=FallbackGeomCandidateFlg, type=Integer, unit=1, desc=当前分支几何回退候选存在标志
 * @field name=FallbackGeomMode, type=Integer, unit=1, desc=当前分支几何回退模式0未尝试1安全几何2原始几何
 * @field name=FallbackHybridCandidateFlg, type=Integer, unit=1, desc=当前分支Hybrid回退候选存在标志
 * @field name=FallbackWinnerKind, type=Integer, unit=1, desc=当前分支回退赢家0无1几何2Hybrid
 * @field name=FallbackDecisionReason, type=Integer, unit=1, desc=当前分支回退决策原因0无1仅有效性2换向次数3路径长度4段数5检查次数
 * @field name=FallbackGeomStpN, type=Integer, unit=1, desc=当前分支几何回退候选换向次数
 * @field name=FallbackGeomPathLen, type=Real, unit=m, desc=当前分支几何回退候选路径总长
 * @field name=FallbackGeomSegNum, type=Integer, unit=1, desc=当前分支几何回退候选段数
 * @field name=FallbackGeomPlnChkCnt, type=Integer, unit=1, desc=当前分支几何回退候选检查次数
 * @field name=FallbackHybridStpN, type=Integer, unit=1, desc=当前分支Hybrid回退候选换向次数
 * @field name=FallbackHybridPathLen, type=Real, unit=m, desc=当前分支Hybrid回退候选路径总长
 * @field name=FallbackHybridSegNum, type=Integer, unit=1, desc=当前分支Hybrid回退候选段数
 * @field name=FallbackHybridPlnChkCnt, type=Integer, unit=1, desc=当前分支Hybrid回退候选检查次数
 * @field name=StpPthTypLN, type=Integer, unit=1, desc=路径类型序列长度
 * @field name=StpPthTyp0, type=Integer, unit=1, desc=路径类型序列首项
 * @field name=StpPthTyp1, type=Integer, unit=1, desc=路径类型序列次项
 * @field name=StpPthTypList, type=Array<Integer, [30]>, unit=1, desc=路径类型序列
 * @field name=FailRs0, type=Integer, unit=1, desc=失败原因首项
 * @field name=FailRs1, type=Integer, unit=1, desc=失败原因次项
 * @field name=FailRsOut, type=Array<Integer, [30]>, unit=1, desc=失败原因序列
 * @version 1.0
 * @date 2026-03-10
 * @author aide
 */

#define VT_SEG_OUT_MAX 9

typedef struct {
  Integer PlanFlg;
  Integer SegNum;
  Real PathLen;
  Integer SegGoDir[VT_SEG_OUT_MAX];
  Integer SegTrnDir[VT_SEG_OUT_MAX];
  Real SegCrv[VT_SEG_OUT_MAX];
  Real SegTht[VT_SEG_OUT_MAX];
  Real SegLen[VT_SEG_OUT_MAX];
  Real SegStX[VT_SEG_OUT_MAX];
  Real SegStY[VT_SEG_OUT_MAX];
  Real SegStTh[VT_SEG_OUT_MAX];
  Real SegCcX[VT_SEG_OUT_MAX];
  Real SegCcY[VT_SEG_OUT_MAX];
  Integer StartPosTyp;
  Real StartRelaX;
  Real StartRelaY;
  Real StartRelaTheta;
  Real ProbeLeftSpace;
  Real ProbeRightSpace;
  Real ResolvedEntrPtX;
  Real ResolvedEntrPtY;
  Integer EntrDgrPtFlg;
  Integer EntrCheckCnt;
  Integer PlnChkCnt;
  Integer StpN;
  Integer PlannerCase;
  Integer FallbackGeomCandidateFlg;
  Integer FallbackGeomMode;
  Integer FallbackHybridCandidateFlg;
  Integer FallbackWinnerKind;
  Integer FallbackDecisionReason;
  Integer FallbackGeomStpN;
  Real FallbackGeomPathLen;
  Integer FallbackGeomSegNum;
  Integer FallbackGeomPlnChkCnt;
  Integer FallbackHybridStpN;
  Real FallbackHybridPathLen;
  Integer FallbackHybridSegNum;
  Integer FallbackHybridPlnChkCnt;
  Integer StpPthTypLN;
  Integer StpPthTyp0;
  Integer StpPthTyp1;
  Integer FailRs0;
  Integer FailRs1;
  Integer StpPthTypList[APA_PATH_SEG_MAX];
  Integer FailRsOut[APA_PATH_SEG_MAX];
} GeoVtPathPlanningOutput_t;

void geoVtPathPlanning(const GeoVtPathPlanningInput_t *input,
                       const GeoVtPathPlanningParam_t *param,
                       GeoVtPathPlanningOutput_t *output);

#endif
