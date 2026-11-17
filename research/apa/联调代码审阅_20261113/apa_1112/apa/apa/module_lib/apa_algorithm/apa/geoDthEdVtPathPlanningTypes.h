/**
 * @file geoDthEdVtPathPlanningTypes.h
 * @brief Oblique-edge VT planner adapter input/output types.
 * @date 2026-04-24
 */
#ifndef GEO_DTH_ED_VT_PATH_PLANNING_TYPES_H
#define GEO_DTH_ED_VT_PATH_PLANNING_TYPES_H

#include "baseTypes.h"
#include "apaHybridPlanningTypes.h"
#include "geoVtPathPlanningTypes.h"

#ifndef GEO_BOUNDARY_LINE_MAX
#define GEO_BOUNDARY_LINE_MAX 64
#endif

#ifndef GEO_BOUNDARY_POINT_MAX
#define GEO_BOUNDARY_POINT_MAX 128
#endif

#ifndef GEO_DTH_ED_VT_SEG_OUT_MAX
#define GEO_DTH_ED_VT_SEG_OUT_MAX 30
#endif

#ifndef GEO_DTH_ED_VT_PATH_LIST_MAX
#define GEO_DTH_ED_VT_PATH_LIST_MAX APA_PATH_SEG_MAX
#endif

#ifndef GEO_DTH_ED_VT_PLANNER_CASE_NONE
#define GEO_DTH_ED_VT_PLANNER_CASE_NONE 0
#endif

#ifndef GEO_DTH_ED_VT_PLANNER_CASE_VT_BRIDGE
#define GEO_DTH_ED_VT_PLANNER_CASE_VT_BRIDGE 1
#endif

#ifndef GEO_DTH_ED_VT_PLANNER_CASE_FRONT_FALLBACK
#define GEO_DTH_ED_VT_PLANNER_CASE_FRONT_FALLBACK 2
#endif

#ifndef GEO_DTH_ED_VT_PLANNER_CASE_GENERIC_FALLBACK
#define GEO_DTH_ED_VT_PLANNER_CASE_GENERIC_FALLBACK 3
#endif

#ifndef GEO_DTH_ED_VT_PLANNER_CASE_HYBRID_FALLBACK
#define GEO_DTH_ED_VT_PLANNER_CASE_HYBRID_FALLBACK 4
#endif

#ifndef GEO_DTH_ED_VT_PLANNER_CASE_PRECHECK_BLOCKED
#define GEO_DTH_ED_VT_PLANNER_CASE_PRECHECK_BLOCKED 5
#endif

#ifndef GEO_DTH_ED_VT_FALLBACK_WINNER_NONE
#define GEO_DTH_ED_VT_FALLBACK_WINNER_NONE 0
#endif

#ifndef GEO_DTH_ED_VT_FALLBACK_WINNER_GEOM
#define GEO_DTH_ED_VT_FALLBACK_WINNER_GEOM 1
#endif

#ifndef GEO_DTH_ED_VT_FALLBACK_WINNER_HYBRID
#define GEO_DTH_ED_VT_FALLBACK_WINNER_HYBRID 2
#endif

/**
 * @brief Oblique-edge VT path planner input.
 * @en_name GeoDthEdVtPathPlanningInput
 * @cn_name 斜边VT路径规划输入
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
 * @field name=PrkType, type=Integer, unit=1, desc=泊车方向
 * @field name=SurdType, type=Integer, unit=1, desc=周边类型
 * @field name=AgrOfst, type=Real, unit=m, desc=可用空间修正量
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=边界线数量
 * @field name=BoundaryLineX1, type=Array<Real, [64]>, unit=m, desc=边界线起点X数组
 * @field name=BoundaryLineY1, type=Array<Real, [64]>, unit=m, desc=边界线起点Y数组
 * @field name=BoundaryLineX2, type=Array<Real, [64]>, unit=m, desc=边界线终点X数组
 * @field name=BoundaryLineY2, type=Array<Real, [64]>, unit=m, desc=边界线终点Y数组
 * @field name=VirtualBoundaryLineNum, type=Integer, unit=1, desc=虚拟边界线数量
 * @field name=VirtualBoundaryP1X, type=Array<Real, [64]>, unit=m, desc=虚拟边界点1X
 * @field name=VirtualBoundaryP1Y, type=Array<Real, [64]>, unit=m, desc=虚拟边界点1Y
 * @field name=VirtualBoundaryP2X, type=Array<Real, [64]>, unit=m, desc=虚拟边界点2X
 * @field name=VirtualBoundaryP2Y, type=Array<Real, [64]>, unit=m, desc=虚拟边界点2Y
 * @field name=BoundarySepPtNum, type=Integer, unit=1, desc=真实离散点数量
 * @field name=BoundarySepPtX, type=Array<Real, [128]>, unit=m, desc=真实离散点X
 * @field name=BoundarySepPtY, type=Array<Real, [128]>, unit=m, desc=真实离散点Y
 * @field name=SlotTrackStopperNum, type=Integer, unit=1, desc=slot stopper线段数量
 * @field name=SlotTrackStopperX1, type=Array<Real, [64]>, unit=m, desc=slot stopper端点1X
 * @field name=SlotTrackStopperY1, type=Array<Real, [64]>, unit=m, desc=slot stopper端点1Y
 * @field name=SlotTrackStopperX2, type=Array<Real, [64]>, unit=m, desc=slot stopper端点2X
 * @field name=SlotTrackStopperY2, type=Array<Real, [64]>, unit=m, desc=slot stopper端点2Y
 * @field name=StopperPtNum, type=Integer, unit=1, desc=挡停点数量
 * @field name=StopperPtX, type=Array<Real, [128]>, unit=m, desc=挡停点X
 * @field name=StopperPtY, type=Array<Real, [128]>, unit=m, desc=挡停点Y
 */
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
  Integer BoundarySepPtNum;
  Real BoundarySepPtX[GEO_BOUNDARY_POINT_MAX];
  Real BoundarySepPtY[GEO_BOUNDARY_POINT_MAX];
  Integer SlotTrackStopperNum;
  Real SlotTrackStopperX1[GEO_BOUNDARY_LINE_MAX];
  Real SlotTrackStopperY1[GEO_BOUNDARY_LINE_MAX];
  Real SlotTrackStopperX2[GEO_BOUNDARY_LINE_MAX];
  Real SlotTrackStopperY2[GEO_BOUNDARY_LINE_MAX];
  Integer StopperPtNum;
  Real StopperPtX[GEO_BOUNDARY_POINT_MAX];
  Real StopperPtY[GEO_BOUNDARY_POINT_MAX];
} GeoDthEdVtPathPlanningInput_t;

/**
 * @brief Oblique-edge VT path planner parameters.
 * @en_name GeoDthEdVtPathPlanningParam
 * @cn_name 斜边VT路径规划参数
 * @tag STRUCT
 * @field name=SltState, type=Integer, unit=1, desc=车位状态
 * @field name=VehW, type=Real, unit=m, desc=车宽
 * @field name=VehLlF, type=Real, unit=m, desc=前悬到参考点长度
 * @field name=VehLfMin, type=Real, unit=m, desc=最小前向安全长度
 * @field name=VehLr, type=Real, unit=m, desc=后悬到参考点长度
 * @field name=FallbackSpace, type=Real, unit=m, desc=边界探测失败时的回退空间
 * @field name=NeedHybridAlgFusion, type=Integer, unit=1, desc=是否允许进入HybridA*融合回退
 * @field name=Diagnostics, type=ApaPlanningDiagnostics_t*, unit=1, desc=本次规划诊断输出指针，仅进程内使用
 * @field name=HybridPolicy, type=ApaPlannerHybridPolicy_t, unit=1, desc=本次搜索的候选比较和准入回调策略
 * @field name=SceneQuery, type=ApaPlanningSceneQuery_t, unit=1, desc=完整场景查询回调及进程内上下文
 */
typedef struct {
  Integer SltState;
  Real VehW;
  Real VehLlF;
  Real VehLfMin;
  Real VehLr;
  Real FallbackSpace;
  Integer NeedHybridAlgFusion;
  ApaPlanningDiagnostics_t *Diagnostics;
  ApaPlannerHybridPolicy_t HybridPolicy;
  ApaPlanningSceneQuery_t SceneQuery;
} GeoDthEdVtPathPlanningParam_t;

/**
 * @brief Oblique-edge VT path planner output.
 * @en_name GeoDthEdVtPathPlanningOutput
 * @cn_name 斜边VT路径规划输出
 * @tag STRUCT
 * @field name=PlanFlg, type=Integer, unit=1, desc=规划成功标志
 * @field name=PthPlnFlg, type=Integer, unit=1, desc=路径规划成功标志
 * @field name=SegNum, type=Integer, unit=1, desc=路径段数
 * @field name=PathLen, type=Real, unit=m, desc=路径总长
 * @field name=SegGoDir, type=Array<Integer, [30]>, unit=1, desc=各段行驶方向
 * @field name=SegTrnDir, type=Array<Integer, [30]>, unit=1, desc=各段转弯方向
 * @field name=SegCrv, type=Array<Real, [30]>, unit=1/m, desc=各段曲率
 * @field name=SegTht, type=Array<Real, [30]>, unit=rad, desc=各段转角
 * @field name=SegLen, type=Array<Real, [30]>, unit=m, desc=各段长度
 * @field name=SegStX, type=Array<Real, [30]>, unit=m, desc=各段起点X
 * @field name=SegStY, type=Array<Real, [30]>, unit=m, desc=各段起点Y
 * @field name=SegStTh, type=Array<Real, [30]>, unit=rad, desc=各段起点航向
 * @field name=SegCcX, type=Array<Real, [30]>, unit=m, desc=各段圆心X
 * @field name=SegCcY, type=Array<Real, [30]>, unit=m, desc=各段圆心Y
 * @field name=Pt1X, type=Real, unit=m, desc=解析入口点X
 * @field name=Pt1Y, type=Real, unit=m, desc=解析入口点Y
 * @field name=DngPtX, type=Real, unit=m, desc=危险点X
 * @field name=DngPtY, type=Real, unit=m, desc=危险点Y
 * @field name=EntrDgrPtFlg, type=Integer, unit=1, desc=入口危险点标志
 * @field name=EntrCheckCnt, type=Integer, unit=1, desc=入口检查次数
 * @field name=DthFtSpc, type=Real, unit=m, desc=斜向前方空间
 * @field name=DthLtSpc, type=Real, unit=m, desc=斜向左侧空间
 * @field name=DthRtSpc, type=Real, unit=m, desc=斜向右侧空间
 * @field name=DthSpaceCheckCnt, type=Integer, unit=1, desc=斜向空间检查次数
 * @field name=DthIntrvRS, type=Integer, unit=1, desc=斜向碰撞检查结果
 * @field name=DthCrshPosX, type=Real, unit=m, desc=碰撞位姿X
 * @field name=DthCrshPosY, type=Real, unit=m, desc=碰撞位姿Y
 * @field name=DthCrshPosTheta, type=Real, unit=rad, desc=碰撞位姿航向
 * @field name=DthCheckPosCX, type=Real, unit=m, desc=检查中心X
 * @field name=DthCheckPosCY, type=Real, unit=m, desc=检查中心Y
 * @field name=DthCheckPosCTheta, type=Real, unit=rad, desc=检查中心航向
 * @field name=PlnChkCnt, type=Integer, unit=1, desc=规划检查次数
 * @field name=StpN, type=Integer, unit=1, desc=换向次数
 * @field name=StpPthTypLN, type=Integer, unit=1, desc=路径类型序列长度
 * @field name=PlannerCase, type=Integer, unit=1, desc=规划分支编号
 * @field name=FallbackGeomCandidateFlg, type=Integer, unit=1, desc=几何候选存在标志
 * @field name=FallbackHybridCandidateFlg, type=Integer, unit=1, desc=Hybrid候选存在标志
 * @field name=FallbackWinnerKind, type=Integer, unit=1, desc=回退赢家0无1几何2Hybrid
 * @field name=FallbackDecisionReason, type=Integer, unit=1, desc=回退决策原因
 * @field name=FallbackGeomStpN, type=Integer, unit=1, desc=几何候选换向次数
 * @field name=FallbackGeomPathLen, type=Real, unit=m, desc=几何候选路径总长
 * @field name=FallbackGeomSegNum, type=Integer, unit=1, desc=几何候选段数
 * @field name=FallbackGeomPlnChkCnt, type=Integer, unit=1, desc=几何候选检查次数
 * @field name=FallbackHybridStpN, type=Integer, unit=1, desc=Hybrid候选换向次数
 * @field name=FallbackHybridPathLen, type=Real, unit=m, desc=Hybrid候选路径总长
 * @field name=FallbackHybridSegNum, type=Integer, unit=1, desc=Hybrid候选段数
 * @field name=FallbackHybridPlnChkCnt, type=Integer, unit=1, desc=Hybrid候选检查次数
 * @field name=StpPthTyp0, type=Integer, unit=1, desc=路径类型序列首项
 * @field name=StpPthTyp1, type=Integer, unit=1, desc=路径类型序列次项
 * @field name=FailRs0, type=Integer, unit=1, desc=失败原因首项
 * @field name=FailRs1, type=Integer, unit=1, desc=失败原因次项
 * @field name=StpPthTypList, type=Array<Integer, [30]>, unit=1, desc=路径类型序列
 * @field name=FailRsOut, type=Array<Integer, [30]>, unit=1, desc=失败原因序列
 */
typedef struct {
  Integer PlanFlg;
  Integer PthPlnFlg;
  Integer SegNum;
  Real PathLen;
  Integer SegGoDir[GEO_DTH_ED_VT_SEG_OUT_MAX];
  Integer SegTrnDir[GEO_DTH_ED_VT_SEG_OUT_MAX];
  Real SegCrv[GEO_DTH_ED_VT_SEG_OUT_MAX];
  Real SegTht[GEO_DTH_ED_VT_SEG_OUT_MAX];
  Real SegLen[GEO_DTH_ED_VT_SEG_OUT_MAX];
  Real SegStX[GEO_DTH_ED_VT_SEG_OUT_MAX];
  Real SegStY[GEO_DTH_ED_VT_SEG_OUT_MAX];
  Real SegStTh[GEO_DTH_ED_VT_SEG_OUT_MAX];
  Real SegCcX[GEO_DTH_ED_VT_SEG_OUT_MAX];
  Real SegCcY[GEO_DTH_ED_VT_SEG_OUT_MAX];
  Real Pt1X;
  Real Pt1Y;
  Real DngPtX;
  Real DngPtY;
  Integer EntrDgrPtFlg;
  Integer EntrCheckCnt;
  Real DthFtSpc;
  Real DthLtSpc;
  Real DthRtSpc;
  Integer DthSpaceCheckCnt;
  Integer DthIntrvRS;
  Real DthCrshPosX;
  Real DthCrshPosY;
  Real DthCrshPosTheta;
  Real DthCheckPosCX;
  Real DthCheckPosCY;
  Real DthCheckPosCTheta;
  Integer PlnChkCnt;
  Integer StpN;
  Integer StpPthTypLN;
  Integer PlannerCase;
  Integer FallbackGeomCandidateFlg;
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
  Integer StpPthTyp0;
  Integer StpPthTyp1;
  Integer FailRs0;
  Integer FailRs1;
  Integer StpPthTypList[GEO_DTH_ED_VT_PATH_LIST_MAX];
  Integer FailRsOut[GEO_DTH_ED_VT_PATH_LIST_MAX];
} GeoDthEdVtPathPlanningOutput_t;

void geoDthEdVtPathPlanning(const GeoDthEdVtPathPlanningInput_t *input,
                            const GeoDthEdVtPathPlanningParam_t *param,
                            GeoDthEdVtPathPlanningOutput_t *output);

#endif
