#ifndef GEO_HT_PATH_PLANNING_TYPES_H
#define GEO_HT_PATH_PLANNING_TYPES_H

#include "baseTypes.h"
#include "apaHybridPlanningTypes.h"

#include "geoCircleIntrvCheckTypes.h"
#include "geoAxisTransTypes.h"
#include "geoDivideTypes.h"
#ifndef GEO_BOUNDARY_LINE_MAX
#define GEO_BOUNDARY_LINE_MAX 64
#endif

#ifndef GEO_BOUNDARY_POINT_MAX
#define GEO_BOUNDARY_POINT_MAX 128
#endif

#ifndef HT_SEG_MAX
#define HT_SEG_MAX 30
#endif

#ifndef HT_SEG_OUT_MAX
#define HT_SEG_OUT_MAX 30
#endif

/**
 * @brief HT路径规划输入
 * @en_name GeoHtPathPlanningInput
 * @cn_name HT路径规划输入
 * @tag STRUCT
 * @field name=VehX, type=Real, unit=m, desc=车辆X
 * @field name=VehY, type=Real, unit=m, desc=车辆Y
 * @field name=VehTheta, type=Real, unit=rad, desc=车辆航向
 * @field name=TarX, type=Real, unit=m, desc=目标X
 * @field name=TarY, type=Real, unit=m, desc=目标Y
 * @field name=TarTheta, type=Real, unit=rad, desc=目标航向
 * @field name=Rmin, type=Real, unit=m, desc=最小转弯半径
 * @field name=SltPt1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SltPt1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SltPt2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SltPt2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SltPt3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SltPt3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SltPt4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SltPt4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=SltEntr, type=Integer, unit=1, desc=车位入口边编号
 * @field name=SlotType, type=Integer, unit=1, desc=MATLAB Slot_Type(0=vertical,1=parallel)
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=障碍边界线段数
 * @field name=BoundaryLineX1, type=Array<Real, [64]>, unit=m, desc=边界线段起点X
 * @field name=BoundaryLineY1, type=Array<Real, [64]>, unit=m, desc=边界线段起点Y
 * @field name=BoundaryLineX2, type=Array<Real, [64]>, unit=m, desc=边界线段终点X
 * @field name=BoundaryLineY2, type=Array<Real, [64]>, unit=m, desc=边界线段终点Y
 * @field name=VirtualBoundaryLineNum, type=Integer, unit=1, desc=虚拟边界线数量
 * @field name=VirtualBoundaryP1X, type=Array<Real, [64]>, unit=m, desc=虚拟边界点1X
 * @field name=VirtualBoundaryP1Y, type=Array<Real, [64]>, unit=m, desc=虚拟边界点1Y
 * @field name=VirtualBoundaryP2X, type=Array<Real, [64]>, unit=m, desc=虚拟边界点2X
 * @field name=VirtualBoundaryP2Y, type=Array<Real, [64]>, unit=m, desc=虚拟边界点2Y
 * @field name=SlotTrackStopperNum, type=Integer, unit=1, desc=车位跟踪挡停线数量
 * @field name=SlotTrackStopperX1, type=Array<Real, [64]>, unit=m, desc=挡停线端点1X
 * @field name=SlotTrackStopperY1, type=Array<Real, [64]>, unit=m, desc=挡停线端点1Y
 * @field name=SlotTrackStopperX2, type=Array<Real, [64]>, unit=m, desc=挡停线端点2X
 * @field name=SlotTrackStopperY2, type=Array<Real, [64]>, unit=m, desc=挡停线端点2Y
 * @field name=BoundarySepPtNum, type=Integer, unit=1, desc=真实离散点数量
 * @field name=BoundarySepPtX, type=Array<Real, [128]>, unit=m, desc=真实离散点X
 * @field name=BoundarySepPtY, type=Array<Real, [128]>, unit=m, desc=真实离散点Y
 * @field name=StopperPtNum, type=Integer, unit=1, desc=挡停点数量
 * @field name=StopperPtX, type=Array<Real, [128]>, unit=m, desc=挡停点X
 * @field name=StopperPtY, type=Array<Real, [128]>, unit=m, desc=挡停点Y
 * @field name=StopperBarrier1, type=Real, unit=m, desc=MATLAB sltstpbp当前行第1列
 * @field name=StopperBarrier2, type=Real, unit=m, desc=MATLAB sltstpbp当前行第2列
 * @field name=StopperBarrier3, type=Real, unit=m, desc=MATLAB sltstpbp当前行第3列
 * @field name=StopperBarrier4, type=Real, unit=m, desc=MATLAB sltstpbp当前行第4列
 * @field name=PrkType, type=Integer, unit=1, desc=Hybrid车位左右类型沿用上游契约
 * @field name=SurdType, type=Integer, unit=1, desc=Hybrid周边障碍类别沿用上游契约
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real VehX;
  Real VehY;
  Real VehTheta;
  Real TarX;
  Real TarY;
  Real TarTheta;
  Real Rmin;
  Real SltPt1X;
  Real SltPt1Y;
  Real SltPt2X;
  Real SltPt2Y;
  Real SltPt3X;
  Real SltPt3Y;
  Real SltPt4X;
  Real SltPt4Y;
  Integer SltEntr;
  Integer SlotType;
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
  Real StopperBarrier1;
  Real StopperBarrier2;
  Real StopperBarrier3;
  Real StopperBarrier4;
  Integer PrkType;
  Integer SurdType;
} GeoHtPathPlanningInput_t;

/**
 * @brief HT路径规划参数
 * @en_name GeoHtPathPlanningParam
 * @cn_name HT路径规划参数
 * @tag STRUCT
 * @field name=SltState, type=Integer, unit=1, desc=车位状态
 * @field name=VehW, type=Real, unit=m, desc=车宽
 * @field name=VehLlF, type=Real, unit=m, desc=后轴参考点到前保险杠长度(para22/k_APA_LLf)
 * @field name=VehLfMin, type=Real, unit=m, desc=最小前向安全长度
 * @field name=VehLr, type=Real, unit=m, desc=后悬到参考点长度
 * @field name=RfoutMin, type=Real, unit=m, desc=最小出口转弯半径(para15)
 * @field name=NeedHybridAlgFusion, type=Integer, unit=1, desc=是否允许进入HybridA*融合回退
 * @field name=Diagnostics, type=ApaPlanningDiagnostics_t*, unit=1, desc=本次规划诊断输出指针，仅进程内使用
 * @field name=HybridPolicy, type=ApaPlannerHybridPolicy_t, unit=1, desc=本次搜索的候选比较和准入回调策略
 * @field name=SceneQuery, type=ApaPlanningSceneQuery_t, unit=1, desc=完整场景查询回调及进程内上下文
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Integer SltState;
  Real VehW;
  Real VehLlF;
  Real VehLfMin;
  Real VehLr;
  Real RfoutMin;
  Integer NeedHybridAlgFusion;
  ApaPlanningDiagnostics_t *Diagnostics;
  ApaPlannerHybridPolicy_t HybridPolicy;
  ApaPlanningSceneQuery_t SceneQuery;
} GeoHtPathPlanningParam_t;

/**
 * @brief HT路径规划输出(与VT共用段数组结构)
 * @en_name GeoHtPathPlanningOutput
 * @cn_name HT路径规划输出
 * @tag STRUCT
 * @field name=PlanFlg, type=Integer, unit=1, desc=规划成功标志
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
 * @field name=DngPtX, type=Real, unit=m, desc=危险点X
 * @field name=DngPtY, type=Real, unit=m, desc=危险点Y
 * @field name=PlnChkCnt, type=Integer, unit=1, desc=规划检查次数
 * @field name=FailRs0, type=Integer, unit=1, desc=失败原因首项
 * @field name=StpN, type=Integer, unit=1, desc=换向次数
 * @field name=PlannerCase, type=Integer, unit=1, desc=规划分支编号
 * @field name=FallbackGeomCandidateFlg, type=Integer, unit=1, desc=几何候选存在标志
 * @field name=FallbackHybridCandidateFlg, type=Integer, unit=1, desc=Hybrid候选存在标志
 * @field name=FallbackWinnerKind, type=Integer, unit=1, desc=回退赢家0无1几何2Hybrid
 * @field name=FallbackDecisionReason, type=Integer, unit=1, desc=回退决策原因0无1仅有效性2换向次数3路径长度4段数5检查次数
 * @field name=FallbackGeomStpN, type=Integer, unit=1, desc=几何候选换向次数
 * @field name=FallbackGeomPathLen, type=Real, unit=m, desc=几何候选路径总长
 * @field name=FallbackGeomSegNum, type=Integer, unit=1, desc=几何候选段数
 * @field name=FallbackGeomPlnChkCnt, type=Integer, unit=1, desc=几何候选检查次数
 * @field name=FallbackHybridStpN, type=Integer, unit=1, desc=Hybrid候选换向次数
 * @field name=FallbackHybridPathLen, type=Real, unit=m, desc=Hybrid候选路径总长
 * @field name=FallbackHybridSegNum, type=Integer, unit=1, desc=Hybrid候选段数
 * @field name=FallbackHybridPlnChkCnt, type=Integer, unit=1, desc=Hybrid候选检查次数
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Integer PlanFlg;
  Integer SegNum;
  Real PathLen;
  Integer SegGoDir[HT_SEG_OUT_MAX];
  Integer SegTrnDir[HT_SEG_OUT_MAX];
  Real SegCrv[HT_SEG_OUT_MAX];
  Real SegTht[HT_SEG_OUT_MAX];
  Real SegLen[HT_SEG_OUT_MAX];
  Real SegStX[HT_SEG_OUT_MAX];
  Real SegStY[HT_SEG_OUT_MAX];
  Real SegStTh[HT_SEG_OUT_MAX];
  Real SegCcX[HT_SEG_OUT_MAX];
  Real SegCcY[HT_SEG_OUT_MAX];
  Real DngPtX;
  Real DngPtY;
  Integer PlnChkCnt;
  Integer FailRs0;
  Integer StpN;
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
} GeoHtPathPlanningOutput_t;

void geoHtPathPlanning(const GeoHtPathPlanningInput_t *input,
                       const GeoHtPathPlanningParam_t *param,
                       GeoHtPathPlanningOutput_t *output);

#endif
