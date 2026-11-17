#ifndef APA_PARKING_PATH_PLANNING_TYPES_H
#define APA_PARKING_PATH_PLANNING_TYPES_H

#include "apaPlanningTypes.h"
#include "baseTypes.h"
#include "apaHybridPlanningTypes.h"
#include "apaPlanningScene.h"

#ifndef APA_PARKING_PATH_SEG_OUT_MAX
#define APA_PARKING_PATH_SEG_OUT_MAX 9
#endif

#ifndef APA_PARKING_PATH_PT_MAX
#define APA_PARKING_PATH_PT_MAX 100
#endif

/**
 * @brief APA泊车路径规划输入
 * @en_name ApaParkingPathPlanningInput
 * @cn_name APA泊车路径规划输入
 * @tag STRUCT
 * @field name=ParkingAction, type=Integer, unit=1, desc=泊车动作1泊入2泊出
 * @field name=TargetSlotKind, type=Integer, unit=1, desc=目标车位类型0手动1VT2HT3DthEdVT4VH；场景源泊出时可保留5但顶层按ParkingAction路由
 * @field name=VehicleX, type=Real, unit=m, desc=车辆X
 * @field name=VehicleY, type=Real, unit=m, desc=车辆Y
 * @field name=VehicleYaw, type=Real, unit=rad, desc=车辆航向
 * @field name=TarX, type=Real, unit=m, desc=目标X
 * @field name=TarY, type=Real, unit=m, desc=目标Y
 * @field name=TarTheta, type=Real, unit=rad, desc=目标航向
 * @field name=GoalPositionTolerance, type=Real, unit=m, desc=本任务终点位置容差为0使用共享默认值
 * @field name=GoalYawTolerance, type=Real, unit=rad, desc=本任务终点航向容差为0使用共享默认值
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
 * @field name=SlotTrackStopperNum, type=Integer, unit=1, desc=slot stopper线段数量
 * @field name=SlotTrackStopperX1, type=Array<Real, [64]>, unit=m, desc=slot stopper端点1X
 * @field name=SlotTrackStopperY1, type=Array<Real, [64]>, unit=m, desc=slot stopper端点1Y
 * @field name=SlotTrackStopperX2, type=Array<Real, [64]>, unit=m, desc=slot stopper端点2X
 * @field name=SlotTrackStopperY2, type=Array<Real, [64]>, unit=m, desc=slot stopper端点2Y
 * @field name=BoundarySepPtNum, type=Integer, unit=1, desc=真实离散点数量
 * @field name=BoundarySepPtX, type=Array<Real, [128]>, unit=m, desc=真实离散点X
 * @field name=BoundarySepPtY, type=Array<Real, [128]>, unit=m, desc=真实离散点Y
 * @field name=StopperPtNum, type=Integer, unit=1, desc=挡停点数量
 * @field name=StopperPtX, type=Array<Real, [128]>, unit=m, desc=挡停点X
 * @field name=StopperPtY, type=Array<Real, [128]>, unit=m, desc=挡停点Y
 * @field name=ShortCutSw, type=Integer, unit=1, desc=是否启用轨迹shortcut
 * @field name=ShortCutThd, type=Real, unit=m, desc=shortcut横向阈值
 * @version 1.0
 * @date 2026-07-28
 * @author aide
 */
typedef struct {
  Integer ParkingAction;
  Integer TargetSlotKind;
  Real VehicleX;
  Real VehicleY;
  Real VehicleYaw;
  Real TarX;
  Real TarY;
  Real TarTheta;
  Real GoalPositionTolerance;
  Real GoalYawTolerance;
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
  Real BoundaryLineX1[APA_SCENARIO_BOUNDARY_LINE_MAX];
  Real BoundaryLineY1[APA_SCENARIO_BOUNDARY_LINE_MAX];
  Real BoundaryLineX2[APA_SCENARIO_BOUNDARY_LINE_MAX];
  Real BoundaryLineY2[APA_SCENARIO_BOUNDARY_LINE_MAX];
  Integer VirtualBoundaryLineNum;
  Real VirtualBoundaryP1X[APA_SCENARIO_BOUNDARY_LINE_MAX];
  Real VirtualBoundaryP1Y[APA_SCENARIO_BOUNDARY_LINE_MAX];
  Real VirtualBoundaryP2X[APA_SCENARIO_BOUNDARY_LINE_MAX];
  Real VirtualBoundaryP2Y[APA_SCENARIO_BOUNDARY_LINE_MAX];
  Integer SlotTrackStopperNum;
  Real SlotTrackStopperX1[APA_SCENARIO_BOUNDARY_LINE_MAX];
  Real SlotTrackStopperY1[APA_SCENARIO_BOUNDARY_LINE_MAX];
  Real SlotTrackStopperX2[APA_SCENARIO_BOUNDARY_LINE_MAX];
  Real SlotTrackStopperY2[APA_SCENARIO_BOUNDARY_LINE_MAX];
  Integer BoundarySepPtNum;
  Real BoundarySepPtX[APA_SCENARIO_BOUNDARY_POINT_MAX];
  Real BoundarySepPtY[APA_SCENARIO_BOUNDARY_POINT_MAX];
  Integer StopperPtNum;
  Real StopperPtX[APA_SCENARIO_BOUNDARY_POINT_MAX];
  Real StopperPtY[APA_SCENARIO_BOUNDARY_POINT_MAX];
  Integer ShortCutSw;
  Real ShortCutThd;
} ApaParkingPathPlanningInput_t;

/**
 * @brief APA泊车路径规划参数
 * @en_name ApaParkingPathPlanningParam
 * @cn_name APA泊车路径规划参数
 * @tag STRUCT
 * @field name=SltState, type=Integer, unit=1, desc=车位状态
 * @field name=VehW, type=Real, unit=m, desc=车宽
 * @field name=VehLlF, type=Real, unit=m, desc=后轴参考点到前保险杠长度
 * @field name=VehLfMin, type=Real, unit=m, desc=最小前向安全长度
 * @field name=VehLr, type=Real, unit=m, desc=后悬到参考点长度
 * @field name=VtFallbackSpace, type=Real, unit=m, desc=VT边界探测失败回退空间
 * @field name=DthFallbackSpace, type=Real, unit=m, desc=DTH边界探测失败回退空间
 * @field name=HtRfoutMin, type=Real, unit=m, desc=HT外侧前角扫掠半径为0按APA_Cal计算
 * @field name=NeedHybridAlgFusion, type=Integer, unit=1, desc=是否允许HybridA融合回退
 * @field name=SceneQuery, type=ApaPlanningSceneQuery_t, unit=1, desc=完整场景查询回调及进程内上下文
 * @field name=Diagnostics, type=ApaPlanningDiagnostics_t*, unit=1, desc=本次规划诊断输出指针，仅进程内使用
 * @field name=HybridPolicy, type=ApaPlannerHybridPolicy_t, unit=1, desc=本次搜索的候选比较和准入回调策略
 * @version 1.0
 * @date 2026-07-28
 * @author aide
 */
typedef struct {
  Integer SltState;
  Real VehW;
  Real VehLlF;
  Real VehLfMin;
  Real VehLr;
  Real VtFallbackSpace;
  Real DthFallbackSpace;
  Real HtRfoutMin;
  Integer NeedHybridAlgFusion;
  ApaPlanningSceneQuery_t SceneQuery; /* Optional, owned by the calling worker. */
  ApaPlanningDiagnostics_t *Diagnostics;
  ApaPlannerHybridPolicy_t HybridPolicy;
} ApaParkingPathPlanningParam_t;

/**
 * @brief APA泊车路径规划输出
 * @en_name ApaParkingPathPlanningOutput
 * @cn_name APA泊车路径规划输出
 * @tag STRUCT
 * @field name=SelectedSlotKind, type=Integer, unit=1, desc=已选择车位类型
 * @field name=PlanFlg, type=Integer, unit=1, desc=规划成功标志
 * @field name=ErrorType, type=Integer, unit=1, desc=已有ApaPathPlanFault故障码；无可靠分支故障时为UNKNOWN
 * @field name=PlannerCase, type=Integer, unit=1, desc=规划分支编号
 * @field name=SegNum, type=Integer, unit=1, desc=路径段数
 * @field name=StpN, type=Integer, unit=1, desc=换向次数
 * @field name=PathLen, type=Real, unit=m, desc=路径总长
 * @field name=PtCount, type=Integer, unit=1, desc=离散点数
 * @field name=StepCount, type=Integer, unit=1, desc=VT/HT/DTH/VH的MATLAB机动步数；ParkingOut不适用为0
 * @field name=St2Idx, type=Integer, unit=1, desc=VT/HT/DTH的MATLAB 1-based首段结束点序号；VH/ParkingOut不适用为0
 * @field name=Stp1PtRX, type=Real, unit=m, desc=仅VT/DTH适用的首个前进点车体坐标X；其他分支为0
 * @field name=Stp1PtRY, type=Real, unit=m, desc=仅VT/DTH适用的首个前进点车体坐标Y；其他分支为0
 * @field name=Stp1PtRTheta, type=Real, unit=rad, desc=仅VT/DTH适用的归一化车体相对航向；其他分支为0
 * @field name=PthPtX, type=Array<Real, [100]>, unit=m, desc=离散点X数组
 * @field name=PthPtY, type=Array<Real, [100]>, unit=m, desc=离散点Y数组
 * @field name=PthPtTht, type=Array<Real, [100]>, unit=rad, desc=离散点航向数组
 * @field name=PthPtOdo, type=Array<Real, [100]>, unit=m, desc=离散点里程数组
 * @version 1.0
 * @date 2026-07-28
 * @author aide
 */
typedef struct {
  Integer SelectedSlotKind;
  Integer PlanFlg;
  Integer ErrorType;
  Integer PlannerCase;
  Integer SegNum;
  Integer StpN;
  Real PathLen;
  Integer PtCount;
  Integer StepCount;
  Integer St2Idx;
  Real Stp1PtRX;
  Real Stp1PtRY;
  Real Stp1PtRTheta;
  Real PthPtX[APA_PARKING_PATH_PT_MAX];
  Real PthPtY[APA_PARKING_PATH_PT_MAX];
  Real PthPtTht[APA_PARKING_PATH_PT_MAX];
  Real PthPtOdo[APA_PARKING_PATH_PT_MAX];
} ApaParkingPathPlanningOutput_t;

#endif
