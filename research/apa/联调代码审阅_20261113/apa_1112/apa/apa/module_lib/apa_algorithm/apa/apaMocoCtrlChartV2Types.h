#ifndef APA_MOCO_CTRL_CHART_V2_TYPES_H
#define APA_MOCO_CTRL_CHART_V2_TYPES_H

#include "baseTypes.h"
#include "apaMocoExecution.h"
#include "apaMocoPathGeometry.h"

#ifndef PATH_PT_MAX
#define PATH_PT_MAX 100
#endif

#ifndef APA_MOCO_V2_MAX_SEG
#define APA_MOCO_V2_MAX_SEG 30
#endif

/**
 * @brief MoCo v2状态机输入
 * @en_name ApaMocoCtrlChartV2Input
 * @cn_name MoCo v2状态机输入
 * @tag STRUCT
 * @field name=VehicleX, type=Real, unit=m, desc=车辆X
 * @field name=VehicleY, type=Real, unit=m, desc=车辆Y
 * @field name=VehicleYaw, type=Real, unit=rad, desc=车辆航向
 * @field name=GoalX, type=Real, unit=m, desc=任务请求目标后轴X不使用路径末点替代
 * @field name=GoalY, type=Real, unit=m, desc=任务请求目标后轴Y
 * @field name=GoalYaw, type=Real, unit=rad, desc=任务请求目标航向
 * @field name=GearPos, type=Integer, unit=1, desc=实际档位(0=未知,1=D,2=R; 未知档位不放行)
 * @field name=PathUpdateIdx, type=Integer, unit=1, desc=路径刷新序号
 * @field name=TargetSlotKind, type=Integer, unit=1, desc=目标车位类型
 * @field name=ParkingAction, type=Integer, unit=1, desc=泊车动作(1泊入,2泊出)
 * @field name=PtCount, type=Integer, unit=1, desc=有效路径点数量
 * @field name=PthPtX, type=Array<Real, [100]>, unit=m, desc=路径点X
 * @field name=PthPtY, type=Array<Real, [100]>, unit=m, desc=路径点Y
 * @field name=PthPtTht, type=Array<Real, [100]>, unit=rad, desc=路径点航向
 * @field name=PthPtOdo, type=Array<Real, [100]>, unit=m, desc=路径点里程
 * @field name=VehicleSpeed, type=Real, unit=m/s, desc=车辆有符号实测速度
 * @field name=GoalBlocked, type=Boolean, unit=1, desc=当前最终位姿存在实车碰撞或不满足正常余量
 * @field name=BrakeDistance, type=Real, unit=m, desc=安全制动距离预算
 * @field name=ApproachDistance, type=Real, unit=m, desc=独立于到点制动的接近减速距离
 * @field name=CreepDistance, type=Real, unit=m, desc=段末低速接近区长度
 * @field name=SteeringPending, type=Boolean, unit=1, desc=目标转角尚未到位
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Real VehicleX;
  Real VehicleY;
  Real VehicleYaw;
  Real GoalX;
  Real GoalY;
  Real GoalYaw;
  Integer GearPos;
  Integer PathUpdateIdx;
  Integer TargetSlotKind;
  Integer ParkingAction;
  Integer PtCount;
  Real PthPtX[PATH_PT_MAX];
  Real PthPtY[PATH_PT_MAX];
  Real PthPtTht[PATH_PT_MAX];
  Real PthPtOdo[PATH_PT_MAX];
  Real VehicleSpeed; /* signed measured speed, m/s */
  Real BrakeDistance;
  Boolean GoalBlocked;   /* 终点车身及净距检查回调。 */
  Real ApproachDistance; /* distance-profile horizon, separate from arrival */
  Real CreepDistance;
  Boolean SteeringPending;
} ApaMocoCtrlChartV2Input_t;

/**
 * @brief MoCo v2状态机参数
 * @en_name ApaMocoCtrlChartV2Param
 * @cn_name MoCo v2状态机参数
 * @tag STRUCT
 * @field name=DistOverStp, type=Real, unit=m, desc=保留参数，不参与到点或切段判定
 * @field name=PointReachedDistSq, type=Real, unit=m2, desc=保留的到达距离平方阈值
 * @field name=StopDwellCycles, type=Integer, unit=1, desc=换向停车等待周期
 * @field name=GearWaitCycles, type=Integer, unit=1, desc=档位等待周期
 * @field name=KPathMinStp, type=Real, unit=m, desc=保留的路径步长参数，不触发输入路径重采样
 * @field name=TerminalYawTol, type=Real, unit=rad, desc=保留的终端航向阈值
 * @field name=CuspPositionTolerance, type=Real, unit=m, desc=方向段端点的位置容差
 * @field name=CuspYawTolerance, type=Real, unit=rad, desc=方向段端点的航向容差
 * @field name=GoalPositionTolerance, type=Real, unit=m, desc=任务最终目标的位置容差
 * @field name=GoalYawTolerance, type=Real, unit=rad, desc=任务最终目标的航向容差
 * @field name=StandstillSpeed, type=Real, unit=m/s, desc=停稳判定的速度阈值
 * @field name=GoalDwellCycles, type=Integer, unit=1, desc=最终到位停稳确认控制周期数
 * @field name=GearTimeoutCycles, type=Integer, unit=1, desc=挡位确认超时控制周期数
 * @field name=StallTimeoutCycles, type=Integer, unit=1, desc=无进展超时控制周期数
 * @field name=SegmentTimeoutCycles, type=Integer, unit=1, desc=方向段执行超时控制周期数
 * @field name=SearchBackSteps, type=Integer, unit=1, desc=投影搜索向后扩展的路径区间数
 * @field name=SearchAheadSteps, type=Integer, unit=1, desc=投影搜索向前扩展的路径区间数
 * @field name=MaxProgressAdvance, type=Real, unit=m, desc=单周期允许的最大进度推进量
 * @field name=MaxReferenceDistance, type=Real, unit=m, desc=车辆到路径参考的最大距离
 * @field name=MaxHeadingError, type=Real, unit=rad, desc=车辆与参考航向的最大偏差
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Real DistOverStp;
  Real PointReachedDistSq;
  Integer StopDwellCycles;
  Integer GearWaitCycles;
  Real KPathMinStp;
  Real TerminalYawTol;
  Real CuspPositionTolerance;
  Real CuspYawTolerance;
  Real GoalPositionTolerance;
  Real GoalYawTolerance;
  Real StandstillSpeed;
  Integer GoalDwellCycles;
  Integer GearTimeoutCycles;
  Integer StallTimeoutCycles;
  Integer SegmentTimeoutCycles;
  Integer SearchBackSteps;
  Integer SearchAheadSteps;
  Real MaxProgressAdvance;
  Real MaxReferenceDistance;
  Real MaxHeadingError;
} ApaMocoCtrlChartV2Param_t;

/**
 * @brief MoCo v2状态机状态
 * @en_name ApaMocoCtrlChartV2State
 * @cn_name MoCo v2状态机状态
 * @tag STRUCT
 * @field name=Phase, type=Integer, unit=1, desc=状态机阶段
 * @field name=StpNum, type=Integer, unit=1, desc=总段数
 * @field name=StpNow, type=Integer, unit=1, desc=当前段0-based
 * @field name=StopCounter, type=Integer, unit=1, desc=停车等待计数
 * @field name=GearCounter, type=Integer, unit=1, desc=档位等待计数
 * @field name=LastPtCount, type=Integer, unit=1, desc=上一周期路径点数
 * @field name=LastPathUpdateIdx, type=Integer, unit=1, desc=上一周期路径刷新序号
 * @field name=SegStartIdx, type=Array<Integer, [30]>, unit=1, desc=段起点
 * @field name=SegEndIdx, type=Array<Integer, [30]>, unit=1, desc=段终点
 * @field name=SegGoDir, type=Array<Integer, [30]>, unit=1, desc=段方向
 * @field name=ProjectionIndex, type=Integer, unit=1, desc=连续投影所在路径区间的零基索引
 * @field name=Progress, type=Real, unit=m, desc=当前方向段的累计进度
 * @field name=ProgressAnchor, type=Real, unit=m, desc=无进展检查使用的进度基准
 * @field name=StallCounter, type=Integer, unit=1, desc=无进展累计控制周期数
 * @field name=SegmentCounter, type=Integer, unit=1, desc=当前方向段累计控制周期数
 * @field name=GearTimeoutCounter, type=Integer, unit=1, desc=挡位确认累计控制周期数
 * @field name=StopReason, type=Integer, unit=1, desc=控制停止原因代码
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Integer Phase;
  Integer StpNum;
  Integer StpNow;
  Integer StopCounter;
  Integer GearCounter;
  Integer LastPtCount;
  Integer LastPathUpdateIdx;
  Integer SegStartIdx[APA_MOCO_V2_MAX_SEG];
  Integer SegEndIdx[APA_MOCO_V2_MAX_SEG];
  Integer SegGoDir[APA_MOCO_V2_MAX_SEG];
  Integer ProjectionIndex;
  Real Progress;
  Real ProgressAnchor;
  Integer StallCounter;
  Integer SegmentCounter;
  Integer GearTimeoutCounter;
  Integer StopReason;
} ApaMocoCtrlChartV2State_t;

/**
 * @brief MoCo v2状态机输出
 * @en_name ApaMocoCtrlChartV2Output
 * @cn_name MoCo v2状态机输出
 * @tag STRUCT
 * @field name=CtrlPhase, type=Integer, unit=1, desc=0空闲1初始化2确认档位3跟踪4停稳5保留6完成7减速8低速接近9故障
 * @field name=ApaStpNum, type=Integer, unit=1, desc=总段数
 * @field name=ApaStpNow, type=Integer, unit=1, desc=当前段1-based
 * @field name=ApaGoDir, type=Integer, unit=1, desc=当前段方向
 * @field name=Dist2Go, type=Real, unit=m, desc=当前段剩余距离
 * @field name=TargetGearReq, type=Integer, unit=1, desc=目标档位请求
 * @field name=TerminalMode, type=Integer, unit=1, desc=最后一段终端模式
 * @field name=ParkingCate, type=Integer, unit=1, desc=泊车类型
 * @field name=SegmentStartIdx, type=Integer, unit=1, desc=当前段全局起点
 * @field name=SegmentEndIdx, type=Integer, unit=1, desc=当前段全局终点
 * @field name=LocalPtCount, type=Integer, unit=1, desc=局部段路径点数量
 * @field name=LocalPathX, type=Array<Real, [100]>, unit=m, desc=局部路径X
 * @field name=LocalPathY, type=Array<Real, [100]>, unit=m, desc=局部路径Y
 * @field name=LocalPathYaw, type=Array<Real, [100]>, unit=rad, desc=局部路径航向
 * @field name=LocalPathOdo, type=Array<Real, [100]>, unit=m, desc=局部路径里程
 * @field name=PathOverflowFlag, type=Integer, unit=1, desc=局部路径容量截断标志
 * @field name=PathRefreshAccepted, type=Integer, unit=1, desc=本周期路径刷新接受标志
 * @field name=DoneReady, type=Integer, unit=1, desc=终点位置航向和持续停稳均达标
 * @field name=Projection, type=ApaMocoPathProjection_t, unit=1, desc=当前方向段的连续路径投影
 * @field name=GoalDistance, type=Real, unit=m, desc=实测后轴到任务目标的位置误差
 * @field name=GoalYawError, type=Real, unit=rad, desc=实测航向到任务目标的绝对角度误差
 * @field name=StopReason, type=Integer, unit=1, desc=控制停止原因代码
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Integer CtrlPhase;
  Integer ApaStpNum;
  Integer ApaStpNow;
  Integer ApaGoDir;
  Real Dist2Go;
  Integer TargetGearReq;
  Integer TerminalMode;
  Integer ParkingCate;
  Integer SegmentStartIdx;
  Integer SegmentEndIdx;
  Integer LocalPtCount;
  Real LocalPathX[PATH_PT_MAX];
  Real LocalPathY[PATH_PT_MAX];
  Real LocalPathYaw[PATH_PT_MAX];
  Real LocalPathOdo[PATH_PT_MAX];
  Integer PathOverflowFlag;
  Integer PathRefreshAccepted;
  Integer DoneReady;
  ApaMocoPathProjection_t Projection;
  Real GoalDistance;
  Real GoalYawError;
  Integer StopReason;
} ApaMocoCtrlChartV2Output_t;

void apaMocoCtrlChartV2(const ApaMocoCtrlChartV2Input_t *input,
                        const ApaMocoCtrlChartV2Param_t *param,
                        ApaMocoCtrlChartV2State_t *state,
                        ApaMocoCtrlChartV2Output_t *output);

#endif
