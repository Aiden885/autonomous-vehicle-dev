#ifndef APA_CTRL_REF_HANDOFF_TYPES_H
#define APA_CTRL_REF_HANDOFF_TYPES_H

#include "baseTypes.h"

#ifndef PATH_PT_MAX
#define PATH_PT_MAX 100
#endif

/* 段表最大段数(对应 MATLAB APA_Ctrl_Chart 的 StpNum 上限) */
#ifndef APA_CTRL_REF_HANDOFF_MAX_SEG
#define APA_CTRL_REF_HANDOFF_MAX_SEG 30
#endif

/**
 * @brief 参考交接辅助输入
 * @en_name ApaCtrlRefHandoffInput
 * @cn_name 参考交接辅助输入
 * @tag STRUCT
 * @field name=VehicleX, type=Real, unit=m, desc=车辆X
 * @field name=VehicleY, type=Real, unit=m, desc=车辆Y
 * @field name=VehicleYaw, type=Real, unit=rad, desc=车辆航向
 * @field name=GearPos, type=Integer, unit=1, desc=实际档位(0=未接入/未知,1=D,2=R; 未知档位不放行)
 * @field name=PathUpdateIdx, type=Integer, unit=1, desc=路径刷新序号(对应MATLAB Path_Update_Idx)
 * @field name=PtCount, type=Integer, unit=1, desc=有效路径点数量
 * @field name=PthPtX, type=Array<Real, [100]>, unit=m, desc=路径点X数组
 * @field name=PthPtY, type=Array<Real, [100]>, unit=m, desc=路径点Y数组
 * @field name=PthPtTht, type=Array<Real, [100]>, unit=rad, desc=路径点航向数组
 * @field name=PthPtOdo, type=Array<Real, [100]>, unit=m, desc=路径点里程数组
 * @field name=RawRefX, type=Real, unit=m, desc=原始参考点X
 * @field name=RawRefY, type=Real, unit=m, desc=原始参考点Y
 * @field name=RawRefYaw, type=Real, unit=rad, desc=原始参考点航向
 * @field name=RawRefSpeed, type=Real, unit=m/s, desc=原始参考速度
 * @field name=RawRefIndex, type=Integer, unit=1, desc=原始参考点索引
 * @version 1.0
 * @date 2026-03-09
 * @author aide
 */
typedef struct {
  Real VehicleX;
  Real VehicleY;
  Real VehicleYaw;
  Integer GearPos;
  Integer PathUpdateIdx;
  Integer PtCount;
  Real PthPtX[PATH_PT_MAX];
  Real PthPtY[PATH_PT_MAX];
  Real PthPtTht[PATH_PT_MAX];
  Real PthPtOdo[PATH_PT_MAX];
  Real RawRefX;
  Real RawRefY;
  Real RawRefYaw;
  Real RawRefSpeed;
  Integer RawRefIndex;
} ApaCtrlRefHandoffInput_t;

/**
 * @brief 参考交接辅助参数
 * @en_name ApaCtrlRefHandoffParam
 * @cn_name 参考交接辅助参数
 * @tag STRUCT
 * @field name=ForwardRefSpeed, type=Real, unit=m/s, desc=前进参考速度
 * @field name=ReverseRefSpeed, type=Real, unit=m/s, desc=倒车参考速度
 * @field name=StopRefSpeed, type=Real, unit=m/s, desc=停止参考速度
 * @field name=DistOverStp, type=Real, unit=m, desc=段切换距离阈值(对应MATLAB Dist_OverStp)
 * @field name=PointReachedDistSq, type=Real, unit=m2, desc=目标到达距离平方阈值
 * @field name=StopDwellCycles, type=Integer, unit=1, desc=换向停车过渡持续周期数
 * @field name=GearWaitCycles, type=Integer, unit=1, desc=档位等待模拟周期数
 * @field name=StallTimeout, type=Integer, unit=1, desc=段内索引停滞超时周期数(对应MATLAB MocoBlock)
 * @field name=ExpectStartDir, type=Integer, unit=1, desc=期望起步方向(0=自动推断 1=前进 -1=倒车 对应MATLAB expect_start_direction)
 * @field name=ExpectDirOdoThresh, type=Real, unit=m, desc=起步方向偏好生效的最小段长阈值(对应MATLAB expect_direction_odo_thresh)
 * @field name=EnableGenRevPt, type=Integer, unit=1, desc=是否使能倒车点生成(0=否 1=是 对应MATLAB enable_generate_reverse_point)
 * @field name=ReverseDist, type=Real, unit=m, desc=倒车点距离(对应MATLAB reverse_dist)
 * @version 3.0
 * @date 2026-04-12
 * @author aide
 */
typedef struct {
  Real ForwardRefSpeed;
  Real ReverseRefSpeed;
  Real StopRefSpeed;
  Real DistOverStp;
  Real PointReachedDistSq;
  Integer StopDwellCycles;
  Integer GearWaitCycles;
  Integer StallTimeout;
  Integer ExpectStartDir;
  Real ExpectDirOdoThresh;
  Integer EnableGenRevPt;
  Real ReverseDist;
} ApaCtrlRefHandoffParam_t;

/**
 * @brief 参考交接辅助状态（按段推进状态机）
 * @en_name ApaCtrlRefHandoffState
 * @cn_name 参考交接辅助状态
 * @tag STRUCT
 * @field name=Phase, type=Integer, unit=1, desc=状态机阶段(0=IDLE,1=SEG_INIT,2=GEAR_WAIT,3=SEG_TRACK,4=STOP_WAIT,5=SEG_ADVANCE,6=ARRIVED)
 * @field name=StpNum, type=Integer, unit=1, desc=路径总段数(对应MATLAB APA_StpNum)
 * @field name=StpNow, type=Integer, unit=1, desc=当前段号0-based(对应MATLAB APA_StpNow)
 * @field name=CurIndx, type=Integer, unit=1, desc=段内当前参考点索引(对应MATLAB Cur_Indx)
 * @field name=StopCounter, type=Integer, unit=1, desc=停车过渡已持续周期数
 * @field name=GearCounter, type=Integer, unit=1, desc=档位等待已持续周期数
 * @field name=SegTableBuilt, type=Integer, unit=1, desc=段表是否已构建(0=未构建,1=已构建)
 * @field name=LastPtCount, type=Integer, unit=1, desc=上一周期路径点数(用于检测路径刷新)
 * @field name=LastPathUpdateIdx, type=Integer, unit=1, desc=上一周期路径刷新序号
 * @field name=StallCounter, type=Integer, unit=1, desc=段内索引停滞计数器(阻塞检测)
 * @field name=OdoAccum, type=Real, unit=m, desc=当前段内车辆行驶里程累积(对应MATLAB LocationTrack)
 * @field name=PathOdoAccum, type=Real, unit=m, desc=当前段内路径odo累积到CurIndx
 * @field name=PrevVehX, type=Real, unit=m, desc=上一步车辆X(用于计算步长)
 * @field name=PrevVehY, type=Real, unit=m, desc=上一步车辆Y(用于计算步长)
 * @field name=SegStartIdx, type=Array<Integer, [30]>, unit=1, desc=每段起始点索引(对应MATLAB RevPtIdx)
 * @field name=SegEndIdx, type=Array<Integer, [30]>, unit=1, desc=每段结束点索引
 * @field name=SegGoDir, type=Array<Integer, [30]>, unit=1, desc=每段方向(+1=前进,-1=倒车)
 * @version 2.0
 * @date 2026-03-14
 * @author aide
 */
typedef struct {
  Integer Phase;
  Integer StpNum;
  Integer StpNow;
  Integer CurIndx;
  Integer StopCounter;
  Integer GearCounter;
  Integer SegTableBuilt;
  Integer LastPtCount;
  Integer LastPathUpdateIdx;
  Integer StallCounter;
  Real OdoAccum;
  Real PathOdoAccum;
  Real PrevVehX;
  Real PrevVehY;
  Integer SegStartIdx[APA_CTRL_REF_HANDOFF_MAX_SEG];
  Integer SegEndIdx[APA_CTRL_REF_HANDOFF_MAX_SEG];
  Integer SegGoDir[APA_CTRL_REF_HANDOFF_MAX_SEG];
} ApaCtrlRefHandoffState_t;

/**
 * @brief 参考交接辅助输出
 * @en_name ApaCtrlRefHandoffOutput
 * @cn_name 参考交接辅助输出
 * @tag STRUCT
 * @field name=RefX, type=Real, unit=m, desc=参考点X
 * @field name=RefY, type=Real, unit=m, desc=参考点Y
 * @field name=RefYaw, type=Real, unit=rad, desc=参考点航向
 * @field name=RefSpeed, type=Real, unit=m/s, desc=参考速度
 * @field name=RefIndex, type=Integer, unit=1, desc=参考点索引
 * @field name=RefCurvature, type=Real, unit=1/m, desc=参考点处路径曲率(仿MATLAB sin(dθ)/ds)
 * @field name=LongitudinalError, type=Real, unit=m, desc=车辆到参考点沿路径切线方向的纵向偏差(前正后负)
 * @field name=LateralError, type=Real, unit=m, desc=车辆到路径切线的横向偏差(右正左负)
 * @field name=HeadingError, type=Real, unit=rad, desc=车辆航向与路径航向偏差
 * @field name=CtrlPhase, type=Integer, unit=1, desc=APA_Ctrl_Chart阶段(0=NoAct,1=StpInitial,2=GearCheck,3=StepRun,4=StopWait,5=StpSwitch,6=Done)
 * @field name=ApaStpNum, type=Integer, unit=1, desc=路径总段数(对应MATLAB APA_StpNum)
 * @field name=ApaStpNow, type=Integer, unit=1, desc=当前段号1-based(对应MATLAB APA_StpNow)
 * @field name=ApaGoDir, type=Integer, unit=1, desc=当前段行驶方向(1=前进,-1=倒车)
 * @field name=Dist2Go, type=Real, unit=m, desc=当前段剩余路径距离(对应MATLAB Dist2Go)
 * @field name=TargetGearReq, type=Integer, unit=1, desc=目标档位请求(1=D,2=R)
 * @field name=SegSearchStart, type=Integer, unit=1, desc=当前段搜索起始索引(传递给pathPointSelector)
 * @field name=SegSearchEnd, type=Integer, unit=1, desc=当前段搜索结束索引(传递给pathPointSelector)
 * @version 3.0
 * @date 2026-03-14
 * @author aide
 */
typedef struct {
  Real RefX;
  Real RefY;
  Real RefYaw;
  Real RefSpeed;
  Integer RefIndex;
  Real RefCurvature;
  Real LongitudinalError;
  Real LateralError;
  Real HeadingError;
  Integer CtrlPhase;
  Integer ApaStpNum;
  Integer ApaStpNow;
  Integer ApaGoDir;
  Real Dist2Go;
  Integer TargetGearReq;
  Integer SegSearchStart;
  Integer SegSearchEnd;
} ApaCtrlRefHandoffOutput_t;

void apaCtrlRefHandoff(const ApaCtrlRefHandoffInput_t *input,
                       const ApaCtrlRefHandoffParam_t *param,
                       ApaCtrlRefHandoffState_t *state,
                       ApaCtrlRefHandoffOutput_t *output);

#endif
