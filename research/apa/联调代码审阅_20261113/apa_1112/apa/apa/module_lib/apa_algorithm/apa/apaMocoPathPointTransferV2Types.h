#ifndef APA_MOCO_PATH_POINT_TRANSFER_V2_TYPES_H
#define APA_MOCO_PATH_POINT_TRANSFER_V2_TYPES_H

#include "baseTypes.h"
#include "apaMocoPathGeometry.h"

#ifndef PATH_PT_MAX
#define PATH_PT_MAX 100
#endif

/**
 * @brief MoCo v2路径点转换输入
 * @en_name ApaMocoPathPointTransferV2Input
 * @cn_name MoCo v2路径点转换输入
 * @tag STRUCT
 * @field name=VehicleX, type=Real, unit=m, desc=车辆X
 * @field name=VehicleY, type=Real, unit=m, desc=车辆Y
 * @field name=VehicleYaw, type=Real, unit=rad, desc=车辆航向
 * @field name=VehicleSpeed, type=Real, unit=m/s, desc=同帧实测有符号车速
 * @field name=VehicleLateralSpeed, type=Real, unit=m/s, desc=同帧实测后轴横向速度(车体+Y向右为正)
 * @field name=SteeringAngle, type=Real, unit=rad, desc=名义前轮转角估计
 * @field name=Wheelbase, type=Real, unit=m, desc=车辆轴距
 * @field name=ResponseTime, type=Real, unit=s, desc=控制参考与预测位姿的共同响应时间
 * @field name=CtrlPhase, type=Integer, unit=1, desc=控制阶段
 * @field name=ApaGoDir, type=Integer, unit=1, desc=当前段方向
 * @field name=SegmentStartIdx, type=Integer, unit=1, desc=当前段全局起点
 * @field name=SegmentEndIdx, type=Integer, unit=1, desc=当前段全局终点
 * @field name=TerminalMode, type=Integer, unit=1, desc=终端模式
 * @field name=ParkingCate, type=Integer, unit=1, desc=泊车类型
 * @field name=LocalPtCount, type=Integer, unit=1, desc=局部路径点数量
 * @field name=LocalPathX, type=Array<Real, [100]>, unit=m, desc=局部路径X
 * @field name=LocalPathY, type=Array<Real, [100]>, unit=m, desc=局部路径Y
 * @field name=LocalPathYaw, type=Array<Real, [100]>, unit=rad, desc=局部路径航向
 * @field name=LocalPathOdo, type=Array<Real, [100]>, unit=m, desc=局部路径里程
 * @field name=UseProjection, type=Boolean, unit=1, desc=使用输入的连续路径投影
 * @field name=Projection, type=ApaMocoPathProjection_t, unit=1, desc=当前方向段的连续路径投影
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Real VehicleX;
  Real VehicleY;
  Real VehicleYaw;
  Real VehicleSpeed;
  Real VehicleLateralSpeed; /* m/s, body +Y (right) at rear axle; 0 when unmeasured. */
  Real SteeringAngle;       /* rad, estimated road-wheel angle. */
  Real Wheelbase;           /* m, used when ResponseTime > 0. */
  Real ResponseTime;        /* s, matched curvature/pose prediction horizon. */
  Integer CtrlPhase;
  Integer ApaGoDir;
  Integer SegmentStartIdx;
  Integer SegmentEndIdx;
  Integer TerminalMode;
  Integer ParkingCate;
  Integer LocalPtCount;
  Real LocalPathX[PATH_PT_MAX];
  Real LocalPathY[PATH_PT_MAX];
  Real LocalPathYaw[PATH_PT_MAX];
  Real LocalPathOdo[PATH_PT_MAX];
  Boolean UseProjection;
  ApaMocoPathProjection_t Projection;
} ApaMocoPathPointTransferV2Input_t;

/**
 * @brief MoCo v2路径点转换参数
 * @en_name ApaMocoPathPointTransferV2Param
 * @cn_name MoCo v2路径点转换参数
 * @tag STRUCT
 * @field name=LookAheadSteps, type=Integer, unit=1, desc=前视点偏移
 * @field name=SearchBackSteps, type=Integer, unit=1, desc=向后搜索步数
 * @field name=SearchAheadSteps, type=Integer, unit=1, desc=向前搜索步数
 * @field name=PreviewTime, type=Real, unit=s, desc=最小距离基础上的速度预瞄时间
 * @field name=PreviewDistMin, type=Real, unit=m, desc=最小预瞄距离
 * @field name=PreviewDistMax, type=Real, unit=m, desc=最大预瞄距离
 * @field name=TerminalPreviewDist, type=Real, unit=m, desc=终端最小预瞄距离
 * @field name=MaxReferenceDistance, type=Real, unit=m, desc=车辆到路径参考的最大距离
 * @field name=MaxHeadingError, type=Real, unit=rad, desc=车辆与参考航向的最大偏差
 * @field name=MaxProgressAdvance, type=Real, unit=m, desc=单周期允许的最大进度推进量
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Integer LookAheadSteps;
  Integer SearchBackSteps;
  Integer SearchAheadSteps;
  Real PreviewTime;
  Real PreviewDistMin;
  Real PreviewDistMax;
  Real TerminalPreviewDist;
  Real MaxReferenceDistance;
  Real MaxHeadingError;
  Real MaxProgressAdvance;
} ApaMocoPathPointTransferV2Param_t;

/**
 * @brief MoCo v2路径点转换状态
 * @en_name ApaMocoPathPointTransferV2State
 * @cn_name MoCo v2路径点转换状态
 * @tag STRUCT
 * @field name=LastLocalIndex, type=Integer, unit=1, desc=上一周期局部最近点
 * @field name=LastSegmentStartIdx, type=Integer, unit=1, desc=上一周期段起点
 * @field name=LastProgress, type=Real, unit=m, desc=上一周期已接受的段内进度
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Integer LastLocalIndex;
  Integer LastSegmentStartIdx;
  Real LastProgress;
} ApaMocoPathPointTransferV2State_t;

/**
 * @brief MoCo v2路径点转换输出
 * @en_name ApaMocoPathPointTransferV2Output
 * @cn_name MoCo v2路径点转换输出
 * @tag STRUCT
 * @field name=RefX, type=Real, unit=m, desc=参考点X
 * @field name=RefY, type=Real, unit=m, desc=参考点Y
 * @field name=RefYaw, type=Real, unit=rad, desc=参考点航向
 * @field name=RefCurvature, type=Real, unit=1/m, desc=参考曲率
 * @field name=RefIndex, type=Integer, unit=1, desc=全局参考点索引
 * @field name=LocalRefIndex, type=Integer, unit=1, desc=局部参考点索引
 * @field name=LongitudinalError, type=Real, unit=m, desc=纵向误差
 * @field name=LateralError, type=Real, unit=m, desc=横向误差
 * @field name=CrossTrackError, type=Real, unit=m, desc=当前最近点横向跟踪误差
 * @field name=HeadingError, type=Real, unit=rad, desc=航向误差
 * @field name=GoalDistance, type=Real, unit=m, desc=距离当前段终点距离
 * @field name=TerminalYawError, type=Real, unit=rad, desc=终点航向误差绝对值
 * @field name=PathPointNum, type=Integer, unit=1, desc=局部路径点数量
 * @field name=TarPtX, type=Real, unit=m, desc=当前目标点X
 * @field name=TarPtY, type=Real, unit=m, desc=当前目标点Y
 * @field name=TarPtYaw, type=Real, unit=rad, desc=当前目标点航向
 * @field name=NxtPtX, type=Real, unit=m, desc=下一目标点X
 * @field name=NxtPtY, type=Real, unit=m, desc=下一目标点Y
 * @field name=NxtPtYaw, type=Real, unit=rad, desc=下一目标点航向
 * @field name=PreviewDistance, type=Real, unit=m, desc=本周期实际预瞄距离
 * @field name=FeedbackDistance, type=Real, unit=m, desc=未按段末截短的正值反馈调度距离
 * @field name=CurLocalIndex, type=Integer, unit=1, desc=当前局部最近点索引
 * @field name=TargetLocalIndex, type=Integer, unit=1, desc=预瞄目标局部索引
 * @field name=PreviewClamped, type=Integer, unit=1, desc=预瞄距离被限幅标志
 * @field name=ControlCurvature, type=Real, unit=1/m, desc=响应时刻的控制前馈曲率
 * @field name=ControlCrossTrackError, type=Real, unit=m, desc=响应时刻的预测横向误差
 * @field name=ControlHeadingError, type=Real, unit=rad, desc=响应时刻的预测航向误差
 * @field name=ResponseDistance, type=Real, unit=m, desc=当前段内的响应预看距离
 * @field name=ReferenceValid, type=Boolean, unit=1, desc=参考位姿有效标志
 * @field name=Progress, type=Real, unit=m, desc=当前方向段的累计进度
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Real RefX;
  Real RefY;
  Real RefYaw;
  Real RefCurvature;
  Integer RefIndex;
  Integer LocalRefIndex;
  Real LongitudinalError;
  Real LateralError;
  Real CrossTrackError;
  Real HeadingError;
  Real GoalDistance;
  Real TerminalYawError;
  Integer PathPointNum;
  Real TarPtX;
  Real TarPtY;
  Real TarPtYaw;
  Real NxtPtX;
  Real NxtPtY;
  Real NxtPtYaw;
  Real PreviewDistance;
  Real FeedbackDistance;
  Integer CurLocalIndex;
  Integer TargetLocalIndex;
  Integer PreviewClamped;
  Boolean ReferenceValid;
  Real Progress;
  Real ControlCurvature;       /* 1/m, response-aligned feedforward. */
  Real ControlCrossTrackError; /* m, predicted pose vs matched reference. */
  Real ControlHeadingError;    /* rad, predicted pose vs matched reference. */
  Real ResponseDistance;       /* m, clamped active-segment advance. */
} ApaMocoPathPointTransferV2Output_t;

void apaMocoPathPointTransferV2(const ApaMocoPathPointTransferV2Input_t *input,
                                const ApaMocoPathPointTransferV2Param_t *param,
                                ApaMocoPathPointTransferV2State_t *state,
                                ApaMocoPathPointTransferV2Output_t *output);

#endif
