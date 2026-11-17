#ifndef APA_MOCO_LONGI_CTRL_V2_TYPES_H
#define APA_MOCO_LONGI_CTRL_V2_TYPES_H

#include "baseTypes.h"

/**
 * @brief MoCo v2纵向控制输入
 * @en_name ApaMocoLongiCtrlV2Input
 * @cn_name MoCo v2纵向控制输入
 * @tag STRUCT
 * @field name=CtrlPhase, type=Integer, unit=1, desc=控制阶段
 * @field name=ApaStpNow, type=Integer, unit=1, desc=当前段1-based
 * @field name=ApaStpNum, type=Integer, unit=1, desc=总段数
 * @field name=ApaGoDir, type=Integer, unit=1, desc=当前段方向
 * @field name=Dist2Go, type=Real, unit=m, desc=段剩余距离
 * @field name=BrakeDistance, type=Real, unit=m, desc=按实测运动和执行延迟计算的停车距离
 * @field name=SegmentLength, type=Real, unit=m, desc=当前活动段完整弧长
 * @field name=LongitudinalError, type=Real, unit=m, desc=纵向误差
 * @field name=GoalDistance, type=Real, unit=m, desc=终点距离
 * @field name=TerminalYawError, type=Real, unit=rad, desc=终点航向误差
 * @field name=HeadingError, type=Real, unit=rad, desc=参考航向误差
 * @field name=LateralError, type=Real, unit=m, desc=横向误差
 * @field name=CrossTrackError, type=Real, unit=m, desc=投影点横偏，不含预瞄点的名义横向坐标
 * @field name=TerminalMode, type=Integer, unit=1, desc=终端模式
 * @field name=ParkingCate, type=Integer, unit=1, desc=泊车类型
 * @field name=DecelerationReq, type=Integer, unit=1, desc=未使用
 * @field name=VehicleSpeed, type=Real, unit=m/s, desc=车体前向有符号实际速度
 * @field name=VehicleAcceleration, type=Real, unit=m/s2, desc=车体前向有符号实际加速度
 * @field name=TargetGearReq, type=Integer, unit=1, desc=目标档位请求(0无效1D2R3P)
 * @field name=GearPos, type=Integer, unit=1, desc=实际档位(0N或未知1D2R3P)
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Integer CtrlPhase;
  Integer ApaStpNow;
  Integer ApaStpNum;
  Integer ApaGoDir;
  Real Dist2Go;
  Real BrakeDistance;
  Real SegmentLength;
  Real LongitudinalError;
  Real GoalDistance;
  Real TerminalYawError;
  Real HeadingError;
  Real LateralError;
  Real CrossTrackError;
  Integer TerminalMode;
  Integer ParkingCate;
  Integer DecelerationReq;
  Real VehicleSpeed;
  Real VehicleAcceleration;
  Integer TargetGearReq;
  Integer GearPos;
} ApaMocoLongiCtrlV2Input_t;

/**
 * @brief MoCo v2纵向控制参数
 * @en_name ApaMocoLongiCtrlV2Param
 * @cn_name MoCo v2纵向控制参数
 * @tag STRUCT
 * @field name=ForwardRefSpeed, type=Real, unit=m/s, desc=前进参考速度
 * @field name=ReverseRefSpeed, type=Real, unit=m/s, desc=倒车参考速度
 * @field name=TerminalKeepSpeed, type=Real, unit=m/s, desc=非平行终端航向修正保持速度
 * @field name=ParallelTerminalKeepSpeed, type=Real, unit=m/s, desc=平行终端保持速度
 * @field name=CuspApproachSpeed, type=Real, unit=m/s, desc=中间换向点低速接近速度
 * @field name=ShortSegmentLength, type=Real, unit=m, desc=短换向段完整长度上限
 * @field name=ShortSegmentSpeed, type=Real, unit=m/s, desc=短换向段行驶速度上限
 * @field name=TerminalDistThd, type=Real, unit=m, desc=终端航向保持距离阈值
 * @field name=ParallelTerminalDistThd, type=Real, unit=m, desc=平行终端航向保持距离阈值
 * @field name=TerminalYawTol, type=Real, unit=rad, desc=终端航向停车阈值
 * @field name=StopDistThd, type=Real, unit=m, desc=停车距离阈值
 * @field name=SlowDistThd, type=Real, unit=m, desc=每段段末降速距离阈值
 * @field name=VerticalMinTrackingSpeed, type=Real, unit=m/s, desc=垂直车位末段最小跟踪速度
 * @field name=DiagonalMinTrackingSpeed, type=Real, unit=m/s, desc=斜列车位末段最小跟踪速度
 * @field name=HeadInMinTrackingSpeed, type=Real, unit=m/s, desc=头入车位末段最小跟踪速度
 * @field name=ParallelMinTrackingSpeed, type=Real, unit=m/s, desc=平行车位末段最小跟踪速度
 * @field name=SpeedUpperLimit, type=Real, unit=m/s, desc=速度上限
 * @field name=SpeedLowerLimit, type=Real, unit=m/s, desc=速度下限
 * @field name=CycleTime, type=Real, unit=s, desc=加速度闭环控制周期
 * @field name=KpSpeed, type=Real, unit=1/s, desc=速度误差比例增益
 * @field name=KiSpeed, type=Real, unit=1/s2, desc=速度误差积分增益
 * @field name=AccelerationFeedbackGain, type=Real, unit=1, desc=实际加速度反馈阻尼增益
 * @field name=IntegralUpperLimit, type=Real, unit=m, desc=速度误差积分上限
 * @field name=IntegralLowerLimit, type=Real, unit=m, desc=速度误差积分下限
 * @field name=AccelerationUpperLimit, type=Real, unit=m/s2, desc=车体前向加速度上限
 * @field name=AccelerationLowerLimit, type=Real, unit=m/s2, desc=车体前向加速度下限
 * @field name=JerkUpperLimit, type=Real, unit=m/s3, desc=加加速度上限
 * @field name=JerkLowerLimit, type=Real, unit=m/s3, desc=加加速度下限
 * @field name=StopDeceleration, type=Real, unit=m/s2, desc=停车制动减速度幅值
 * @field name=StandstillSpeedThreshold, type=Real, unit=m/s, desc=停车和换挡静止阈值
 * @field name=TrackingErrorSoft, type=Real, unit=m, desc=车身误差开始平滑降速的阈值
 * @field name=TrackingErrorHard, type=Real, unit=m, desc=车身误差降到纠偏巡航速度的阈值
 * @field name=TrackingCornerRadius, type=Real, unit=m, desc=后轴到最远车身角点的半径
 * @field name=TrackingMinSpeed, type=Real, unit=m/s, desc=误差降速的最低巡航速度，段末仍可更低
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Real ForwardRefSpeed;
  Real ReverseRefSpeed;
  Real TerminalKeepSpeed;
  Real ParallelTerminalKeepSpeed;
  Real CuspApproachSpeed;
  Real ShortSegmentLength;
  Real ShortSegmentSpeed;
  Real TerminalDistThd;
  Real ParallelTerminalDistThd;
  Real TerminalYawTol;
  Real StopDistThd;
  Real SlowDistThd;
  Real VerticalMinTrackingSpeed;
  Real DiagonalMinTrackingSpeed;
  Real HeadInMinTrackingSpeed;
  Real ParallelMinTrackingSpeed;
  Real SpeedUpperLimit;
  Real SpeedLowerLimit;
  Real CycleTime;
  Real KpSpeed;
  Real KiSpeed;
  Real AccelerationFeedbackGain;
  Real IntegralUpperLimit;
  Real IntegralLowerLimit;
  Real AccelerationUpperLimit;
  Real AccelerationLowerLimit;
  Real JerkUpperLimit;
  Real JerkLowerLimit;
  Real StopDeceleration;
  Real StandstillSpeedThreshold;
  Real TrackingErrorSoft;
  Real TrackingErrorHard;
  Real TrackingCornerRadius;
  Real TrackingMinSpeed;
} ApaMocoLongiCtrlV2Param_t;

/**
 * @brief MoCo v2纵向控制状态
 * @en_name ApaMocoLongiCtrlV2State
 * @cn_name MoCo v2纵向控制状态
 * @tag STRUCT
 * @field name=LastSpeed, type=Real, unit=m/s, desc=上一周期速度
 * @field name=TerminalLatch, type=Integer, unit=1, desc=最后一段终端捕获锁存
 * @field name=IntegralError, type=Real, unit=m, desc=速度误差积分
 * @field name=LastAcceleration, type=Real, unit=m/s2, desc=上一周期目标加速度
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Real LastSpeed;
  Integer TerminalLatch;
  Real IntegralError;
  Real LastAcceleration;
} ApaMocoLongiCtrlV2State_t;

/**
 * @brief MoCo v2纵向控制输出
 * @en_name ApaMocoLongiCtrlV2Output
 * @cn_name MoCo v2纵向控制输出
 * @tag STRUCT
 * @field name=TargetSpeed, type=Real, unit=m/s, desc=目标速度
 * @field name=Dist2GoCmd, type=Real, unit=m, desc=纵向控制距离命令
 * @field name=LongiBlockFlag, type=Integer, unit=1, desc=纵向阻塞标志
 * @field name=StopReq, type=Integer, unit=1, desc=停车请求
 * @field name=LongiPhase, type=Integer, unit=1, desc=纵向相位(1Cruise,2Decel,3Creep,4Stop,5Blocked)
 * @field name=SpeedScale, type=Real, unit=1, desc=基础速度缩放
 * @field name=TerminalCreepActive, type=Integer, unit=1, desc=终端低速修正激活
 * @field name=StopReason, type=Integer, unit=1, desc=停车原因
 * @field name=TargetAcceleration, type=Real, unit=m/s2, desc=车体前向有符号目标加速度
 * @field name=DirectionSign, type=Integer, unit=1, desc=目标档位方向(+1前进-1倒车0无效)
 * @field name=TargetGearReq, type=Integer, unit=1, desc=目标档位请求
 * @field name=ControlEnabled, type=Integer, unit=1, desc=加速度闭环驱动使能
 * @field name=GearChangeAllowed, type=Integer, unit=1, desc=车辆静止且目标档位有效时允许换挡
 * @field name=AccelerationSaturated, type=Integer, unit=1, desc=加速度或加加速度限幅标志
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Real TargetSpeed;
  Real Dist2GoCmd;
  Integer LongiBlockFlag;
  Integer StopReq;
  Integer LongiPhase;
  Real SpeedScale;
  Integer TerminalCreepActive;
  Integer StopReason;
  Real TargetAcceleration;
  Integer DirectionSign;
  Integer TargetGearReq;
  Integer ControlEnabled;
  Integer GearChangeAllowed;
  Integer AccelerationSaturated;
} ApaMocoLongiCtrlV2Output_t;

void apaMocoLongiCtrlV2(const ApaMocoLongiCtrlV2Input_t *input,
                        const ApaMocoLongiCtrlV2Param_t *param,
                        ApaMocoLongiCtrlV2State_t *state,
                        ApaMocoLongiCtrlV2Output_t *output);

#endif
