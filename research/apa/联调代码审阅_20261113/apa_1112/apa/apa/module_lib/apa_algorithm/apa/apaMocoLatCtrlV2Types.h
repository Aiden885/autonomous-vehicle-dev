#ifndef APA_MOCO_LAT_CTRL_V2_TYPES_H
#define APA_MOCO_LAT_CTRL_V2_TYPES_H

#include "baseTypes.h"

/**
 * @brief MoCo v2横向控制输入
 * @en_name ApaMocoLatCtrlV2Input
 * @cn_name MoCo v2横向控制输入
 * @tag STRUCT
 * @field name=RefCurvature, type=Real, unit=1/m, desc=参考曲率
 * @field name=TargetSpeed, type=Real, unit=m/s, desc=纵向目标速度
 * @field name=Dist2Go, type=Real, unit=m, desc=当前段有符号剩余里程
 * @field name=LongitudinalError, type=Real, unit=m, desc=纵向误差
 * @field name=LateralError, type=Real, unit=m, desc=横向误差
 * @field name=FeedbackDistance, type=Real, unit=m, desc=独立于段末取点的反馈调度距离
 * @field name=CrossTrackError, type=Real, unit=m, desc=当前最近点横向跟踪误差
 * @field name=HeadingError, type=Real, unit=rad, desc=航向误差
 * @field name=GoalDistance, type=Real, unit=m, desc=终点距离
 * @field name=TerminalYawError, type=Real, unit=rad, desc=终点航向误差
 * @field name=CtrlPhase, type=Integer, unit=1, desc=控制阶段
 * @field name=ApaGoDir, type=Integer, unit=1, desc=当前段方向
 * @field name=TerminalMode, type=Integer, unit=1, desc=终端模式
 * @field name=ParkingCate, type=Integer, unit=1, desc=泊车类型
 * @field name=PathPointNum, type=Integer, unit=1, desc=局部路径点数量
 * @field name=VehicleSpeed, type=Real, unit=m/s, desc=实测有符号车速
 * @field name=VehicleLateralSpeed, type=Real, unit=m/s, desc=实测后轴横向速度(车体+Y向右为正)
 * @field name=CycleTime, type=Real, unit=s, desc=有效仿真控制周期
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Real RefCurvature;
  Real TargetSpeed;
  Real Dist2Go;
  Real LongitudinalError;
  Real LateralError;
  Real FeedbackDistance;
  Real CrossTrackError;
  Real HeadingError;
  Real GoalDistance;
  Real TerminalYawError;
  Integer CtrlPhase;
  Integer ApaGoDir;
  Integer TerminalMode;
  Integer ParkingCate;
  Integer PathPointNum;
  Real VehicleSpeed;
  Real VehicleLateralSpeed;
  Real CycleTime;
} ApaMocoLatCtrlV2Input_t;

/**
 * @brief MoCo v2横向控制参数
 * @en_name ApaMocoLatCtrlV2Param
 * @cn_name MoCo v2横向控制参数
 * @tag STRUCT
 * @field name=Wheelbase, type=Real, unit=m, desc=轴距
 * @field name=Kpp, type=Real, unit=1, desc=空间横向误差反馈增益
 * @field name=SteeringRatio, type=Real, unit=1, desc=保留的转向传动比标定
 * @field name=SteerGapMax, type=Real, unit=rad, desc=单步间隙限幅
 * @field name=SteerRateLimit, type=Real, unit=rad, desc=转角速率限制
 * @field name=SteerUpperLimit, type=Real, unit=rad, desc=前轮转角上限
 * @field name=SteerLowerLimit, type=Real, unit=rad, desc=前轮转角下限
 * @field name=TerminalYawTol, type=Real, unit=rad, desc=末段航向捕获阈值
 * @field name=AllowEyThreshold, type=Real, unit=m, desc=横向误差允许阈值
 * @field name=StaticSpeedThd, type=Real, unit=m/s, desc=近静止转角保持速度阈值
 * @field name=ShortPathPointCount, type=Integer, unit=1, desc=不参与转角回正判定
 * @field name=IntegralGain, type=Real, unit=1/m3, desc=按实际行驶距离累计的横偏积分增益
 * @field name=IntegralCurvatureLimit, type=Real, unit=1/m, desc=积分曲率补偿绝对上限
 * @field name=HeadingDampingRatio, type=Real, unit=1, desc=空间航向阻尼倍率，1保留原阻尼
 * @field name=HeadingDeadband, type=Real, unit=rad, desc=航向反馈死区
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Real Wheelbase;
  Real Kpp;
  Real SteeringRatio;
  Real SteerGapMax;
  Real SteerRateLimit;
  Real SteerUpperLimit;
  Real SteerLowerLimit;
  Real TerminalYawTol;
  Real AllowEyThreshold;
  Real StaticSpeedThd;
  Integer ShortPathPointCount;
  Real IntegralGain;
  Real IntegralCurvatureLimit;
  Real HeadingDampingRatio;
  Real HeadingDeadband;
} ApaMocoLatCtrlV2Param_t;

/**
 * @brief MoCo v2横向控制状态
 * @en_name ApaMocoLatCtrlV2State
 * @cn_name MoCo v2横向控制状态
 * @tag STRUCT
 * @field name=LastSteerAngle, type=Real, unit=rad, desc=上一周期前轮转角
 * @field name=StrZeroFlagLast, type=Integer, unit=1, desc=上一周期完成或故障回正标志
 * @field name=IntegralCurvature, type=Real, unit=1/m, desc=有界积分曲率补偿
 * @field name=LastGoDir, type=Integer, unit=1, desc=上一周期行驶方向
 * @field name=LastRefCurvature, type=Real, unit=1/m, desc=上一周期参考曲率
 * @field name=SteerTarget, type=Real, unit=rad, desc=上一周期限幅后的目标前轮转角
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Real LastSteerAngle;
  Integer StrZeroFlagLast;
  Real IntegralCurvature;
  Integer LastGoDir;
  Real LastRefCurvature;
  Real SteerTarget;
} ApaMocoLatCtrlV2State_t;

/**
 * @brief MoCo v2横向控制输出
 * @en_name ApaMocoLatCtrlV2Output
 * @cn_name MoCo v2横向控制输出
 * @tag STRUCT
 * @field name=SteerAngle, type=Real, unit=rad, desc=前轮转角
 * @field name=StrPP, type=Real, unit=rad, desc=横向/纯跟踪转角项
 * @field name=StrH, type=Real, unit=rad, desc=航向转角项
 * @field name=DisErr, type=Real, unit=m, desc=横向误差调试量
 * @field name=HeadErr, type=Real, unit=rad, desc=航向误差调试量
 * @field name=DecelerationReq, type=Integer, unit=1, desc=横向控制降速请求
 * @field name=SteerSaturated, type=Integer, unit=1, desc=转角限幅标志
 * @field name=AllowEy, type=Integer, unit=1, desc=横向误差允许门控标志
 * @field name=StrZeroFlag, type=Integer, unit=1, desc=完成或故障相位转角归零标志
 * @field name=IntegralCurvature, type=Real, unit=1/m, desc=本周期采用的积分曲率补偿
 * @version 1.0
 * @date 2026-05-18
 * @author aide
 */
typedef struct {
  Real SteerAngle;
  Real StrPP;
  Real StrH;
  Real DisErr;
  Real HeadErr;
  Integer DecelerationReq;
  Integer SteerSaturated;
  Integer AllowEy;
  Integer StrZeroFlag;
  Real IntegralCurvature;
} ApaMocoLatCtrlV2Output_t;

void apaMocoLatCtrlV2(const ApaMocoLatCtrlV2Input_t *input,
                      const ApaMocoLatCtrlV2Param_t *param,
                      ApaMocoLatCtrlV2State_t *state,
                      ApaMocoLatCtrlV2Output_t *output);

#endif
