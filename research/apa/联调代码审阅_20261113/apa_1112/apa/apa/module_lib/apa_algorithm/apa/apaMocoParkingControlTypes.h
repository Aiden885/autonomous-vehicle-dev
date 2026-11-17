#ifndef APA_MOCO_PARKING_CONTROL_TYPES_H
#define APA_MOCO_PARKING_CONTROL_TYPES_H
#include "apaMocoCtrlChartV2Types.h"
#include "apaMocoPathPointTransferV2Types.h"
#include "apaMocoLatCtrlV2Types.h"
#include "apaMocoLongiCtrlV2Types.h"

/**
 * @brief Measured vehicle feedback and an unchanged public path.
 * @en_name ApaMocoParkingControlInput
 * @cn_name 泊车执行控制输入
 * @tag STRUCT
 * @field name=Chart, type=ApaMocoCtrlChartV2Input_t, unit=1, desc=后轴位姿实际档位与原路径
 * @field name=VehicleSpeed, type=Real, unit=m/s, desc=实测有符号速度
 * @field name=VehicleLateralSpeed, type=Real, unit=m/s, desc=实测后轴横向速度(车体+Y向右为正)
 * @field name=VehicleAcceleration, type=Real, unit=m/s2, desc=实测有符号加速度
 * @field name=AvailableDistance, type=Real, unit=m, desc=场景给出的当前段可运动距离
 * @field name=AvailableDistanceLimited, type=Boolean, unit=1, desc=是否存在障碍距离约束
 */
typedef struct {
  ApaMocoCtrlChartV2Input_t Chart;
  Real VehicleSpeed;
  Real VehicleLateralSpeed; /* m/s, body +Y (right) at rear axle; 0 when unmeasured. */
  Real VehicleAcceleration;
  Real AvailableDistance; /* m, scene-provided free motion before an obstacle. */
  Boolean AvailableDistanceLimited;
} ApaMocoParkingControlInput_t;

/**
 * @brief Explicit shared calibration for candidate validation and execution.
 * @en_name ApaMocoParkingControlParam
 * @cn_name 泊车执行控制参数
 * @tag STRUCT
 * @field name=Chart, type=ApaMocoCtrlChartV2Param_t, unit=1, desc=到点位姿容差档位和停稳等待参数
 * @field name=Path, type=ApaMocoPathPointTransferV2Param_t, unit=1, desc=原参考点标定
 * @field name=Lat, type=ApaMocoLatCtrlV2Param_t, unit=1, desc=原横向标定
 * @field name=Longi, type=ApaMocoLongiCtrlV2Param_t, unit=1, desc=原加速度闭环标定
 * @field name=ReactionTime, type=Real, unit=s, desc=停车请求响应预算
 * @field name=StopBuffer, type=Real, unit=m, desc=制动距离之外的位置余量
 * @field name=CurveRefSpeed, type=Real, unit=m/s, desc=最大转向曲率处的速度上限
 * @field name=CurvaturePreviewDistance, type=Real, unit=m, desc=当前段弯道限速预看距离
 * @field name=SteeringResponseTime, type=Real, unit=s, desc=名义一阶前轮转向响应时间
 */
typedef struct {
  ApaMocoCtrlChartV2Param_t Chart;
  ApaMocoPathPointTransferV2Param_t Path;
  ApaMocoLatCtrlV2Param_t Lat;
  ApaMocoLongiCtrlV2Param_t Longi;
  Real ReactionTime;
  Real StopBuffer;
  Real CurveRefSpeed;            /* m/s at maximum steering curvature. */
  Real CurvaturePreviewDistance; /* m, active-segment speed constraint horizon. */
  Real SteeringResponseTime;     /* s, nominal first-order road-wheel response. */
} ApaMocoParkingControlParam_t;

/**
 * @brief 泊车执行控制状态。
 * @en_name ApaMocoParkingControlState
 * @cn_name 泊车执行控制状态
 * @tag STRUCT
 * @field name=Chart, type=ApaMocoCtrlChartV2State_t, unit=1, desc=原段表与状态
 * @field name=Path, type=ApaMocoPathPointTransferV2State_t, unit=1, desc=参考点状态
 * @field name=Lat, type=ApaMocoLatCtrlV2State_t, unit=1, desc=横向状态
 * @field name=Longi, type=ApaMocoLongiCtrlV2State_t, unit=1, desc=纵向状态
 * @field name=EstimatedSteerAngle, type=Real, unit=rad, desc=共享响应模型的前轮转角估计
 */
typedef struct {
  ApaMocoCtrlChartV2State_t Chart;
  ApaMocoPathPointTransferV2State_t Path;
  ApaMocoLatCtrlV2State_t Lat;
  ApaMocoLongiCtrlV2State_t Longi;
  Real EstimatedSteerAngle; /* rad, nominal road-wheel state. */
} ApaMocoParkingControlState_t;

/**
 * @brief Existing complete outputs plus explicit stop-policy diagnostics.
 * @en_name ApaMocoParkingControlOutput
 * @cn_name 泊车执行控制输出
 * @tag STRUCT
 * @field name=Chart, type=ApaMocoCtrlChartV2Output_t, unit=1, desc=原状态与局部路径
 * @field name=Path, type=ApaMocoPathPointTransferV2Output_t, unit=1, desc=原路径参考
 * @field name=Lat, type=ApaMocoLatCtrlV2Output_t, unit=1, desc=转角命令
 * @field name=Longi, type=ApaMocoLongiCtrlV2Output_t, unit=1, desc=加速度命令与静止换挡守卫
 * @field name=StopDistance, type=Real, unit=m, desc=本周期提前停车距离
 * @field name=HoldingForStandstill, type=Boolean, unit=1, desc=车辆仍运动时禁止累计换向等待
 * @field name=CurveSpeedLimit, type=Real, unit=m/s, desc=弯道绝对速度上限
 * @field name=EndpointSpeedLimit, type=Real, unit=m/s, desc=段末制动包络速度上限
 * @field name=ObstacleSpeedLimit, type=Real, unit=m/s, desc=障碍可运动距离速度上限
 * @field name=AppliedSpeedLimit, type=Real, unit=m/s, desc=本周期物理约束合成上限
 */
typedef struct {
  ApaMocoCtrlChartV2Output_t Chart;
  ApaMocoPathPointTransferV2Output_t Path;
  ApaMocoLatCtrlV2Output_t Lat;
  ApaMocoLongiCtrlV2Output_t Longi;
  Real StopDistance;
  Boolean HoldingForStandstill;
  Real CurveSpeedLimit;
  Real EndpointSpeedLimit;
  Real ObstacleSpeedLimit;
  Real AppliedSpeedLimit;
} ApaMocoParkingControlOutput_t;
#endif
