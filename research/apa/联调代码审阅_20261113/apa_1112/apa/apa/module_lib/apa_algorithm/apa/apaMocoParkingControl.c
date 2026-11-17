#include "apaMocoParkingControl.h"
#include "apaVehicleDefaults.h"
#include "apaEnums.h"
#include "apaGoalTolerance.h"
#include "apaMocoBraking.h"
#include "apaMocoMotionResponse.h"
#include <math.h>

static Real apaMocoParkingControlStopDistance(const ApaMocoParkingControlInput_t *input,
                                              const ApaMocoParkingControlParam_t *param,
                                              Real lastAcceleration) {
  return apaMocoParkingBrakingProfile(
             input->VehicleSpeed, input->VehicleAcceleration, lastAcceleration, param)
      .Distance;
}

Real apaMocoParkingSpeedForDistance(const ApaMocoParkingControlInput_t *input,
                                    const ApaMocoParkingControlParam_t *param,
                                    Real lastAcceleration,
                                    Real distance,
                                    Integer direction) {

  Real low = 0.0f;
  Real high = direction < 0 ? -param->Longi.SpeedLowerLimit : param->Longi.SpeedUpperLimit;
  if (!(param->Longi.KpSpeed > 0.0f))
    return 0.0f;
  const Real responseTime = apaMocoParkingSpeedResponseTime(param);
  ApaMocoParkingControlInput_t trial = *input;
  for (Integer i = 0; i < 12; ++i) {
    const Real speed = (low + high) / 2.0f;
    trial.VehicleSpeed = direction * speed;
    if (speed * responseTime + apaMocoParkingControlStopDistance(&trial, param, lastAcceleration) <=
        distance)
      low = speed;
    else
      high = speed;
  }
  return low;
}

static void apaMocoParkingSpeedEnvelope(const ApaMocoParkingControlInput_t *input,
                                        const ApaMocoParkingControlParam_t *param,
                                        const ApaMocoParkingControlState_t *state,
                                        ApaMocoParkingControlOutput_t *output) {
  const Integer direction = output->Chart.ApaGoDir < 0 || (output->Chart.ApaGoDir == 0 &&
                                                           input->Chart.GearPos == APA_GEAR_POS_R)
                                ? -1
                                : 1;
  const Real maximum = direction < 0 ? -param->Longi.SpeedLowerLimit : param->Longi.SpeedUpperLimit;
  output->CurveSpeedLimit = output->EndpointSpeedLimit = output->ObstacleSpeedLimit = maximum;
  if (input->AvailableDistanceLimited)
    output->ObstacleSpeedLimit = apaMocoParkingSpeedForDistance(
        input, param, state->Longi.LastAcceleration, input->AvailableDistance, direction);
  if (apaMocoMotionPhase(output->Chart.CtrlPhase)) {
    Real peak = fabsf(tanf(state->Lat.LastSteerAngle) / param->Lat.Wheelbase);
    Real pathMinCurvature = 0.0f;
    Real pathMaxCurvature = 0.0f;
    Boolean hasPathCurvature = false;
    const Real reversalPreview = fmaxf(param->CurvaturePreviewDistance, 0.65f);
    Real distance = 0.0f;
    const Integer count = output->Chart.LocalPtCount;
    for (Integer i = 0; i + 1 < count; ++i) {
      const Real length = hypotf(output->Chart.LocalPathX[i + 1] - output->Chart.LocalPathX[i],
                                 output->Chart.LocalPathY[i + 1] - output->Chart.LocalPathY[i]);
      if (distance + length >= output->Chart.Projection.Progress &&
          distance <= output->Chart.Projection.Progress + reversalPreview &&
          fabsf(output->Chart.LocalPathOdo[i]) > 1.e-5f) {
        const Real yaw =
            atan2f(sinf(output->Chart.LocalPathYaw[i + 1] - output->Chart.LocalPathYaw[i]),
                   cosf(output->Chart.LocalPathYaw[i + 1] - output->Chart.LocalPathYaw[i]));
        const Real curvature = yaw / output->Chart.LocalPathOdo[i];
        if (distance <= output->Chart.Projection.Progress + param->CurvaturePreviewDistance) {
          peak = fmaxf(peak, fabsf(curvature));
        }
        if (!hasPathCurvature) {
          pathMinCurvature = curvature;
          pathMaxCurvature = curvature;
          hasPathCurvature = true;
        } else {
          pathMinCurvature = fminf(pathMinCurvature, curvature);
          pathMaxCurvature = fmaxf(pathMaxCurvature, curvature);
        }
      }
      distance += length;
    }
    Real span = 0.0f;

    if (hasPathCurvature && param->CurvaturePreviewDistance > 0.0f) {
      Real minCurvature = pathMinCurvature;
      Real maxCurvature = pathMaxCurvature;
      if (fabsf(input->VehicleSpeed) > param->Chart.StandstillSpeed &&
          isfinite(state->EstimatedSteerAngle)) {
        const Real wheelCurvature = tanf(state->EstimatedSteerAngle) / param->Lat.Wheelbase;
        minCurvature = fminf(minCurvature, wheelCurvature);
        maxCurvature = fmaxf(maxCurvature, wheelCurvature);
      }
      if (minCurvature * maxCurvature < -1.e-6f) {
        span = maxCurvature - minCurvature;
      }
    }
    const Real effectiveCurvature = peak + 1.5f * span;
    const Real maximumCurvature =
        fmaxf(fabsf(tanf(param->Lat.SteerUpperLimit)), fabsf(tanf(param->Lat.SteerLowerLimit))) /
        param->Lat.Wheelbase;
    if (effectiveCurvature > 1.e-5f)
      output->CurveSpeedLimit =
          fminf(maximum, param->CurveRefSpeed * sqrtf(maximumCurvature / effectiveCurvature));

    const Real responseDistance =
        param->Longi.StandstillSpeedThreshold * apaMocoParkingSpeedResponseTime(param);
    output->EndpointSpeedLimit =
        apaMocoParkingSpeedForDistance(input,
                                       param,
                                       state->Longi.LastAcceleration,
                                       output->Chart.Dist2Go + responseDistance,
                                       direction);
  }
  output->AppliedSpeedLimit =
      fminf(output->CurveSpeedLimit, fminf(output->EndpointSpeedLimit, output->ObstacleSpeedLimit));
}

void apaMocoParkingControlDefaultParam(ApaMocoParkingControlParam_t *param) {
  *param = (ApaMocoParkingControlParam_t){0};
  param->Chart.DistOverStp = 0.0f;
  param->Chart.PointReachedDistSq = 0.05f * 0.05f;
  param->Chart.StopDwellCycles = 10;
  param->Chart.GearWaitCycles = 10;
  param->Chart.KPathMinStp = 0.10f;
  param->Chart.TerminalYawTol = 0.035f;

  param->Chart.CuspPositionTolerance = 0.20f;
  param->Chart.CuspYawTolerance = 0.10471975512f;
  param->Chart.GoalPositionTolerance = APA_GOAL_POSITION_TOLERANCE_M;
  param->Chart.GoalYawTolerance = APA_GOAL_YAW_TOLERANCE_RAD;
  param->Chart.StandstillSpeed = 0.02f;
  param->Chart.GoalDwellCycles = 25;
  param->Chart.GearTimeoutCycles = 150;
  param->Chart.StallTimeoutCycles = 500;
  param->Chart.SegmentTimeoutCycles = 15000;

  param->Path.LookAheadSteps = 1;
  param->Path.SearchBackSteps = 2;
  param->Path.SearchAheadSteps = 24;

  param->Path.PreviewTime = 1.0f / 3.0f;
  param->Path.PreviewDistMin = 0.60f;
  param->Path.PreviewDistMax = 1.00f;
  param->Path.TerminalPreviewDist = 0.18f;
  param->Path.MaxReferenceDistance = 0.50f;
  param->Path.MaxHeadingError = 0.60f;
  param->Path.MaxProgressAdvance = 0.30f;
  param->Chart.SearchBackSteps = param->Path.SearchBackSteps;
  param->Chart.SearchAheadSteps = param->Path.SearchAheadSteps;
  param->Chart.MaxProgressAdvance = param->Path.MaxProgressAdvance;
  param->Chart.MaxReferenceDistance = param->Path.MaxReferenceDistance;
  param->Chart.MaxHeadingError = param->Path.MaxHeadingError;

  param->Lat.Wheelbase = APA_VEHICLE_WHEELBASE_M;
  param->Lat.Kpp = 0.9f;
  param->Lat.SteeringRatio = 20.0f;
  param->Lat.SteerGapMax = 1.6f;
  param->Lat.SteerRateLimit = 0.0f;
  param->Lat.SteerUpperLimit = APA_VEHICLE_MAX_STEER_RAD;
  param->Lat.SteerLowerLimit = -APA_VEHICLE_MAX_STEER_RAD;
  param->Lat.TerminalYawTol = 0.035f;
  param->Lat.AllowEyThreshold = 0.002f;
  param->Lat.StaticSpeedThd = 0.03f;
  param->Lat.ShortPathPointCount = 3;
  param->Lat.IntegralGain = 0.25f;
  param->Lat.IntegralCurvatureLimit = 0.06f;
  param->Lat.HeadingDampingRatio = 1.0f;
  param->Lat.HeadingDeadband = 0.0008726646f;

  param->Longi.ForwardRefSpeed = 1.10f;
  param->Longi.ReverseRefSpeed = -0.60f;
  param->Longi.TerminalKeepSpeed = 0.08f;
  param->Longi.ParallelTerminalKeepSpeed = 0.08f;
  param->Longi.CuspApproachSpeed = 0.15f;
  param->Longi.ShortSegmentLength = 1.0f;
  param->Longi.ShortSegmentSpeed = 0.30f;
  param->Longi.TerminalDistThd = 0.18f;
  param->Longi.ParallelTerminalDistThd = 0.12f;
  param->Longi.TerminalYawTol = 0.035f;
  param->Longi.StopDistThd = 0.04f;
  param->Longi.SlowDistThd = 1.5f;
  param->Longi.VerticalMinTrackingSpeed = 0.08f;
  param->Longi.DiagonalMinTrackingSpeed = 0.08f;
  param->Longi.HeadInMinTrackingSpeed = 0.07f;
  param->Longi.ParallelMinTrackingSpeed = 0.08f;
  param->Longi.SpeedUpperLimit = 2.0f;
  param->Longi.SpeedLowerLimit = -2.0f;

  param->Longi.CycleTime = 0.02f;
  param->Longi.KpSpeed = 1.5f;

  param->Longi.KiSpeed = 0.0f;
  param->Longi.AccelerationFeedbackGain = 0.0f;
  param->Longi.IntegralUpperLimit = 3.0f;
  param->Longi.IntegralLowerLimit = -3.0f;
  param->Longi.AccelerationUpperLimit = 1.2f;
  param->Longi.AccelerationLowerLimit = -1.5f;
  param->Longi.JerkUpperLimit = 3.0f;
  param->Longi.JerkLowerLimit = -4.0f;
  param->Longi.StopDeceleration = 1.2f;
  param->Longi.StandstillSpeedThreshold = 0.02f;
  param->Longi.TrackingErrorSoft = 0.02f;
  param->Longi.TrackingErrorHard = 0.08f;
  param->Longi.TrackingMinSpeed = 0.30f;
  param->Longi.TrackingCornerRadius =
      hypotf(APA_VEHICLE_REAR_TO_FRONT_M, 0.5f * APA_VEHICLE_WIDTH_M);

  param->ReactionTime = 0.12f;
  param->StopBuffer = 0.005f;
  param->CurveRefSpeed = 0.40f;
  param->CurvaturePreviewDistance = 0.35f;
  param->SteeringResponseTime = 0.12f;
}

static void apaMocoParkingControlPathInput(const ApaMocoParkingControlInput_t *input,
                                           const ApaMocoCtrlChartV2Output_t *chart,
                                           const ApaMocoParkingControlParam_t *param,
                                           Real steeringAngle,
                                           ApaMocoPathPointTransferV2Input_t *path) {
  Integer index;
  *path = (ApaMocoPathPointTransferV2Input_t){0};
  path->VehicleX = input->Chart.VehicleX;
  path->VehicleY = input->Chart.VehicleY;
  path->VehicleYaw = input->Chart.VehicleYaw;
  path->VehicleSpeed = input->VehicleSpeed;
  path->VehicleLateralSpeed = input->VehicleLateralSpeed;
  path->SteeringAngle = steeringAngle;
  path->Wheelbase = param->Lat.Wheelbase;
  path->ResponseTime = param->SteeringResponseTime;
  path->CtrlPhase = chart->CtrlPhase;
  path->ApaGoDir = chart->ApaGoDir;
  path->SegmentStartIdx = chart->SegmentStartIdx;
  path->SegmentEndIdx = chart->SegmentEndIdx;
  path->TerminalMode = chart->TerminalMode;
  path->ParkingCate = chart->ParkingCate;
  path->UseProjection = true;
  path->Projection = chart->Projection;
  path->LocalPtCount = chart->LocalPtCount;
  index = 0;
  while (index < chart->LocalPtCount) {
    path->LocalPathX[index] = chart->LocalPathX[index];
    path->LocalPathY[index] = chart->LocalPathY[index];
    path->LocalPathYaw[index] = chart->LocalPathYaw[index];
    path->LocalPathOdo[index] = chart->LocalPathOdo[index];
    index = index + 1;
  }
}

static void apaMocoParkingControlLongiInput(const ApaMocoParkingControlInput_t *input,
                                            const ApaMocoParkingControlOutput_t *output,
                                            ApaMocoLongiCtrlV2Input_t *longi) {
  *longi = (ApaMocoLongiCtrlV2Input_t){0};
  longi->CtrlPhase = output->Chart.CtrlPhase;
  longi->ApaStpNow = output->Chart.ApaStpNow;
  longi->ApaStpNum = output->Chart.ApaStpNum;
  longi->ApaGoDir = output->Chart.ApaGoDir;
  longi->Dist2Go = output->Chart.Dist2Go;
  longi->BrakeDistance = output->StopDistance;
  longi->SegmentLength = output->Chart.Projection.Length;
  longi->LongitudinalError = output->Path.LongitudinalError;
  longi->GoalDistance = output->Path.GoalDistance;
  longi->TerminalYawError = output->Path.TerminalYawError;
  longi->HeadingError = output->Path.HeadingError;
  longi->LateralError = output->Path.LateralError;
  longi->CrossTrackError = output->Path.CrossTrackError;
  longi->TerminalMode = output->Chart.TerminalMode;
  longi->ParkingCate = output->Chart.ParkingCate;
  longi->DecelerationReq = 0;
  longi->VehicleSpeed = input->VehicleSpeed;
  longi->VehicleAcceleration = input->VehicleAcceleration;
  longi->TargetGearReq = output->Chart.TargetGearReq;
  longi->GearPos = input->Chart.GearPos;
}

static void apaMocoParkingControlLatInput(const ApaMocoParkingControlInput_t *input,
                                          const ApaMocoParkingControlParam_t *param,
                                          const ApaMocoParkingControlOutput_t *output,
                                          ApaMocoLatCtrlV2Input_t *lat) {
  *lat = (ApaMocoLatCtrlV2Input_t){0};
  lat->RefCurvature = output->Path.ControlCurvature;
  lat->TargetSpeed = output->Longi.TargetSpeed;
  lat->Dist2Go = output->Chart.Dist2Go;
  lat->LongitudinalError = output->Path.LongitudinalError;
  lat->LateralError = output->Path.LateralError;
  lat->FeedbackDistance = output->Path.FeedbackDistance;
  lat->CrossTrackError = output->Path.ControlCrossTrackError;
  lat->HeadingError = output->Path.ControlHeadingError;
  lat->GoalDistance = output->Path.GoalDistance;
  lat->TerminalYawError = output->Path.TerminalYawError;
  lat->CtrlPhase = output->Chart.CtrlPhase;
  lat->ApaGoDir = output->Chart.ApaGoDir;
  lat->TerminalMode = output->Chart.TerminalMode;
  lat->ParkingCate = output->Chart.ParkingCate;
  lat->PathPointNum = output->Path.PathPointNum;
  lat->VehicleSpeed = input->VehicleSpeed;
  lat->VehicleLateralSpeed = input->VehicleLateralSpeed;
  lat->CycleTime = param->Longi.CycleTime;
}

/**
 * @brief Execute the public MoCo chain with braking-before-cusp and standstill dwell.
 * @en_name apaMocoParkingControl
 * @cn_name 泊车安全分段控制
 * @type widget_module
 * @config cn_name=停车响应预算, en_name=ReactionTime, desc=制动命令及执行反馈响应时间, unit=s, type=Real, range=[0,1], default=0.12
 * @config cn_name=停车位置余量, en_name=StopBuffer, desc=制动距离额外留空, unit=m, type=Real, range=[0,1], default=0.005
 * @config cn_name=弯道参考速度, en_name=CurveRefSpeed, desc=最大转向曲率处的绝对速度上限, unit=m/s, type=Real, range=[0.02,2], default=0.40
 * @config cn_name=弯道预看距离, en_name=CurvaturePreviewDistance, desc=仅在当前方向段内预看曲率峰值, unit=m, type=Real, range=[0,1], default=0.35
 * @config cn_name=转向响应时间, en_name=SteeringResponseTime, desc=控制预瞄与执行预测共用名义一阶时间常数, unit=s, type=Real, range=[0,1], default=0.12
 * @param[IN] ApaMocoCtrlChartV2Input_t Chart 实测位姿实际档位及完整原路径
 * @param[IN] Real VehicleSpeed 实测有符号速度
 * @param[IN] Real VehicleLateralSpeed 实测后轴横向速度(左正)
 * @param[IN] Real VehicleAcceleration 实测有符号加速度
 * @param[IN] Real AvailableDistance 场景可运动距离
 * @param[IN] Boolean AvailableDistanceLimited 障碍距离约束有效
 * @param[OUT] ApaMocoCtrlChartV2Output_t Chart 活动段输出
 * @param[OUT] ApaMocoPathPointTransferV2Output_t Path 跟踪参考
 * @param[OUT] ApaMocoLatCtrlV2Output_t Lat 转角输出
 * @param[OUT] ApaMocoLongiCtrlV2Output_t Longi 加速度与换挡许可
 * @param[OUT] Real CurveSpeedLimit 弯道绝对速度上限
 * @param[OUT] Real EndpointSpeedLimit 段末包络速度上限
 * @param[OUT] Real ObstacleSpeedLimit 障碍距离速度上限
 * @param[OUT] Real AppliedSpeedLimit 本周期物理约束上限
 * @param[OUT] Real StopDistance 本周期提前制动距离
 * @param[OUT] Boolean HoldingForStandstill 静止等待守卫
 * @var ApaMocoCtrlChartV2State_t Chart 原状态机状态
 * @var ApaMocoPathPointTransferV2State_t Path 参考状态
 * @var ApaMocoLatCtrlV2State_t Lat 横向状态
 * @var ApaMocoLongiCtrlV2State_t Longi 纵向状态
 * @var Real EstimatedSteerAngle 名义前轮转角估计
 * @granularity composite
 * @tag_level1 泊车控制
 * @tag_level2 安全执行
 * @source README.md
 * @version 1.0
 * @date 2026-10-01
 * @author aide
 */
void apaMocoParkingControl(const ApaMocoParkingControlInput_t *input,
                           const ApaMocoParkingControlParam_t *param,
                           ApaMocoParkingControlState_t *state,
                           ApaMocoParkingControlOutput_t *output) {
  ApaMocoCtrlChartV2Param_t chartParam;
  ApaMocoPathPointTransferV2Input_t pathInput;
  ApaMocoLongiCtrlV2Input_t longiInput;
  ApaMocoLongiCtrlV2Param_t longiParam;
  ApaMocoLatCtrlV2Param_t latParam;
  ApaMocoLatCtrlV2Input_t latInput;
  ApaMocoCtrlChartV2Input_t chartInput;
  *output = (ApaMocoParkingControlOutput_t){0};
  state->EstimatedSteerAngle = apaMocoSteeringResponse(state->EstimatedSteerAngle,
                                                       state->Lat.LastSteerAngle,
                                                       param->SteeringResponseTime,
                                                       param->Longi.CycleTime);
  chartParam = param->Chart;
  output->StopDistance =
      apaMocoParkingControlStopDistance(input, param, state->Longi.LastAcceleration);
  chartInput = input->Chart;
  const Real steeringTolerance = 0.034906585f;
  const Real target = state->Lat.SteerTarget;
  chartInput.SteeringPending = !isfinite(target) || !isfinite(state->EstimatedSteerAngle) ||
                               !isfinite(state->Lat.LastSteerAngle) ||
                               fabsf(state->EstimatedSteerAngle - target) > steeringTolerance ||
                               fabsf(state->Lat.LastSteerAngle - target) > steeringTolerance;
  chartInput.VehicleSpeed = isfinite(input->VehicleAcceleration) ? input->VehicleSpeed : NAN;
  chartInput.BrakeDistance = output->StopDistance;
  chartInput.ApproachDistance = fmaxf(chartInput.ApproachDistance, param->Longi.SlowDistThd);
  chartInput.CreepDistance = input->Chart.TargetSlotKind == 2 ? param->Longi.ParallelTerminalDistThd
                                                              : param->Longi.TerminalDistThd;
  chartParam.SearchBackSteps = param->Path.SearchBackSteps;
  chartParam.SearchAheadSteps = param->Path.SearchAheadSteps;
  chartParam.MaxProgressAdvance = param->Path.MaxProgressAdvance;
  chartParam.MaxReferenceDistance = param->Path.MaxReferenceDistance;
  chartParam.MaxHeadingError = param->Path.MaxHeadingError;
  apaMocoCtrlChartV2(&chartInput, &chartParam, &state->Chart, &output->Chart);
  output->HoldingForStandstill = output->Chart.CtrlPhase == APA_MOCO_STOP_WAIT &&
                                 fabsf(input->VehicleSpeed) > param->Chart.StandstillSpeed;
  apaMocoParkingControlPathInput(
      input, &output->Chart, param, state->EstimatedSteerAngle, &pathInput);
  apaMocoPathPointTransferV2(&pathInput, &param->Path, &state->Path, &output->Path);
  if (apaMocoMotionPhase(output->Chart.CtrlPhase) && !output->Path.ReferenceValid) {
    state->Chart.Phase = APA_MOCO_FAULT;
    state->Chart.StopReason = APA_MOCO_STOP_REFERENCE;
    output->Chart.CtrlPhase = APA_MOCO_FAULT;
    output->Chart.StopReason = APA_MOCO_STOP_REFERENCE;
  }
  apaMocoParkingControlLongiInput(input, output, &longiInput);
  longiParam = param->Longi;
  apaMocoParkingSpeedEnvelope(input, param, state, output);
  longiParam.SpeedUpperLimit = fminf(longiParam.SpeedUpperLimit, output->AppliedSpeedLimit);
  longiParam.SpeedLowerLimit = fmaxf(longiParam.SpeedLowerLimit, -output->AppliedSpeedLimit);
  apaMocoLongiCtrlV2(&longiInput, &longiParam, &state->Longi, &output->Longi);
  apaMocoParkingControlLatInput(input, param, output, &latInput);
  latParam = param->Lat;
  latParam.TerminalYawTol = param->Chart.GoalYawTolerance;
  apaMocoLatCtrlV2(&latInput, &latParam, &state->Lat, &output->Lat);
  if (output->Chart.CtrlPhase == APA_MOCO_FAULT) {
    output->Lat.SteerAngle = 0.0f;
    output->Longi.StopReason = output->Chart.StopReason;
  }
}
