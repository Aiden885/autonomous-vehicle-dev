#include "apa_tracking_execution.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <limits>
extern "C" {
#include "apaEnums.h"
#include "apaMocoExecution.h"
#include "apaMocoActuator.h"
#include "apaMocoMotionResponse.h"
#include "apaTrackingPolicy.h"
#include "vehicleDynamicStepTypes.h"
}

namespace pangu::modules {
ApaMotionPredictionPolicy BuildApaMotionPredictionPolicy(Real speed,
    Real acceleration, Real previous_acceleration,
    const ApaMocoParkingControlParam_t& calibration) {
  ApaMotionPredictionPolicy policy;
  policy.braking = apaMocoParkingBrakingProfile(speed,acceleration,
      previous_acceleration,&calibration);
  policy.wheelbase = calibration.Lat.Wheelbase;
  policy.steering_response_time = calibration.SteeringResponseTime;
  policy.minimum_steer = calibration.Lat.SteerLowerLimit;
  policy.maximum_steer = calibration.Lat.SteerUpperLimit;
  const double rate = calibration.Lat.SteerRateLimit;
  const double gap = calibration.Lat.SteerGapMax;
  const double cycle = calibration.Longi.CycleTime;
  if (!std::isfinite(rate) || !std::isfinite(gap) ||
      !std::isfinite(cycle) || cycle<=0) {
    policy.braking.Valid = false;
  } else {
    double per_cycle = std::numeric_limits<double>::infinity();
    if (rate>1.e-6) per_cycle = std::min(per_cycle,rate);
    if (gap>1.e-6) per_cycle = std::min(per_cycle,gap);
    if (std::isfinite(per_cycle)) policy.steer_rate_per_sec=per_cycle/cycle;
  }
  return policy;
}

namespace {

void StepExecutionMotion(const ApaMocoActuatorOutput_t& pedals,
    VehicleDynamicStepInput_t command, const VehicleDynamicStepParam_t& model,
    Real gravity, VehicleDynamicStepState_t* plant, VehicleDynamicStepOutput_t* next) {
  const Real before = plant->VehicleSpeed;
  const Real direction = pedals.Gear == APA_GEAR_POS_R ? -1.0f : 1.0f;
  const Real drive = direction*pedals.Throttle*APA_MOCO_DRIVE_GAIN_MPS2;
  const Real steering = std::tan(command.SteerAngle);
  const Real resistance = APA_MOCO_CREEP_THROTTLE*APA_MOCO_DRIVE_GAIN_MPS2*(1+steering*steering);
  const Real brake = (pedals.HandBrake ? 1.0f : pedals.Brake)*APA_MOCO_BRAKE_GAIN_MPS2;
  const Real force = drive+gravity;
  const Real friction = brake+resistance;
  Real after;
  if (std::fabs(before)>1.e-6f) {
    const Real acceleration=force-std::copysign(friction,before);
    after=before+acceleration*model.CycleTime;
    if (before*after<0) {
      const Real remaining=model.CycleTime+before/acceleration;
      after=std::copysign(std::max(0.0f,std::fabs(force)-friction)*remaining,force);
    }
  } else {
    after=std::copysign(std::max(0.0f,std::fabs(force)-friction)*model.CycleTime,force);
  }
  VehicleDynamicStepState_t motion{};
  motion.VehicleSpeed = (before+after)*0.5f;
  command.AccelerationCmd = 0.0f;
  command.GearPos = motion.VehicleSpeed < 0.0f ? APA_GEAR_POS_R : APA_GEAR_POS_D;

  vehicleDynamicStep(&command,&model,&motion,next);
  plant->VehicleSpeed = next->VehicleSpeed = after;
  plant->VehicleAcceleration = next->VehicleAcceleration = (after-before)/model.CycleTime;
}
}

static ApaMotionPrediction TrackingPrediction(const ApaMotionPrediction* measured,
    const ApaMocoParkingControlState_t& state, const ApaMocoParkingControlOutput_t& control,
    const ApaMocoParkingControlParam_t& calibration) {
  auto prediction = measured ? *measured : ApaMotionPrediction{};
  prediction.commanded_curvature = std::tan(control.Lat.SteerAngle)/calibration.Lat.Wheelbase;
  prediction.response_reference_valid = control.Path.ReferenceValid;
  prediction.reference_curvature = control.Path.ControlCurvature;
  prediction.steering_angle = state.EstimatedSteerAngle;
  return prediction;
}

static ApaTrackingStep StepApaTrackingInScene(const ApaSafetyScene& scene, const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path, int path_update,
    const ApaMocoParkingControlParam_t& calibration,
    ApaMocoParkingControlState_t* state, const ApaClearanceRecovery* recovery,
    const ApaMotionPrediction* motion,
    bool observe_rejection = false) {
  ApaTrackingStep result;
  ApaMocoParkingControlInput_t control{};
  auto& chart = control.Chart;
  chart.VehicleX = input.vehicle_x(); chart.VehicleY = input.vehicle_y(); chart.VehicleYaw = input.vehicle_yaw();
  chart.GoalX = input.target_x(); chart.GoalY = input.target_y(); chart.GoalYaw = input.target_theta();
  chart.GearPos = input.gear_pos(); chart.PathUpdateIdx = path_update;
  chart.TargetSlotKind = input.target_slot_kind(); chart.ParkingAction = input.parking_action();
  chart.PtCount = path.pathPointCount;
  for (int i=0; i<chart.PtCount; ++i) {
    chart.PthPtX[i] = path.pathPointX[i]; chart.PthPtY[i] = path.pathPointY[i];
    chart.PthPtTht[i] = path.pathPointTheta[i]; chart.PthPtOdo[i] = path.pathPointOdometer[i];
  }
  control.VehicleSpeed = input.vehicle_speed(); control.VehicleAcceleration = input.vehicle_acceleration();
  control.VehicleLateralSpeed = (motion && motion->measured && std::isfinite(motion->lateral_speed))
      ? static_cast<Real>(motion->lateral_speed) : 0.0f;
  result.prediction_policy = BuildApaMotionPredictionPolicy(control.VehicleSpeed,
      control.VehicleAcceleration,state->Longi.LastAcceleration,calibration);
  if (std::hypot(input.target_x()-input.vehicle_x(), input.target_y()-input.vehicle_y()) <=
      calibration.Chart.GoalPositionTolerance)
    chart.GoalBlocked = scene.CheckTrackingPose(input,path,state->Chart).blocked();
  auto effective = calibration;
  if (recovery && recovery->info.enabled &&
      ApaPathProgress(path,state->Chart) <= recovery->info.prefix_length+1.e-4) {
    const Real speed = APA_CLEARANCE_RECOVERY_MAX_SPEED_MPS;
    effective.Longi.ForwardRefSpeed = std::min(effective.Longi.ForwardRefSpeed,speed);
    effective.Longi.ReverseRefSpeed = std::max(effective.Longi.ReverseRefSpeed,-speed);
    effective.Longi.SpeedUpperLimit = std::min(effective.Longi.SpeedUpperLimit,speed);
    effective.Longi.SpeedLowerLimit = std::max(effective.Longi.SpeedLowerLimit,-speed);
  }
  auto trial = *state;
  apaMocoParkingControl(&control,&effective,&trial,&result.control);
  ApaAvailableMotion available;
  const bool stalled = result.control.Chart.CtrlPhase == APA_MOCO_FAULT &&
      result.control.Chart.StopReason == APA_MOCO_STOP_STALLED && apaMocoMotionPhase(state->Chart.Phase);
  if (apaMocoMotionPhase(result.control.Chart.CtrlPhase) || stalled) {
    const int direction = stalled ? state->Chart.SegGoDir[state->Chart.StpNow] : result.control.Chart.ApaGoDir;
    const double steer = stalled ? state->Lat.LastSteerAngle : result.control.Lat.SteerAngle;
    const double curvature = std::tan(steer)/effective.Lat.Wheelbase;
    auto forecast = TrackingPrediction(motion,trial,result.control,effective);
    forecast.commanded_curvature = curvature;

    const Real current_speed=std::max(0.0f,direction*control.VehicleSpeed);
    const Real target_speed=std::fabs(result.control.Longi.TargetSpeed);
    const Real acceleration=std::max(0.0f,direction*result.control.Longi.TargetAcceleration);
    const Real reachable=current_speed+acceleration*(effective.ReactionTime+effective.Longi.CycleTime);
    const Real guidance_speed=direction*std::max(current_speed,std::min(target_speed,reachable));
    const Real prospective_acceleration=direction*std::max({0.0f,
        direction*state->Longi.LastAcceleration,direction*result.control.Longi.TargetAcceleration});
    auto guidance_policy = BuildApaMotionPredictionPolicy(guidance_speed,
        control.VehicleAcceleration,prospective_acceleration,effective);
    guidance_policy.speed_response_time=apaMocoParkingSpeedResponseTime(&effective);
    guidance_policy.follow_path_direction=true;
    const double horizon=guidance_policy.braking.Speed*guidance_policy.speed_response_time+
        guidance_policy.braking.Distance;
    available = scene.CheckAvailableMotion(input,path,trial.Chart,
        guidance_policy,forecast,recovery,horizon,
        observe_rejection ? &result.motion_observation : nullptr);
    control.AvailableDistanceLimited = available.obstacle.blocked();
    control.AvailableDistance = available.distance;
  }
  apaMocoParkingControl(&control,&effective,state,&result.control);
  result.prediction = TrackingPrediction(motion,*state,result.control,effective);
  const Real threshold = std::min(calibration.Longi.CuspApproachSpeed,
      calibration.Longi.TerminalKeepSpeed);
  if (stalled && available.obstacle.blocked() && result.control.ObstacleSpeedLimit <= threshold)
    result.motion_limit = available.obstacle;
  return result;
}

ApaTrackingStep StepApaTracking(const ApaSafetyScene& scene, const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path, int path_update,
    const ApaMocoParkingControlParam_t& calibration,
    ApaMocoParkingControlState_t* state, const ApaClearanceRecovery* recovery,
    const ApaMotionPrediction* motion) {
  return StepApaTrackingInScene(scene,input,path,path_update,
      calibration,state,recovery,motion);
}

ApaTrackingStep StepApaTracking(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path, int path_update,
    const ApaMocoParkingControlParam_t& calibration,
    ApaMocoParkingControlState_t* state, const ApaClearanceRecovery* recovery,
    const ApaMotionPrediction* motion) {
  return StepApaTracking(ApaSafetyScene(input),input,path,path_update,
      calibration,state,recovery,motion);
}

bool CheckApaTrackingExecution(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path,
    const ApaMocoParkingControlParam_t& calibration, const ApaClearanceRecovery* recovery) {
  return CheckApaTrackingExecution(ApaSafetyScene(input),input,path,calibration,recovery);
}

bool CheckApaTrackingExecution(const ApaSafetyScene& scene, const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path,
    const ApaMocoParkingControlParam_t& calibration, const ApaClearanceRecovery* recovery) {
  const auto began = std::chrono::steady_clock::now();
  double tracking_seconds=0.0, safety_seconds=0.0;
  auto observed = input;
  ApaMocoParkingControlState_t state{};
  VehicleDynamicStepState_t plant{};
  plant.VehicleSpeed = input.vehicle_speed();
  plant.VehicleAcceleration = input.vehicle_acceleration();
  VehicleDynamicStepParam_t model{};
  model.CycleTime = calibration.Longi.CycleTime;
  model.WheelBase = calibration.Lat.Wheelbase;
  const Real speed_limit = std::max(calibration.Longi.SpeedUpperLimit,
      -calibration.Longi.SpeedLowerLimit);
  model.ForwardSpeedLimit = model.ReverseSpeedLimit = speed_limit +
      APA_MOCO_OVERSPEED_TOLERANCE_MPS + APA_MOCO_DRIVE_GAIN_MPS2*model.CycleTime;
  model.AccelerationUpperLimit = calibration.Longi.AccelerationUpperLimit;
  model.AccelerationLowerLimit = calibration.Longi.AccelerationLowerLimit;
  model.StraightSteerThreshold = 0.0001f;
  ApaClearanceRecovery clearance = recovery ? *recovery : ApaClearanceRecovery{};
  ApaMotionPrediction motion{};
  if (!std::isfinite(calibration.ReactionTime) || calibration.ReactionTime < 0.0f) return false;
  ApaMocoActuatorState_t actuator{};
  Real steering = input.vehicle_steering_command_rad();
  state.EstimatedSteerAngle = steering;
  state.Lat.LastSteerAngle = state.Lat.SteerTarget = steering;
  const int max_steps = calibration.Chart.SegmentTimeoutCycles *
      std::max(1,static_cast<int>(path.directionChangeCount)+1);
  const char* reason = "execution_budget";
  ApaOnlineSafety rejection;
  ApaCandidateSafetyObservation observation;
  bool passed = false;
  int steps = 0;
  for (; steps < max_steps; ++steps) {
    if (clearance.info.enabled && ApaPathProgress(path,state.Chart)<=clearance.info.prefix_length+1.e-4 &&
        (steps*model.CycleTime>APA_CLEARANCE_RECOVERY_MAX_DURATION_SEC ||
         std::fabs(observed.vehicle_speed())>APA_CLEARANCE_RECOVERY_MAX_SPEED_MPS+0.05)) {
      reason="clearance_recovery_execution_limit"; break;
    }
    const auto tracking_began=std::chrono::steady_clock::now();
    const auto step = StepApaTrackingInScene(scene,observed,path,1,calibration,&state,&clearance,&motion,true);
    tracking_seconds+=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-tracking_began).count();
    const auto& out = step.control;
    if (out.Chart.CtrlPhase == APA_MOCO_FAULT) { reason = "controller_fault"; break; }
    if (step.motion_limit.blocked()) {
      reason = "motion_limited"; rejection = step.motion_limit;
      observation = step.motion_observation; break;
    }
    motion = step.prediction;
    const auto safety_began=std::chrono::steady_clock::now();
    const auto safety = scene.CheckOnlineSafety(observed,&path,state.Chart,out.StopDistance,
        std::fabs(observed.vehicle_speed()),&clearance,&motion,
        &step.prediction_policy,&observation);
    safety_seconds+=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-safety_began).count();
    const bool arrival_brake = out.Chart.CtrlPhase == APA_MOCO_STOP_WAIT && safety.blocked() &&
        (safety.reason()=="clearance_predicted" || safety.reason()=="motion_blocked");
    if (arrival_brake) state.Longi = {};
    if (safety.blocked() && !arrival_brake) {
      reason = "execution_clearance"; rejection = safety; break;
    }
    if (out.Chart.CtrlPhase == APA_MOCO_DONE) { reason = "completed"; passed = true; break; }
    ApaMocoActuatorInput_t actuator_input{};
    actuator_input.VehicleSpeed = observed.vehicle_speed();
    actuator_input.VehicleMotionSpeed = std::fabs(observed.vehicle_speed());
    actuator_input.VehicleAcceleration = observed.vehicle_acceleration();
    actuator_input.GravityAcceleration = observed.road_gravity_x()*std::cos(observed.vehicle_yaw()) +
        observed.road_gravity_y()*std::sin(observed.vehicle_yaw());
    actuator_input.TargetSpeed = out.Longi.TargetSpeed;
    actuator_input.TargetAcceleration = out.Longi.TargetAcceleration;
    actuator_input.SpeedLimit = speed_limit;
    actuator_input.CycleTime = model.CycleTime;
    actuator_input.FeedbackGear = observed.gear_pos();
    actuator_input.TargetGear = out.Longi.TargetGearReq;
    actuator_input.Enable = out.Longi.ControlEnabled && !arrival_brake;
    actuator_input.StopReq = out.Longi.StopReq || arrival_brake;
    actuator_input.GearChangeAllowed = out.Longi.GearChangeAllowed;
    ApaMocoActuatorOutput_t pedals{};
    apaMocoActuate(&actuator_input,&actuator,&pedals);
    if (pedals.Overspeed) { reason = "actuator_overspeed"; break; }
    VehicleDynamicStepInput_t command{};

    command.GearPos = pedals.Gear;
    const Real next_steering = apaMocoSteeringResponse(steering,out.Lat.SteerAngle,
        calibration.SteeringResponseTime,model.CycleTime);
    command.SteerAngle = (steering+next_steering)*0.5f;
    steering = next_steering;
    VehicleDynamicStepOutput_t next{};
    StepExecutionMotion(pedals,command,model,actuator_input.GravityAcceleration,&plant,&next);
    if (!next.ModelValid) { reason = "invalid_execution_model"; break; }
    motion.measured = true;
    motion.longitudinal_speed = next.VehicleX/model.CycleTime;
    motion.lateral_speed = next.VehicleY/model.CycleTime;
    motion.yaw_rate = next.VehicleYaw/model.CycleTime;
    const double c=std::cos(observed.vehicle_yaw()), s=std::sin(observed.vehicle_yaw());
    observed.set_vehicle_x(observed.vehicle_x()+c*next.VehicleX-s*next.VehicleY);
    observed.set_vehicle_y(observed.vehicle_y()+s*next.VehicleX+c*next.VehicleY);
    const double yaw=observed.vehicle_yaw()+next.VehicleYaw;
    observed.set_vehicle_yaw(std::atan2(std::sin(yaw),std::cos(yaw)));
    observed.set_vehicle_speed(next.VehicleSpeed); observed.set_vehicle_acceleration(next.VehicleAcceleration);
    observed.set_gear_pos(pedals.Gear);
    observed.set_vehicle_steering_command_rad(out.Lat.SteerAngle);
  }
  if (rejection.blocked()) {
    std::fprintf(stderr,"[APA execution rejection] task=%llu scene=%llu reason=%s safety_reason=%s "
        "obstacle=%s kind=%s pose_valid=%d model=%d sample_m=%.6f "
        "actual_x=%.9f actual_y=%.9f actual_yaw=%.9f speed=%.6f "
        "checked_x=%.9f checked_y=%.9f checked_yaw=%.9f "
        "current_gap_m=%.6f checked_gap_m=%.6f checked_swept_gap_m=%.6f "
        "front_margin_m=%.6f rear_margin_m=%.6f side_margin_m=%.6f execution_gap_floor_m=%.6f\n",
        static_cast<unsigned long long>(input.task_id()),
        static_cast<unsigned long long>(input.scene().version()),reason,rejection.reason().c_str(),
        rejection.obstacle_id().c_str(),observation.obstacle_kind.c_str(),observation.pose_valid,
        observation.prediction_model,observation.sample_distance_m,
        observed.vehicle_x(),observed.vehicle_y(),observed.vehicle_yaw(),observed.vehicle_speed(),
        observation.x,observation.y,observation.yaw,observation.current_gap_m,
        observation.checked_gap_m,observation.checked_swept_gap_m,
        observation.margins_m[0],observation.margins_m[1],observation.margins_m[2],
        observation.execution_gap_floor_m);
  }
  std::fprintf(stderr,"[APA execution] passed=%d reason=%s steps=%d segment=%ld phase=%ld stop=%ld goal=%.4f yaw=%.4f "
      "tracking_sec=%.6f safety_sec=%.6f elapsed=%.6f\n",
      passed,reason,steps,static_cast<long>(state.Chart.StpNow+1),
      static_cast<long>(state.Chart.Phase),static_cast<long>(state.Chart.StopReason),
      std::hypot(observed.vehicle_x()-input.target_x(),observed.vehicle_y()-input.target_y()),
      observed.vehicle_yaw()-input.target_theta(),tracking_seconds,safety_seconds,
      std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count());
  return passed;
}
}
