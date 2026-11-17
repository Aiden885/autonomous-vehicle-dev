#include "ApaModule.h"
#include "apa_input_contract.h"
#include "apa_online_safety.h"
#include "apa_control_limits.h"
#include "apaVehicleDefaults.h"
#include "apaTrackingPolicy.h"

extern "C" {
#include "apaPlanningTypes.h"
#include "apaEnums.h"
}

#include <google/protobuf/text_format.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <chrono>
#include <stdexcept>

namespace pangu {
namespace modules {
namespace {

float ToReal(double value) { return static_cast<float>(value); }

int ClampPathCount(int count) {
  if (count < 0) {
    return 0;
  }
  if (count > PATH_PT_MAX) {
    return PATH_PT_MAX;
  }
  return count;
}

template <typename ArrayT, typename RepeatedT>
void CopyRepeated(const RepeatedT& source, ArrayT* target) {
  if (static_cast<size_t>(source.size()) > target->size())
    throw std::invalid_argument("planner array exceeds capacity");
  const int count = source.size();
  for (int i = 0; i < count; ++i) {
    (*target)[i] = static_cast<::apa::Real>(source.Get(i));
  }
}

::apa::ApaParkingPathPlanningInputData BuildPlannerInput(
    const pangu::modules::ApaInput& input) {
  ::apa::ApaParkingPathPlanningInputData planner_input{};
  planner_input.parkingAction = input.parking_action();
  planner_input.targetSlotKind = input.target_slot_kind();
  planner_input.vehicleX = static_cast<::apa::Real>(input.vehicle_x());
  planner_input.vehicleY = static_cast<::apa::Real>(input.vehicle_y());
  planner_input.vehicleYaw = static_cast<::apa::Real>(input.vehicle_yaw());
  planner_input.targetX = static_cast<::apa::Real>(input.target_x());
  planner_input.targetY = static_cast<::apa::Real>(input.target_y());
  planner_input.targetTheta = static_cast<::apa::Real>(input.target_theta());
  planner_input.minimumTurningRadius = static_cast<::apa::Real>(
      input.r_min());
  planner_input.entrancePointX = static_cast<::apa::Real>(input.entry_point_x());
  planner_input.entrancePointY = static_cast<::apa::Real>(input.entry_point_y());
  planner_input.slotPoint1X = static_cast<::apa::Real>(input.slot_point1_x());
  planner_input.slotPoint1Y = static_cast<::apa::Real>(input.slot_point1_y());
  planner_input.slotPoint2X = static_cast<::apa::Real>(input.slot_point2_x());
  planner_input.slotPoint2Y = static_cast<::apa::Real>(input.slot_point2_y());
  planner_input.slotPoint3X = static_cast<::apa::Real>(input.slot_point3_x());
  planner_input.slotPoint3Y = static_cast<::apa::Real>(input.slot_point3_y());
  planner_input.slotPoint4X = static_cast<::apa::Real>(input.slot_point4_x());
  planner_input.slotPoint4Y = static_cast<::apa::Real>(input.slot_point4_y());
  planner_input.slotEntrance = input.slot_entry();
  planner_input.parkingType = input.parking_type();
  planner_input.surroundingType = input.surrounding_type();
  planner_input.availableSpaceOffset = static_cast<::apa::Real>(input.angle_offset());
  planner_input.shortcutSwitch = input.shortcut_switch() ? 1 : 0;
  planner_input.shortcutThreshold = static_cast<::apa::Real>(input.shortcut_threshold());
  planner_input.needHybridAlgorithmFusion =
      input.need_hybrid_algorithm_fusion() ? 1 : 0;

  planner_input.boundaryLineCount = input.boundary_line_num();
  planner_input.virtualBoundaryLineCount = input.virtual_boundary_line_num();
  planner_input.slotTrackStopperCount = input.slot_track_stopper_num();
  planner_input.boundarySepPointCount = input.boundary_sep_point_num();
  planner_input.stopperPointCount = input.stopper_point_num();

  CopyRepeated(input.boundary_line_x1(), &planner_input.boundaryLineX1);
  CopyRepeated(input.boundary_line_y1(), &planner_input.boundaryLineY1);
  CopyRepeated(input.boundary_line_x2(), &planner_input.boundaryLineX2);
  CopyRepeated(input.boundary_line_y2(), &planner_input.boundaryLineY2);
  CopyRepeated(input.virtual_boundary_p1_x(), &planner_input.virtualBoundaryPoint1X);
  CopyRepeated(input.virtual_boundary_p1_y(), &planner_input.virtualBoundaryPoint1Y);
  CopyRepeated(input.virtual_boundary_p2_x(), &planner_input.virtualBoundaryPoint2X);
  CopyRepeated(input.virtual_boundary_p2_y(), &planner_input.virtualBoundaryPoint2Y);
  CopyRepeated(input.slot_track_stopper_x1(), &planner_input.slotTrackStopperX1);
  CopyRepeated(input.slot_track_stopper_y1(), &planner_input.slotTrackStopperY1);
  CopyRepeated(input.slot_track_stopper_x2(), &planner_input.slotTrackStopperX2);
  CopyRepeated(input.slot_track_stopper_y2(), &planner_input.slotTrackStopperY2);
  CopyRepeated(input.boundary_sep_point_x(), &planner_input.boundarySepPointX);
  CopyRepeated(input.boundary_sep_point_y(), &planner_input.boundarySepPointY);
  CopyRepeated(input.stopper_point_x(), &planner_input.stopperPointX);
  CopyRepeated(input.stopper_point_y(), &planner_input.stopperPointY);
  return planner_input;
}

}

REGISTER_NODE_DEFINE(apa);
REGISTER_CALLBACK(apa, apa_input, pangu::modules::ApaInput);

void apa::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SHOWVERSION();
}

bool apa::Init(const std::string& ptstr) {
  SetVersion();
  if (!LoadConfig(ptstr)) {
    PLOG_ERROR("can't load proto str for module:{}", module_name_);
    return false;
  }
  if (!InitIO()) {
    PLOG_ERROR("can't create io for module:{}", module_name_);
    return false;
  }
  planning_limits_.set_attempt_timeout_sec(0.0);
  planning_limits_.set_recovery_timeout_sec(15.0);
  planning_limits_.set_max_attempts(4);
  planning_limits_.set_retry_delay_sec(0.5);
  if (apa_params_.has_planning_limits()) planning_limits_ = apa_params_.planning_limits();
  if (!std::isfinite(planning_limits_.attempt_timeout_sec()) || planning_limits_.attempt_timeout_sec() < 0.0 ||
      planning_limits_.attempt_timeout_sec() > 3600.0 ||
      !std::isfinite(planning_limits_.recovery_timeout_sec()) ||
      planning_limits_.recovery_timeout_sec() <= 0.0 ||
      planning_limits_.recovery_timeout_sec() > 3600.0 ||
      !std::isfinite(planning_limits_.retry_delay_sec()) || planning_limits_.retry_delay_sec() < 0.0 ||
      planning_limits_.max_attempts() == 0) return false;

  recovery_decision_param_.planningReserveSec = 0.0;
  recovery_decision_ = ::apa::ApaRecoveryDecision{recovery_decision_param_};
  std::string control_error;
  if (!BuildApaControlCalibration(apa_params_, &control_param_, &control_error)) {
    PLOG_ERROR("APA invalid control calibration: {}", control_error);
    return false;
  }
  control_period_sec_ = apa_params_.control_period_sec() > 0.0
      ? apa_params_.control_period_sec() : control_param_.Longi.CycleTime;
  if (!std::isfinite(control_period_sec_) || !std::isfinite(apa_params_.timer_duration()) ||
      std::fabs(control_period_sec_ - control_param_.Longi.CycleTime) > 1.0e-6 ||
      apa_params_.timer_duration() < 1.0 || apa_params_.timer_duration() > 20.0) {
    PLOG_ERROR("APA requires the C model period and a scheduler wakeup of 1..20 ms");
    return false;
  }
  ResetMocoState();
  if (!CREATE_TIMER_TASK(apa, static_cast<int>(apa_params_.timer_duration()), apa::Proc)) {
    PLOG_ERROR("APA control scheduler registration failed");
    return false;
  }
  PLOG_INFO("APA simulation clock: control_period_sec={}, scheduler_wakeup_ms={}",
            control_period_sec_, apa_params_.timer_duration());
  std::fprintf(stderr, "[APA clock] config control_period_sec=%.6f scheduler_wakeup_ms=%.0f\n",
               control_period_sec_, apa_params_.timer_duration());
  const auto effective_limits = DescribeApaActuationLimits(control_param_);
  std::fprintf(stderr, "[APA limits] speed=%.4f accel=%.4f steer=%.4f normal_brake=%.4f\n",
      effective_limits.max_abs_target_speed_mps(), effective_limits.max_abs_accel_mps2(),
      effective_limits.max_abs_steer_rad(), control_param_.Longi.StopDeceleration);
  std::fprintf(stderr, "[APA tracking] preview_min=%.3f preview_max=%.3f preview_time=%.3f kpp=%.3f ki=%.3f integral_limit=%.3f parallel_min_speed=%.3f parallel_terminal_speed=%.3f\n",
      control_param_.Path.PreviewDistMin, control_param_.Path.PreviewDistMax,
      control_param_.Path.PreviewTime, control_param_.Lat.Kpp,
      control_param_.Lat.IntegralGain, control_param_.Lat.IntegralCurvatureLimit,
      control_param_.Longi.ParallelMinTrackingSpeed, control_param_.Longi.ParallelTerminalKeepSpeed);
  PLOG_INFO("apa init: target_speed_mps={}, max_accel_mps2={}, max_steer_rad={}, enable_control={}, forward_ref_speed={}, reverse_ref_speed={}, terminal_speed={}, cusp_approach_speed={}, short_segment_length={}, short_segment_speed={}, minimum_tracking_speed={}, slowdown_distance={}",
            apa_params_.target_speed_mps(), apa_params_.max_accel_mps2(),
            apa_params_.max_steer_rad(), apa_params_.enable_control(),
            control_param_.Longi.ForwardRefSpeed,
            control_param_.Longi.ReverseRefSpeed,
            control_param_.Longi.TerminalKeepSpeed,
            control_param_.Longi.CuspApproachSpeed,
            control_param_.Longi.ShortSegmentLength,
            control_param_.Longi.ShortSegmentSpeed,
            control_param_.Longi.VerticalMinTrackingSpeed,
            control_param_.Longi.SlowDistThd);
  return true;
}

bool apa::InitIO() {
  CREATE_OUTPUT_RETURN_IF_ERROR(apa_output, pangu::modules::ApaOutput);
  CREATE_INPUT_RETURN_IF_ERROR(apa, apa_input, pangu::modules::ApaInput);
  PLOG_INFO("apa InitIO end");
  return true;
}

bool apa::LoadConfig(const std::string& ptstr) {
  if (!google::protobuf::TextFormat::ParseFromString(ptstr, &apa_params_)) {
    PLOG_ERROR("Can't get conf file of [{}]", module_name_);
    return false;
  }
  return true;
}

apa::~apa() { planning_job_.Cancel(); }

void apa::ResetMocoState() {
  std::memset(&control_state_, 0, sizeof(control_state_));
  actuator_state_ = {};
  planner_output_ = {};
  has_plan_ = false;
  path_update_idx_ = 1;
  last_control_time_ = -1.0;
  control_step_count_ = 0;
  last_control_log_time_ = -1.0;
  last_clock_log_time_ = -1.0;
  last_clock_status_.clear();
  planning_job_.Cancel();
  planning_started_.reset();
  planning_time_ = {};
  replan_stage_ = ReplanStage::Idle;
  recovery_id_ = path_version_ = planning_attempts_ = 0;
  recovery_progress_ = {};
  planner_error_type_ = 0;
  recovery_reason_.clear();
  recovery_context_.Reset();
  recovery_decision_ = ::apa::ApaRecoveryDecision{recovery_decision_param_};
  motion_history_valid_ = false;
  motion_prediction_ = {};
  motion_prediction_policy_ = {};
  motion_limit_.Clear();
  pending_clearance_ = {};
  execution_clearance_ = {};
  clearance_started_sim_time_ = -1.0;
  fault_.clear();
  input_error_.clear();
  request_fingerprint_.clear();
  safety_state_.Clear();
  safety_state_.set_state("none");
  online_safety_.Clear();
  planning_origin_scene_version_ = 0;
  plan_scene_version_ = 0;
  task_control_param_ = control_param_;
}

void apa::InvalidatePath() {
  motion_limit_.Clear();
  planning_job_.Cancel();
  FinishPlanningTime();
  last_control_time_ = -1.0;
  has_plan_ = false;
  planner_output_ = {};
  control_state_ = {};
  pending_clearance_ = {};
  execution_clearance_ = {};
  clearance_started_sim_time_ = -1.0;
}

void apa::LatchInputFault(const std::string& reason) {
  if (input_error_.empty()) {
    input_error_ = reason.substr(0, 512);
    InvalidatePath();
    std::fprintf(stderr, "[APA input] task=%llu fault=%s\n",
        static_cast<unsigned long long>(task_id_), input_error_.c_str());
  }
  fault_ = "input_error";
  replan_stage_ = ReplanStage::Failed;
  recovery_reason_ = fault_;
}

void apa::BeginRecovery(const std::string& reason, const pangu::modules::ApaInput& input) {
  const double position = std::hypot(input.target_x()-input.vehicle_x(),
      input.target_y()-input.vehicle_y());
  const double yaw = std::fabs(std::atan2(std::sin(input.target_theta()-input.vehicle_yaw()),
      std::cos(input.target_theta()-input.vehicle_yaw())));
  const double pos_tolerance = input.has_goal_tolerance() ?
      std::min<double>(task_control_param_.Chart.GoalPositionTolerance,input.goal_tolerance().position_m()) :
      task_control_param_.Chart.GoalPositionTolerance;
  const double yaw_tolerance = input.has_goal_tolerance() ?
      std::min<double>(task_control_param_.Chart.GoalYawTolerance,input.goal_tolerance().yaw_rad()) :
      task_control_param_.Chart.GoalYawTolerance;
  const double error = std::max(position/pos_tolerance,yaw/yaw_tolerance);
  const bool progress = recovery_progress_.Begin(error,planning_limits_.max_attempts());
  InvalidatePath();
  recovery_context_.Reset();
  recovery_decision_ = ::apa::ApaRecoveryDecision{recovery_decision_param_};
  ++recovery_id_;
  recovery_reason_ = reason;
  planner_error_type_ = 0;
  recovery_started_ = next_attempt_ = ApaPlanningJob::Clock::now();
  planning_time_ = {};
  replan_stage_ = ReplanStage::Stopping;
  planning_attempts_ = 0;
  std::fprintf(stderr, "[APA replan] task=%llu recovery=%llu stopping reason=%s\n",
      static_cast<unsigned long long>(task_id_), static_cast<unsigned long long>(recovery_id_), reason.c_str());
  if (!progress) FailRecovery("recovery_no_progress");
}

void apa::FailRecovery(const std::string& reason, const std::string& status) {
  const std::string failure_reason = reason;
  InvalidatePath();
  replan_stage_ = ReplanStage::Failed;
  recovery_reason_ = failure_reason;
  fault_ = status.empty() ? failure_reason : status;
  std::fprintf(stderr, "[APA replan] task=%llu recovery=%llu failed reason=%s planner_error=%d\n",
      static_cast<unsigned long long>(task_id_), static_cast<unsigned long long>(recovery_id_),
      failure_reason.c_str(), planner_error_type_);
}

double apa::RecoveryRemainingSec(ApaPlanningJob::Clock::time_point now) const {
  const auto waiting = planning_started_.value_or(now) - recovery_started_ - planning_time_;
  return planning_limits_.recovery_timeout_sec() - std::chrono::duration<double>(waiting).count();
}

void apa::FinishPlanningTime() {
  if (!planning_started_) return;
  planning_time_ += ApaPlanningJob::Clock::now() - *planning_started_;
  planning_started_.reset();
}

void apa::StartPlanning(const pangu::modules::ApaInput& input) {
  auto planner_input = BuildPlannerInput(input);
  task_control_param_ = control_param_;
  if (input.has_goal_tolerance()) {
    task_control_param_.Chart.GoalPositionTolerance = std::min(
        task_control_param_.Chart.GoalPositionTolerance, ToReal(input.goal_tolerance().position_m()));
    task_control_param_.Chart.GoalYawTolerance = std::min(
        task_control_param_.Chart.GoalYawTolerance, ToReal(input.goal_tolerance().yaw_rad()));
  }
  planner_input.goalPositionTolerance = task_control_param_.Chart.GoalPositionTolerance;
  planner_input.goalYawTolerance = task_control_param_.Chart.GoalYawTolerance;
  planning_origin_scene_version_ = input.scene().version();
  recovery_context_.RecordAttempt(input);
  ++planning_attempts_;
  planner_error_type_ = 0;
  const auto now = ApaPlanningJob::Clock::now();
  const double remaining = RecoveryRemainingSec(now);
  if (remaining <= 0.0) { FailRecovery("recovery_timeout"); return; }
  planning_started_ = now;
  pending_clearance_ = PrepareApaClearanceRecovery(input);
  planning_job_.Start(planner_input, input, planning_limits_.attempt_timeout_sec(), task_control_param_);
  replan_stage_ = ReplanStage::Planning;
  std::fprintf(stderr, "[APA replan] task=%llu recovery=%llu attempt=%u scene=%llu planning\n",
      static_cast<unsigned long long>(task_id_), static_cast<unsigned long long>(recovery_id_), planning_attempts_,
      static_cast<unsigned long long>(input.scene().version()));
}

bool apa::UpdatePlanning(const pangu::modules::ApaInput& input, ApaPlanningJob::Clock::time_point now) {
  if (replan_stage_ == ReplanStage::Idle || replan_stage_ == ReplanStage::Ready || replan_stage_ == ReplanStage::Failed) return false;
  const auto assessment = recovery_context_.Assess(input, online_safety_);
  ::apa::ApaRecoveryDecisionTraits::Input decision_input{};
  decision_input.risk = assessment.risk;
  decision_input.blockerChanged = assessment.blocker_changed;
  decision_input.blockerSpeedMps = assessment.blocker_speed_mps;
  decision_input.stopped = input.recovery().recovery_id() == recovery_id_ &&
      input.recovery().stop().state() == "stopped" &&
      input.recovery().stop().standstill_sec()+1e-6 >= task_control_param_.Chart.StopDwellCycles*control_period_sec_ &&
      std::fabs(input.vehicle_speed()) <= task_control_param_.Chart.StandstillSpeed;

  decision_input.escapeAvailable = assessment.risk == ::apa::ApaRecoveryRisk::VirtualStartConstraint &&
      PrepareApaClearanceRecovery(input).info.enabled;
  decision_input.plannerRunning = planning_job_.Running();
  decision_input.waitForSceneChange = replan_stage_ == ReplanStage::WaitingScene && recovery_context_.HasAttempt();

  decision_input.problemChanged = !decision_input.waitForSceneChange || recovery_context_.ProblemChanged(input);
  decision_input.retryDelayElapsed = now >= next_attempt_;
  decision_input.attemptsAvailable = planning_attempts_ < planning_limits_.max_attempts();
  decision_input.simTimeSec = input.sim_time_sec();
  decision_input.remainingSec = RecoveryRemainingSec(now);
  ::apa::ApaRecoveryDecisionTraits::Output decision_output{};
  recovery_decision_.run(decision_input, decision_output);
  using Action = ::apa::ApaRecoveryAction;
  if (decision_output.action == Action::Fail) {
    FailRecovery(ApaRecoveryFailureReason(decision_output.failure), decision_input.stopped ? "plan_failed" : "");
    return true;
  }
  if (decision_output.action == Action::Hold) {
    if (replan_stage_ != ReplanStage::Stopping) {
      planning_job_.Cancel();
      FinishPlanningTime();
      recovery_context_.ForgetAttempt();
      pending_clearance_ = {};
      replan_stage_ = ReplanStage::Stopping;
      recovery_reason_ = "standstill_lost";
      return true;
    }
    return false;
  }
  if (decision_output.action == Action::Observe) {
    if (planning_job_.Running()) {
      planning_job_.Cancel();
      FinishPlanningTime();
      recovery_context_.ForgetAttempt();
      pending_clearance_ = {};
    }
    const std::string reason = online_safety_.blocked() ? online_safety_.reason() : "obstacle_clear_confirmation";
    const bool changed = replan_stage_ != ReplanStage::WaitingScene || recovery_reason_ != reason;
    replan_stage_ = ReplanStage::WaitingScene;
    recovery_reason_ = reason;
    return changed;
  }
  if (decision_output.action == Action::WaitForScene) return false;
  if (decision_output.action == Action::PollPlanner) return PollPlanningResult(input, now);
  try { StartPlanning(input); }
  catch (const std::exception& error) {
    FailRecovery("planner_start_failed");
    PLOG_ERROR("APA planner start: {}", error.what());
  }
  return true;
}

bool apa::PollPlanningResult(const pangu::modules::ApaInput& input, ApaPlanningJob::Clock::time_point now) {
  const auto state = planning_job_.Poll(now);
  if (state == ApaPlanningJob::State::Running) return false;
  if (state == ApaPlanningJob::State::Timeout) {
    FailRecovery("planning_timeout", "plan_failed"); return true;
  }
  if (state == ApaPlanningJob::State::Failed) {
    FailRecovery("planner_process_failed", "plan_failed"); return true;
  }
  const auto& result = planning_job_.Result();
  planner_error_type_ = result.path.errorType;
  bool retry_same_scene = false;
  if (result.code != ApaPlanResultCode::Success) {
    recovery_reason_ = result.code == ApaPlanResultCode::StartContact ? "start_contact" :
        result.code == ApaPlanResultCode::ClearanceNoSolution ? "clearance_recovery_no_solution" :
        result.code == ApaPlanResultCode::ClearanceCapacity ? "clearance_recovery_capacity" :
        result.code == ApaPlanResultCode::SceneCapacity ? "planning_scene_capacity" :
        result.code == ApaPlanResultCode::RefinementLimit ? "planning_refinement_limit" :
        result.code == ApaPlanResultCode::UnsafeCandidate ? "unsafe_candidate" : "no_solution";

    if (path_version_ == 0 || result.code == ApaPlanResultCode::SceneCapacity ||
        result.code == ApaPlanResultCode::RefinementLimit || pending_clearance_.info.enabled) {
      FailRecovery(recovery_reason_, "plan_failed");
      return true;
    }
  } else {
    const auto& path = result.path;
    if (static_cast<bool>(result.recovery.enabled) != static_cast<bool>(pending_clearance_.info.enabled)) {
      FailRecovery("clearance_context_mismatch", "plan_failed"); return true;
    }

    auto admitted_recovery = pending_clearance_;
    if (admitted_recovery.info.enabled) admitted_recovery.info.prefix_length = result.recovery.prefix_length;
    const double position_error = std::hypot(input.vehicle_x()-path.pathPointX[0], input.vehicle_y()-path.pathPointY[0]);
    const double yaw_error = std::atan2(std::sin(input.vehicle_yaw()-path.pathPointTheta[0]),
        std::cos(input.vehicle_yaw()-path.pathPointTheta[0]));
    if (position_error > 0.1 || std::fabs(yaw_error) > 0.05) {
      recovery_reason_ = "stale_plan_start"; retry_same_scene = true;
    } else if (const auto candidate_check = CheckApaCandidatePath(input, path, &admitted_recovery,
        task_control_param_.Chart.GoalPositionTolerance, task_control_param_.Chart.GoalYawTolerance);
        candidate_check.blocked()) {
      recovery_reason_ = "candidate_blocked";
      std::fprintf(stderr, "[APA candidate] stage=receiver task=%llu origin_scene=%llu scene=%llu obstacle=%s reason=%s distance=%.3f\n",
          static_cast<unsigned long long>(task_id_), static_cast<unsigned long long>(planning_origin_scene_version_),
          static_cast<unsigned long long>(candidate_check.scene_version()), candidate_check.obstacle_id().c_str(),
          candidate_check.reason().c_str(), candidate_check.checked_distance_m());
      if (path_version_ == 0) {
        FailRecovery(recovery_reason_, "plan_failed");
        return true;
      }
    } else {
      if (RecoveryRemainingSec(ApaPlanningJob::Clock::now()) <= 0.0) {
        FailRecovery("recovery_timeout"); return true;
      }
      planner_output_ = path;
      execution_clearance_ = admitted_recovery;
      clearance_started_sim_time_ = -1.0;
      online_safety_ = candidate_check;
      plan_scene_version_ = planning_origin_scene_version_;
      has_plan_ = true;
      ++path_update_idx_; ++path_version_;
      control_state_ = {}; last_control_time_ = -1.0;
      control_state_.EstimatedSteerAngle = input.vehicle_steering_command_rad();
      control_state_.Lat.LastSteerAngle = input.vehicle_steering_command_rad();
      control_state_.Lat.SteerTarget = input.vehicle_steering_command_rad();
      replan_stage_ = ReplanStage::Ready;
      FinishPlanningTime();
      recovery_reason_.clear();
      std::fprintf(stderr, "[APA replan] task=%llu recovery=%llu path=%llu origin_scene=%llu accepted_scene=%llu ready\n",
          static_cast<unsigned long long>(task_id_), static_cast<unsigned long long>(recovery_id_),
          static_cast<unsigned long long>(path_version_), static_cast<unsigned long long>(plan_scene_version_),
          static_cast<unsigned long long>(input.scene().version()));
      return true;
    }
  }
  replan_stage_ = ReplanStage::WaitingScene;
  FinishPlanningTime();
  if (retry_same_scene) recovery_context_.ForgetAttempt();
  next_attempt_ = ApaPlanningJob::Clock::now() + std::chrono::duration_cast<ApaPlanningJob::Clock::duration>(
      std::chrono::duration<double>(planning_limits_.retry_delay_sec()));
  if (planning_attempts_ >= planning_limits_.max_attempts()) FailRecovery("replan_exhausted");
  return true;
}

void apa::RunControlStep(const ApaSafetyScene& scene, const pangu::modules::ApaInput& input,
                         pangu::modules::ApaOutput* output) {
  const bool restoring_clearance = execution_clearance_.info.enabled &&
      ApaPathProgress(planner_output_, control_state_.Chart) <= execution_clearance_.info.prefix_length + 1.e-4;
  if (restoring_clearance) {
    if (clearance_started_sim_time_ < 0.0) clearance_started_sim_time_ = input.sim_time_sec();
    if (input.sim_time_sec() - clearance_started_sim_time_ > APA_CLEARANCE_RECOVERY_MAX_DURATION_SEC ||
        std::fabs(input.vehicle_speed()) > static_cast<Real>(APA_CLEARANCE_RECOVERY_MAX_SPEED_MPS) + 0.05) {
      FailRecovery("clearance_recovery_execution_limit");
      return;
    }
  }
  const auto step = StepApaTracking(scene,input,planner_output_,path_update_idx_,task_control_param_,
      &control_state_,&execution_clearance_,&motion_prediction_);
  const auto& control_output = step.control;
  motion_limit_ = step.motion_limit;
  motion_prediction_policy_ = step.prediction_policy;
  motion_prediction_ = step.prediction;
  ++control_step_count_;
  last_control_time_ = input.sim_time_sec();
  output->set_stop_req(control_output.Longi.StopReq != 0 || control_output.Chart.CtrlPhase == 6);
  output->set_target_speed_mps(control_output.Longi.TargetSpeed);
  output->set_target_accel_mps2(control_output.Longi.TargetAcceleration);
  output->set_steer_rad(control_output.Lat.SteerAngle);
  output->set_target_gear_req(control_output.Longi.TargetGearReq);
  output->set_direction_sign(control_output.Longi.DirectionSign);
  output->set_gear_change_allowed(control_output.Longi.GearChangeAllowed != 0);
  output->set_ctrl_phase(control_output.Chart.CtrlPhase);
  output->set_enable(control_output.Longi.ControlEnabled != 0);
  output->set_control_dt_sec(control_period_sec_);
  output->set_control_status(control_output.Chart.CtrlPhase == APA_MOCO_DONE ? "complete" :
      control_output.Chart.CtrlPhase == APA_MOCO_FAULT ? "tracking_fault" : "tracking");
  auto* d = output->mutable_control_diagnostic();
  d->set_segment(control_output.Chart.ApaStpNow);
  d->set_segment_count(control_output.Chart.ApaStpNum);
  d->set_reference_index(control_output.Path.RefIndex);
  d->set_remaining_distance_m(control_output.Chart.Dist2Go);
  d->set_stop_distance_m(control_output.StopDistance);
  d->set_cross_track_error_m(control_output.Chart.Projection.CrossTrackError);
  d->set_heading_error_rad(control_output.Chart.Projection.HeadingError);
  d->set_endpoint_distance_m(control_output.Chart.GoalDistance);
  d->set_endpoint_yaw_error_rad(control_output.Chart.GoalYawError);
  d->set_vehicle_speed_mps(input.vehicle_speed());
  d->set_vehicle_acceleration_mps2(input.vehicle_acceleration());
  d->set_feedback_gear(input.gear_pos());
  d->set_stop_reason(control_output.Chart.StopReason);
  d->set_reference_valid(control_output.Path.ReferenceValid);
  d->set_steer_pursuit_rad(control_output.Lat.StrPP);
  d->set_steer_heading_rad(control_output.Lat.StrH);
  d->set_steer_saturated(control_output.Lat.SteerSaturated != 0);
  d->set_progress_m(control_output.Chart.Projection.Progress);
  d->set_reference_x(control_output.Path.RefX);
  d->set_reference_y(control_output.Path.RefY);
  d->set_goal_x(planner_output_.resolvedScene.targetX);
  d->set_goal_y(planner_output_.resolvedScene.targetY);
  d->set_goal_yaw_rad(planner_output_.resolvedScene.targetTheta);
  const auto& previous = cached_output_.control_diagnostic();
  if (last_control_log_time_ < 0.0 || input.sim_time_sec() - last_control_log_time_ >= 0.2 - 1.e-6 ||
      output->ctrl_phase() != cached_output_.ctrl_phase() || d->segment() != previous.segment() ||
      d->stop_reason() != previous.stop_reason()) {
    std::fprintf(stderr, "[APA control] task=%llu frame=%llu t=%.6f x=%.6f y=%.6f yaw=%.6f phase=%d segment=%d/%d "
        "progress=%.4f remaining=%.4f stop_distance=%.4f endpoint=%.4f endpoint_yaw=%.5f "
        "ey=%.4f eheading=%.5f ref=%d valid=%d speed=%.4f acceleration=%.4f "
        "gear=%d request=%d target_speed=%.4f target_accel=%.4f steer=%.5f integral_curvature=%.5f reason=%d\n",
        static_cast<unsigned long long>(input.task_id()),
        static_cast<unsigned long long>(input.carla_frame_id()), input.sim_time_sec(),
        input.vehicle_x(), input.vehicle_y(), input.vehicle_yaw(),
        control_output.Chart.CtrlPhase, d->segment(), d->segment_count(), d->progress_m(),
        d->remaining_distance_m(), d->stop_distance_m(), d->endpoint_distance_m(),
        d->endpoint_yaw_error_rad(), d->cross_track_error_m(), d->heading_error_rad(),
        d->reference_index(), d->reference_valid(), input.vehicle_speed(), input.vehicle_acceleration(),
        input.gear_pos(), output->target_gear_req(), output->target_speed_mps(),
        output->target_accel_mps2(), output->steer_rad(), control_output.Lat.IntegralCurvature, d->stop_reason());
    last_control_log_time_ = input.sim_time_sec();
  }
}

bool apa::Proc() {

  std::unique_lock<std::mutex> control_lock(control_mutex_, std::try_to_lock);
  if (!control_lock.owns_lock()) return true;
  const auto now = ApaPlanningJob::Clock::now();
  const bool recovering = replan_stage_ == ReplanStage::Stopping || replan_stage_ == ReplanStage::Planning ||
      replan_stage_ == ReplanStage::WaitingScene;
  const bool budget_due = recovering && (planning_job_.Expired(now) ||
      RecoveryRemainingSec(now) <= 0.0);
  pangu::modules::ApaInput input;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (processed_revision_ == input_revision_ && !budget_due) return true;
    input = latest_input_;
    processed_revision_ = input_revision_;
  }
  const bool new_task = input.session_id() != session_id_ || input.task_id() != task_id_ ||
      input.parking_enabled() != parking_enabled_ || input.parking_action() != last_parking_action_;
  if (new_task) {
    const bool same_identity = input.session_id() == session_id_ && input.task_id() == task_id_;
    const auto previous_error = same_identity ? input_error_ : std::string{};
    const auto previous_request = same_identity ? request_fingerprint_ : std::string{};
    const auto previous_online_safety = online_safety_;
    const auto previous_fault = same_identity ? fault_ : std::string{};
    ResetMocoState();
    session_id_ = input.session_id();
    task_id_ = input.task_id();
    parking_enabled_ = input.parking_enabled();
    last_parking_action_ = input.parking_action();
    have_feedback_ = false;
    if (!previous_error.empty()) LatchInputFault(previous_error);
    request_fingerprint_ = previous_request;
    if (!previous_fault.empty()) {
      online_safety_ = previous_online_safety;
      fault_ = previous_fault;
      replan_stage_ = ReplanStage::Failed;
      recovery_reason_ = previous_fault;
    }
  }
  const bool had_input_fault = !input_error_.empty();
  std::string invalid;
  if (!input.has_input_validation()) invalid = "missing_input_validation";
  else if (!input.input_validation().error().empty()) invalid = input.input_validation().error();
  else if (!input.input_validation().feedback_valid()) invalid = "feedback_not_validated";
  else if (!ValidateApaFeedback(input, &invalid)) {}
  else if (input.input_validation().request_valid()) {
    if (ValidateApaRequest(input, &invalid)) {
      const auto fingerprint = ApaRequestFingerprint(input);
      if (!request_fingerprint_.empty() && request_fingerprint_ != fingerprint)
        invalid = "request_changed_without_new_task";
      else request_fingerprint_ = fingerprint;
    }
  } else if (has_plan_ || !request_fingerprint_.empty()) invalid = "accepted_request_became_invalid";
  if (!invalid.empty()) LatchInputFault(invalid);
  auto validation = input.input_validation();
  if (!input_error_.empty()) validation.set_error(input_error_);
  auto safety = input.safety_state();
  if (!input.has_safety_state() ||
      (safety.state() != "none" && safety.state() != "braking" &&
       safety.state() != "stopped" && safety.state() != "unconfirmed") ||
      !std::isfinite(safety.standstill_sec()) || safety.standstill_sec() < 0.0 ||
      (safety.state() == "none" && (!safety.reason().empty() || safety.standstill_sec() != 0.0)) ||
      (safety.state() != "none" && safety.reason().empty())) {
    safety.Clear();
    safety.set_reason("invalid_safety_state");
    safety.set_state("unconfirmed");
  }
  bool safety_changed = false;

  if (!safety.reason().empty()) {
    if (safety_state_.reason().empty()) {
      InvalidatePath();
      std::fprintf(stderr, "[APA safety] session=%s task=%llu event=%llu reason=%s\n",
          session_id_.c_str(), static_cast<unsigned long long>(task_id_),
          static_cast<unsigned long long>(safety.event_id()), safety.reason().c_str());
    }
    if (safety_state_.reason().empty() || safety.event_id() >= safety_state_.event_id()) {
      safety_changed = safety_state_.SerializeAsString() != safety.SerializeAsString();
      safety_state_ = safety;
    }
    fault_ = "safety_stop";
    replan_stage_ = ReplanStage::Failed;
    recovery_reason_ = safety.reason();
  } else if (safety_state_.reason().empty()) {
    safety_state_ = safety;
  }
  bool planning_changed = false;
  bool body_checked = false;

  std::optional<ApaSafetyScene> safety_scene;
  const auto scene = [&]() -> const ApaSafetyScene& {
    if (!safety_scene) safety_scene.emplace(input);
    return *safety_scene;
  };
  if (parking_enabled_ && apa_params_.enable_control() && fault_.empty() && validation.request_valid()) {
    if (replan_stage_ == ReplanStage::Ready && control_state_.Chart.Phase != APA_MOCO_DONE &&
        control_state_.Chart.Phase != APA_MOCO_FAULT && input.recovery().recovery_id() == recovery_id_ &&
        !input.recovery().stop().reason().empty() && input.recovery().stop().state() != "stopped") {
      BeginRecovery("standstill_lost", input); planning_changed = true;
    }
    if (replan_stage_ == ReplanStage::Idle) { BeginRecovery("initial_plan", input); planning_changed = true; }
    if (replan_stage_ != ReplanStage::Ready) {
      online_safety_ = scene().CheckOnlineSafety(input, nullptr, control_state_.Chart, 0.0, 0.0,
          nullptr, nullptr, nullptr);
      body_checked = true;
      if (online_safety_.blocked() && !IsApaRecoverableObstacle(online_safety_.reason())) {
        FailRecovery(online_safety_.reason()); planning_changed = true;
      }
    }
    if (fault_.empty()) planning_changed = UpdatePlanning(input, now) || planning_changed;
  }
  const bool same_frame = have_feedback_ && input.carla_frame_id() == last_feedback_frame_ &&
      input.sim_time_sec() == last_feedback_time_;
  if (same_frame && !safety_changed &&
      !planning_changed && (had_input_fault || input_error_.empty())) {

    auto replay = std::make_shared<pangu::modules::ApaOutput>(cached_output_);
    *replay->mutable_input_validation() = validation;
    PUB_MSG(apa_output, replay);
    return true;
  }

  auto output = std::make_shared<pangu::modules::ApaOutput>();
  output->set_vehicle_profile_id(APA_VEHICLE_PROFILE_ID);
  output->set_frame_id(input.frame_id());
  output->set_carla_frame_id(input.carla_frame_id());
  output->set_sim_time_sec(input.sim_time_sec());
  output->set_session_id(input.session_id());
  output->set_task_id(input.task_id());
  output->set_planning_only(input.planning_only());
  *output->mutable_safety_state() = safety_state_;
  *output->mutable_input_validation() = validation;
  output->set_stop_req(true);
  output->set_target_accel_mps2(-1.0);
  output->set_target_gear_req(input.gear_pos());
  output->set_control_status("manual");

  const double step = input.cycle_time();
  const double elapsed = input.sim_time_sec() - last_feedback_time_;
  const double tolerance = 1.0e-5;
  const double ratio = control_period_sec_ / step;
  const bool clock_valid = !input.session_id().empty() && input.carla_frame_id() != 0 &&
      std::isfinite(input.sim_time_sec()) && input.sim_time_sec() >= 0.0 &&
      std::isfinite(step) && step > 0.0 && step <= control_period_sec_ + tolerance &&
      std::fabs(ratio - std::round(ratio)) < 1.0e-4 &&
      (!have_feedback_ || same_frame || (input.carla_frame_id() > last_feedback_frame_ &&
                          std::fabs(elapsed - step) < tolerance));

  if (parking_enabled_ && !clock_valid && fault_.empty()) FailRecovery("timing_error");
  motion_prediction_.measured = false;
  if (clock_valid && motion_history_valid_ && !same_frame && elapsed > 0.0) {
    const double dx = input.vehicle_x()-motion_x_, dy = input.vehicle_y()-motion_y_;
    const double c = std::cos(input.vehicle_yaw()), sn = std::sin(input.vehicle_yaw());
    motion_prediction_.measured = true;
    motion_prediction_.longitudinal_speed = (c*dx+sn*dy)/elapsed;
    motion_prediction_.lateral_speed = (-sn*dx+c*dy)/elapsed;
    motion_prediction_.yaw_rate = std::atan2(std::sin(input.vehicle_yaw()-motion_yaw_),
        std::cos(input.vehicle_yaw()-motion_yaw_))/elapsed;
  }
  motion_x_ = input.vehicle_x(); motion_y_ = input.vehicle_y(); motion_yaw_ = input.vehicle_yaw();
  motion_history_valid_ = clock_valid;
  last_feedback_frame_ = input.carla_frame_id();
  last_feedback_time_ = input.sim_time_sec();
  have_feedback_ = true;

  if (parking_enabled_ && apa_params_.enable_control() && fault_.empty() && validation.request_valid()) {
    output->set_control_status(replan_stage_ == ReplanStage::Stopping ? "replan_stopping" :
        replan_stage_ == ReplanStage::WaitingScene ? "replan_waiting" : "planning");
    if (has_plan_ && !input.planning_only()) {
      const double control_elapsed = input.sim_time_sec() - last_control_time_;
      if (last_control_time_ < 0.0 || std::fabs(control_elapsed - control_period_sec_) < tolerance) {
        RunControlStep(scene(), input, output.get());
        if (output->control_status() == "tracking_fault") {
          const int reason = output->control_diagnostic().stop_reason();
          if (reason == APA_MOCO_STOP_REFERENCE)
            BeginRecovery("tracking_deviation", input);
          else if (reason == APA_MOCO_STOP_ENDPOINT)
            BeginRecovery("endpoint_deviation", input);
          else if (reason == APA_MOCO_STOP_STALLED && motion_limit_.blocked())
            BeginRecovery("tracking_blocked", input);
          if (replan_stage_ == ReplanStage::Stopping)
            output->set_control_status("replan_stopping");

        }
      } else if (control_elapsed > control_period_sec_ + tolerance) {
        FailRecovery("timing_error");
      } else {

        *output = cached_output_;
        output->set_frame_id(input.frame_id());
        output->set_carla_frame_id(input.carla_frame_id());
        output->set_sim_time_sec(input.sim_time_sec());
        output->set_control_dt_sec(0.0);
        output->set_control_status("holding");
      }
    }
  }
  if (parking_enabled_ && fault_.empty() && !validation.request_valid())
    output->set_control_status("waiting_request");
  if (parking_enabled_ && fault_.empty() && (!body_checked || has_plan_)) {

    const double preview_speed = std::fabs(input.vehicle_speed());
    const bool following = has_plan_ && !input.planning_only() && control_state_.Chart.Phase != APA_MOCO_DONE &&
        control_state_.Chart.Phase != APA_MOCO_FAULT;
    motion_prediction_.commanded_curvature = std::tan(output->steer_rad())/task_control_param_.Lat.Wheelbase;
    online_safety_ = scene().CheckOnlineSafety(input, following ? &planner_output_ : nullptr,
        control_state_.Chart, output->control_diagnostic().stop_distance_m(), preview_speed,
        &execution_clearance_, &motion_prediction_, &motion_prediction_policy_);

    if (!online_safety_.blocked() && following && motion_limit_.blocked())
      online_safety_ = motion_limit_;
    online_safety_.set_plan_scene_version(plan_scene_version_);
    const bool braking = following && control_state_.Chart.Phase == APA_MOCO_STOP_WAIT &&
        (online_safety_.reason() == "clearance_predicted" || online_safety_.reason() == "motion_blocked");
    if (online_safety_.blocked()) {
      if (!braking && IsApaRecoverableObstacle(online_safety_.reason()) &&
          replan_stage_ == ReplanStage::Ready && following &&
          !(control_state_.Chart.Phase == APA_MOCO_STOP_WAIT &&
            control_state_.Chart.StpNow == control_state_.Chart.StpNum - 1))
        BeginRecovery(online_safety_.reason(), input);
      else if (!braking) FailRecovery(online_safety_.reason());
      std::fprintf(stderr, "[APA obstacle] task=%llu scene=%llu obstacle=%s reason=%s distance=%.3f\n",
          static_cast<unsigned long long>(task_id_), static_cast<unsigned long long>(input.scene().version()),
          online_safety_.obstacle_id().c_str(), online_safety_.reason().c_str(), online_safety_.checked_distance_m());
    } else if (has_plan_ && following) {
      if (execution_clearance_.info.enabled &&
          ApaPathProgress(planner_output_, control_state_.Chart) > execution_clearance_.info.prefix_length + 1.e-4) {
        execution_clearance_ = {};
        std::fprintf(stderr, "[APA clearance recovery] task=%llu restored normal margins\n",
            static_cast<unsigned long long>(task_id_));
      }
    }
    if (braking) {

      control_state_.Longi = {};
      output->set_enable(false); output->set_stop_req(true);
      output->set_target_speed_mps(0.0); output->set_target_accel_mps2(0.0);
      output->set_target_gear_req(input.gear_pos());
      output->set_direction_sign(input.gear_pos() == 2 ? -1 : input.gear_pos() == 1 ? 1 : 0);
      output->set_gear_change_allowed(false);
      output->set_control_status("motion_braking");
    }
  }
  if (fault_.empty() && (replan_stage_ == ReplanStage::Stopping || replan_stage_ == ReplanStage::WaitingScene ||
      replan_stage_ == ReplanStage::Planning)) {
    output->set_enable(false); output->set_stop_req(true);
    output->set_target_speed_mps(0.0); output->set_target_accel_mps2(0.0);
    output->set_steer_rad(input.vehicle_steering_command_rad());
    output->set_gear_change_allowed(false);
    output->set_control_dt_sec(0.0);
    output->set_control_status(replan_stage_ == ReplanStage::Stopping ? "replan_stopping" :
        replan_stage_ == ReplanStage::WaitingScene ? "replan_waiting" : "planning");
  }
  *output->mutable_online_safety() = online_safety_;

  if (!online_safety_.blocked()) output->mutable_online_safety()->set_scene_version(input.scene().version());
  if (!fault_.empty()) {
    output->set_enable(false);
    output->set_stop_req(true);
    output->set_target_speed_mps(0.0);
    output->set_target_accel_mps2(-1.0);
    output->set_steer_rad(0.0);
    output->set_control_status(fault_);
    if (!input_error_.empty()) output->mutable_input_validation()->set_error(input_error_);
    if (fault_ == "safety_stop") {
      output->set_target_accel_mps2(0.0);
      output->set_ctrl_phase(APA_MOCO_FAULT);
      output->mutable_control_diagnostic()->set_reference_valid(false);
    }
    if (fault_ == "obstacle_stop") {
      output->set_ctrl_phase(APA_MOCO_FAULT);
      output->mutable_control_diagnostic()->set_reference_valid(false);
    }
  }
  if (input.planning_only()) {

    output->set_enable(false); output->set_stop_req(true);
    output->set_target_speed_mps(0.0); output->set_target_accel_mps2(0.0);
    output->set_steer_rad(0.0);
    output->set_target_gear_req(APA_GEAR_POS_P);
    output->set_gear_change_allowed(true);
    output->set_control_dt_sec(0.0);
    if (fault_.empty() && has_plan_ && replan_stage_ == ReplanStage::Ready)
      output->set_control_status("plan_ready");
  }

  ApaMocoActuatorInput_t actuator_input{};
  actuator_input.VehicleSpeed=input.vehicle_speed();
  actuator_input.VehicleMotionSpeed=input.vehicle_motion_speed();
  actuator_input.VehicleAcceleration=input.vehicle_acceleration();
  actuator_input.GravityAcceleration=input.road_gravity_x()*std::cos(input.vehicle_yaw()) +
      input.road_gravity_y()*std::sin(input.vehicle_yaw());
  actuator_input.TargetSpeed=output->target_speed_mps();
  actuator_input.TargetAcceleration=output->target_accel_mps2();
  actuator_input.SpeedLimit=std::max(task_control_param_.Longi.SpeedUpperLimit,
      -task_control_param_.Longi.SpeedLowerLimit);
  actuator_input.CycleTime=input.cycle_time();
  actuator_input.FeedbackGear=input.gear_pos();
  actuator_input.TargetGear=output->target_gear_req();
  actuator_input.Enable=output->enable();
  actuator_input.StopReq=output->stop_req();
  actuator_input.GearChangeAllowed=output->gear_change_allowed();
  ApaMocoActuatorOutput_t pedals{};
  apaMocoActuate(&actuator_input,&actuator_state_,&pedals);
  if (pedals.Overspeed && parking_enabled_) {
    fault_="tracking_fault";
    output->set_enable(false); output->set_stop_req(true);
    output->set_target_speed_mps(0); output->set_target_accel_mps2(0);
    output->set_steer_rad(0); output->set_gear_change_allowed(false);
    output->set_ctrl_phase(APA_MOCO_FAULT); output->set_control_status(fault_);
    output->mutable_control_diagnostic()->set_stop_reason(APA_MOCO_STOP_OVERSPEED);
    output->mutable_control_diagnostic()->set_reference_valid(false);
  }
  auto* actuator=output->mutable_actuator();
  actuator->set_owner("C:apaMocoActuate:v2");
  actuator->set_launch_hold(pedals.LaunchHold);
  actuator->set_throttle(pedals.Throttle); actuator->set_brake(pedals.Brake);
  actuator->set_hand_brake(pedals.HandBrake); actuator->set_gear_pos(pedals.Gear);
  auto* replan = output->mutable_replan();
  *output->mutable_actuation_limits() = DescribeApaActuationLimits(task_control_param_);
  replan->set_recovery_id(recovery_id_); replan->set_path_version(path_version_);
  replan->set_attempt(planning_attempts_); replan->set_reason(recovery_reason_);
  replan->set_planner_error_type(planner_error_type_);
  replan->set_state(replan_stage_ == ReplanStage::Idle ? "idle" : replan_stage_ == ReplanStage::Stopping ? "stopping" :
      replan_stage_ == ReplanStage::Planning ? "planning" : replan_stage_ == ReplanStage::WaitingScene ? "waiting_scene" :
      replan_stage_ == ReplanStage::Ready ? "ready" : "failed");
  if (recovering || replan_stage_ == ReplanStage::Stopping || replan_stage_ == ReplanStage::Planning ||
      replan_stage_ == ReplanStage::WaitingScene)
    replan->set_elapsed_sec(std::max(0.0, std::chrono::duration<double>(now-recovery_started_).count()));
  output->set_control_step_count(control_step_count_);
  output->clear_path_point_x();
  output->clear_path_point_y();
  output->clear_path_point_yaw();
  output->clear_path_point_odo();
  if (has_plan_) {
    const int count = ClampPathCount(planner_output_.pathPointCount);
    for (int i = 0; i < count; ++i) {
      output->add_path_point_x(planner_output_.pathPointX[i]);
      output->add_path_point_y(planner_output_.pathPointY[i]);
      output->add_path_point_yaw(planner_output_.pathPointTheta[i]);
      output->add_path_point_odo(planner_output_.pathPointOdometer[i]);
    }
  }
  cached_output_ = *output;
  PUB_MSG(apa_output, output);

  if (last_clock_status_ != output->control_status() || last_clock_log_time_ < 0.0 ||
      input.sim_time_sec() - last_clock_log_time_ >= 0.2) {
    std::fprintf(stderr,
        "[APA clock] session=%s task=%llu frame=%llu sim_time=%.6f dt=%.6f steps=%llu status=%s\n",
        session_id_.c_str(), static_cast<unsigned long long>(task_id_),
        static_cast<unsigned long long>(input.carla_frame_id()), input.sim_time_sec(),
        output->control_dt_sec(), static_cast<unsigned long long>(control_step_count_),
        output->control_status().c_str());
    last_clock_log_time_ = input.sim_time_sec();
    last_clock_status_ = output->control_status();
    PLOG_INFO("APA clock: session={}, task={}, frame={}, sim_time={}, dt={}, steps={}, status={}",
              session_id_, task_id_, input.carla_frame_id(), input.sim_time_sec(),
              output->control_dt_sec(), control_step_count_, output->control_status());
  }
  return true;
}

void apa::apa_inputCallback(const pangu::modules::ApaInput& input) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (input_revision_ != 0 && input.session_id() == latest_input_.session_id()) {
    if (input.task_id() < latest_input_.task_id()) return;
    if (input.task_id() == latest_input_.task_id() &&
        input.carla_frame_id() < latest_input_.carla_frame_id()) return;
  }
  const auto previous_safety = latest_input_.safety_state();
  const auto previous_validation = latest_input_.input_validation();
  const bool keep_input_error = input.session_id() == latest_input_.session_id() &&
      input.task_id() == latest_input_.task_id() && !previous_validation.error().empty();
  const bool keep_safety = input.session_id() == latest_input_.session_id() &&
      input.task_id() == latest_input_.task_id() && !previous_safety.reason().empty() &&
      (input.safety_state().reason().empty() ||
       input.safety_state().event_id() < previous_safety.event_id());
  latest_input_ = input;
  if (keep_safety) *latest_input_.mutable_safety_state() = previous_safety;
  if (keep_input_error) latest_input_.mutable_input_validation()->set_error(previous_validation.error());
  ++input_revision_;
}

DEFINE_CALLBACK_HEAD(apa, apa_input, pangu::modules::ApaInput) {
  std::string inputer;
  auto msg = GET_TRIGGER_MSG_DATA_PROTO(inputer, pangu::modules::ApaInput);
  if (msg == nullptr) {
    PLOG_WARN("apa_input callback got null message from {}", inputer);
    return;
  }

  apa_inputCallback(*msg);
}

}
}
