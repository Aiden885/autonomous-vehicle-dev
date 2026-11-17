#include "apa_recovery_context.h"

#include <cmath>
#include "apaMocoExecution.h"

namespace pangu::modules {
namespace {
bool SameGeometry(const ApaObstacle& left, const ApaObstacle& right) {
  if (left.id() != right.id() || left.footprint_size() != right.footprint_size()) return false;
  for (int i = 0; i < left.footprint_size(); ++i) {
    if (static_cast<float>(left.footprint(i).x()) != static_cast<float>(right.footprint(i).x()) ||
        static_cast<float>(left.footprint(i).y()) != static_cast<float>(right.footprint(i).y())) return false;
  }
  return true;
}

bool HasPrefix(const std::string& value, const char* prefix) {
  return value.compare(0, std::char_traits<char>::length(prefix), prefix) == 0;
}
}

bool ApaRecoveryProgress::Begin(double error, unsigned limit) {
  if (error + 1.0 < best_error) {
    best_error = error;
    stagnant_recoveries = 0;
  } else ++stagnant_recoveries;
  return stagnant_recoveries < limit;
}

void ApaRecoveryContext::Reset() {
  ForgetAttempt();
  blocker_id_.clear();
  blocker_.Clear();
}

void ApaRecoveryContext::ForgetAttempt() {
  has_attempt_ = false;
  planned_scene_.Clear();
}

void ApaRecoveryContext::RecordAttempt(const ApaInput& input) {
  planned_x_ = static_cast<float>(input.vehicle_x());
  planned_y_ = static_cast<float>(input.vehicle_y());
  planned_yaw_ = static_cast<float>(input.vehicle_yaw());
  planned_scene_ = input.scene();
  has_attempt_ = true;
}

bool ApaRecoveryContext::ProblemChanged(const ApaInput& input) const {

  if (!has_attempt_ || planned_x_ != static_cast<float>(input.vehicle_x()) ||
      planned_y_ != static_cast<float>(input.vehicle_y()) ||
      planned_yaw_ != static_cast<float>(input.vehicle_yaw()) ||
      planned_scene_.obstacles_size() != input.scene().obstacles_size()) return true;
  for (int i = 0; i < planned_scene_.obstacles_size(); ++i) {
    const auto& previous = planned_scene_.obstacles(i);
    const auto& current = input.scene().obstacles(i);

    if (!SameGeometry(previous, current) || previous.velocity_x() != current.velocity_x() ||
        previous.velocity_y() != current.velocity_y()) return true;
  }
  return false;
}

ApaRecoveryAssessment ApaRecoveryContext::Assess(const ApaInput& input,
    const ApaOnlineSafety& safety) {
  ApaRecoveryAssessment result;
  result.blocker_changed = blocker_id_ != safety.obstacle_id();
  blocker_id_ = safety.obstacle_id();
  if (!safety.blocked()) {
    blocker_.Clear();
    return result;
  }
  using Risk = ::apa::ApaRecoveryRisk;
  if (safety.reason() == "body_blocked" || safety.reason() == "line_contact") result.risk = Risk::PhysicalOverlap;
  else if (safety.reason() == "body_predicted") result.risk = Risk::PredictedIntrusion;
  else if (safety.reason() == "path_blocked" || safety.reason() == "clearance_deficit" ||
           safety.reason() == "clearance_predicted" || safety.reason() == "motion_blocked" ||
           safety.reason() == "clearance_recovery_deviation") result.risk = Risk::PathBlocked;
  else if (safety.reason() == "planning_clearance") {
    result.risk = Risk::Invalid;
    for (const auto& obstacle : input.scene().obstacles()) {
      if (obstacle.id() != safety.obstacle_id()) continue;
      result.blocker_changed = result.blocker_changed || !SameGeometry(blocker_, obstacle);
      blocker_ = obstacle;
      result.risk = Risk::ActorStartConstraint;
      result.blocker_speed_mps = std::hypot(obstacle.velocity_x(), obstacle.velocity_y());
      return result;
    }

    if (HasPrefix(safety.obstacle_id(), "json:virtual:")) result.risk = Risk::VirtualStartConstraint;
    else if (HasPrefix(safety.obstacle_id(), "json:boundary:") ||
             HasPrefix(safety.obstacle_id(), "json:point:") ||
             HasPrefix(safety.obstacle_id(), "json:slot_stopper:") ||
             HasPrefix(safety.obstacle_id(), "json:stopper:")) result.risk = Risk::FixedStartConstraint;
  } else result.risk = Risk::Invalid;
  blocker_.Clear();
  return result;
}

const char* ApaRecoveryFailureReason(::apa::ApaRecoveryFailure failure) {
  using Failure = ::apa::ApaRecoveryFailure;
  switch (failure) {
    case Failure::None: return "";
    case Failure::RecoveryTimeout: return "recovery_timeout";
    case Failure::ReplanExhausted: return "replan_exhausted";
    case Failure::VirtualStartRequiresEscape: return "virtual_start_requires_escape";
    case Failure::FixedStartBlocked: return "fixed_start_blocked";
    case Failure::StationaryStartBlocked: return "stationary_start_blocked";
    case Failure::PhysicalOverlap: return "body_blocked";
    case Failure::InvalidSafety: return "invalid_recovery_safety";
    case Failure::ObstacleWaitTimeout: return "obstacle_wait_timeout";
    case Failure::UnchangedProblem: return "unchanged_planning_problem";
  }
  return "invalid_recovery_decision";
}

}
