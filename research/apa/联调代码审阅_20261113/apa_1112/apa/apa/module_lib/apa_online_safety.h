#pragma once
extern "C" {
#include "apaMocoCtrlChartV2Types.h"
#include "apaVirtualRecovery.h"
}

#include "apa_config.pb.h"
#include "ApaParkingPathPlanningTypes.hpp"
#include "apa_clearance_recovery.h"
#include "apa_motion_prediction.h"
#include <string>
#include <vector>
#include <memory>
#include <array>
#include <limits>
#include <cstdint>

namespace pangu::modules {
struct ApaCandidateSafetyObservation;
inline constexpr double kApaObstaclePredictionSec = 1.5;
struct ApaScenePoint { double x, y; };
std::vector<ApaScenePoint> ApaObstacleSweptFootprint(const ApaObstacle& obstacle);
VehicleCollisionCheckInput_t BuildApaPlanningEnvironment(const ApaInput& input);
double ApaPathProgress(const ::apa::ApaParkingPathPlanningOutputData& path,
                       const ApaMocoCtrlChartV2State_t& chart);

// Query/context are local to one worker request; neither crosses IPC.
class ApaPlannerScene {
 public:
  explicit ApaPlannerScene(const ApaInput& snapshot);
  ~ApaPlannerScene();
  ApaPlanningSceneQuery_t query();
  size_t pose_queries() const;
  size_t obstacle_hits() const;
  const std::string& last_obstacle_id() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
inline bool IsApaRecoverableObstacle(const std::string& reason) {
  return reason == "path_blocked" || reason == "body_predicted" || reason == "planning_clearance" ||
      reason == "clearance_deficit" || reason == "clearance_predicted" || reason == "motion_blocked" ||
      reason == "clearance_recovery_deviation";
}

// Caller owns task/path admission. Active motion requires a same-step prediction policy.
// Segment and progress are owned by the controller.
ApaOnlineSafety CheckApaOnlineSafety(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData* path,
    const ApaMocoCtrlChartV2State_t& chart, double stop_distance, double preview_speed,
    ApaClearanceRecovery* recovery = nullptr,
    const ApaMotionPrediction* motion = nullptr,
    const ApaMotionPredictionPolicy* prediction_policy = nullptr,
    ApaCandidateSafetyObservation* observation = nullptr);

ApaOnlineSafety CheckApaCandidatePath(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path,
    const ApaClearanceRecovery* recovery = nullptr,
    Real goal_position_tolerance = 0.0f, Real goal_yaw_tolerance = 0.0f);

// Diagnostic only; pose_valid is false when no geometric check occurred.
struct ApaCandidateSafetyObservation {
  bool pose_valid = false;
  double x = 0.0, y = 0.0, yaw = 0.0, sample_distance_m = 0.0;
  std::string stage;
  // -1: current pose; -2: reference path; 0/1: commanded/measured model.
  int prediction_model = -1;
  std::string obstacle_kind;
  // Unsigned body gap: 0 = contact/overlap; NaN = no matching scene geometry.
  double current_gap_m = std::numeric_limits<double>::quiet_NaN();
  double checked_gap_m = std::numeric_limits<double>::quiet_NaN();
  double checked_swept_gap_m = std::numeric_limits<double>::quiet_NaN();
  std::array<double, 3> margins_m{};
  double execution_gap_floor_m = std::numeric_limits<double>::quiet_NaN();
};
ApaOnlineSafety CheckApaCandidatePathObserved(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path,
    ApaCandidateSafetyObservation* observation,
    const ApaClearanceRecovery* recovery = nullptr,
    Real goal_position_tolerance = 0.0f, Real goal_yaw_tolerance = 0.0f);

ApaOnlineSafety CheckApaPlanningStart(const ApaInput& input);

// Zero-margin hard contact only, including slot centerlines.
ApaOnlineSafety CheckApaBody(const ApaInput& input);
struct ApaAvailableMotion {
  double distance = 0.0;
  ApaOnlineSafety obstacle;
};

// Frozen request geometry. Observations may change only vehicle pose, speed,
// acceleration and gear; live callers require a fresh scene each frame.
class ApaSafetyScene {
 public:
  explicit ApaSafetyScene(const ApaInput& snapshot);
  ~ApaSafetyScene();
  ApaOnlineSafety CheckPlanningStart(const ApaInput& observation) const;
  ApaOnlineSafety CheckTrackingPose(const ApaInput& observation,
      const ::apa::ApaParkingPathPlanningOutputData& path,
      const ApaMocoCtrlChartV2State_t& chart) const;
  ApaAvailableMotion CheckAvailableMotion(const ApaInput& observation,
      const ::apa::ApaParkingPathPlanningOutputData& path,
      const ApaMocoCtrlChartV2State_t& chart,
      const ApaMotionPredictionPolicy& policy, const ApaMotionPrediction& motion,
      const ApaClearanceRecovery* recovery, double horizon = 1.0,
      ApaCandidateSafetyObservation* diagnostic = nullptr) const;
  ApaOnlineSafety CheckOnlineSafety(const ApaInput& observation,
      const ::apa::ApaParkingPathPlanningOutputData* path,
      const ApaMocoCtrlChartV2State_t& chart, double stop_distance, double preview_speed,
      ApaClearanceRecovery* recovery, const ApaMotionPrediction* motion,
      const ApaMotionPredictionPolicy* prediction_policy,
      ApaCandidateSafetyObservation* diagnostic = nullptr) const;
 private:
  struct Impl;
  std::unique_ptr<const Impl> impl_;
};
ApaAvailableMotion CheckApaAvailableMotion(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path,
    const ApaMocoCtrlChartV2State_t& chart,
    const ApaMotionPredictionPolicy& policy, const ApaMotionPrediction& motion,
    const ApaClearanceRecovery* recovery, double horizon = 1.0);
ApaClearanceRecovery PrepareApaClearanceRecovery(const ApaInput& input);

// Origin-relative bounded recovery; prefers a 15 cm join gap.
// target_reached ranks this request only and does not cross IPC.
bool BuildApaClearancePrefix(const ApaInput& input, const ApaClearanceRecovery& recovery,
    double curvature, int direction, ::apa::ApaParkingPathPlanningOutputData* prefix,
    bool* target_reached = nullptr, double* join_gap = nullptr);

}
