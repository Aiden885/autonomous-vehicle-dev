#pragma once

extern "C" {
#include "apaMocoBraking.h"
}
#include <array>
#include <vector>

namespace pangu::modules {
// Same-frame rear-axle motion in vehicle coordinates; stop distance includes braking delay.
struct ApaMotionPrediction {
  bool measured = false;
  double longitudinal_speed = 0.0;
  double lateral_speed = 0.0;
  double yaw_rate = 0.0;
  double commanded_curvature = 0.0;
  bool response_reference_valid = false;
  double reference_curvature = 0.0;
  double steering_angle = 0.0;
};

struct ApaMotionPose { double x, y, yaw; };
// Borrowed arrays have count elements; the owner validates capacity.
struct ApaMotionPathView {
  const Real *x = nullptr, *y = nullptr, *yaw = nullptr, *odometer = nullptr;
  int count = 0, direction = 0;
  double progress = 0.0;
};
struct ApaMotionPredictionPolicy {
  ApaMocoBrakingProfile_t braking{};
  double wheelbase = 0.0, minimum_steer = 0.0, maximum_steer = 0.0;
  double steering_response_time = 0.0; // Nominal first-order response, s.
  double steer_rate_per_sec = 0.0; // Zero means no configured rate limit.
  // Guidance response travel; zero for actual-speed stopping queries.
  double speed_response_time = 0.0;
  bool follow_path_direction = false;
};
ApaMotionPose AdvanceApaMotionPose(ApaMotionPose origin,
    double longitudinal, double lateral, double angle);

// Prediction starts at the actual pose with measured drift and steering response.
// It stays in the active gear and retains terminal curvature on overrun.
class ApaTrackingMotionPredictor {
 public:
  ApaTrackingMotionPredictor(ApaMotionPose origin, const ApaMotionPathView& path,
      const ApaMotionPredictionPolicy& policy, const ApaMotionPrediction& motion,
      double signed_speed);
  bool valid() const { return valid_; }
  int model_count() const { return count_; }
  double maximum_angular_motion() const { return angular_bound_; }
  const std::array<ApaMotionPose, 2>& advance(double distance);
  double path_travel(int model) const;
 private:
  struct Section { double end, curvature; };
  struct Model {
    double longitudinal = 0.0, lateral = 0.0, yaw_residual = 0.0;
    double steering = 0.0;
    bool follows_reference = false;
  };
  double reference_curvature(double distance) const;
  double distance_at_time(double time) const;
  double time_at_distance(double distance) const;
  ApaMotionPredictionPolicy policy_;
  std::vector<Section> sections_;
  std::array<Model, 2> models_{};
  std::array<ApaMotionPose, 2> poses_{};
  double progress_ = 0.0, distance_ = 0.0, time_ = 0.0;
  double correction_ = 0.0, initial_steering_ = 0.0, angular_bound_ = 0.0;
  int count_ = 0;
  bool valid_ = false;
};
}
