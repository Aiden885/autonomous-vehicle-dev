#pragma once

#include "ApaRecoveryDecisionTypes.hpp"
#include "apa_config.pb.h"
#include "apa_online_safety.h"

#include <string>
#include <limits>

namespace pangu::modules {

// Attempts are recovery-local; repeated recoveries require error improvement
// measured in arrival-tolerance units.
struct ApaRecoveryProgress {
  double best_error = std::numeric_limits<double>::infinity();
  unsigned stagnant_recoveries = 0;
  bool Begin(double error, unsigned limit);
};

struct ApaRecoveryAssessment {
  ::apa::ApaRecoveryRisk risk = ::apa::ApaRecoveryRisk::Clear;
  bool blocker_changed = false;
  double blocker_speed_mps = 0.0;
};

// Geometry comparisons use C float precision without spatial rounding.
class ApaRecoveryContext {
 public:
  void Reset();
  void ForgetAttempt();
  void RecordAttempt(const ApaInput& input);
  bool ProblemChanged(const ApaInput& input) const;
  bool HasAttempt() const { return has_attempt_; }
  ApaRecoveryAssessment Assess(const ApaInput& input, const ApaOnlineSafety& safety);

 private:
  bool has_attempt_ = false;
  float planned_x_ = 0.0f, planned_y_ = 0.0f, planned_yaw_ = 0.0f;
  ApaScene planned_scene_;
  std::string blocker_id_;
  ApaObstacle blocker_;
};

const char* ApaRecoveryFailureReason(::apa::ApaRecoveryFailure failure);

}
