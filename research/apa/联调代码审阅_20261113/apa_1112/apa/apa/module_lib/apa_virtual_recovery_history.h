#pragma once

#include "apa_online_safety.h"

#include <deque>

namespace pangu::modules {

// Task/session-local measured poses with normal admission only.
// Recovery-admitted poses cannot widen the exemption.
class ApaVirtualRecoveryHistory {
 public:
  void Reset();
  void Remember(const ApaInput& input);
  bool Prepare(const ApaInput& input, ApaVirtualRecoveryContext_t* context) const;

 private:
  struct Pose {
    double x, y, yaw, sim_time;
  };
  std::deque<Pose> poses_;
};

}
