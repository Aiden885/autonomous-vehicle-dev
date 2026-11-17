#pragma once

#include <string>
#include <vector>

namespace pangu::modules {

// IPC carries only this descriptor; the receiver derives allowances from its snapshot.
struct ApaClearanceRecoveryInfo {
  bool enabled = false;
  double origin_x = 0.0, origin_y = 0.0, origin_yaw = 0.0;
  double prefix_length = 0.0;
};

struct ApaClearanceLimit {
  std::string geometry_key;
  double minimum_gap = 0.0;
  // Frozen origin gap; allowances cannot accumulate across samples.
};

struct ApaClearanceRecovery {
  ApaClearanceRecoveryInfo info;
  std::vector<ApaClearanceLimit> limits;
};

}
