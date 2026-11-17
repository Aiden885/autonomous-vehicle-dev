#include "apa_virtual_recovery_history.h"

#include <cmath>
#include <cstdio>

namespace pangu::modules {

void ApaVirtualRecoveryHistory::Reset() { poses_.clear(); }

void ApaVirtualRecoveryHistory::Remember(const ApaInput& input) {
  if (!poses_.empty() && std::hypot(input.vehicle_x() - poses_.back().x,
      input.vehicle_y() - poses_.back().y) < 0.04) return;
  poses_.push_back({input.vehicle_x(), input.vehicle_y(), input.vehicle_yaw(), input.sim_time_sec()});
  while (poses_.size() > 40) poses_.pop_front();
}

bool ApaVirtualRecoveryHistory::Prepare(const ApaInput& input,
    ApaVirtualRecoveryContext_t* context) const {
  *context = {};
  if (poses_.empty()) {
    std::fprintf(stderr, "[APA escape history] eligible=0 reason=no_safe_history origin=(%.3f,%.3f,%.6f)\n",
        input.vehicle_x(), input.vehicle_y(), input.vehicle_yaw());
    return false;
  }
  auto environment = BuildApaPlanningEnvironment(input);
  ApaVirtualRecoveryParam_t param{};
  apaVirtualRecoveryDefaultParam(&param);
  ApaPlannerScene scene(input);
  const auto query = scene.query();
  size_t candidates = 0;
  for (auto it = poses_.rbegin(); it != poses_.rend(); ++it) {
    const double age = input.sim_time_sec() - it->sim_time;
    if (age < 0.0 || age > 10.0 ||
        std::hypot(input.vehicle_x() - it->x, input.vehicle_y() - it->y) > param.MaxLength) continue;
    ++candidates;
    if (apaVirtualRecoveryInitialize(&environment, it->x, it->y, it->yaw, &param, &query, context)) {
      std::fprintf(stderr,
          "[APA escape history] eligible=1 poses=%zu candidates=%zu origin=(%.3f,%.3f,%.6f) anchor=(%.3f,%.3f)\n",
          poses_.size(), candidates, environment.VehPosX, environment.VehPosY, environment.VehPosTheta,
          it->x, it->y);
      return true;
    }
  }
  std::fprintf(stderr,
      "[APA escape history] eligible=0 reason=%s poses=%zu candidates=%zu origin=(%.3f,%.3f,%.6f) "
      "scene_queries=%zu scene_hits=%zu scene_obstacle=%s\n",
      candidates == 0 ? "no_recent_anchor" : "anchor_validation_failed", poses_.size(), candidates,
      environment.VehPosX, environment.VehPosY, environment.VehPosTheta,
      scene.pose_queries(), scene.obstacle_hits(), scene.last_obstacle_id().c_str());
  *context = {};
  return false;
}

}
