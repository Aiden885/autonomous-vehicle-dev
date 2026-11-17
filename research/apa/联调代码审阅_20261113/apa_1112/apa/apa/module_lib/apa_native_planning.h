#pragma once

#include "ApaNativePlanning.hpp"
extern "C" {
#include "apaParkingSlotPathPlanning.h"
#include "apaParkingSearchFallback.h"
#include "apaVirtualRecovery.h"
}

namespace apa::native {

/** A nonzero ID is valid only during this scope, on this thread. */
class SceneScope final {
 public:
  explicit SceneScope(ApaPlanningSceneQuery_t query);
  ~SceneScope();
  SceneScope(const SceneScope&) = delete;
  SceneScope& operator=(const SceneScope&) = delete;
  UInteger id() const noexcept { return id_; }
 private:
  UInteger id_;
};
ApaRecoveryEnvironment ToModel(const VehicleCollisionCheckInput_t& source);
VehicleCollisionCheckInput_t ToNative(const ApaRecoveryEnvironment& source);
ApaParkingManeuverAssessment_t AssessPath(const ApaParkingPathPlanningInputData& input,
    const ApaParkingPathPlanningOutputData& path, Real reserve);
ApaVirtualRecoveryContext ToModel(const ApaVirtualRecoveryContext_t& source);
ApaVirtualRecoveryContext_t ToNative(const ApaVirtualRecoveryContext& source);
ApaVirtualRecoveryPrefix ToModel(const ApaVirtualRecoveryPrefix_t& source);
ApaVirtualRecoveryPrefix_t ToNative(const ApaVirtualRecoveryPrefix& source);
}
