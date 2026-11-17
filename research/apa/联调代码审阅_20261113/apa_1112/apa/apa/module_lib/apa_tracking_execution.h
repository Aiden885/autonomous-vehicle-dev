#pragma once

#include "apa_online_safety.h"
extern "C" {
#include "apaMocoParkingControl.h"
}

namespace pangu::modules {
ApaMotionPredictionPolicy BuildApaMotionPredictionPolicy(Real speed,
    Real acceleration, Real previous_acceleration,
    const ApaMocoParkingControlParam_t& calibration);
struct ApaTrackingStep {
  ApaMocoParkingControlOutput_t control{};
  ApaMotionPredictionPolicy prediction_policy{};
  ApaMotionPrediction prediction{};
  // Persistent guidance blockage; routine slowing does not set this flag.
  ApaOnlineSafety motion_limit;
  ApaCandidateSafetyObservation motion_observation;
};

// Shared live/replay C step; caller owns time, motion and accepted-path identity.
ApaTrackingStep StepApaTracking(const ApaSafetyScene& scene, const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path, int path_update,
    const ApaMocoParkingControlParam_t& calibration,
    ApaMocoParkingControlState_t* state,
    const ApaClearanceRecovery* recovery = nullptr,
    const ApaMotionPrediction* motion = nullptr);
ApaTrackingStep StepApaTracking(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path, int path_update,
    const ApaMocoParkingControlParam_t& calibration,
    ApaMocoParkingControlState_t* state,
    const ApaClearanceRecovery* recovery = nullptr,
    const ApaMotionPrediction* motion = nullptr);

// Nominal bicycle-model candidate check with task calibration and motion limits.
bool CheckApaTrackingExecution(const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path,
    const ApaMocoParkingControlParam_t& calibration,
    const ApaClearanceRecovery* recovery = nullptr);
bool CheckApaTrackingExecution(const ApaSafetyScene& scene, const ApaInput& input,
    const ::apa::ApaParkingPathPlanningOutputData& path,
    const ApaMocoParkingControlParam_t& calibration,
    const ApaClearanceRecovery* recovery = nullptr);
}
