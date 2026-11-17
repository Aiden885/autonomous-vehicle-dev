#pragma once

namespace apa {

enum class ApaRecoveryRisk {
  Clear,
  PathBlocked,
  PredictedIntrusion,
  ActorStartConstraint,
  FixedStartConstraint,
  VirtualStartConstraint,
  PhysicalOverlap,
  Invalid
};

enum class ApaRecoveryAction { Hold, Observe, WaitForScene, PollPlanner, Plan, Fail };

enum class ApaRecoveryFailure {
  None,
  RecoveryTimeout,
  ReplanExhausted,
  VirtualStartRequiresEscape,
  FixedStartBlocked,
  StationaryStartBlocked,
  PhysicalOverlap,
  InvalidSafety,
  ObstacleWaitTimeout,
  UnchangedProblem
};

}
