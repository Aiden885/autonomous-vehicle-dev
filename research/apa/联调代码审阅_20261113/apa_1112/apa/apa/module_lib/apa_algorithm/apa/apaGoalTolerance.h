#ifndef APA_GOAL_TOLERANCE_H
#define APA_GOAL_TOLERANCE_H

#include "baseTypes.h"
#include "apaVehicleProfileGenerated.h"
#include <math.h>

/* Shared defaults for standalone C planning and control. Per-task values are
 * supplied by the APA adapter after combining configuration and request. */
#define APA_GOAL_POSITION_TOLERANCE_M APA_DEFAULT_GOAL_POSITION_TOLERANCE_M
#define APA_GOAL_YAW_TOLERANCE_RAD APA_DEFAULT_GOAL_YAW_TOLERANCE_RAD

static inline Boolean apaGoalToleranceValid(Real value) {
  return isfinite(value) && value >= 0.0f;
}

static inline Real apaGoalPositionTolerance(Real value) {
  return value == 0.0f ? APA_GOAL_POSITION_TOLERANCE_M : value;
}

static inline Real apaGoalYawTolerance(Real value) {
  return value == 0.0f ? APA_GOAL_YAW_TOLERANCE_RAD : value;
}

#endif
