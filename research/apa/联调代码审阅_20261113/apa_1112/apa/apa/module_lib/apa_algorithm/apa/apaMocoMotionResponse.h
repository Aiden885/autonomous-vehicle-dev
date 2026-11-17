#ifndef APA_MOCO_MOTION_RESPONSE_H
#define APA_MOCO_MOTION_RESPONSE_H

#include "baseTypes.h"
#include <math.h>

/* Nominal road-wheel response shared by control, safety and offline execution.
 * Zero response time represents an ideal actuator for comparison. */
static inline Real
apaMocoSteeringResponse(Real angle, Real command, Real responseTime, Real cycleTime) {
  return responseTime > 0.0f ? angle + (command - angle) * (1.0f - expf(-cycleTime / responseTime))
                             : command;
}
#endif
