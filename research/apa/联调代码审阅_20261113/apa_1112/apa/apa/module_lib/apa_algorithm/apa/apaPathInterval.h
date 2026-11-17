#ifndef APA_PATH_INTERVAL_H
#define APA_PATH_INTERVAL_H

#include "apaCoordUtils.h"
#include "baseTypes.h"
#include <math.h>

/* Odo belongs to the outgoing interval. Allow chord/arc and float rounding,
 * but never join distant points using a short or oppositely signed step. */
static inline Boolean
apaPathIntervalIsConsistent(Real x0, Real y0, Real yaw0, Real odo, Real x1, Real y1, Real yaw1) {
  Real dx = x1 - x0, dy = y1 - y0;
  Real distance = hypotf(dx, dy);
  Real midYaw = yaw0 + 0.5f * apaCoordWrapToHalfOpenPi(yaw1 - yaw0);
  Real along = dx * cosf(midYaw) + dy * sinf(midYaw);
  return isfinite(distance) && isfinite(along) && isfinite(odo) &&
         fabsf(distance - fabsf(odo)) <= 0.02f + 0.02f * distance && along * odo >= -1.0e-6f;
}

#endif
