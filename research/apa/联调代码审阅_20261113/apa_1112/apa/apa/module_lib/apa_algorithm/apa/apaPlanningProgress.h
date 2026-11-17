#ifndef APA_PLANNING_PROGRESS_H
#define APA_PLANNING_PROGRESS_H

#include "apaMathUtils.h"
#include <math.h>

/* Numerical motion progress, not an executable minimum or a short-run rule.
 * A partial collision result with zero available length cannot advance a
 * geometric maneuver. Keep the existing branch failure/fallback contract. */
static inline Boolean apaPlanningMotionHasProgress(Real distance) {
  return isfinite(distance) && fabsf(distance) > 1.e-4f;
}

#endif
