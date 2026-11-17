#ifndef APA_COUNT_UTILS_H
#define APA_COUNT_UTILS_H

#include "baseTypes.h"

/**
 * @brief Clamp a non-negative item count to an inclusive upper bound.
 *
 * The caller must provide a non-negative maximum count.
 */
static inline Integer apaClampCount(Integer count, Integer maxCount) {
  Integer result0;

  result0 = count;
  if (result0 < 0) {
    result0 = 0;
  }
  if (result0 > maxCount) {
    result0 = maxCount;
  }
  return result0;
}

/** @brief Clamp a non-negative item count to both inclusive upper bounds. */
static inline Integer apaClampCountToLimits(Integer count, Integer firstMax, Integer secondMax) {
  Integer result0;

  result0 = apaClampCount(count, firstMax);
  result0 = apaClampCount(result0, secondMax);
  return result0;
}

#endif
