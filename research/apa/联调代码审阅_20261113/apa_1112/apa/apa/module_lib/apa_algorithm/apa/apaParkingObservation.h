#ifndef APA_PARKING_OBSERVATION_H
#define APA_PARKING_OBSERVATION_H

#include "apaParkingPathPlanningTypes.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Content ID, independent of enumeration order and struct padding. The ID
 * describes the sampled path, not its scene; correlate with the worker log. */
static inline uint64_t apaParkingPathId(const ApaParkingPathPlanningOutput_t *path) {
  uint64_t hash = UINT64_C(14695981039346656037);
  const Real *fields[] = {path->PthPtX, path->PthPtY, path->PthPtTht, path->PthPtOdo};
  for (Integer i = -1; i < path->PtCount; ++i) {
    for (Integer field = 0; field < (i < 0 ? 1 : 4); ++field) {
      uint32_t bits = (uint32_t)path->PtCount;
      if (i >= 0)
        memcpy(&bits, &fields[field][i], sizeof(bits));
      for (unsigned byte = 0; byte < 4; ++byte) {
        hash ^= (bits >> (byte * 8)) & 255u;
        hash *= UINT64_C(1099511628211);
      }
    }
  }
  return hash;
}

/* Diagnostic threshold only. Merge adjacent edges with the same direction,
 * including across curvature changes; ignore the terminal zero outgoing odo. */
static inline void apaParkingLogRuns(const ApaParkingPathPlanningOutput_t *path,
                                     const char *event) {
  double lengths[APA_PARKING_PATH_PT_MAX] = {0};
  int directions[APA_PARKING_PATH_PT_MAX] = {0};
  Integer count = 0, shortCount = 0, boundaryCount = 0;
  for (Integer i = 0; i + 1 < path->PtCount; ++i) {
    const double distance = path->PthPtOdo[i];
    if (fabs(distance) < 1.e-6)
      continue;
    const int direction = distance > 0 ? 1 : -1;
    if (!count || directions[count - 1] != direction)
      directions[count++] = direction;
    lengths[count - 1] += fabs(distance);
  }
  for (Integer i = 0; i < count; ++i) {
    shortCount += lengths[i] < 0.4 - 1.e-5;
    boundaryCount += fabs(lengths[i] - 0.4) <= 1.e-5;
  }
  fprintf(stderr,
          "[APA candidate runs] event=%s id=%016llx points=%ld runs=%ld short_lt_0.4=%ld "
          "boundary_0.4=%ld signed_lengths_m=",
          event,
          (unsigned long long)apaParkingPathId(path),
          (long)path->PtCount,
          (long)count,
          (long)shortCount,
          (long)boundaryCount);
  for (Integer i = 0; i < count; ++i)
    fprintf(stderr, "%s%.6f", i ? "," : "", directions[i] * lengths[i]);
  fputc('\n', stderr);
}
#endif
