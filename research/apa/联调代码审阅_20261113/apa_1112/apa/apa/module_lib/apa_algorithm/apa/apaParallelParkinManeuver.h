#ifndef APA_PARALLEL_PARKIN_MANEUVER_H
#define APA_PARALLEL_PARKIN_MANEUVER_H

#include "apaPlannerHybridParity.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  Integer Loops;
  Boolean Adjusted;
  const char *Reason;
} ApaParallelParkinManeuverDiag_t;

/* ClassParallelParkOut.m's parking-IN tail: generate from the fixed slot
 * target towards the aisle, then reverse. The live vehicle pose determines
 * the side and eligibility, not the starting point of this maneuver. */
Boolean apaParallelParkinManeuver(const ApaSlot_t *slot,
                                  const Real vehicle[3],
                                  ApaPlannerHybridCollisionContext_t *collision,
                                  ApaHybridAStarMotionPath_t *tail,
                                  ApaParallelParkinManeuverDiag_t *diag);

/* Retry longitudinal goals only inside the task's unchanged position region.
 * Every returned motion keeps its actual endpoint; the caller admits it. */
Boolean apaParallelParkinManeuverInRegion(const ApaSlot_t *slot,
                                          const Real vehicle[3],
                                          Real tolerance,
                                          ApaPlannerHybridCollisionContext_t *collision,
                                          ApaHybridAStarMotionPath_t *tail,
                                          ApaParallelParkinManeuverDiag_t *diag);

/* Enumerate nominal, +/- half and +/- full longitudinal tolerance anchors.
 * A successful tail does not suppress other anchors when its connector fails. */
Boolean apaParallelParkinManeuverRegionCandidate(const ApaSlot_t *slot,
                                                 const Real vehicle[3],
                                                 Real tolerance,
                                                 Integer index,
                                                 ApaPlannerHybridCollisionContext_t *collision,
                                                 ApaHybridAStarMotionPath_t *tail,
                                                 ApaParallelParkinManeuverDiag_t *diag);

#ifdef __cplusplus
}
#endif
#endif
