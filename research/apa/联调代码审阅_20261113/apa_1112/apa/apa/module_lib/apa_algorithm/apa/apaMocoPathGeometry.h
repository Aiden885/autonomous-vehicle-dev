#ifndef APA_MOCO_PATH_GEOMETRY_H
#define APA_MOCO_PATH_GEOMETRY_H
#include "baseTypes.h"
typedef struct {
  Integer Index;
  Real Fraction;
  Real X, Y, Yaw;
  Real Progress, Length, Remaining;
  Real CrossTrackError, HeadingError;
  Boolean Valid;
} ApaMocoPathProjection_t;

/* Search only the active segment, bounded by the previously accepted progress. */
void apaMocoProjectPath(const Real *x,
                        const Real *y,
                        const Real *yaw,
                        Integer count,
                        Real vehicleX,
                        Real vehicleY,
                        Real vehicleYaw,
                        Integer lastIndex,
                        Real lastProgress,
                        Integer searchBack,
                        Integer searchAhead,
                        Real maxAdvance,
                        Real maxDistance,
                        Real maxHeading,
                        ApaMocoPathProjection_t *projection);
void apaMocoPathAtDistance(const Real *x,
                           const Real *y,
                           const Real *yaw,
                           Integer count,
                           Real distance,
                           Integer *index,
                           Real *targetX,
                           Real *targetY,
                           Real *targetYaw);
#endif
