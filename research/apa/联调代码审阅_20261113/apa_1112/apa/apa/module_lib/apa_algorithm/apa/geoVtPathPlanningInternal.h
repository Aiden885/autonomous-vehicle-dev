#ifndef GEO_VT_PATH_PLANNING_INTERNAL_H
#define GEO_VT_PATH_PLANNING_INTERNAL_H

#include "geoVtPathPlanningTypes.h"
#include "apaTargetSlotConstraintTypes.h"

typedef Boolean (*GeoVtCandidateAccept_t)(const GeoVtPathPlanningOutput_t *candidate,
                                          void *userData);

/* Optional, single-call APA policy. The source-reference entry has no policy. */
typedef struct {
  const ApaTargetSlotConstraint_t *SlotConstraint;
  GeoVtCandidateAccept_t AcceptCandidate;
  void *UserData;
} GeoVtPathPlanningPolicy_t;

void geoVtPathPlanningWithPolicy(const GeoVtPathPlanningInput_t *input,
                                 const GeoVtPathPlanningParam_t *param,
                                 const GeoVtPathPlanningPolicy_t *policy,
                                 GeoVtPathPlanningOutput_t *output);
Boolean geoVtPathPlanningGeometryIsContinuous(const GeoVtPathPlanningInput_t *input,
                                              const GeoVtPathPlanningOutput_t *candidate,
                                              Integer *rejectedSegment);

#endif
