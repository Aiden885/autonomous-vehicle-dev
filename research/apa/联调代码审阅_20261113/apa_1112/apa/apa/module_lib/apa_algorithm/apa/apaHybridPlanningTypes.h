#ifndef APA_HYBRID_PLANNING_TYPES_H
#define APA_HYBRID_PLANNING_TYPES_H

#include "apaEnums.h"
#include "apaPlanningScene.h"

#ifndef APA_PATH_SEG_MAX
#define APA_PATH_SEG_MAX 30
#endif
#ifndef APA_PATH_CC_MAX
#define APA_PATH_CC_MAX 60
#endif

/* Per-call diagnostics and policy are owned by the planner process. Neither
 * pointers nor callbacks are part of the worker IPC or the public path. */
typedef struct {
  Integer ErrorType;
  const char *Stage;
  Integer Priority;
  Integer SearchFault;
  Integer RejectedCandidates;
  Integer AcceptedCandidates;
  Integer TruncatedPrimitives;
} ApaPlanningDiagnostics_t;

static inline void apaPlanningRecordFailure(ApaPlanningDiagnostics_t *diagnostic,
                                            Integer error,
                                            const char *stage,
                                            Integer priority) {
  if (diagnostic && error != APA_FAULT_NO_ERR &&
      (!diagnostic->Stage || priority > diagnostic->Priority)) {
    diagnostic->ErrorType = error;
    diagnostic->Stage = stage;
    diagnostic->Priority = priority;
  }
}

static inline Integer apaPlanningSearchError(Integer fault) {
  switch (fault) {
  case APA_HYBRID_FAULT_ILLEGAL_START_GRID:
    return APA_FAULT_START_GRID_ILLEGAL;
  case APA_HYBRID_FAULT_ILLEGAL_GOAL_GRID:
    return APA_FAULT_GOAL_GRID_ILLEGAL;
  case APA_HYBRID_FAULT_HYBRID_OPEN_RUN_OUT:
    return APA_FAULT_HYBRID_OPEN_RUN_OUT;
  case APA_HYBRID_FAULT_HYBRID_CLOSE_RUN_OUT:
    return APA_FAULT_HYBRID_CLOSE_RUN_OUT;
  case APA_HYBRID_FAULT_HYBRID_OPEN_EMPTY:
    return APA_FAULT_HYBRID_OPEN_EMPTY;
  case APA_HYBRID_FAULT_PATH_OUT_OF_RANGE:
    return APA_FAULT_HYBRID_PATH_OUT_OF_RANGE;
  case APA_HYBRID_FAULT_PATH_RESOLUTION_TOO_LARGE:
    return APA_FAULT_HYBRID_PATH_RES_TOO_LARGE;
  case APA_HYBRID_FAULT_EXCEED_CAL_POWER:
    return APA_FAULT_MASSIVE_CAL_POWER_LACK;
  case APA_HYBRID_FAULT_PARENT_INVALID:
    return APA_FAULT_CLOSE_PARENT_INVALID;
  default:
    return APA_FAULT_HYBRID_UNKNOWN_ERR;
  }
}

typedef struct {
  Integer SegNum;
  Integer StpN;
  Integer PlnChkCnt;
  Integer PlannerCase;
  Real PathLen;
  Integer PathGoDir[APA_PATH_SEG_MAX];
  Integer PathTrnDir[APA_PATH_SEG_MAX];
  Real PathCrv[APA_PATH_SEG_MAX];
  Real PathTht[APA_PATH_SEG_MAX];
  Real PathLenArr[APA_PATH_SEG_MAX];
  Real PathStx[APA_PATH_SEG_MAX];
  Real PathSty[APA_PATH_SEG_MAX];
  Real PathStTh[APA_PATH_SEG_MAX];
  Real PathCC[APA_PATH_CC_MAX];
} ApaPlannerHybridSegments_t;

typedef struct {
  Boolean (*AcceptCandidate)(const ApaPlannerHybridSegments_t *, void *);
  Integer (*CompareCandidates)(const ApaPlannerHybridSegments_t *,
                               const ApaPlannerHybridSegments_t *,
                               void *);
  void *UserData;
  Real GoalPositionTolerance; /* Task region for optional parallel tail retries. */
} ApaPlannerHybridPolicy_t;

#endif
