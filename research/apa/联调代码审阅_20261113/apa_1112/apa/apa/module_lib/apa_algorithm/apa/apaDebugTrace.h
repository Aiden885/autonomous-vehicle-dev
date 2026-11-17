#ifndef APA_DEBUG_TRACE_H
#define APA_DEBUG_TRACE_H

#include "apaParkingPathPlanningTypes.h"

/* Single-instance planner/MoCo diagnostic correlation. */
UInteger apaDebugTraceBeginFrame(void);
UInteger apaDebugTraceCurrentFrame(void);
void apaDebugTracePlanningInput(UInteger frameId,
                                const ApaParkingPathPlanningInput_t *input,
                                const ApaParkingPathPlanningParam_t *param);
void apaDebugTracePlanningOutput(const char *stage,
                                 UInteger frameId,
                                 const ApaParkingPathPlanningOutput_t *output);
void apaDebugTraceGear(UInteger frameId,
                       const char *stage,
                       const char *source,
                       Integer pathIndex,
                       Real odo,
                       Integer segGoDir,
                       Integer targetGearReq);
void apaDebugTraceParkoutInput(const char *stage,
                               UInteger frameId,
                               const ApaParkingPathPlanningInput_t *input);
void apaDebugTraceParkoutPath(const char *stage,
                              UInteger frameId,
                              const ApaParkingPathPlanningInput_t *input,
                              const ApaParkingPathPlanningOutput_t *output);

#endif
