#ifndef APA_PARKING_PATH_POST_PROCESS_H
#define APA_PARKING_PATH_POST_PROCESS_H

#include "apaParkingPathPlanningTypes.h"

/* ClassPath.jumpPoints at the common stopped park-in boundary. Each splice
 * must improve shared quality and pass the complete production checks.
 * executionBudget is the remainder of the request's candidate replay budget. */
Boolean apaParkingPathPostProcess(const ApaParkingPathPlanningInput_t *input,
                                  const ApaParkingPathPlanningParam_t *param,
                                  Integer executionBudget,
                                  ApaParkingPathPlanningOutput_t *output);

#endif
