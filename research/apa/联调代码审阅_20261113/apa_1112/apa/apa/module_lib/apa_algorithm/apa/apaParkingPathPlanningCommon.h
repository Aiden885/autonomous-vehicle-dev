#ifndef APA_PARKING_PATH_PLANNING_COMMON_H
#define APA_PARKING_PATH_PLANNING_COMMON_H

#include "apaParkingPathPlanningTypes.h"

void apaParkingPathPlanningZeroOutput(ApaParkingPathPlanningOutput_t *output);
void apaParkingPathPlanningCopyOutput(const ApaParkingPathPlanningOutput_t *src,
                                      ApaParkingPathPlanningOutput_t *dst);

#endif
