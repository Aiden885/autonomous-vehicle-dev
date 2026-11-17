#ifndef APA_PARKING_PATH_SAFETY_GATE_TYPES_H
#define APA_PARKING_PATH_SAFETY_GATE_TYPES_H

#include "apaParkingPathPlanningTypes.h"

typedef struct {
  ApaParkingPathPlanningInput_t PlannerInput;
  ApaParkingPathPlanningParam_t PlannerParam;
  ApaParkingPathPlanningOutput_t CandidateOutput;
} ApaParkingPathSafetyGateInput_t;

typedef struct {
  ApaParkingPathPlanningOutput_t ValidatedOutput;
  Boolean Rejected;
} ApaParkingPathSafetyGateOutput_t;

#endif
