#ifndef APA_PARKING_SLOT_PATH_PLANNING_H
#define APA_PARKING_SLOT_PATH_PLANNING_H

#include "apaParkingSlotPathPlanningTypes.h"

void apaParkingPathPlanningFromSlotDefaultParam(ApaParkingPathPlanningFromSlotParam_t *param);
void apaParkingPathPlanningFromSlot(const ApaParkingPathPlanningFromSlotInput_t *input,
                                    const ApaParkingPathPlanningFromSlotParam_t *param,
                                    ApaParkingPathPlanningFromSlotOutput_t *output);
#endif
