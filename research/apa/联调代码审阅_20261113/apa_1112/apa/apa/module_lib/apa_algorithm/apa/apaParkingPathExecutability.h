#ifndef APA_PARKING_PATH_EXECUTABILITY_H
#define APA_PARKING_PATH_EXECUTABILITY_H
#include "apaParkingPathExecutabilityTypes.h"
void apaParkingPathExecutabilityDefaultParam(ApaParkingPathExecutabilityParam_t *param);
Boolean apaParkingPathExecutabilityParamIsValid(const ApaParkingPathExecutabilityParam_t *param);
void apaParkingPathExecutabilityEnvelope(const ApaParkingPathPlanningParam_t *vehicle,
                                         const ApaParkingPathExecutabilityParam_t *param,
                                         ApaParkingPathPlanningParam_t *envelope);
Boolean apaParkingPathExecutabilityMotionIsSafe(const ApaParkingPathPlanningInput_t *scene,
                                                const ApaParkingPathPlanningParam_t *vehicle,
                                                const Real previousPose[3],
                                                const Real nextPose[3]);
void apaParkingPathExecutability(const ApaParkingPathExecutabilityInput_t *input,
                                 const ApaParkingPathExecutabilityParam_t *param,
                                 ApaParkingPathExecutabilityOutput_t *output);
#endif
