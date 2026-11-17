#ifndef APA_PARKING_SEARCH_FALLBACK_H
#define APA_PARKING_SEARCH_FALLBACK_H

#include "apaParkingPathPlanningTypes.h"
#include "apaParkingSearch.h"

/* Common parking candidate selection, independent of action and geometric slot cases. */
#define APA_PARKING_SEARCH_PLANNER_CASE 100
#define APA_PARKING_CONNECTION_PLANNER_CASE 101
ApaParkingQuality_t apaParkingManeuverQuality(const ApaParkingPathPlanningOutput_t *path,
                                              double cost);
Real apaParkingManeuverCost(const ApaParkingPathPlanningOutput_t *path,
                            const ApaParkingPathPlanningInput_t *input,
                            const ApaParkingPathPlanningParam_t *param);
typedef struct {
  ApaParkingQuality_t Quality;
  Boolean HasClearanceReserve;
} ApaParkingManeuverAssessment_t;

/* Optional clearance reserve affects candidate preference, independently of admission. */
ApaParkingManeuverAssessment_t apaParkingAssessManeuver(const ApaParkingPathPlanningOutput_t *path,
                                                        const ApaParkingPathPlanningInput_t *input,
                                                        const ApaParkingPathPlanningParam_t *param,
                                                        Real reserve);
/* Keep the template as a candidate, improve it with the common search, then
 * choose the best executable member of the same bounded set for every kind. */
Boolean apaParkingSelectCandidate(const ApaParkingPathPlanningInput_t *input,
                                  const ApaParkingPathPlanningParam_t *param,
                                  ApaParkingPathPlanningOutput_t *output);

#endif
