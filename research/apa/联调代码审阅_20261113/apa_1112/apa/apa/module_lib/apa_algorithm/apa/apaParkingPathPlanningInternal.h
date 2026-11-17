#ifndef APA_PARKING_PATH_PLANNING_INTERNAL_H
#define APA_PARKING_PATH_PLANNING_INTERNAL_H

#include "apaPathTypes.h"
#include "geoVtPathPlanningTypes.h"
#include "geoHtPathPlanningTypes.h"
#include "geoDthEdVtPathPlanningTypes.h"
#include "geoVhPathPlanningTypes.h"
#include "geoPthDiscretVtTypes.h"
#include "geoPthDiscretHtTypes.h"
#include "geoPthDiscretVhTypes.h"
#include "apaDthEdVtParkingPathPlanningTypes.h"
#include "apaHtParkingPathPlanningTypes.h"
#include "apaParallelParkoutPlan.h"
#include "apaParkingOutPathPlanningTypes.h"
#include "apaParkingPathPlanningCommon.h"
#include "apaVhParkingPathPlanningTypes.h"
#include "apaVtParkingPathPlanningTypes.h"
#include "vehicleCollisionCheckTypes.h"
#include "apaTargetSlotConstraint.h"

typedef struct {
  const ApaParkingPathPlanningInput_t *Input;
  const ApaParkingPathPlanningParam_t *Param;
} ApaParkingHybridTrial_t;

Integer apaParkingPathPlanningCompareHybridSegments(const ApaPlannerHybridSegments_t *origin,
                                                    const ApaPlannerHybridSegments_t *candidate,
                                                    void *userData);

Boolean apaParkingPathPlanningAcceptHybridSegments(const ApaPlannerHybridSegments_t *segments,
                                                   void *userData);

Boolean apaParkingPathPlanningBuildHybridCandidate(const ApaPlannerHybridSegments_t *segments,
                                                   const ApaParkingHybridTrial_t *trial,
                                                   ApaParkingPathPlanningOutput_t *candidate);

void apaParkingPathPlanningBuildTargetSlotConstraint(const ApaParkingPathPlanningInput_t *input,
                                                     const ApaParkingPathPlanningParam_t *param,
                                                     ApaTargetSlotConstraint_t *constraint);

void apaParkingPathPlanningFillVtInput(const ApaParkingPathPlanningInput_t *input,
                                       GeoVtPathPlanningInput_t *vtInput);
void apaParkingPathPlanningFillHtInput(const ApaParkingPathPlanningInput_t *input,
                                       GeoHtPathPlanningInput_t *htInput);
void apaParkingPathPlanningFillDthInput(const ApaParkingPathPlanningInput_t *input,
                                        GeoDthEdVtPathPlanningInput_t *dthInput);
void apaParkingPathPlanningFillVhInput(const ApaParkingPathPlanningInput_t *input,
                                       GeoVhPathPlanningInput_t *vhInput);
void apaParkingPathPlanningFillVtParam(const ApaParkingPathPlanningParam_t *param,
                                       GeoVtPathPlanningParam_t *vtParam);
void apaParkingPathPlanningFillHtParam(const ApaParkingPathPlanningParam_t *param,
                                       GeoHtPathPlanningParam_t *htParam);
void apaParkingPathPlanningFillDthParam(const ApaParkingPathPlanningParam_t *param,
                                        GeoDthEdVtPathPlanningParam_t *dthParam);
void apaParkingPathPlanningFillVhParam(const ApaParkingPathPlanningParam_t *param,
                                       GeoVhPathPlanningParam_t *vhParam);
void apaParkingPathPlanningFillVtDiscretInput(const ApaParkingPathPlanningInput_t *input,
                                              const GeoVtPathPlanningOutput_t *plannerOutput,
                                              GeoPthDiscretVtInput_t *discretInput);
void apaParkingPathPlanningFillHtDiscretInput(const ApaParkingPathPlanningInput_t *input,
                                              const GeoHtPathPlanningOutput_t *plannerOutput,
                                              GeoPthDiscretHtInput_t *discretInput);
void apaParkingPathPlanningFillDthDiscretInput(const ApaParkingPathPlanningInput_t *input,
                                               const GeoDthEdVtPathPlanningOutput_t *plannerOutput,
                                               GeoPthDiscretVtInput_t *discretInput);
void apaParkingPathPlanningFillVhDiscretInput(const ApaParkingPathPlanningInput_t *input,
                                              const GeoVhPathPlanningOutput_t *plannerOutput,
                                              GeoPthDiscretVhInput_t *discretInput);
Real apaParkingPathPlanningCalcParkoutPathLen(const ApaPath_t *path);
Boolean apaParkingPathPlanningShouldRetryParkoutSearch(const ApaComplexPath_t *parkoutPath);
Boolean
apaParkingPathPlanningShouldRetryParkoutSearchAfterFailure(const ApaParallelParkoutDiag_t *diag);
void apaParkingPathPlanningCopyVtPath(const GeoPthDiscretVtOutput_t *discretOutput,
                                      ApaParkingPathPlanningOutput_t *output);
void apaParkingPathPlanningRefreshOutputOdo(ApaParkingPathPlanningOutput_t *output);
void apaParkingPathPlanningReverseOutputPath(ApaParkingPathPlanningOutput_t *output);
void apaParkingPathPlanningRefreshVtStp1Metadata(const ApaParkingPathPlanningInput_t *input,
                                                 ApaParkingPathPlanningOutput_t *output);
void apaParkingPathPlanningSwapInputStartTargetPose(ApaParkingPathPlanningInput_t *input);
Boolean apaParkingPathPlanningShouldTryReverseVtPath(const ApaParkingPathPlanningInput_t *input);
Boolean apaParkingPathPlanningOutputPathIsSafe(const ApaParkingPathPlanningInput_t *input,
                                               const ApaParkingPathPlanningParam_t *param,
                                               const ApaParkingPathPlanningOutput_t *output,
                                               Boolean checkEndPose);
Boolean apaParkingPathPlanningSuccessPayloadIsValid(const ApaParkingPathPlanningOutput_t *output,
                                                    Integer segmentMaximum);
Boolean
apaParkingPathPlanningCandidateIsStructurallyValid(const ApaParkingPathPlanningInput_t *input,
                                                   const ApaParkingPathPlanningOutput_t *output);
Real apaParkingPathPlanningInterpolateThetaShortest(Real previousTheta,
                                                    Real currentTheta,
                                                    Real ratio);
void apaParkingPathPlanningFinalizeL1Output(ApaParkingPathPlanningOutput_t *output,
                                            Integer segmentMaximum);
void apaParkingPathPlanningRejectUnsafeOutput(ApaParkingPathPlanningOutput_t *output);
void apaParkingPathPlanningCopyHtPath(const GeoPthDiscretHtOutput_t *discretOutput,
                                      ApaParkingPathPlanningOutput_t *output);
void apaParkingPathPlanningCopyVhPath(const GeoPthDiscretVhOutput_t *discretOutput,
                                      ApaParkingPathPlanningOutput_t *output);
void apaParkingPathPlanningCopyParkoutPath(const ApaComplexPath_t *parkoutPath,
                                           ApaParkingPathPlanningOutput_t *output);
void apaParkingPathPlanningBuildPathVccInput(const ApaParkingPathPlanningInput_t *input,
                                             const ApaParkingPathPlanningParam_t *param,
                                             VehicleCollisionCheckInput_t *vccInput);
#endif
