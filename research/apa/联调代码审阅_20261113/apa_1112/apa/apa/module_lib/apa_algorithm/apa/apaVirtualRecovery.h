#ifndef APA_VIRTUAL_RECOVERY_H
#define APA_VIRTUAL_RECOVERY_H

#include "apaPlanningScene.h"

#include "apaVirtualRecoveryTypes.h"

void apaVirtualRecoveryDefaultParam(ApaVirtualRecoveryParam_t *param);

/* Context must be disabled. */
Boolean apaVirtualRecoveryInitialize(const VehicleCollisionCheckInput_t *start,
                                     Real anchorX,
                                     Real anchorY,
                                     Real anchorYaw,
                                     const ApaVirtualRecoveryParam_t *param,
                                     const ApaPlanningSceneQuery_t *scene,
                                     ApaVirtualRecoveryContext_t *context);

Boolean apaVirtualRecoveryContextMatches(const VehicleCollisionCheckInput_t *pose,
                                         const ApaVirtualRecoveryContext_t *context);

/* Checks all supplied lines; an enabled waiver is rejected. */
void apaVirtualRecoveryCollisionCheck(const VehicleCollisionCheckInput_t *pose,
                                      const ApaVirtualRecoveryContext_t *context,
                                      Real pathDistance,
                                      VehicleCollisionCheckOutput_t *output);

void apaVirtualRecoveryPrefixPlan(const VehicleCollisionCheckInput_t *start,
                                  const ApaVirtualRecoveryContext_t *context,
                                  Integer primitive,
                                  Real minimumRadius,
                                  const ApaPlanningSceneQuery_t *scene,
                                  ApaVirtualRecoveryPrefix_t *prefix);

Boolean apaVirtualRecoveryRestored(const VehicleCollisionCheckInput_t *pose,
                                   const ApaVirtualRecoveryContext_t *context,
                                   const ApaPlanningSceneQuery_t *scene);

#endif
