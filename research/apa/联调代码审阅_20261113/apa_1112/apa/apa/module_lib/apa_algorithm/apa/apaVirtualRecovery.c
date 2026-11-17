#include "apaVirtualRecovery.h"
#include "apaTrackingPolicy.h"

#include <string.h>

void apaVirtualRecoveryDefaultParam(ApaVirtualRecoveryParam_t *param) {

  param->MaxLength = 0.8f;
  param->MaxInitialDepth = 0.15f;
  param->RestoredClearance = APA_PLANNING_TRACKING_RESERVE_M + 0.02f;
  param->MaxSpeed = 0.15f;
  param->MaxDuration = 15.0f;
}

Boolean apaVirtualRecoveryInitialize(const VehicleCollisionCheckInput_t *start,
                                     Real anchorX,
                                     Real anchorY,
                                     Real anchorYaw,
                                     const ApaVirtualRecoveryParam_t *param,
                                     const ApaPlanningSceneQuery_t *scene,
                                     ApaVirtualRecoveryContext_t *context) {
  (void)start;
  (void)anchorX;
  (void)anchorY;
  (void)anchorYaw;
  (void)param;
  (void)scene;
  memset(context, 0, sizeof(*context));
  return false;
}

Boolean apaVirtualRecoveryContextMatches(const VehicleCollisionCheckInput_t *pose,
                                         const ApaVirtualRecoveryContext_t *context) {
  (void)pose;
  (void)context;
  return false;
}

void apaVirtualRecoveryCollisionCheck(const VehicleCollisionCheckInput_t *pose,
                                      const ApaVirtualRecoveryContext_t *context,
                                      Real pathDistance,
                                      VehicleCollisionCheckOutput_t *output) {
  (void)pathDistance;
  vehicleCollisionCheck(pose, output);

  if (context && context->Enabled && !output->IsCollision) {
    output->IsCollision = true;
    output->CollisionType = VCC_COLLISION_VIRTUAL;
    output->CollisionIndex = 0;
  }
}

Boolean apaVirtualRecoveryRestored(const VehicleCollisionCheckInput_t *pose,
                                   const ApaVirtualRecoveryContext_t *context,
                                   const ApaPlanningSceneQuery_t *scene) {
  (void)pose;
  (void)context;
  (void)scene;
  return false;
}

void apaVirtualRecoveryPrefixPlan(const VehicleCollisionCheckInput_t *start,
                                  const ApaVirtualRecoveryContext_t *context,
                                  Integer primitive,
                                  Real minimumRadius,
                                  const ApaPlanningSceneQuery_t *scene,
                                  ApaVirtualRecoveryPrefix_t *prefix) {
  (void)start;
  (void)context;
  (void)primitive;
  (void)minimumRadius;
  (void)scene;
  memset(prefix, 0, sizeof(*prefix));
}
