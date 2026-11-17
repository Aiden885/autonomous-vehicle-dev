/** @file apaHybridAStarCollision.h
 * @brief Hybrid A* 运动碰撞检查。
 */
/* MATLAB sources: VehicleCollisionCheck.m */
#ifndef APA_HYBRID_ASTAR_COLLISION_H
#define APA_HYBRID_ASTAR_COLLISION_H

#include "apaPlannerGlobal.h"
#include "apaSlotTypes.h"
#include "apaVehicleCollisionCheck.h"

typedef ApaVehicleCollisionLineSeg_t ApaLineSeg2d_t;
typedef ApaVehicleCollisionCheckContext_t ApaHybridAStarCollisionEnv_t;

static inline void apaHybridAStarCollisionEnvInit(ApaHybridAStarCollisionEnv_t *env) {
  apaVehicleCollisionCheckContextInit(env);
}

static inline void
apaHybridAStarCollisionEnvSetPlannerGlobal(ApaHybridAStarCollisionEnv_t *env,
                                           ApaPlannerGlobalState_t *plannerGlobal) {
  apaVehicleCollisionCheckContextSetPlannerGlobal(env, plannerGlobal);
}

static inline void apaHybridAStarCollisionEnvSetCheckType(ApaHybridAStarCollisionEnv_t *env,
                                                          Integer checkType) {
  apaVehicleCollisionCheckContextSetCheckType(env, checkType);
}

static inline void apaHybridAStarCollisionEnvSetCollisionDemand(ApaHybridAStarCollisionEnv_t *env,
                                                                Integer collisionDemand) {
  apaVehicleCollisionCheckContextSetCollisionDemand(env, collisionDemand);
}

static inline void apaHybridAStarCollisionEnvSetSlot(ApaHybridAStarCollisionEnv_t *env,
                                                     const ApaSlot_t *slot) {
  apaVehicleCollisionCheckContextSetSlot(env, slot);
}

static inline void apaHybridAStarCollisionEnvClearSlot(ApaHybridAStarCollisionEnv_t *env) {
  apaVehicleCollisionCheckContextClearSlot(env);
}

static inline void apaHybridAStarCollisionEnvSetParkingOutFlag(ApaHybridAStarCollisionEnv_t *env,
                                                               Boolean parkingOutFlag) {
  apaVehicleCollisionCheckContextSetParkingOutFlag(env, parkingOutFlag);
}

static inline void apaHybridAStarCollisionEnvSetVerticalLastStep(ApaHybridAStarCollisionEnv_t *env,
                                                                 Boolean verticalLastStep) {
  apaVehicleCollisionCheckContextSetVerticalLastStep(env, verticalLastStep);
}

static inline void apaHybridAStarCollisionEnvSetTargetPose(ApaHybridAStarCollisionEnv_t *env,
                                                           const Real targetPose[3]) {
  apaVehicleCollisionCheckContextSetTargetPose(env, targetPose);
}

static inline void apaHybridAStarCollisionEnvClearTargetPose(ApaHybridAStarCollisionEnv_t *env) {
  apaVehicleCollisionCheckContextClearTargetPose(env);
}

static inline Boolean apaHybridAStarCollisionEnvAddBoundaryLine(
    ApaHybridAStarCollisionEnv_t *env, Real x1, Real y1, Real x2, Real y2) {
  return apaVehicleCollisionObsAddBoundaryLine(&env->Obs, x1, y1, x2, y2);
}

static inline Boolean
apaHybridAStarCollisionEnvAddBoundaryPoint(ApaHybridAStarCollisionEnv_t *env, Real x, Real y) {
  return apaVehicleCollisionObsAddBoundarySeparatePoint(&env->Obs, x, y);
}

static inline Boolean
apaHybridAStarCollisionEnvAddSeparateStopper(ApaHybridAStarCollisionEnv_t *env, Real x, Real y) {
  return apaVehicleCollisionObsAddSeparateStopper(&env->Obs, x, y);
}

static inline Boolean apaHybridAStarCollisionEnvAddTrackStopper(
    ApaHybridAStarCollisionEnv_t *env, Real x1, Real y1, Real x2, Real y2) {
  return apaVehicleCollisionObsAddSlotTrackStopper(&env->Obs, x1, y1, x2, y2);
}

static inline Boolean apaHybridAStarCollisionEnvAddVirtualLine(
    ApaHybridAStarCollisionEnv_t *env, Real x1, Real y1, Real x2, Real y2) {
  return apaVehicleCollisionObsAddVirtualBoundaryLine(&env->Obs, x1, y1, x2, y2);
}

static inline Boolean apaHybridAStarCollisionCheckDefault(const ApaHybridNode_t *fromNode,
                                                          const ApaHybridNode_t *toNode,
                                                          void *userData) {
  return apaVehicleCollisionCheckMotionDefault(fromNode, toNode, userData);
}

static inline Boolean
apaHybridAStarCollisionCheckMotionDetailed(const ApaHybridNode_t *fromNode,
                                           const ApaHybridNode_t *toNode,
                                           void *userData,
                                           Integer collisionDemand,
                                           ApaVehicleCollisionCheckResult_t *result,
                                           Real *collisionTravel) {
  return apaVehicleCollisionCheckMotionDetailed(
      fromNode, toNode, userData, collisionDemand, result, collisionTravel);
}

#endif
