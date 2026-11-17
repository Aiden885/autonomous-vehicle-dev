/**
 * @file apaParkoutUtils.h
 * @brief APA park-out helper utilities
 *
 * This header starts the second D3.6 helper batch and rewrites:
 *   - get_parallel_space.m
 *
 * The helpers are kept internal and header-only so ClassParallelParkOut
 * rewriting can consume them directly in later batches.
 *
 * @date 2026-04-13
 */
#ifndef APA_PARKOUT_UTILS_H
#define APA_PARKOUT_UTILS_H

#include "baseTypes.h"
#include "apaCoordUtils.h"
#include "apaMathUtils.h"
#include "apaPlannerConfig.h"
#include "apaSlotTypes.h"
#include "vehicleCollisionCheckTypes.h"

static inline void
apaParkoutGetSlotPointByOffset(const ApaSlot_t *slot, Integer pointOffset, Real *outX, Real *outY) {
  Integer xIdx;
  Integer yIdx;

  xIdx = apaSlotWrapCoordIndex(2 * slot->EnterEdge + 2 * pointOffset - 1);
  yIdx = apaSlotWrapCoordIndex(2 * slot->EnterEdge + 2 * pointOffset);
  *outX = slot->SlotPts[xIdx];
  *outY = slot->SlotPts[yIdx];
}

static inline Real
apaParkoutExploreFrontBackDistance(const Real startPose[3],
                                   Integer direction,
                                   const VehicleCollisionCheckInput_t *collisionTemplate,
                                   Real sampleStep,
                                   Real maxDistance) {
  Real moveDistance;
  Boolean exploreOpen;

  moveDistance = sampleStep;
  exploreOpen = (moveDistance < maxDistance);
  while (exploreOpen) {
    VehicleCollisionCheckInput_t vccInput;
    VehicleCollisionCheckOutput_t vccOutput;
    Real moveSign;
    Real posX;
    Real posY;

    moveSign = 1.0f;
    if (direction < 0) {
      moveSign = -1.0f;
    }

    posX = startPose[0] + moveDistance * cosf(startPose[2]) * moveSign;
    posY = startPose[1] + moveDistance * sinf(startPose[2]) * moveSign;

    vccInput = *collisionTemplate;
    vccInput.CheckType = VCC_CHECK_TYPE_PLANNING;
    vccInput.CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
    vccInput.CollisionDemand = VCC_COLLISION_DEMAND_ALL;
    vccInput.ParkingOutFlag = true;
    vccInput.UseAutoMargin = true;
    vccInput.VehPosX = posX;
    vccInput.VehPosY = posY;
    vccInput.VehPosTheta = startPose[2];
    vehicleCollisionCheck(&vccInput, &vccOutput);
    if (vccOutput.IsCollision) {
      exploreOpen = false;
    } else {
      moveDistance = moveDistance + sampleStep;
      exploreOpen = (moveDistance < maxDistance);
    }
  }

  moveDistance = moveDistance - sampleStep;
  if (apaMathLess(moveDistance, 0.0f)) {
    moveDistance = 0.0f;
  }
  return moveDistance;
}

static inline void
apaParkoutCalcParallelSpace(const ApaSlot_t *slot,
                            const ApaPlannerVehConfig_t *vehConfig,
                            const VehicleCollisionCheckInput_t *collisionTemplate,
                            Real *backSpace,
                            Real *frontSpace) {
  Integer slotClass;
  Real backSlotNearX;
  Real backSlotNearY;
  Real backSlotNearLocalX;
  Real backSlotNearLocalY;
  Real backSlotNearLocalYaw;
  Real lateralDelta;
  Real clearance;
  Real startPose[3];

  *backSpace = 0.0f;
  *frontSpace = 0.0f;

  slotClass = slot->SlotClass;
  if (slotClass == APA_SLOT_CLASS_UNKNOWN) {
    slotClass = apaSlotCalcSlotClass(slot->SlotPts, slot->EnterEdge);
  }
  if (slotClass != APA_SLOT_CLASS_PARALLEL) {
    return;
  }

  apaParkoutGetSlotPointByOffset(slot, 1, &backSlotNearX, &backSlotNearY);
  apaCoordGround2Veh(slot->TarPos[0],
                     slot->TarPos[1],
                     slot->TarPos[2],
                     backSlotNearX,
                     backSlotNearY,
                     0.0f,
                     &backSlotNearLocalX,
                     &backSlotNearLocalY,
                     &backSlotNearLocalYaw);
  (void)backSlotNearLocalX;
  (void)backSlotNearLocalYaw;

  lateralDelta = 0.0f;
  clearance = fabsf(backSlotNearLocalY) - 0.5f * vehConfig->W - 0.01f;
  if (apaMathGreater(clearance, 0.0f)) {
    if (apaMathGreater(backSlotNearLocalY, 0.0f)) {
      lateralDelta = clearance;
    } else {
      lateralDelta = -clearance;
    }
  }

  apaCoordVeh2Ground(slot->TarPos[0],
                     slot->TarPos[1],
                     slot->TarPos[2],
                     0.0f,
                     lateralDelta,
                     0.0f,
                     &startPose[0],
                     &startPose[1],
                     &startPose[2]);

  *backSpace = apaParkoutExploreFrontBackDistance(startPose, -1, collisionTemplate, 0.03f, 4.0f);
  *frontSpace = apaParkoutExploreFrontBackDistance(startPose, 1, collisionTemplate, 0.03f, 4.0f);
}

#endif
