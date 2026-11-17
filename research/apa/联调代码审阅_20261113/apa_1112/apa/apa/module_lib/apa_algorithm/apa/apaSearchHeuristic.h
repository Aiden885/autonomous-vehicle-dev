/**
 * @file apaSearchHeuristic.h
 * @brief Heuristic-cost rewrite for APA Hybrid A* search.
 *
 * Source:
 *   - getHeuristicCost.m
 *   - getTotalCost.m
 *
 * Date: 2026-04-14
 */
#ifndef APA_SEARCH_HEURISTIC_H
#define APA_SEARCH_HEURISTIC_H

#include "apaCoordUtils.h"
#include "apaMathUtils.h"
#include "apaSearchTypes.h"
#include "apaVehicleDefaults.h"

#define APA_REVERSE_TARGET_ANGLE_DEG 90.0f
#define APA_REVERSE_TARGET_ANGLE_MIN_DEG 80.0f
#define APA_REVERSE_TARGET_ANGLE_DIVISOR 57.3f
#define APA_REVERSE_LAT_ERR_THRESHOLD 1.3f
#define APA_VERTICAL_LAT_BLEND_THRESHOLD 1.0f
#define APA_PARALLEL_LAT_BLEND_THRESHOLD 0.8f
#define APA_REVERSE_DONE_YAW_THRESHOLD 0.5f

typedef struct {
  Real MinCircle;
  Real HCost;
  Boolean UseGCost;
  UInteger8 ReverseFlag;
  Real ReverseTheta;
} ApaPlannerHeuristicContext_t;

typedef struct {
  Real TarPos[3];
  Integer SlotClass;
  Integer SlotSubStatus;
} ApaSlotHeuristicView_t;

static inline void apaPlannerHeuristicContextInitDefault(ApaPlannerHeuristicContext_t *context) {
  context->MinCircle = APA_VEHICLE_MIN_TURN_RADIUS_M;
  context->HCost = 5.0f;
  context->UseGCost = true;
  context->ReverseFlag = 0U;
  context->ReverseTheta = 0.0f;
}

static inline void apaSlotHeuristicViewInit(ApaSlotHeuristicView_t *slot) {
  slot->TarPos[0] = 0.0f;
  slot->TarPos[1] = 0.0f;
  slot->TarPos[2] = 0.0f;
  slot->SlotClass = APA_SLOT_CLASS_UNKNOWN;
  slot->SlotSubStatus = APA_SLOT_SUB_END_NONE;
}

static inline Real apaSearchScaleReverseTargetAngle(Real latErr, Integer parkingType) {
  Real targetAngleDeg;
  Real absLatErr;

  targetAngleDeg = APA_REVERSE_TARGET_ANGLE_DEG;
  absLatErr = apaMathAbs(latErr);

  if ((parkingType == APA_PARKING_TYPE_RIGHT && latErr < 0.0f) ||
      (parkingType == APA_PARKING_TYPE_LEFT && latErr > 0.0f)) {
    if (absLatErr > APA_REVERSE_LAT_ERR_THRESHOLD) {
      targetAngleDeg = APA_REVERSE_TARGET_ANGLE_MIN_DEG;
    } else {
      targetAngleDeg =
          APA_REVERSE_TARGET_ANGLE_DEG - ((absLatErr / APA_REVERSE_LAT_ERR_THRESHOLD) * 10.0f);
    }
  }

  return targetAngleDeg;
}

static inline Real apaSearchGetHeuristicCost(const ApaSlotHeuristicView_t *slot,
                                             const ApaHybridNode_t *workNode,
                                             const ApaHybridNode_t *goalNode,
                                             Boolean reversePlanFlag,
                                             Integer parkingType,
                                             ApaPlannerHeuristicContext_t *context) {
  Real rmin;
  Real errYaw;
  Real costYaw;
  Real goalPose[3];
  Real workPose[3];
  Real goalLocal[3];
  Real ptTemp[3];
  Real headingErr;
  Real latErr;
  Real targetAngleDeg;
  Real targetAngleRad;
  Real localDirection[3];
  Real globalDirection[3];
  Real cost2Norm;
  Real costLatDis;
  Real costLongDis;
  Real reverseThetaErr;
  Real disThresh;
  Real cost;
  Real costAStar;
  Real costPotentialField;
  Real costRs;

  costAStar = 0.0f;
  costPotentialField = 0.0f;
  costRs = 0.0f;

  rmin = context->MinCircle;
  errYaw = apaCoordWrapToPi(goalNode->Theta - workNode->Theta);
  costYaw = rmin * apaMathAbs(errYaw);

  goalPose[0] = goalNode->X;
  goalPose[1] = goalNode->Y;
  goalPose[2] = goalNode->Theta;
  workPose[0] = workNode->X;
  workPose[1] = workNode->Y;
  workPose[2] = workNode->Theta;

  if (reversePlanFlag) {
    apaCoordGroundToVehPose(slot->TarPos, goalPose, goalLocal);
    headingErr = apaCoordWrapToPi(apaMathAbs(goalLocal[2]));
    latErr = goalLocal[1];
    targetAngleDeg = apaSearchScaleReverseTargetAngle(latErr, parkingType);
    targetAngleRad = targetAngleDeg / APA_REVERSE_TARGET_ANGLE_DIVISOR;

    if (slot->SlotClass == APA_SLOT_CLASS_VERTICAL &&
        slot->SlotSubStatus == APA_SLOT_SUB_END_NONE &&
        headingErr > apaCoordWrapToPi(targetAngleRad)) {
      localDirection[0] = 0.0f;
      localDirection[1] = 0.0f;
      if (parkingType == APA_PARKING_TYPE_RIGHT) {
        localDirection[2] = apaCoordWrapToPi(-targetAngleRad);
        apaCoordVehToGroundPose(slot->TarPos, localDirection, globalDirection);
        errYaw = apaCoordWrapToPi(globalDirection[2] - workNode->Theta);
        costYaw = rmin * apaMathAbs(errYaw);
      } else if (parkingType == APA_PARKING_TYPE_LEFT) {
        localDirection[2] = apaCoordWrapToPi(targetAngleRad);
        apaCoordVehToGroundPose(slot->TarPos, localDirection, globalDirection);
        errYaw = apaCoordWrapToPi(globalDirection[2] - workNode->Theta);
        costYaw = rmin * apaMathAbs(errYaw);
      }
    }
  }

  cost2Norm = apaMathNorm2D(workNode->X - goalNode->X, workNode->Y - goalNode->Y);

  apaCoordGroundToVehPose(goalPose, workPose, ptTemp);
  latErr = ptTemp[1];
  costLatDis = apaMathAbs(latErr);
  costLongDis = apaMathAbs(ptTemp[0]);

  (void)costAStar;
  (void)costPotentialField;
  (void)costRs;
  (void)cost2Norm;
  (void)costLongDis;

  if ((slot->SlotSubStatus == APA_SLOT_SUB_END_FIRST ||
       slot->SlotSubStatus == APA_SLOT_SUB_END_SECOND) &&
      slot->SlotClass == APA_SLOT_CLASS_VERTICAL) {
    reverseThetaErr = apaCoordWrapToPi(context->ReverseTheta - workNode->Theta);
    if (context->ReverseFlag == 0U) {
      cost = apaMathAbs(reverseThetaErr) * rmin;
    } else {
      cost = costLatDis + costYaw;
    }

    if (apaMathLess(apaMathAbs(reverseThetaErr), APA_REVERSE_DONE_YAW_THRESHOLD) &&
        apaMathGreater(workNode->D, 0.0f) && context->ReverseFlag == 0U) {
      context->ReverseFlag = 1U;
    }

    return cost;
  }

  if (slot->SlotClass == APA_SLOT_CLASS_VERTICAL) {
    if (apaMathLess(apaMathAbs(latErr), APA_VERTICAL_LAT_BLEND_THRESHOLD)) {
      return costYaw;
    }
    return costLatDis + costYaw;
  }

  disThresh = APA_PARALLEL_LAT_BLEND_THRESHOLD;
  if (apaMathLess(apaMathAbs(latErr), disThresh)) {
    return costLatDis + ((disThresh - (apaMathAbs(latErr) / disThresh)) * costYaw);
  }
  return costLatDis;
}

#endif
