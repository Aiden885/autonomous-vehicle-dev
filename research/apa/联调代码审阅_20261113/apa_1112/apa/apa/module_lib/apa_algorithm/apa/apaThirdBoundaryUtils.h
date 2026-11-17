/**
 * @file apaThirdBoundaryUtils.h
 * @brief APA third-boundary skeleton helpers
 *
 * This header starts the B1 batch for get_3rd_boundary.m. It does not try to
 * rewrite the full MATLAB orchestration in one shot. Instead it extracts the
 * first reusable layer:
 *   - target-slot context loading
 *   - parking direction / slot occupancy helpers
 *   - vertical-slot upper/lower boundary seed construction
 *   - third-boundary line container append helpers
 *
 * Later B2/B3 work can keep filling collision-search and diagonal-slot
 * compensation on top of this skeleton.
 *
 * @date 2026-04-13
 */
#ifndef APA_THIRD_BOUNDARY_UTILS_H
#define APA_THIRD_BOUNDARY_UTILS_H

#include "baseTypes.h"
#include "apaCoordUtils.h"
#include "apaGeoUtils.h"
#include "apaMathUtils.h"
#include "apaPlannerConfig.h"
#include "rectCollisionCheckForBoundaryTypes.h"
#include "apaSlotTypes.h"
#include "apaVehicleUtils.h"
#include "vehicleCollisionCheckTypes.h"

#define APA_THIRD_BOUNDARY_MAX_LINES 20
#define APA_THIRD_BOUNDARY_ENV_MAX_LEN 12.0f
#define APA_THIRD_BOUNDARY_ENV_MIN_LEN 5.0f
#define APA_THIRD_BOUNDARY_ENV_OFFSET 3.0f
#define APA_THIRD_BOUNDARY_SIDE_MARGIN 0.06f

typedef struct {
  Real Lines[APA_THIRD_BOUNDARY_MAX_LINES][4];
  Integer Count;
} ApaThirdBoundaryLineSet_t;

typedef struct {
  ApaSlot_t Slot;
  Real VehPos[3];
  Real LnLeft[4];
  Real LnRight[4];
  Real SlotBounds[4];
  Integer SlotClass;
  Integer ParkingType;
  Integer ParkingDirec;
  Integer SurroundingType;
  Integer TargetSlotIdx;
  Boolean Valid;
} ApaThirdBoundaryTarget_t;

typedef struct {
  Real LnLeft[4];
  Real LnRight[4];
  Real BoundaryUp[4];
  Real BoundaryBottom[4];
  Real BoundaryDownLt[4];
  Real BoundaryDownRt[4];
  Real FrontDir[2];
  Real FrontNormal[2];
  Real LeftDir[2];
  Real RightDir[2];
  Real EnterEdgeLen;
  Real BottomEdgeLen;
  Integer ParkingDirec;
  Boolean VehInSlot;
  Boolean VehNearSlot;
} ApaThirdBoundaryVerticalSeed_t;

typedef struct {
  Real LnLeft[4];
  Real LnRight[4];
  Real LnLeftCollision[4];
  Real LnRightCollision[4];
  Real CollisionRect[8];
  Real PLong[2];
  Real PLongBase[2];
  Real PLat[2];
  Real PLatBase[2];
  Real PBackBase[2];
} ApaThirdBoundaryEnterCollisionSeed_t;

typedef struct {
  Real LeftTargetClear[4];
  Real RightTargetClear[4];
  Real LeftVehExtended[4];
  Real RightVehExtended[4];
  Real LeftVehClear[4];
  Real RightVehClear[4];
  Real LeftLateralFlip[4];
  Real RightLateralFlip[4];
  Real SlotLateralUnit[2];
  Real BottomLateralUnit[2];
  Real LeftLateralBias;
  Real RightLateralBias;
  Boolean LeftVehCollision;
  Boolean RightVehCollision;
  Boolean LeftFlipMissing;
  Boolean RightFlipMissing;
} ApaThirdBoundaryInSlotSeed_t;

static inline void apaThirdBoundaryResetLineSet(ApaThirdBoundaryLineSet_t *lineSet) {
  Integer lineIdx;

  lineSet->Count = 0;
  for (lineIdx = 0; lineIdx < APA_THIRD_BOUNDARY_MAX_LINES; lineIdx = lineIdx + 1) {
    Integer valueIdx;

    for (valueIdx = 0; valueIdx < 4; valueIdx = valueIdx + 1) {
      lineSet->Lines[lineIdx][valueIdx] = 0.0f;
    }
  }
}

static inline Boolean apaThirdBoundaryAppendLine(ApaThirdBoundaryLineSet_t *lineSet,
                                                 const Real line[4]) {
  Integer dstIdx;

  if (lineSet->Count >= APA_THIRD_BOUNDARY_MAX_LINES) {
    return false;
  }

  dstIdx = lineSet->Count;
  lineSet->Lines[dstIdx][0] = line[0];
  lineSet->Lines[dstIdx][1] = line[1];
  lineSet->Lines[dstIdx][2] = line[2];
  lineSet->Lines[dstIdx][3] = line[3];
  lineSet->Count = lineSet->Count + 1;
  return true;
}

static inline void apaThirdBoundaryCopy3(const Real src[3], Real dst[3]) {
  Integer idx;

  for (idx = 0; idx < 3; idx = idx + 1) {
    dst[idx] = src[idx];
  }
}

static inline void apaThirdBoundaryCopy4(const Real src[4], Real dst[4]) {
  Integer idx;

  for (idx = 0; idx < 4; idx = idx + 1) {
    dst[idx] = src[idx];
  }
}

static inline void apaThirdBoundaryCopy8(const Real src[8], Real dst[8]) {
  Integer idx;

  for (idx = 0; idx < 8; idx = idx + 1) {
    dst[idx] = src[idx];
  }
}

static inline void apaThirdBoundaryInitTarget(ApaThirdBoundaryTarget_t *target) {
  Integer idx;

  apaSlotInit(&target->Slot);
  for (idx = 0; idx < 3; idx = idx + 1) {
    target->VehPos[idx] = 0.0f;
  }
  for (idx = 0; idx < 4; idx = idx + 1) {
    target->LnLeft[idx] = 0.0f;
    target->LnRight[idx] = 0.0f;
    target->SlotBounds[idx] = 0.0f;
  }
  target->SlotClass = APA_SLOT_CLASS_UNKNOWN;
  target->ParkingType = APA_PARKING_TYPE_UNKNOWN;
  target->ParkingDirec = APA_PARKING_DIREC_INVALID;
  target->SurroundingType = APA_SURROUND_UNKNOWN;
  target->TargetSlotIdx = -1;
  target->Valid = false;
}

static inline void apaThirdBoundaryInitVerticalSeed(ApaThirdBoundaryVerticalSeed_t *seed) {
  Integer idx;

  for (idx = 0; idx < 4; idx = idx + 1) {
    seed->LnLeft[idx] = 0.0f;
    seed->LnRight[idx] = 0.0f;
    seed->BoundaryUp[idx] = 0.0f;
    seed->BoundaryBottom[idx] = 0.0f;
    seed->BoundaryDownLt[idx] = 0.0f;
    seed->BoundaryDownRt[idx] = 0.0f;
  }
  for (idx = 0; idx < 2; idx = idx + 1) {
    seed->FrontDir[idx] = 0.0f;
    seed->FrontNormal[idx] = 0.0f;
    seed->LeftDir[idx] = 0.0f;
    seed->RightDir[idx] = 0.0f;
  }
  seed->EnterEdgeLen = 0.0f;
  seed->BottomEdgeLen = 0.0f;
  seed->ParkingDirec = APA_PARKING_DIREC_INVALID;
  seed->VehInSlot = false;
  seed->VehNearSlot = false;
}

static inline void
apaThirdBoundaryInitEnterCollisionSeed(ApaThirdBoundaryEnterCollisionSeed_t *seed) {
  Integer idx;

  for (idx = 0; idx < 4; idx = idx + 1) {
    seed->LnLeft[idx] = 0.0f;
    seed->LnRight[idx] = 0.0f;
    seed->LnLeftCollision[idx] = 0.0f;
    seed->LnRightCollision[idx] = 0.0f;
  }
  for (idx = 0; idx < 8; idx = idx + 1) {
    seed->CollisionRect[idx] = 0.0f;
  }
  for (idx = 0; idx < 2; idx = idx + 1) {
    seed->PLong[idx] = 0.0f;
    seed->PLongBase[idx] = 0.0f;
    seed->PLat[idx] = 0.0f;
    seed->PLatBase[idx] = 0.0f;
    seed->PBackBase[idx] = 0.0f;
  }
}

static inline void apaThirdBoundaryInitInSlotSeed(ApaThirdBoundaryInSlotSeed_t *seed) {
  Integer idx;

  for (idx = 0; idx < 4; idx = idx + 1) {
    seed->LeftTargetClear[idx] = 0.0f;
    seed->RightTargetClear[idx] = 0.0f;
    seed->LeftVehExtended[idx] = 0.0f;
    seed->RightVehExtended[idx] = 0.0f;
    seed->LeftVehClear[idx] = 0.0f;
    seed->RightVehClear[idx] = 0.0f;
    seed->LeftLateralFlip[idx] = 0.0f;
    seed->RightLateralFlip[idx] = 0.0f;
  }
  for (idx = 0; idx < 2; idx = idx + 1) {
    seed->SlotLateralUnit[idx] = 0.0f;
    seed->BottomLateralUnit[idx] = 0.0f;
  }
  seed->LeftLateralBias = 0.0f;
  seed->RightLateralBias = 0.0f;
  seed->LeftVehCollision = false;
  seed->RightVehCollision = false;
  seed->LeftFlipMissing = false;
  seed->RightFlipMissing = false;
}

static inline void
apaThirdBoundaryLineTranslate(const Real srcLine[4], const Real shiftVec[2], Real dstLine[4]) {
  dstLine[0] = srcLine[0] + shiftVec[0];
  dstLine[1] = srcLine[1] + shiftVec[1];
  dstLine[2] = srcLine[2] + shiftVec[0];
  dstLine[3] = srcLine[3] + shiftVec[1];
}

static inline void
apaThirdBoundaryTranslateLineStart(const Real srcLine[4], const Real shiftVec[2], Real dstLine[4]) {
  dstLine[0] = srcLine[0] + shiftVec[0];
  dstLine[1] = srcLine[1] + shiftVec[1];
  dstLine[2] = srcLine[2];
  dstLine[3] = srcLine[3];
}

static inline void
apaThirdBoundaryTranslateLineEnd(const Real srcLine[4], const Real shiftVec[2], Real dstLine[4]) {
  dstLine[0] = srcLine[0];
  dstLine[1] = srcLine[1];
  dstLine[2] = srcLine[2] + shiftVec[0];
  dstLine[3] = srcLine[3] + shiftVec[1];
}

static inline void apaThirdBoundaryBuildQuadFromLines(const Real leftLine[4],
                                                      const Real rightLine[4],
                                                      Real rectPts[8]) {
  rectPts[0] = leftLine[0];
  rectPts[1] = leftLine[1];
  rectPts[2] = leftLine[2];
  rectPts[3] = leftLine[3];
  rectPts[4] = rightLine[2];
  rectPts[5] = rightLine[3];
  rectPts[6] = rightLine[0];
  rectPts[7] = rightLine[1];
}

static inline void apaThirdBoundaryBuildVehicleRectPts(const Real vehPos[3],
                                                       const ApaPlannerVehConfig_t *vehConfig,
                                                       Real frontMargin,
                                                       Real rearMargin,
                                                       Real sideMargin,
                                                       Real rectPts[8]) {
  Real rectPtsX[4];
  Real rectPtsY[4];
  Integer idx;

  apaVehicleCornerPoint(vehPos[0],
                        vehPos[1],
                        vehPos[2],
                        vehConfig->LF,
                        vehConfig->LB,
                        0.5f * vehConfig->W,
                        frontMargin,
                        rearMargin,
                        sideMargin,
                        rectPtsX,
                        rectPtsY);
  idx = 0;
  while (idx < 4) {
    rectPts[2 * idx] = rectPtsX[idx];
    rectPts[2 * idx + 1] = rectPtsY[idx];
    idx = idx + 1;
  }
}

static inline void apaThirdBoundaryBuildHalfRect(const Real lnLeft[4],
                                                 const Real lnRight[4],
                                                 Real leftCollision[4],
                                                 Real rightCollision[4],
                                                 Real rectPts[8]) {
  leftCollision[0] = lnLeft[0];
  leftCollision[1] = lnLeft[1];
  leftCollision[2] = 0.5f * (lnLeft[0] + lnLeft[2]);
  leftCollision[3] = 0.5f * (lnLeft[1] + lnLeft[3]);

  rightCollision[0] = lnRight[0];
  rightCollision[1] = lnRight[1];
  rightCollision[2] = 0.5f * (lnRight[0] + lnRight[2]);
  rightCollision[3] = 0.5f * (lnRight[1] + lnRight[3]);

  apaThirdBoundaryBuildQuadFromLines(leftCollision, rightCollision, rectPts);
}

static inline void apaThirdBoundaryBuildSideSearchRect(const Real nearLine[4],
                                                       const Real farLine[4],
                                                       Real rectPts[8]) {
  rectPts[0] = nearLine[0];
  rectPts[1] = nearLine[1];
  rectPts[2] = nearLine[2];
  rectPts[3] = nearLine[3];
  rectPts[4] = 0.5f * (nearLine[2] + farLine[2]);
  rectPts[5] = 0.5f * (nearLine[3] + farLine[3]);
  rectPts[6] = 0.5f * (nearLine[0] + farLine[0]);
  rectPts[7] = 0.5f * (nearLine[1] + farLine[1]);
}

static inline void apaThirdBoundaryBuildEnterCollisionSeed(
    const Real lnLeft[4], const Real lnRight[4], ApaThirdBoundaryEnterCollisionSeed_t *seed) {
  Real midStartX;
  Real midStartY;
  Real midEndX;
  Real midEndY;
  Real pLongLen;
  Real pLatLen;

  apaThirdBoundaryInitEnterCollisionSeed(seed);
  apaThirdBoundaryCopy4(lnLeft, seed->LnLeft);
  apaThirdBoundaryCopy4(lnRight, seed->LnRight);
  apaThirdBoundaryBuildHalfRect(
      lnLeft, lnRight, seed->LnLeftCollision, seed->LnRightCollision, seed->CollisionRect);

  midStartX = 0.5f * (seed->LnLeftCollision[0] + seed->LnRightCollision[0]);
  midStartY = 0.5f * (seed->LnLeftCollision[1] + seed->LnRightCollision[1]);
  midEndX = 0.5f * (seed->LnLeftCollision[2] + seed->LnRightCollision[2]);
  midEndY = 0.5f * (seed->LnLeftCollision[3] + seed->LnRightCollision[3]);

  seed->PLong[0] = midStartX - midEndX;
  seed->PLong[1] = midStartY - midEndY;
  pLongLen = apaMathNorm2D(seed->PLong[0], seed->PLong[1]);
  seed->PLongBase[0] = seed->PLong[0] / apaMathDivideZeroProcess(pLongLen);
  seed->PLongBase[1] = seed->PLong[1] / apaMathDivideZeroProcess(pLongLen);
  seed->PBackBase[0] = -seed->PLongBase[0];
  seed->PBackBase[1] = -seed->PLongBase[1];

  seed->PLat[0] = seed->LnLeftCollision[0] - seed->LnRightCollision[0];
  seed->PLat[1] = seed->LnLeftCollision[1] - seed->LnRightCollision[1];
  pLatLen = apaMathNorm2D(seed->PLat[0], seed->PLat[1]);
  seed->PLatBase[0] = seed->PLat[0] / apaMathDivideZeroProcess(pLatLen);
  seed->PLatBase[1] = seed->PLat[1] / apaMathDivideZeroProcess(pLatLen);
}

static inline void
apaThirdBoundaryExtendCollisionLines(const ApaThirdBoundaryEnterCollisionSeed_t *seed,
                                     Real transitionPosLen,
                                     Real leftExtended[4],
                                     Real rightExtended[4]) {
  Real leftDir[2];
  Real rightDir[2];
  Real leftLen;
  Real rightLen;
  Real shiftVec[2];

  leftDir[0] = seed->LnLeft[0] - seed->LnLeft[2];
  leftDir[1] = seed->LnLeft[1] - seed->LnLeft[3];
  leftLen = apaMathNorm2D(leftDir[0], leftDir[1]);
  leftDir[0] = leftDir[0] / apaMathDivideZeroProcess(leftLen);
  leftDir[1] = leftDir[1] / apaMathDivideZeroProcess(leftLen);
  shiftVec[0] = leftDir[0] * transitionPosLen;
  shiftVec[1] = leftDir[1] * transitionPosLen;
  apaThirdBoundaryTranslateLineEnd(seed->LnLeft, shiftVec, leftExtended);

  rightDir[0] = seed->LnRight[0] - seed->LnRight[2];
  rightDir[1] = seed->LnRight[1] - seed->LnRight[3];
  rightLen = apaMathNorm2D(rightDir[0], rightDir[1]);
  rightDir[0] = rightDir[0] / apaMathDivideZeroProcess(rightLen);
  rightDir[1] = rightDir[1] / apaMathDivideZeroProcess(rightLen);
  shiftVec[0] = rightDir[0] * transitionPosLen;
  shiftVec[1] = rightDir[1] * transitionPosLen;
  apaThirdBoundaryTranslateLineEnd(seed->LnRight, shiftVec, rightExtended);
}

static inline void apaThirdBoundaryShiftLineUntilClearRect(const Real srcLine[4],
                                                           const Real shiftUnit[2],
                                                           Real shiftSign,
                                                           Real maxBias,
                                                           Real stepLen,
                                                           const Real rectPts[8],
                                                           Real outLine[4]) {
  Real biasLen;
  Real shiftVec[2];
  Boolean lineHit;
  Boolean foundClear;
  Boolean loopActive;

  biasLen = 0.0f;
  foundClear = false;
  apaThirdBoundaryCopy4(srcLine, outLine);
  loopActive = apaMathLess(biasLen, maxBias);
  while (loopActive) {
    shiftVec[0] = shiftUnit[0] * shiftSign * biasLen;
    shiftVec[1] = shiftUnit[1] * shiftSign * biasLen;
    apaThirdBoundaryLineTranslate(srcLine, shiftVec, outLine);
    lineHit = apaGeoLineIntersectRect(outLine[0], outLine[1], outLine[2], outLine[3], rectPts);
    if (!lineHit) {
      foundClear = true;
    } else {
      biasLen = biasLen + stepLen;
    }
    loopActive = apaMathLess(biasLen, maxBias);
    if (loopActive) {
      loopActive = !foundClear;
    }
  }
}

static inline void
apaThirdBoundaryExtendLineStartForward(const Real srcLine[4], Real extendLen, Real outLine[4]) {
  Real dirX;
  Real dirY;
  Real dirLen;
  Real shiftVec[2];

  dirX = srcLine[0] - srcLine[2];
  dirY = srcLine[1] - srcLine[3];
  dirLen = apaMathNorm2D(dirX, dirY);
  shiftVec[0] = dirX / apaMathDivideZeroProcess(dirLen) * extendLen;
  shiftVec[1] = dirY / apaMathDivideZeroProcess(dirLen) * extendLen;
  apaThirdBoundaryTranslateLineStart(srcLine, shiftVec, outLine);
}

static inline Boolean apaThirdBoundaryShiftLineStartUntilVehClear(const Real targetClearLine[4],
                                                                  const Real extendedLine[4],
                                                                  const Real vehRectPts[8],
                                                                  Real stepLen,
                                                                  Real outLine[4]) {
  Real dirX;
  Real dirY;
  Real dirLen;
  Real biasLen;
  Real shiftVec[2];
  Boolean wasCollision;
  Boolean foundClear;
  Boolean loopActive;
  Boolean lineHit;

  dirX = targetClearLine[2] - targetClearLine[0];
  dirY = targetClearLine[3] - targetClearLine[1];
  dirLen = apaMathNorm2D(dirX, dirY);
  wasCollision = apaGeoLineIntersectRect(
      extendedLine[0], extendedLine[1], extendedLine[2], extendedLine[3], vehRectPts);
  foundClear = false;
  apaThirdBoundaryCopy4(targetClearLine, outLine);

  if (wasCollision) {
    biasLen = stepLen;
    loopActive = apaMathLess(biasLen, dirLen);
    while (loopActive) {
      shiftVec[0] = dirX / apaMathDivideZeroProcess(dirLen) * biasLen;
      shiftVec[1] = dirY / apaMathDivideZeroProcess(dirLen) * biasLen;
      apaThirdBoundaryCopy4(targetClearLine, outLine);
      outLine[0] = outLine[0] + shiftVec[0];
      outLine[1] = outLine[1] + shiftVec[1];
      lineHit = apaGeoLineIntersectRect(outLine[0], outLine[1], outLine[2], outLine[3], vehRectPts);
      if (!lineHit) {
        foundClear = true;
      } else {
        biasLen = biasLen + stepLen;
      }
      loopActive = apaMathLess(biasLen, dirLen);
      if (loopActive) {
        loopActive = !foundClear;
      }
    }
  }

  if (wasCollision) {
    if (!foundClear) {
      apaThirdBoundaryCopy4(targetClearLine, outLine);
    }
  } else {
    apaThirdBoundaryCopy4(targetClearLine, outLine);
  }
  return wasCollision;
}

static inline void apaThirdBoundarySearchLineLateralFlip(const Real baseLine[4],
                                                         const Real lateralUnit[2],
                                                         Real expandDirection,
                                                         const Real vehRectPts[8],
                                                         Real maxBias,
                                                         Real stepLen,
                                                         Real outLine[4],
                                                         Real *outBias,
                                                         Boolean *flipMissing) {
  Boolean initialCollision;
  Boolean flipFound;
  Boolean loopActive;
  Boolean lineHit;
  Real biasLen;
  Real shiftVec[2];

  initialCollision =
      apaGeoLineIntersectRect(baseLine[0], baseLine[1], baseLine[2], baseLine[3], vehRectPts);
  flipFound = false;
  biasLen = stepLen;
  apaThirdBoundaryCopy4(baseLine, outLine);

  loopActive = apaMathLess(biasLen, maxBias);
  while (loopActive) {
    shiftVec[0] = lateralUnit[0] * expandDirection * biasLen;
    shiftVec[1] = lateralUnit[1] * expandDirection * biasLen;
    apaThirdBoundaryLineTranslate(baseLine, shiftVec, outLine);
    lineHit = apaGeoLineIntersectRect(outLine[0], outLine[1], outLine[2], outLine[3], vehRectPts);
    if (lineHit != initialCollision) {
      flipFound = true;
    } else {
      biasLen = biasLen + stepLen;
    }
    loopActive = apaMathLess(biasLen, maxBias);
    if (loopActive) {
      loopActive = !flipFound;
    }
  }

  *outBias = biasLen;
  *flipMissing = !flipFound;
}

static inline void
apaThirdBoundaryCalcOriginPose(const Real lnLeft[4], const Real lnRight[4], Real originPose[3]) {
  Real deltaX;
  Real deltaY;
  Real slopeK;

  originPose[0] = 0.5f * (lnLeft[0] + lnRight[0]);
  originPose[1] = 0.5f * (lnLeft[1] + lnRight[1]);
  deltaX = lnLeft[0] - lnLeft[2];
  deltaY = lnLeft[1] - lnLeft[3];
  slopeK = deltaY / apaMathDivideZeroProcess(deltaX);
  originPose[2] = atanf(slopeK);
}

static inline void apaThirdBoundaryAccumulateLocalPoint(const Real originPose[3],
                                                        Real groundX,
                                                        Real groundY,
                                                        Real *minX,
                                                        Real *minY,
                                                        Real *maxX,
                                                        Real *maxY) {
  Real localX;
  Real localY;
  Real localYaw;

  apaCoordGround2Veh(originPose[0],
                     originPose[1],
                     originPose[2],
                     groundX,
                     groundY,
                     0.0f,
                     &localX,
                     &localY,
                     &localYaw);
  (void)localYaw;

  if (apaMathLess(localX, *minX)) {
    *minX = localX;
  }
  if (apaMathLess(localY, *minY)) {
    *minY = localY;
  }
  if (apaMathGreater(localX, *maxX)) {
    *maxX = localX;
  }
  if (apaMathGreater(localY, *maxY)) {
    *maxY = localY;
  }
}

static inline void
apaThirdBoundaryCalcEnvironmentLocalBounds(const Real originPose[3],
                                           const VehicleCollisionCheckInput_t *obsInput,
                                           Real *minX,
                                           Real *minY,
                                           Real *maxX,
                                           Real *maxY) {
  Integer lineIdx;
  Integer ptIdx;

  *minX = 1.0e9f;
  *minY = 1.0e9f;
  *maxX = -1.0e9f;
  *maxY = -1.0e9f;

  lineIdx = 0;
  while (lineIdx < obsInput->BoundaryLineNum) {
    apaThirdBoundaryAccumulateLocalPoint(originPose,
                                         obsInput->BoundaryLineX1[lineIdx],
                                         obsInput->BoundaryLineY1[lineIdx],
                                         minX,
                                         minY,
                                         maxX,
                                         maxY);
    apaThirdBoundaryAccumulateLocalPoint(originPose,
                                         obsInput->BoundaryLineX2[lineIdx],
                                         obsInput->BoundaryLineY2[lineIdx],
                                         minX,
                                         minY,
                                         maxX,
                                         maxY);
    lineIdx = lineIdx + 1;
  }

  ptIdx = 0;
  while (ptIdx < obsInput->BoundarySepPtsNum) {
    apaThirdBoundaryAccumulateLocalPoint(originPose,
                                         obsInput->BoundarySepPtsX[ptIdx],
                                         obsInput->BoundarySepPtsY[ptIdx],
                                         minX,
                                         minY,
                                         maxX,
                                         maxY);
    ptIdx = ptIdx + 1;
  }
}

static inline void
apaThirdBoundaryClampEnvironmentBounds(Real *minX, Real *minY, Real *maxX, Real *maxY) {
  if (apaMathGreater(*maxX, APA_THIRD_BOUNDARY_ENV_MAX_LEN)) {
    *maxX = APA_THIRD_BOUNDARY_ENV_MAX_LEN;
  }
  if (apaMathGreater(*maxY, APA_THIRD_BOUNDARY_ENV_MAX_LEN)) {
    *maxY = APA_THIRD_BOUNDARY_ENV_MAX_LEN;
  }
  if (apaMathLess(*minX, -APA_THIRD_BOUNDARY_ENV_MAX_LEN)) {
    *minX = -APA_THIRD_BOUNDARY_ENV_MAX_LEN;
  }
  if (apaMathLess(*minY, -APA_THIRD_BOUNDARY_ENV_MAX_LEN)) {
    *minY = -APA_THIRD_BOUNDARY_ENV_MAX_LEN;
  }

  if (apaMathLess(*maxX, APA_THIRD_BOUNDARY_ENV_MIN_LEN)) {
    *maxX = APA_THIRD_BOUNDARY_ENV_MIN_LEN;
  }
  if (apaMathLess(*maxY, APA_THIRD_BOUNDARY_ENV_MIN_LEN)) {
    *maxY = APA_THIRD_BOUNDARY_ENV_MIN_LEN;
  }
  if (apaMathGreater(*minX, -APA_THIRD_BOUNDARY_ENV_MIN_LEN)) {
    *minX = -APA_THIRD_BOUNDARY_ENV_MIN_LEN;
  }
  if (apaMathGreater(*minY, -APA_THIRD_BOUNDARY_ENV_MIN_LEN)) {
    *minY = -APA_THIRD_BOUNDARY_ENV_MIN_LEN;
  }

  *maxX = *maxX + APA_THIRD_BOUNDARY_ENV_OFFSET;
  *maxY = *maxY + APA_THIRD_BOUNDARY_ENV_OFFSET;
  *minX = *minX - APA_THIRD_BOUNDARY_ENV_OFFSET;
  *minY = *minY - APA_THIRD_BOUNDARY_ENV_OFFSET;
}

static inline void
apaThirdBoundaryAppendEnvironmentBoundary(const Real lnLeft[4],
                                          const Real lnRight[4],
                                          const VehicleCollisionCheckInput_t *obsInput,
                                          ApaThirdBoundaryLineSet_t *lineSet) {
  Real originPose[3];
  Real minX;
  Real minY;
  Real maxX;
  Real maxY;
  Real localPts[4][2];
  Real groundX;
  Real groundY;
  Real groundYaw;
  Real line[4];
  Integer idx;
  Integer nextIdx;

  apaThirdBoundaryCalcOriginPose(lnLeft, lnRight, originPose);
  apaThirdBoundaryCalcEnvironmentLocalBounds(originPose, obsInput, &minX, &minY, &maxX, &maxY);
  apaThirdBoundaryClampEnvironmentBounds(&minX, &minY, &maxX, &maxY);

  localPts[0][0] = maxX;
  localPts[0][1] = maxY;
  localPts[1][0] = minX;
  localPts[1][1] = maxY;
  localPts[2][0] = minX;
  localPts[2][1] = minY;
  localPts[3][0] = maxX;
  localPts[3][1] = minY;

  idx = 0;
  while (idx < 4) {
    nextIdx = idx + 1;
    if (nextIdx >= 4) {
      nextIdx = 0;
    }

    apaCoordVeh2Ground(originPose[0],
                       originPose[1],
                       originPose[2],
                       localPts[idx][0],
                       localPts[idx][1],
                       0.0f,
                       &groundX,
                       &groundY,
                       &groundYaw);
    line[0] = groundX;
    line[1] = groundY;
    apaCoordVeh2Ground(originPose[0],
                       originPose[1],
                       originPose[2],
                       localPts[nextIdx][0],
                       localPts[nextIdx][1],
                       0.0f,
                       &groundX,
                       &groundY,
                       &groundYaw);
    line[2] = groundX;
    line[3] = groundY;
    apaThirdBoundaryAppendLine(lineSet, line);
    idx = idx + 1;
  }
}

static inline Boolean apaThirdBoundaryLineIntersectsVehicle(const Real line[4],
                                                            const Real vehPos[3],
                                                            const ApaPlannerVehConfig_t *vehConfig,
                                                            Real frontMargin,
                                                            Real rearMargin,
                                                            Real sideMargin) {
  Real rectPtsX[4];
  Real rectPtsY[4];
  Real rectPts[8];
  Integer idx;

  apaVehicleCornerPoint(vehPos[0],
                        vehPos[1],
                        vehPos[2],
                        vehConfig->LF,
                        vehConfig->LB,
                        0.5f * vehConfig->W,
                        frontMargin,
                        rearMargin,
                        sideMargin,
                        rectPtsX,
                        rectPtsY);
  idx = 0;
  while (idx < 4) {
    rectPts[2 * idx] = rectPtsX[idx];
    rectPts[2 * idx + 1] = rectPtsY[idx];
    idx = idx + 1;
  }

  return apaGeoLineIntersectRect(line[0], line[1], line[2], line[3], rectPts);
}

static inline void apaThirdBoundaryDeleteCollisionWithVeh(const Real vehPos[3],
                                                          const ApaPlannerVehConfig_t *vehConfig,
                                                          Real frontMargin,
                                                          Real rearMargin,
                                                          Real sideMargin,
                                                          ApaThirdBoundaryLineSet_t *lineSet) {
  Integer idx;

  idx = 0;
  while (idx < lineSet->Count) {
    Boolean lineHit;

    lineHit = apaThirdBoundaryLineIntersectsVehicle(
        lineSet->Lines[idx], vehPos, vehConfig, frontMargin, rearMargin, sideMargin);
    if (lineHit) {
      Integer lastIdx;

      lastIdx = lineSet->Count - 1;
      lineSet->Lines[idx][0] = lineSet->Lines[lastIdx][0];
      lineSet->Lines[idx][1] = lineSet->Lines[lastIdx][1];
      lineSet->Lines[idx][2] = lineSet->Lines[lastIdx][2];
      lineSet->Lines[idx][3] = lineSet->Lines[lastIdx][3];
      lineSet->Count = lineSet->Count - 1;
    } else {
      idx = idx + 1;
    }
  }
}

static inline void
apaThirdBoundaryDeleteCollisionWithVehDefault(const Real vehPos[3],
                                              const ApaPlannerVehConfig_t *vehConfig,
                                              ApaThirdBoundaryLineSet_t *lineSet) {
  apaThirdBoundaryDeleteCollisionWithVeh(
      vehPos, vehConfig, 0.0f, 0.0f, APA_THIRD_BOUNDARY_SIDE_MARGIN, lineSet);
}

static inline void
apaThirdBoundaryWriteToCollisionInput(const ApaThirdBoundaryLineSet_t *lineSet,
                                      VehicleCollisionCheckInput_t *collisionInput) {
  Integer idx;
  Boolean copyLoop;

  collisionInput->VirtualBoundaryLineNum = 0;
  idx = 0;
  while (idx < VCC_MAX_VIRTUAL_LINES) {
    collisionInput->VirtualBoundaryLineX1[idx] = 0.0f;
    collisionInput->VirtualBoundaryLineY1[idx] = 0.0f;
    collisionInput->VirtualBoundaryLineX2[idx] = 0.0f;
    collisionInput->VirtualBoundaryLineY2[idx] = 0.0f;
    idx = idx + 1;
  }

  idx = 0;
  copyLoop = (idx < lineSet->Count);
  if (copyLoop) {
    copyLoop = (idx < VCC_MAX_VIRTUAL_LINES);
  }
  while (copyLoop) {
    collisionInput->VirtualBoundaryLineX1[idx] = lineSet->Lines[idx][0];
    collisionInput->VirtualBoundaryLineY1[idx] = lineSet->Lines[idx][1];
    collisionInput->VirtualBoundaryLineX2[idx] = lineSet->Lines[idx][2];
    collisionInput->VirtualBoundaryLineY2[idx] = lineSet->Lines[idx][3];
    idx = idx + 1;
    copyLoop = (idx < lineSet->Count);
    if (copyLoop) {
      copyLoop = (idx < VCC_MAX_VIRTUAL_LINES);
    }
  }
  collisionInput->VirtualBoundaryLineNum = idx;
}

static inline void
apaThirdBoundaryRectTranslate(const Real srcRect[8], const Real shiftVec[2], Real dstRect[8]) {
  Integer idx;

  idx = 0;
  while (idx < 4) {
    dstRect[2 * idx] = srcRect[2 * idx] + shiftVec[0];
    dstRect[2 * idx + 1] = srcRect[2 * idx + 1] + shiftVec[1];
    idx = idx + 1;
  }
}

static inline Boolean
apaThirdBoundaryRectCollisionWithObs(const Real rectPts[8],
                                     const VehicleCollisionCheckInput_t *obsInput) {
  RectCollisionCheckForBoundaryInput_t rectInput;
  RectCollisionCheckForBoundaryOutput_t rectOutput;
  Integer idx;

  rectInput = (RectCollisionCheckForBoundaryInput_t){0};
  rectInput.RectX1 = rectPts[0];
  rectInput.RectY1 = rectPts[1];
  rectInput.RectX2 = rectPts[2];
  rectInput.RectY2 = rectPts[3];
  rectInput.RectX3 = rectPts[4];
  rectInput.RectY3 = rectPts[5];
  rectInput.RectX4 = rectPts[6];
  rectInput.RectY4 = rectPts[7];

  rectInput.BoundaryLineNum = obsInput->BoundaryLineNum;
  if (rectInput.BoundaryLineNum > MAX_BOUNDARY_LINES) {
    rectInput.BoundaryLineNum = MAX_BOUNDARY_LINES;
  }
  idx = 0;
  while (idx < rectInput.BoundaryLineNum) {
    rectInput.BoundaryLineX1[idx] = obsInput->BoundaryLineX1[idx];
    rectInput.BoundaryLineY1[idx] = obsInput->BoundaryLineY1[idx];
    rectInput.BoundaryLineX2[idx] = obsInput->BoundaryLineX2[idx];
    rectInput.BoundaryLineY2[idx] = obsInput->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  rectInput.SeparatePtsNum = obsInput->BoundarySepPtsNum;
  if (rectInput.SeparatePtsNum > MAX_SEPARATE_PTS) {
    rectInput.SeparatePtsNum = MAX_SEPARATE_PTS;
  }
  idx = 0;
  while (idx < rectInput.SeparatePtsNum) {
    rectInput.SeparatePtsX[idx] = obsInput->BoundarySepPtsX[idx];
    rectInput.SeparatePtsY[idx] = obsInput->BoundarySepPtsY[idx];
    idx = idx + 1;
  }

  rectCollisionCheckForBoundary(&rectInput, &rectOutput);
  return rectOutput.IsCollision;
}

static inline Real
apaThirdBoundaryFindRectCollisionPos(const Real collisionRect[8],
                                     const Real baseDir[2],
                                     Real maxLen,
                                     Real initLen,
                                     Real stepLen,
                                     Real safetyOffset,
                                     const VehicleCollisionCheckInput_t *obsInput) {
  Real biasLen;
  Boolean hitFlag;
  Boolean loopActive;

  biasLen = initLen;
  hitFlag = false;
  loopActive = !apaMathGreater(biasLen, maxLen);
  while (loopActive) {
    Real shiftVec[2];
    Real movedRect[8];
    Boolean isCollision;

    shiftVec[0] = baseDir[0] * biasLen;
    shiftVec[1] = baseDir[1] * biasLen;
    apaThirdBoundaryRectTranslate(collisionRect, shiftVec, movedRect);
    isCollision = apaThirdBoundaryRectCollisionWithObs(movedRect, obsInput);
    if (isCollision) {
      hitFlag = true;
    } else {
      biasLen = biasLen + stepLen;
    }
    loopActive = !apaMathGreater(biasLen, maxLen);
    if (loopActive) {
      loopActive = !hitFlag;
    }
  }

  if (hitFlag) {
    biasLen = biasLen - stepLen - safetyOffset;
  }
  return biasLen;
}

static inline void
apaThirdBoundarySlotEnterCollisionCheck(const Real lnLeft[4],
                                        const Real lnRight[4],
                                        Integer surroundingType,
                                        Integer parkingType,
                                        Boolean useSideExtendFlag,
                                        const ApaPlannerConfConfig_t *confConfig,
                                        const VehicleCollisionCheckInput_t *obsInput,
                                        Real outLeft[4],
                                        Real outRight[4]) {
  ApaThirdBoundaryEnterCollisionSeed_t seed;
  Real searchRect[8];
  Real shiftVec[2];
  Real biasLen;

  (void)surroundingType;
  (void)parkingType;

  apaThirdBoundaryBuildEnterCollisionSeed(lnLeft, lnRight, &seed);
  apaThirdBoundaryCopy4(lnLeft, outLeft);
  apaThirdBoundaryCopy4(lnRight, outRight);

  apaThirdBoundaryRectTranslate(seed.CollisionRect, seed.PLong, searchRect);
  shiftVec[0] = 0.5f * seed.PLat[0];
  shiftVec[1] = 0.5f * seed.PLat[1];
  apaThirdBoundaryRectTranslate(searchRect, shiftVec, searchRect);
  biasLen = apaThirdBoundaryFindRectCollisionPos(
      searchRect, seed.PBackBase, 3.0f, -0.3f, 0.1f, 0.0f, obsInput);
  shiftVec[0] = seed.PBackBase[0] * biasLen;
  shiftVec[1] = seed.PBackBase[1] * biasLen;
  apaThirdBoundaryTranslateLineStart(outLeft, shiftVec, outLeft);

  apaThirdBoundaryRectTranslate(seed.CollisionRect, seed.PLong, searchRect);
  shiftVec[0] = -0.5f * seed.PLat[0];
  shiftVec[1] = -0.5f * seed.PLat[1];
  apaThirdBoundaryRectTranslate(searchRect, shiftVec, searchRect);
  biasLen = apaThirdBoundaryFindRectCollisionPos(
      searchRect, seed.PBackBase, 3.0f, -0.3f, 0.1f, 0.0f, obsInput);
  shiftVec[0] = seed.PBackBase[0] * biasLen;
  shiftVec[1] = seed.PBackBase[1] * biasLen;
  apaThirdBoundaryTranslateLineStart(outRight, shiftVec, outRight);

  if (!useSideExtendFlag) {
    return;
  }

  {
    Real transitionPosLen;
    Real leftExtended[4];
    Real rightExtended[4];

    if (confConfig != (const ApaPlannerConfConfig_t *)0) {
      transitionPosLen = confConfig->TransitionPosLen;
    } else {
      transitionPosLen = 0.0f;
    }

    apaThirdBoundaryExtendCollisionLines(&seed, transitionPosLen, leftExtended, rightExtended);

    apaThirdBoundaryBuildSideSearchRect(leftExtended, rightExtended, searchRect);
    biasLen = apaThirdBoundaryFindRectCollisionPos(
        searchRect, seed.PLatBase, 1.0f, 0.0f, 0.1f, 0.25f, obsInput);
    shiftVec[0] = seed.PLatBase[0] * biasLen;
    shiftVec[1] = seed.PLatBase[1] * biasLen;
    apaThirdBoundaryLineTranslate(outLeft, shiftVec, outLeft);

    apaThirdBoundaryBuildSideSearchRect(rightExtended, leftExtended, searchRect);
    shiftVec[0] = -seed.PLatBase[0];
    shiftVec[1] = -seed.PLatBase[1];
    biasLen = apaThirdBoundaryFindRectCollisionPos(
        searchRect, shiftVec, 1.0f, 0.0f, 0.1f, 0.25f, obsInput);
    shiftVec[0] = -seed.PLatBase[0] * biasLen;
    shiftVec[1] = -seed.PLatBase[1] * biasLen;
    apaThirdBoundaryLineTranslate(outRight, shiftVec, outRight);
  }
}

static inline void apaThirdBoundaryPrepareInSlotSeed(const Real vehPos[3],
                                                     const Real tarPos[3],
                                                     const Real lnLeft[4],
                                                     const Real lnRight[4],
                                                     const ApaPlannerVehConfig_t *vehConfig,
                                                     ApaThirdBoundaryInSlotSeed_t *seed) {
  Real tarRectPts[8];
  Real vehRectPts[8];
  Real slotLatLen;
  Real bottomLatLen;
  Real leftLateralDirection;
  Real rightLateralDirection;

  apaThirdBoundaryInitInSlotSeed(seed);
  seed->SlotLateralUnit[0] = lnLeft[0] - lnRight[0];
  seed->SlotLateralUnit[1] = lnLeft[1] - lnRight[1];
  slotLatLen = apaMathNorm2D(seed->SlotLateralUnit[0], seed->SlotLateralUnit[1]);
  seed->SlotLateralUnit[0] = seed->SlotLateralUnit[0] / apaMathDivideZeroProcess(slotLatLen);
  seed->SlotLateralUnit[1] = seed->SlotLateralUnit[1] / apaMathDivideZeroProcess(slotLatLen);

  seed->BottomLateralUnit[0] = lnLeft[2] - lnRight[2];
  seed->BottomLateralUnit[1] = lnLeft[3] - lnRight[3];
  bottomLatLen = apaMathNorm2D(seed->BottomLateralUnit[0], seed->BottomLateralUnit[1]);
  seed->BottomLateralUnit[0] = seed->BottomLateralUnit[0] / apaMathDivideZeroProcess(bottomLatLen);
  seed->BottomLateralUnit[1] = seed->BottomLateralUnit[1] / apaMathDivideZeroProcess(bottomLatLen);

  apaThirdBoundaryBuildVehicleRectPts(tarPos, vehConfig, 0.0f, 0.0f, 0.0f, tarRectPts);
  apaThirdBoundaryShiftLineUntilClearRect(
      lnLeft, seed->SlotLateralUnit, 1.0f, 0.8f, 0.1f, tarRectPts, seed->LeftTargetClear);
  apaThirdBoundaryShiftLineUntilClearRect(
      lnRight, seed->SlotLateralUnit, -1.0f, 0.8f, 0.1f, tarRectPts, seed->RightTargetClear);

  apaThirdBoundaryBuildVehicleRectPts(vehPos, vehConfig, 0.0f, 0.0f, 0.0f, vehRectPts);
  apaThirdBoundaryExtendLineStartForward(lnLeft, 4.0f, seed->LeftVehExtended);
  apaThirdBoundaryExtendLineStartForward(lnRight, 4.0f, seed->RightVehExtended);
  seed->LeftVehCollision = apaThirdBoundaryShiftLineStartUntilVehClear(
      seed->LeftTargetClear, seed->LeftVehExtended, vehRectPts, 0.1f, seed->LeftVehClear);
  seed->RightVehCollision = apaThirdBoundaryShiftLineStartUntilVehClear(
      seed->RightTargetClear, seed->RightVehExtended, vehRectPts, 0.1f, seed->RightVehClear);
  if (!seed->LeftVehCollision) {
    apaThirdBoundaryCopy4(seed->LeftTargetClear, seed->LeftVehClear);
  }
  if (!seed->RightVehCollision) {
    apaThirdBoundaryCopy4(seed->RightTargetClear, seed->RightVehClear);
  }

  leftLateralDirection = -1.0f;
  if (seed->LeftVehCollision) {
    leftLateralDirection = 1.0f;
  }
  rightLateralDirection = 1.0f;
  if (seed->RightVehCollision) {
    rightLateralDirection = -1.0f;
  }

  apaThirdBoundarySearchLineLateralFlip(seed->LeftVehExtended,
                                        seed->BottomLateralUnit,
                                        leftLateralDirection,
                                        vehRectPts,
                                        2.2f,
                                        0.1f,
                                        seed->LeftLateralFlip,
                                        &seed->LeftLateralBias,
                                        &seed->LeftFlipMissing);
  apaThirdBoundarySearchLineLateralFlip(seed->RightVehExtended,
                                        seed->BottomLateralUnit,
                                        rightLateralDirection,
                                        vehRectPts,
                                        2.2f,
                                        0.1f,
                                        seed->RightLateralFlip,
                                        &seed->RightLateralBias,
                                        &seed->RightFlipMissing);
}

static inline void
apaThirdBoundaryAppendInSlotLinesByScene(const ApaThirdBoundaryInSlotSeed_t *seed,
                                         Integer surroundingType,
                                         Boolean isNormalScene,
                                         ApaThirdBoundaryLineSet_t *lineSet) {
  if (!isNormalScene) {
    if (surroundingType == APA_SURROUND_OBS_EXIST_RIGHT_REAR_ONLY) {
      apaThirdBoundaryAppendLine(lineSet, seed->RightVehClear);
    } else if (surroundingType == APA_SURROUND_OBS_EXIST_LEFT_FRONT_ONLY) {
      apaThirdBoundaryAppendLine(lineSet, seed->LeftVehClear);
    } else if (surroundingType == APA_SURROUND_OBS_EXIST_NONE) {
      return;
    } else {
      apaThirdBoundaryAppendLine(lineSet, seed->LeftVehClear);
      apaThirdBoundaryAppendLine(lineSet, seed->RightVehClear);
    }
    return;
  }

  apaThirdBoundaryAppendLine(lineSet, seed->LeftVehClear);
  apaThirdBoundaryAppendLine(lineSet, seed->RightVehClear);
}

static inline void apaThirdBoundaryCalcVerticalBasis(const Real lnLeft[4],
                                                     const Real lnRight[4],
                                                     Real frontDir[2],
                                                     Real frontNormal[2],
                                                     Real leftDir[2],
                                                     Real rightDir[2],
                                                     Real *enterEdgeLen,
                                                     Real *bottomEdgeLen) {
  Real rawFrontX;
  Real rawFrontY;
  Real rawLeftX;
  Real rawLeftY;
  Real rawRightX;
  Real rawRightY;
  Real rawNormalX;
  Real rawNormalY;
  Real dotValue;

  rawFrontX = lnLeft[0] - lnLeft[2];
  rawFrontY = lnLeft[1] - lnLeft[3];
  rawLeftX = lnLeft[0] - lnRight[0];
  rawLeftY = lnLeft[1] - lnRight[1];
  rawRightX = lnRight[0] - lnLeft[0];
  rawRightY = lnRight[1] - lnLeft[1];
  rawNormalX = -rawLeftY;
  rawNormalY = rawLeftX;
  dotValue = rawNormalX * rawFrontX + rawNormalY * rawFrontY;
  if (apaMathLess(dotValue, 0.0f)) {
    rawNormalX = -rawNormalX;
    rawNormalY = -rawNormalY;
  }

  *enterEdgeLen = apaMathNorm2D(rawLeftX, rawLeftY);
  *bottomEdgeLen = apaMathNorm2D(lnLeft[2] - lnRight[2], lnLeft[3] - lnRight[3]);

  frontDir[0] = rawFrontX / apaMathDivideZeroProcess(apaMathNorm2D(rawFrontX, rawFrontY));
  frontDir[1] = rawFrontY / apaMathDivideZeroProcess(apaMathNorm2D(rawFrontX, rawFrontY));
  frontNormal[0] = rawNormalX / apaMathDivideZeroProcess(apaMathNorm2D(rawNormalX, rawNormalY));
  frontNormal[1] = rawNormalY / apaMathDivideZeroProcess(apaMathNorm2D(rawNormalX, rawNormalY));
  leftDir[0] = rawLeftX / apaMathDivideZeroProcess(*enterEdgeLen);
  leftDir[1] = rawLeftY / apaMathDivideZeroProcess(*enterEdgeLen);
  rightDir[0] = rawRightX / apaMathDivideZeroProcess(*enterEdgeLen);
  rightDir[1] = rawRightY / apaMathDivideZeroProcess(*enterEdgeLen);
}

static inline void apaThirdBoundaryBuildVerticalBottomSeed(const Real lnLeft[4],
                                                           const Real lnRight[4],
                                                           Real boundaryBottom[4]) {
  Real biasX;
  Real biasY;
  Real biasLen;
  Real unitX;
  Real unitY;
  Real expandLen;

  biasX = lnLeft[2] - lnRight[2];
  biasY = lnLeft[3] - lnRight[3];
  biasLen = apaMathNorm2D(biasX, biasY);
  unitX = biasX / apaMathDivideZeroProcess(biasLen);
  unitY = biasY / apaMathDivideZeroProcess(biasLen);
  expandLen = 1.5f * biasLen;

  boundaryBottom[0] = lnLeft[2] + unitX * expandLen;
  boundaryBottom[1] = lnLeft[3] + unitY * expandLen;
  boundaryBottom[2] = lnRight[2] - unitX * expandLen;
  boundaryBottom[3] = lnRight[3] - unitY * expandLen;
}

static inline void apaThirdBoundaryBuildVerticalDownSeeds(const Real lnLeft[4],
                                                          const Real lnRight[4],
                                                          const Real frontNormal[2],
                                                          Real boundaryDownLt[4],
                                                          Real boundaryDownRt[4]) {
  Real shiftVec[2];
  Real baseLine[4];

  baseLine[0] = lnLeft[0];
  baseLine[1] = lnLeft[1];
  baseLine[2] = lnRight[0];
  baseLine[3] = lnRight[1];
  shiftVec[0] = frontNormal[0] * 0.3f;
  shiftVec[1] = frontNormal[1] * 0.3f;

  apaThirdBoundaryLineTranslate(baseLine, shiftVec, boundaryDownLt);
  apaThirdBoundaryLineTranslate(baseLine, shiftVec, boundaryDownRt);
}

static inline void apaThirdBoundaryBuildVerticalUpperSeed(const Real lnLeft[4],
                                                          const Real lnRight[4],
                                                          const Real frontNormal[2],
                                                          const Real leftDir[2],
                                                          const Real rightDir[2],
                                                          Integer parkingDirec,
                                                          Integer slotSubStatus,
                                                          Real boundaryUp[4]) {
  Real frontExtend;
  Real startShift[2];
  Real endShift[2];
  Real baseLine[4];

  baseLine[0] = lnLeft[0];
  baseLine[1] = lnLeft[1];
  baseLine[2] = lnRight[0];
  baseLine[3] = lnRight[1];

  if (parkingDirec == APA_PARKING_DIREC_FRONT) {
    frontExtend = 7.0f;
  } else {
    frontExtend = 5.5f;
  }
  if (slotSubStatus != APA_SLOT_SUB_END_NONE) {
    frontExtend = frontExtend + 3.5f;
    if (apaMathGreater(frontExtend, 10.0f)) {
      frontExtend = 10.0f;
    }
  }

  startShift[0] = frontNormal[0] * frontExtend + leftDir[0] * 15.0f;
  startShift[1] = frontNormal[1] * frontExtend + leftDir[1] * 15.0f;
  endShift[0] = frontNormal[0] * frontExtend + rightDir[0] * 15.0f;
  endShift[1] = frontNormal[1] * frontExtend + rightDir[1] * 15.0f;

  boundaryUp[0] = baseLine[0] + startShift[0];
  boundaryUp[1] = baseLine[1] + startShift[1];
  boundaryUp[2] = baseLine[2] + endShift[0];
  boundaryUp[3] = baseLine[3] + endShift[1];
}

static inline void
apaThirdBoundaryAdjustVerticalUpperForVehicle(const Real vehPos[3],
                                              const Real slotPts[8],
                                              const Real tarPos[3],
                                              const ApaPlannerVehConfig_t *vehConfig,
                                              const Real frontNormal[2],
                                              Real boundaryUp[4]) {
  Real slotBounds[4];
  Boolean vehNearSlot;
  Boolean initialCollision;
  Boolean referenceCollision;
  Boolean tempCollision;
  Boolean foundFlip;
  Boolean loopActive;
  Real expandDirection;
  Real biasLen;
  Real shiftVec[2];
  Real boundaryTemp[4];
  Real angleAbs;
  Real kLBias;
  Real finalBiasLen;

  apaSlotCalcSlotBounds(slotPts, slotBounds);
  vehNearSlot = apaSlotCalcVehNearSlotFlag(vehPos, slotPts, slotBounds, vehConfig);
  if (!vehNearSlot) {
    initialCollision =
        apaThirdBoundaryLineIntersectsVehicle(boundaryUp, vehPos, vehConfig, 0.0f, 0.0f, 0.0f);
    expandDirection = -1.0f;
    if (initialCollision) {
      expandDirection = 1.0f;
    }

    referenceCollision = false;
    biasLen = 0.1f;
    foundFlip = false;
    loopActive = apaMathLess(biasLen, 5.0f);
    while (loopActive) {
      shiftVec[0] = frontNormal[0] * expandDirection * biasLen;
      shiftVec[1] = frontNormal[1] * expandDirection * biasLen;
      apaThirdBoundaryLineTranslate(boundaryUp, shiftVec, boundaryTemp);
      tempCollision =
          apaThirdBoundaryLineIntersectsVehicle(boundaryTemp, vehPos, vehConfig, 0.0f, 0.0f, 0.0f);
      if (tempCollision != referenceCollision) {
        foundFlip = true;
      } else {
        biasLen = biasLen + 0.1f;
      }
      loopActive = apaMathLess(biasLen, 5.0f);
      if (loopActive) {
        loopActive = !foundFlip;
      }
    }

    if (foundFlip) {
      angleAbs = apaMathAbs(apaCoordWrapToPi(tarPos[2] - vehPos[2]));
      kLBias = 2.0f;
      if (apaMathGreater(angleAbs, 70.0f / 180.0f * APA_PI)) {
        kLBias = 3.0f;
      }

      finalBiasLen = 0.0f;
      if (referenceCollision) {
        finalBiasLen = biasLen + kLBias;
      } else if (apaMathLess(biasLen, kLBias)) {
        finalBiasLen = kLBias - biasLen;
      }

      shiftVec[0] = frontNormal[0] * finalBiasLen;
      shiftVec[1] = frontNormal[1] * finalBiasLen;
      apaThirdBoundaryLineTranslate(boundaryUp, shiftVec, boundaryUp);
    }
  }
}

static inline Boolean apaThirdBoundaryLoadTargetSlot(const ApaSlotListArr_t *slotList,
                                                     Integer targetSlotIdx,
                                                     const Real vehPos[3],
                                                     Integer surroundingType,
                                                     Integer parkingType,
                                                     ApaThirdBoundaryTarget_t *target) {
  if (targetSlotIdx < 0) {
    return false;
  }
  if (targetSlotIdx >= APA_MAX_SLOTS) {
    return false;
  }
  if (slotList->EnterEdgeList[targetSlotIdx] <= 0) {
    return false;
  }

  apaThirdBoundaryInitTarget(target);
  apaSlotInitFromArr(&target->Slot, slotList, targetSlotIdx);
  apaThirdBoundaryCopy3(vehPos, target->VehPos);
  target->SlotClass = apaSlotGetSlotClass(&target->Slot);
  target->ParkingType = parkingType;
  target->SurroundingType = surroundingType;
  target->TargetSlotIdx = targetSlotIdx;

  apaSlotCalcSideLine(target->Slot.SlotPts,
                      target->Slot.EnterEdge,
                      target->Slot.TarPos,
                      target->SlotClass,
                      target->LnLeft,
                      target->LnRight);
  apaSlotCalcSlotBounds(target->Slot.SlotPts, target->SlotBounds);
  target->ParkingDirec =
      apaSlotCalcParkingDirecFromSideLines(target->LnLeft, target->LnRight, target->Slot.TarPos);
  target->Slot.SlotClass = target->SlotClass;
  target->Slot.ParkingType = parkingType;
  target->Slot.ParkingDirec = target->ParkingDirec;
  target->Slot.SurroundingType = surroundingType;
  target->Valid = true;
  return true;
}

static inline Boolean apaThirdBoundaryPrepareVerticalSeed(const ApaThirdBoundaryTarget_t *target,
                                                          const ApaPlannerVehConfig_t *vehConfig,
                                                          ApaThirdBoundaryVerticalSeed_t *seed) {
  if (!target->Valid) {
    return false;
  }
  if (target->SlotClass != APA_SLOT_CLASS_VERTICAL) {
    return false;
  }

  apaThirdBoundaryInitVerticalSeed(seed);
  apaThirdBoundaryCopy4(target->LnLeft, seed->LnLeft);
  apaThirdBoundaryCopy4(target->LnRight, seed->LnRight);
  seed->ParkingDirec = target->ParkingDirec;

  apaThirdBoundaryCalcVerticalBasis(target->LnLeft,
                                    target->LnRight,
                                    seed->FrontDir,
                                    seed->FrontNormal,
                                    seed->LeftDir,
                                    seed->RightDir,
                                    &seed->EnterEdgeLen,
                                    &seed->BottomEdgeLen);
  apaThirdBoundaryBuildVerticalUpperSeed(target->LnLeft,
                                         target->LnRight,
                                         seed->FrontNormal,
                                         seed->LeftDir,
                                         seed->RightDir,
                                         target->ParkingDirec,
                                         target->Slot.SlotSubStatus,
                                         seed->BoundaryUp);
  apaThirdBoundaryBuildVerticalBottomSeed(target->LnLeft, target->LnRight, seed->BoundaryBottom);
  apaThirdBoundaryBuildVerticalDownSeeds(target->LnLeft,
                                         target->LnRight,
                                         seed->FrontNormal,
                                         seed->BoundaryDownLt,
                                         seed->BoundaryDownRt);
  seed->VehInSlot =
      apaSlotCalcVehInSlotFlag(target->VehPos, target->Slot.SlotPts, target->SlotBounds, vehConfig);
  seed->VehNearSlot = apaSlotCalcVehNearSlotFlag(
      target->VehPos, target->Slot.SlotPts, target->SlotBounds, vehConfig);
  return true;
}

static inline void apaThirdBoundaryCalcSlotPoseError(const Real vehPos[3],
                                                     const Real tarPos[3],
                                                     const Real lnLeft[4],
                                                     const Real lnRight[4],
                                                     Real *yawErr,
                                                     Real *distErr) {
  Real pBias[2];
  Real ptLt[2];
  Real ptRt[2];
  Real lineCos;
  Real lineSin;
  Real len;
  (void)len;
  Real A, B, cLt, cRt;
  Real disLt, disRt;

  *yawErr = vehPos[2] - tarPos[2];

  lineCos = cosf(tarPos[2]);
  lineSin = sinf(tarPos[2]);
  A = -lineSin;
  B = lineCos;

  pBias[0] = lnLeft[0] - lnRight[0];
  pBias[1] = lnLeft[1] - lnRight[1];

  ptLt[0] = tarPos[0] + pBias[0];
  ptLt[1] = tarPos[1] + pBias[1];
  ptRt[0] = tarPos[0] - pBias[0];
  ptRt[1] = tarPos[1] - pBias[1];

  cLt = -(A * ptLt[0] + B * ptLt[1]);
  cRt = -(A * ptRt[0] + B * ptRt[1]);

  disLt = fabsf(A * vehPos[0] + B * vehPos[1] + cLt);
  disRt = fabsf(A * vehPos[0] + B * vehPos[1] + cRt);

  if (apaMathGreater(disLt, disRt)) {
    *distErr = -(disLt - 0.5f * (disLt + disRt));
  } else {
    *distErr = disRt - 0.5f * (disLt + disRt);
  }
}

static inline void apaThirdBoundaryGetVirtualBoundaryInSlot(const Real vehPos[3],
                                                            const Real tarPos[3],
                                                            Integer surroundingType,
                                                            const Real lnLeft[4],
                                                            const Real lnRight[4],
                                                            Boolean isNormalScene,
                                                            const ApaPlannerVehConfig_t *vehConfig,
                                                            ApaThirdBoundaryLineSet_t *lineSet) {
  ApaThirdBoundaryInSlotSeed_t seed;
  Real yawErr;
  Real distErr;
  Real kExistRiskDis = 0.8f;
  Real kNoRiskDis = 0.2f;
  Real disThresh = 0.05f;
  Real yawThresh = 1.5f / 180.0f * APA_PI;
  Real baseShiftLeft[2];
  Real baseShiftRight[2];
  Real ptBias[2];
  Real leftLineOut[4];
  Real rightLineOut[4];
  Boolean bothFlipMissing;

  apaThirdBoundaryPrepareInSlotSeed(vehPos, tarPos, lnLeft, lnRight, vehConfig, &seed);

  bothFlipMissing = false;
  if (seed.LeftFlipMissing) {
    if (seed.RightFlipMissing) {
      bothFlipMissing = true;
    }
  }
  if (bothFlipMissing) {
    if (!isNormalScene) {
      apaThirdBoundaryAppendInSlotLinesByScene(&seed, surroundingType, isNormalScene, lineSet);
      return;
    }
    apaThirdBoundaryAppendLine(lineSet, seed.LeftVehClear);
    apaThirdBoundaryAppendLine(lineSet, seed.RightVehClear);
    return;
  }

  apaThirdBoundaryCalcSlotPoseError(vehPos, tarPos, lnLeft, lnRight, &yawErr, &distErr);

  baseShiftLeft[0] = seed.BottomLateralUnit[0];
  baseShiftLeft[1] = seed.BottomLateralUnit[1];
  baseShiftRight[0] = -seed.BottomLateralUnit[0];
  baseShiftRight[1] = -seed.BottomLateralUnit[1];

  apaThirdBoundaryCopy4(seed.LeftLateralFlip, leftLineOut);
  apaThirdBoundaryCopy4(seed.RightLateralFlip, rightLineOut);

  if (apaMathGreater(distErr, disThresh)) {
    if (apaMathGreater(yawErr, yawThresh)) {
      if (seed.LeftVehCollision) {
        ptBias[0] = baseShiftLeft[0] * kExistRiskDis;
        ptBias[1] = baseShiftLeft[1] * kExistRiskDis;
        apaThirdBoundaryLineTranslate(leftLineOut, ptBias, leftLineOut);
      } else {
        if (apaMathLess(seed.LeftLateralBias, kExistRiskDis)) {
          ptBias[0] = baseShiftLeft[0] * kExistRiskDis;
          ptBias[1] = baseShiftLeft[1] * kExistRiskDis;
          apaThirdBoundaryLineTranslate(leftLineOut, ptBias, leftLineOut);
        } else {
          apaThirdBoundaryCopy4(seed.LeftVehClear, leftLineOut);
        }
      }
    } else {
      if (seed.LeftVehCollision) {
        ptBias[0] = baseShiftLeft[0] * kNoRiskDis;
        ptBias[1] = baseShiftLeft[1] * kNoRiskDis;
        apaThirdBoundaryLineTranslate(leftLineOut, ptBias, leftLineOut);
      } else {
        if (apaMathLess(seed.LeftLateralBias, kNoRiskDis)) {
          ptBias[0] = baseShiftLeft[0] * kNoRiskDis;
          ptBias[1] = baseShiftLeft[1] * kNoRiskDis;
          apaThirdBoundaryLineTranslate(leftLineOut, ptBias, leftLineOut);
        } else {
          apaThirdBoundaryCopy4(seed.LeftVehClear, leftLineOut);
        }
      }
    }

    if (seed.RightVehCollision) {
      ptBias[0] = baseShiftRight[0] * kExistRiskDis;
      ptBias[1] = baseShiftRight[1] * kExistRiskDis;
      apaThirdBoundaryLineTranslate(rightLineOut, ptBias, rightLineOut);
    } else {
      if (apaMathLess(seed.RightLateralBias, kExistRiskDis)) {
        ptBias[0] = baseShiftRight[0] * kExistRiskDis;
        ptBias[1] = baseShiftRight[1] * kExistRiskDis;
        apaThirdBoundaryLineTranslate(rightLineOut, ptBias, rightLineOut);
      } else {
        apaThirdBoundaryCopy4(seed.RightVehClear, rightLineOut);
      }
    }

  } else if (apaMathLess(distErr, -disThresh)) {
    if (seed.LeftVehCollision) {
      ptBias[0] = baseShiftLeft[0] * kExistRiskDis;
      ptBias[1] = baseShiftLeft[1] * kExistRiskDis;
      apaThirdBoundaryLineTranslate(leftLineOut, ptBias, leftLineOut);
    } else {
      if (apaMathLess(seed.LeftLateralBias, kExistRiskDis)) {
        ptBias[0] = baseShiftLeft[0] * kExistRiskDis;
        ptBias[1] = baseShiftLeft[1] * kExistRiskDis;
        apaThirdBoundaryLineTranslate(leftLineOut, ptBias, leftLineOut);
      } else {
        apaThirdBoundaryCopy4(seed.LeftVehClear, leftLineOut);
      }
    }

    if (apaMathLess(yawErr, -yawThresh)) {
      if (seed.RightVehCollision) {
        ptBias[0] = baseShiftRight[0] * kExistRiskDis;
        ptBias[1] = baseShiftRight[1] * kExistRiskDis;
        apaThirdBoundaryLineTranslate(rightLineOut, ptBias, rightLineOut);
      } else {
        if (apaMathLess(seed.RightLateralBias, kExistRiskDis)) {
          ptBias[0] = baseShiftRight[0] * kExistRiskDis;
          ptBias[1] = baseShiftRight[1] * kExistRiskDis;
          apaThirdBoundaryLineTranslate(rightLineOut, ptBias, rightLineOut);
        } else {
          apaThirdBoundaryCopy4(seed.RightVehClear, rightLineOut);
        }
      }
    } else {
      if (seed.RightVehCollision) {
        ptBias[0] = baseShiftRight[0] * kNoRiskDis;
        ptBias[1] = baseShiftRight[1] * kNoRiskDis;
        apaThirdBoundaryLineTranslate(rightLineOut, ptBias, rightLineOut);
      } else {
        if (apaMathLess(seed.RightLateralBias, kNoRiskDis)) {
          ptBias[0] = baseShiftRight[0] * kNoRiskDis;
          ptBias[1] = baseShiftRight[1] * kNoRiskDis;
          apaThirdBoundaryLineTranslate(rightLineOut, ptBias, rightLineOut);
        } else {
          apaThirdBoundaryCopy4(seed.RightVehClear, rightLineOut);
        }
      }
    }
  } else {
    if (apaMathGreater(yawErr, yawThresh)) {
      if (seed.LeftVehCollision) {
        ptBias[0] = baseShiftLeft[0] * kExistRiskDis;
        ptBias[1] = baseShiftLeft[1] * kExistRiskDis;
        apaThirdBoundaryLineTranslate(leftLineOut, ptBias, leftLineOut);
      } else {
        if (apaMathLess(seed.LeftLateralBias, kExistRiskDis)) {
          ptBias[0] = baseShiftLeft[0] * kExistRiskDis;
          ptBias[1] = baseShiftLeft[1] * kExistRiskDis;
          apaThirdBoundaryLineTranslate(leftLineOut, ptBias, leftLineOut);
        } else {
          apaThirdBoundaryCopy4(seed.LeftVehClear, leftLineOut);
        }
      }

      if (seed.RightVehCollision) {
        ptBias[0] = baseShiftRight[0] * kNoRiskDis;
        ptBias[1] = baseShiftRight[1] * kNoRiskDis;
        apaThirdBoundaryLineTranslate(rightLineOut, ptBias, rightLineOut);
      } else {
        if (apaMathLess(seed.RightLateralBias, kNoRiskDis)) {
          ptBias[0] = baseShiftRight[0] * kNoRiskDis;
          ptBias[1] = baseShiftRight[1] * kNoRiskDis;
          apaThirdBoundaryLineTranslate(rightLineOut, ptBias, rightLineOut);
        } else {
          apaThirdBoundaryCopy4(seed.RightVehClear, rightLineOut);
        }
      }
    } else if (apaMathLess(yawErr, -yawThresh)) {
      if (seed.LeftVehCollision) {
        ptBias[0] = baseShiftLeft[0] * kNoRiskDis;
        ptBias[1] = baseShiftLeft[1] * kNoRiskDis;
        apaThirdBoundaryLineTranslate(leftLineOut, ptBias, leftLineOut);
      } else {
        if (apaMathLess(seed.LeftLateralBias, kNoRiskDis)) {
          ptBias[0] = baseShiftLeft[0] * kNoRiskDis;
          ptBias[1] = baseShiftLeft[1] * kNoRiskDis;
          apaThirdBoundaryLineTranslate(leftLineOut, ptBias, leftLineOut);
        } else {
          apaThirdBoundaryCopy4(seed.LeftVehClear, leftLineOut);
        }
      }

      if (seed.RightVehCollision) {
        ptBias[0] = baseShiftRight[0] * kExistRiskDis;
        ptBias[1] = baseShiftRight[1] * kExistRiskDis;
        apaThirdBoundaryLineTranslate(rightLineOut, ptBias, rightLineOut);
      } else {
        if (apaMathLess(seed.RightLateralBias, kExistRiskDis)) {
          ptBias[0] = baseShiftRight[0] * kExistRiskDis;
          ptBias[1] = baseShiftRight[1] * kExistRiskDis;
          apaThirdBoundaryLineTranslate(rightLineOut, ptBias, rightLineOut);
        } else {
          apaThirdBoundaryCopy4(seed.RightVehClear, rightLineOut);
        }
      }
    } else {
      apaThirdBoundaryCopy4(seed.LeftVehClear, leftLineOut);
      apaThirdBoundaryCopy4(seed.RightVehClear, rightLineOut);
    }
  }

  apaThirdBoundaryCopy4(leftLineOut, seed.LeftVehClear);
  apaThirdBoundaryCopy4(rightLineOut, seed.RightVehClear);

  apaThirdBoundaryAppendInSlotLinesByScene(&seed, surroundingType, isNormalScene, lineSet);
}

static inline void
apaThirdBoundaryParallelSlotSideEnterDeviation(const Real ptFInside[2],
                                               const Real ptFOutside[2],
                                               const Real ptRInside[2],
                                               const Real ptROutside[2],
                                               const VehicleCollisionCheckInput_t *obsInput,
                                               Real pBiasOut[2]) {
  Real lStep = 0.1f;
  Real lBias = 0.0f;
  Real ptRInsideCollision[2];
  Real ptROutsideCollision[2];
  Real rectPts[8];
  Real pLonBias[2];
  Real pLatBias[2];
  Real pLatBiasLen;
  Real pLatBiasUnit[2];
  Boolean isCollision;
  Boolean loopActive;

  ptRInsideCollision[0] = 0.5f * (ptFInside[0] + ptRInside[0]);
  ptRInsideCollision[1] = 0.5f * (ptFInside[1] + ptRInside[1]);
  ptROutsideCollision[0] = 0.5f * (ptFOutside[0] + ptROutside[0]);
  ptROutsideCollision[1] = 0.5f * (ptFOutside[1] + ptROutside[1]);

  rectPts[0] = ptFInside[0];
  rectPts[1] = ptFInside[1];
  rectPts[2] = ptFOutside[0];
  rectPts[3] = ptFOutside[1];
  rectPts[4] = ptROutsideCollision[0];
  rectPts[5] = ptROutsideCollision[1];
  rectPts[6] = ptRInsideCollision[0];
  rectPts[7] = ptRInsideCollision[1];

  pLonBias[0] = ptFInside[0] - ptRInsideCollision[0];
  pLonBias[1] = ptFInside[1] - ptRInsideCollision[1];
  pLatBias[0] = ptFInside[0] - ptFOutside[0];
  pLatBias[1] = ptFInside[1] - ptFOutside[1];

  pLatBiasLen = apaMathNorm2D(pLatBias[0], pLatBias[1]);
  pLatBiasUnit[0] = pLatBias[0] / apaMathDivideZeroProcess(pLatBiasLen);
  pLatBiasUnit[1] = pLatBias[1] / apaMathDivideZeroProcess(pLatBiasLen);

  loopActive = apaMathLess(lBias, 1.2f);
  while (loopActive) {
    Real pBias[2];
    Real rectPtsTemp[8];

    pBias[0] = pLonBias[0] + pLatBias[0] - pLatBiasUnit[0] * lBias;
    pBias[1] = pLonBias[1] + pLatBias[1] - pLatBiasUnit[1] * lBias;

    apaThirdBoundaryRectTranslate(rectPts, pBias, rectPtsTemp);
    isCollision = apaThirdBoundaryRectCollisionWithObs(rectPtsTemp, obsInput);
    if (!isCollision) {
      lBias = lBias + lStep;
    }
    loopActive = apaMathLess(lBias, 1.2f);
    if (loopActive) {
      loopActive = !isCollision;
    }
  }

  pBiasOut[0] = -pLatBiasUnit[0] * lBias;
  pBiasOut[1] = -pLatBiasUnit[1] * lBias;
}

#endif
