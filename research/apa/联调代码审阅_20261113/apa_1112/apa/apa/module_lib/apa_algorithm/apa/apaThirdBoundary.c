#include "apaThirdBoundaryTypes.h"

static void
apaThirdBoundarySelectSlotClass(const ApaSlot_t *slot, Integer inputSlotClass, Integer *slotClass) {
  Integer slotClass0;

  slotClass0 = inputSlotClass;
  if (slotClass0 == APA_SLOT_CLASS_UNKNOWN) {
    slotClass0 = slot->SlotClass;
  }
  if (slotClass0 == APA_SLOT_CLASS_UNKNOWN) {
    slotClass0 = apaSlotCalcSlotClass(slot->SlotPts, slot->EnterEdge);
  }
  *slotClass = slotClass0;
}

static void apaThirdBoundarySelectSlotSubStatus(const ApaSlot_t *slot,
                                                Integer inputSlotSubStatus,
                                                Integer *slotSubStatus) {
  (void)slot;
  *slotSubStatus = inputSlotSubStatus;
}

static void apaThirdBoundaryBuildSideBias(const Real lnLeft[4],
                                          const Real lnRight[4],
                                          Real biasLen,
                                          Real sideBias[2]) {
  Real rawBiasX;
  Real rawBiasY;
  Real rawBiasLen;
  Real divValue;

  rawBiasX = lnLeft[0] - lnRight[0];
  rawBiasY = lnLeft[1] - lnRight[1];
  rawBiasLen = apaMathNorm2D(rawBiasX, rawBiasY);
  divValue = apaMathDivideZeroProcess(rawBiasLen);
  sideBias[0] = rawBiasX / divValue * biasLen;
  sideBias[1] = rawBiasY / divValue * biasLen;
}

static void apaThirdBoundaryScaleVec(const Real vecIn[2], Real scale, Real vecOut[2]) {
  vecOut[0] = vecIn[0] * scale;
  vecOut[1] = vecIn[1] * scale;
}

static void apaThirdBoundaryAppendVerticalSearchLines(const Real lnLeft[4],
                                                      const Real lnRight[4],
                                                      Integer surroundingType,
                                                      Integer parkingType,
                                                      Integer slotSubStatus,
                                                      Boolean useSideExtendFlag,
                                                      const VehicleCollisionCheckInput_t *obsInput,
                                                      ApaThirdBoundaryLineSet_t *lineSet) {
  ApaPlannerConfig_t defaultConfig;
  Real leftCollision[4];
  Real rightCollision[4];
  Real rawSideBias[2];
  Real sideBias[2];
  Real tempBias[2];

  apaPlannerConfigInitDefault(&defaultConfig);
  apaThirdBoundarySlotEnterCollisionCheck(lnLeft,
                                          lnRight,
                                          surroundingType,
                                          parkingType,
                                          useSideExtendFlag,
                                          &defaultConfig.Conf,
                                          obsInput,
                                          leftCollision,
                                          rightCollision);
  apaThirdBoundaryBuildSideBias(lnLeft, lnRight, 1.5f, sideBias);
  rawSideBias[0] = lnLeft[0] - lnRight[0];
  rawSideBias[1] = lnLeft[1] - lnRight[1];

  if (surroundingType == APA_SURROUND_OBS_EXIST_BOTH) {
    apaThirdBoundaryAppendLine(lineSet, leftCollision);
    apaThirdBoundaryAppendLine(lineSet, rightCollision);
  } else if (surroundingType == APA_SURROUND_OBS_EXIST_RIGHT_REAR_ONLY) {
    if (slotSubStatus == APA_SLOT_SUB_END_NONE) {
      apaThirdBoundaryLineTranslate(leftCollision, sideBias, leftCollision);
    } else {
      tempBias[0] = rawSideBias[0] * 2.0f;
      tempBias[1] = rawSideBias[1] * 2.0f;
      apaThirdBoundaryLineTranslate(lnLeft, tempBias, leftCollision);
    }
    apaThirdBoundaryAppendLine(lineSet, leftCollision);
    apaThirdBoundaryAppendLine(lineSet, rightCollision);
  } else if (surroundingType == APA_SURROUND_OBS_EXIST_LEFT_FRONT_ONLY) {
    if (slotSubStatus == APA_SLOT_SUB_END_NONE) {
      tempBias[0] = -sideBias[0];
      tempBias[1] = -sideBias[1];
      apaThirdBoundaryLineTranslate(rightCollision, tempBias, rightCollision);
    } else if (slotSubStatus == APA_SLOT_SUB_END_FIRST) {
      tempBias[0] = -rawSideBias[0] * 2.0f;
      tempBias[1] = -rawSideBias[1] * 2.0f;
      apaThirdBoundaryLineTranslate(lnRight, tempBias, rightCollision);
    } else {
      tempBias[0] = -rawSideBias[0];
      tempBias[1] = -rawSideBias[1];
      apaThirdBoundaryLineTranslate(lnRight, tempBias, rightCollision);
    }
    apaThirdBoundaryAppendLine(lineSet, leftCollision);
    apaThirdBoundaryAppendLine(lineSet, rightCollision);
  } else if (surroundingType == APA_SURROUND_OBS_EXIST_NONE) {
    if (parkingType == APA_PARKING_TYPE_MEANINGLESS) {
      apaThirdBoundaryAppendLine(lineSet, rightCollision);
      apaThirdBoundaryAppendLine(lineSet, leftCollision);
    } else if (slotSubStatus != APA_SLOT_SUB_END_NONE) {
      Real endBias[2];
      Real endBiasScale;

      endBiasScale = 1.0f;
      if (slotSubStatus == APA_SLOT_SUB_END_FIRST) {
        endBiasScale = 2.0f;
      }
      apaThirdBoundaryScaleVec(rawSideBias, endBiasScale, endBias);
      tempBias[0] = endBias[0];
      tempBias[1] = endBias[1];
      apaThirdBoundaryLineTranslate(lnLeft, tempBias, leftCollision);
      tempBias[0] = -endBias[0];
      tempBias[1] = -endBias[1];
      apaThirdBoundaryLineTranslate(lnRight, tempBias, rightCollision);
      apaThirdBoundaryAppendLine(lineSet, leftCollision);
      apaThirdBoundaryAppendLine(lineSet, rightCollision);
    } else {
      apaThirdBoundaryLineTranslate(leftCollision, sideBias, leftCollision);
      tempBias[0] = -sideBias[0];
      tempBias[1] = -sideBias[1];
      apaThirdBoundaryLineTranslate(rightCollision, tempBias, rightCollision);
      apaThirdBoundaryAppendLine(lineSet, leftCollision);
      apaThirdBoundaryAppendLine(lineSet, rightCollision);
    }
  }
}

static void apaThirdBoundaryExtendVerticalDownLineLeft(const ApaThirdBoundaryVerticalSeed_t *seed,
                                                       Real downLine[4]) {
  Real sideUnit[2];
  Real sideLenDiv;

  sideLenDiv = apaMathDivideZeroProcess(seed->EnterEdgeLen);
  sideUnit[0] = (seed->LnLeft[0] - seed->LnRight[0]) / sideLenDiv;
  sideUnit[1] = (seed->LnLeft[1] - seed->LnRight[1]) / sideLenDiv;
  downLine[0] = downLine[0] + sideUnit[0] * 10.0f;
  downLine[1] = downLine[1] + sideUnit[1] * 10.0f;
  downLine[2] = downLine[2] + sideUnit[0] * (seed->EnterEdgeLen + 2.5f);
  downLine[3] = downLine[3] + sideUnit[1] * (seed->EnterEdgeLen + 2.5f);
}

static void apaThirdBoundaryExtendVerticalDownLineRight(const ApaThirdBoundaryVerticalSeed_t *seed,
                                                        Real downLine[4]) {
  Real sideUnit[2];
  Real sideLenDiv;

  sideLenDiv = apaMathDivideZeroProcess(seed->EnterEdgeLen);
  sideUnit[0] = (seed->LnLeft[0] - seed->LnRight[0]) / sideLenDiv;
  sideUnit[1] = (seed->LnLeft[1] - seed->LnRight[1]) / sideLenDiv;
  downLine[0] = downLine[0] - sideUnit[0] * (seed->EnterEdgeLen + 2.5f);
  downLine[1] = downLine[1] - sideUnit[1] * (seed->EnterEdgeLen + 2.5f);
  downLine[2] = downLine[2] - sideUnit[0] * 10.0f;
  downLine[3] = downLine[3] - sideUnit[1] * 10.0f;
}

static void apaThirdBoundaryAppendVerticalDownLines(const ApaThirdBoundaryVerticalSeed_t *seed,
                                                    const Real vehPos[3],
                                                    const ApaPlannerVehConfig_t *vehConfig,
                                                    ApaThirdBoundaryLineSet_t *lineSet) {
  Real downLt[4];
  Real downRt[4];
  Boolean downLtHit;
  Boolean downRtHit;

  apaThirdBoundaryCopy4(seed->BoundaryDownLt, downLt);
  apaThirdBoundaryCopy4(seed->BoundaryDownRt, downRt);
  apaThirdBoundaryExtendVerticalDownLineLeft(seed, downLt);
  apaThirdBoundaryExtendVerticalDownLineRight(seed, downRt);

  downLtHit = apaThirdBoundaryLineIntersectsVehicle(downLt, vehPos, vehConfig, 0.0f, 0.0f, 0.0f);
  downRtHit = apaThirdBoundaryLineIntersectsVehicle(downRt, vehPos, vehConfig, 0.0f, 0.0f, 0.0f);
  if (!downLtHit) {
    apaThirdBoundaryAppendLine(lineSet, downLt);
  }
  if (!downRtHit) {
    apaThirdBoundaryAppendLine(lineSet, downRt);
  }
}

static Real
apaThirdBoundaryPointLineDistance(Real x1, Real y1, Real x2, Real y2, Real x3, Real y3) {
  Real numerator;
  Real denominator;
  Real distance;

  numerator = (y3 - y1) * (x2 - x1) - (x3 - x1) * (y2 - y1);
  if (apaMathLess(numerator, 0.0f)) {
    numerator = -numerator;
  }
  denominator = apaMathNorm2D(x2 - x1, y2 - y1);
  distance = numerator / apaMathDivideZeroProcess(denominator);
  return distance;
}

static void apaThirdBoundaryCheckDiagonalPoints(const Real lineA[4],
                                                const Real lineB[4],
                                                Boolean *isCollinear,
                                                Boolean *isOnePointOnSegment,
                                                Boolean *a2InA1B1,
                                                Boolean *b2InA1B1) {
  Real vecA1B1[2];
  Real vecA1A2[2];
  Real vecA1B2[2];
  Real vecA2B2[2];
  Real cross12;
  Real dotA2;
  Real dotB2;
  Real modSquareA1B1;
  Real distA2Line;
  Boolean collinear0;
  Boolean a2In;
  Boolean b2In;

  *isCollinear = false;
  *isOnePointOnSegment = false;
  *a2InA1B1 = false;
  *b2InA1B1 = false;

  vecA1B1[0] = lineA[2] - lineA[0];
  vecA1B1[1] = lineA[3] - lineA[1];
  vecA1A2[0] = lineB[0] - lineA[0];
  vecA1A2[1] = lineB[1] - lineA[1];
  vecA1B2[0] = lineB[2] - lineA[0];
  vecA1B2[1] = lineB[3] - lineA[1];
  vecA2B2[0] = lineB[2] - lineB[0];
  vecA2B2[1] = lineB[3] - lineB[1];
  cross12 = apaMathCross2D(vecA1B1[0], vecA1B1[1], vecA2B2[0], vecA2B2[1]);
  dotA2 = apaMathDot2D(vecA1A2[0], vecA1A2[1], vecA1B1[0], vecA1B1[1]);
  dotB2 = apaMathDot2D(vecA1B2[0], vecA1B2[1], vecA1B1[0], vecA1B1[1]);
  modSquareA1B1 = apaMathDot2D(vecA1B1[0], vecA1B1[1], vecA1B1[0], vecA1B1[1]) + 0.5f;
  distA2Line =
      apaThirdBoundaryPointLineDistance(lineA[0], lineA[1], lineA[2], lineA[3], lineB[0], lineB[1]);
  collinear0 = false;
  if (apaMathLess(apaMathAbs(cross12), 1.2f)) {
    if (apaMathLess(distA2Line, 0.8f)) {
      collinear0 = true;
    }
  }

  if (collinear0) {
    *isCollinear = true;
    a2In = false;
    if (apaMathLess(0.0f, dotA2)) {
      if (apaMathLess(dotA2, modSquareA1B1)) {
        a2In = true;
      }
    }
    b2In = false;
    if (apaMathLess(0.0f, dotB2)) {
      if (apaMathLess(dotB2, modSquareA1B1)) {
        b2In = true;
      }
    }
    if (a2In) {
      *a2InA1B1 = true;
      if (!b2In) {
        *isOnePointOnSegment = true;
      }
    } else if (b2In) {
      *b2InA1B1 = true;
      *isOnePointOnSegment = true;
    }
  }
}

static void apaThirdBoundaryBuildVerticalUpperForPair(const Real pairLeft[4],
                                                      const Real pairRight[4],
                                                      Integer parkingDirec,
                                                      Integer slotSubStatus,
                                                      Real boundaryUp[4]) {
  Real frontDir[2];
  Real frontNormal[2];
  Real leftDir[2];
  Real rightDir[2];
  Real enterEdgeLen;
  Real bottomEdgeLen;

  apaThirdBoundaryCalcVerticalBasis(
      pairLeft, pairRight, frontDir, frontNormal, leftDir, rightDir, &enterEdgeLen, &bottomEdgeLen);
  apaThirdBoundaryBuildVerticalUpperSeed(
      pairLeft, pairRight, frontNormal, leftDir, rightDir, parkingDirec, slotSubStatus, boundaryUp);
}

static void apaThirdBoundarySetDiagonalDownLines(const Real downBase[4],
                                                 const Real frontNormal[2],
                                                 ApaThirdBoundaryVerticalSeed_t *seed) {
  Real shiftVec[2];

  shiftVec[0] = frontNormal[0] * 0.3f;
  shiftVec[1] = frontNormal[1] * 0.3f;
  apaThirdBoundaryLineTranslate(downBase, shiftVec, seed->BoundaryDownLt);
  apaThirdBoundaryLineTranslate(downBase, shiftVec, seed->BoundaryDownRt);
}

static void apaThirdBoundaryApplyVerticalDiagonalCandidate(const Real baseLeft[4],
                                                           const Real baseRight[4],
                                                           const Real candidateLeft[4],
                                                           const Real candidateRight[4],
                                                           Integer slotSubStatus,
                                                           ApaThirdBoundaryVerticalSeed_t *seed,
                                                           Boolean *foundDiagonal) {
  Boolean isCollinearLt;
  Boolean isOneLt;
  Boolean a2InLt;
  Boolean b2InLt;
  Boolean isCollinearRt;
  Boolean isOneRt;
  Boolean a2InRt;
  Boolean b2InRt;
  Real disA2Front;
  Real disB2Rear;
  Real downBase[4];
  Real pLtBias[2];
  Boolean distanceOk;

  apaThirdBoundaryCheckDiagonalPoints(
      baseLeft, candidateRight, &isCollinearLt, &isOneLt, &a2InLt, &b2InLt);
  apaThirdBoundaryCheckDiagonalPoints(
      baseRight, candidateLeft, &isCollinearRt, &isOneRt, &a2InRt, &b2InRt);

  if (!(*foundDiagonal)) {
    if (isCollinearLt) {
      if (isOneLt) {
        disA2Front = apaThirdBoundaryPointLineDistance(baseLeft[0],
                                                       baseLeft[1],
                                                       baseRight[0],
                                                       baseRight[1],
                                                       candidateLeft[0],
                                                       candidateLeft[1]);
        disB2Rear = apaThirdBoundaryPointLineDistance(baseLeft[2],
                                                      baseLeft[3],
                                                      baseRight[2],
                                                      baseRight[3],
                                                      candidateLeft[2],
                                                      candidateLeft[3]);
        distanceOk = apaMathGreater(disA2Front + disB2Rear, 1.2f);
        if (distanceOk) {
          if (a2InLt) {
            apaThirdBoundaryBuildVerticalUpperForPair(
                baseLeft, candidateLeft, seed->ParkingDirec, slotSubStatus, seed->BoundaryUp);
            pLtBias[0] = candidateLeft[0] - baseLeft[0];
            pLtBias[1] = candidateLeft[1] - baseLeft[1];
            downBase[0] = candidateLeft[0] - pLtBias[0];
            downBase[1] = candidateLeft[1] - pLtBias[1];
            downBase[2] = baseLeft[0] - pLtBias[0];
            downBase[3] = baseLeft[1] - pLtBias[1];
            apaThirdBoundarySetDiagonalDownLines(downBase, seed->FrontNormal, seed);
            *foundDiagonal = true;
          } else if (b2InLt) {
            apaThirdBoundaryBuildVerticalUpperForPair(
                baseRight, candidateRight, seed->ParkingDirec, slotSubStatus, seed->BoundaryUp);
            downBase[0] = candidateRight[0];
            downBase[1] = candidateRight[1];
            downBase[2] = baseRight[0];
            downBase[3] = baseRight[1];
            apaThirdBoundarySetDiagonalDownLines(downBase, seed->FrontNormal, seed);
            *foundDiagonal = true;
          }
        }
      }
    }
  }

  if (!(*foundDiagonal)) {
    if (isCollinearRt) {
      if (isOneRt) {
        disA2Front = apaThirdBoundaryPointLineDistance(baseLeft[0],
                                                       baseLeft[1],
                                                       baseRight[0],
                                                       baseRight[1],
                                                       candidateLeft[0],
                                                       candidateLeft[1]);
        disB2Rear = apaThirdBoundaryPointLineDistance(baseLeft[2],
                                                      baseLeft[3],
                                                      baseRight[2],
                                                      baseRight[3],
                                                      candidateLeft[2],
                                                      candidateLeft[3]);
        distanceOk = apaMathGreater(disA2Front + disB2Rear, 1.2f);
        if (distanceOk) {
          if (a2InRt) {
            apaThirdBoundaryBuildVerticalUpperForPair(
                baseRight, candidateRight, seed->ParkingDirec, slotSubStatus, seed->BoundaryUp);
            pLtBias[0] = baseRight[0] - candidateRight[0];
            pLtBias[1] = baseRight[1] - candidateRight[1];
            downBase[0] = baseRight[0] + pLtBias[0];
            downBase[1] = baseRight[1] + pLtBias[1];
            downBase[2] = candidateRight[0] + pLtBias[0];
            downBase[3] = candidateRight[1] + pLtBias[1];
            apaThirdBoundarySetDiagonalDownLines(downBase, seed->FrontNormal, seed);
            *foundDiagonal = true;
          } else if (b2InRt) {
            apaThirdBoundaryBuildVerticalUpperForPair(
                baseLeft, candidateLeft, seed->ParkingDirec, slotSubStatus, seed->BoundaryUp);
            downBase[0] = baseLeft[0];
            downBase[1] = baseLeft[1];
            downBase[2] = candidateLeft[0];
            downBase[3] = candidateLeft[1];
            apaThirdBoundarySetDiagonalDownLines(downBase, seed->FrontNormal, seed);
            *foundDiagonal = true;
          }
        }
      }
    }
  }
}

static void apaThirdBoundaryApplyVerticalDiagonalSlots(const ApaSlot_t *slotList,
                                                       Integer slotCount,
                                                       Integer targetSlotPtr,
                                                       Integer slotSubStatus,
                                                       ApaThirdBoundaryVerticalSeed_t *seed) {
  Integer idx;
  Integer candidateClass;
  Real candidateLeft[4];
  Real candidateRight[4];
  Boolean foundDiagonal;
  Boolean loopActive;
  Boolean checkCandidate;

  idx = 0;
  foundDiagonal = false;
  loopActive = (idx < slotCount);
  if (loopActive) {
    loopActive = !foundDiagonal;
  }
  while (loopActive) {
    checkCandidate = (idx != targetSlotPtr);
    if (checkCandidate) {
      checkCandidate = (slotList[idx].EnterEdge > 0);
    }
    if (checkCandidate) {
      candidateClass = slotList[idx].SlotClass;
      if (candidateClass == APA_SLOT_CLASS_UNKNOWN) {
        candidateClass = apaSlotCalcSlotClass(slotList[idx].SlotPts, slotList[idx].EnterEdge);
      }
      apaSlotCalcSideLine(slotList[idx].SlotPts,
                          slotList[idx].EnterEdge,
                          slotList[idx].TarPos,
                          candidateClass,
                          candidateLeft,
                          candidateRight);
      apaThirdBoundaryApplyVerticalDiagonalCandidate(seed->LnLeft,
                                                     seed->LnRight,
                                                     candidateLeft,
                                                     candidateRight,
                                                     slotSubStatus,
                                                     seed,
                                                     &foundDiagonal);
    }
    idx = idx + 1;
    loopActive = (idx < slotCount);
    if (loopActive) {
      loopActive = !foundDiagonal;
    }
  }
}

static void apaThirdBoundaryBuildVerticalBottomWithPose(const Real lnLeft[4],
                                                        const Real lnRight[4],
                                                        const Real vehPos[3],
                                                        const Real tarPos[3],
                                                        const ApaPlannerVehConfig_t *vehConfig,
                                                        Real boundaryBottom[4]);

static void apaThirdBoundaryAppendVerticalLines(const ApaSlot_t *slot,
                                                const ApaSlot_t *slotList,
                                                Integer slotCount,
                                                Integer targetSlotPtr,
                                                const Real vehPos[3],
                                                Integer surroundingType,
                                                Integer parkingType,
                                                Integer slotSubStatus,
                                                Boolean isNormalScene,
                                                Boolean useSideExtendFlag,
                                                const VehicleCollisionCheckInput_t *obsInput,
                                                const ApaPlannerVehConfig_t *vehConfig,
                                                const Real lnLeft[4],
                                                const Real lnRight[4],
                                                ApaThirdBoundaryLineSet_t *lineSet) {
  ApaThirdBoundaryVerticalSeed_t seed;
  Real yawErr;
  Real distErr;
  Real crossLeft;
  Real crossRight;
  Boolean isFrontOfSlot;
  Boolean yawSmall10;
  Boolean distSmall05;
  Boolean yawSmall5;
  Boolean distSmall1;
  Boolean yawPositiveSmall;
  Boolean distPositiveSmall;
  Boolean yawNegativeSmall;
  Boolean distNegativeSmall;
  Boolean errAllowable0;
  Boolean errAllowable1;
  Boolean errAllowable2;
  Boolean errAllowable3;
  Boolean errAllowable;
  Boolean isFrontAndAllowable;

  apaThirdBoundaryInitVerticalSeed(&seed);
  apaThirdBoundaryCopy4(lnLeft, seed.LnLeft);
  apaThirdBoundaryCopy4(lnRight, seed.LnRight);
  seed.ParkingDirec = apaSlotCalcParkingDirecFromSideLines(lnLeft, lnRight, slot->TarPos);
  apaThirdBoundaryCalcVerticalBasis(lnLeft,
                                    lnRight,
                                    seed.FrontDir,
                                    seed.FrontNormal,
                                    seed.LeftDir,
                                    seed.RightDir,
                                    &seed.EnterEdgeLen,
                                    &seed.BottomEdgeLen);
  apaThirdBoundaryBuildVerticalUpperSeed(lnLeft,
                                         lnRight,
                                         seed.FrontNormal,
                                         seed.LeftDir,
                                         seed.RightDir,
                                         seed.ParkingDirec,
                                         slotSubStatus,
                                         seed.BoundaryUp);
  apaThirdBoundaryBuildVerticalBottomWithPose(
      lnLeft, lnRight, vehPos, slot->TarPos, vehConfig, seed.BoundaryBottom);
  apaThirdBoundaryBuildVerticalDownSeeds(
      lnLeft, lnRight, seed.FrontNormal, seed.BoundaryDownLt, seed.BoundaryDownRt);
  apaThirdBoundaryApplyVerticalDiagonalSlots(
      slotList, slotCount, targetSlotPtr, slotSubStatus, &seed);
  apaThirdBoundaryAdjustVerticalUpperForVehicle(
      vehPos, slot->SlotPts, slot->TarPos, vehConfig, seed.FrontNormal, seed.BoundaryUp);

  if (slotSubStatus != APA_SLOT_SUB_END_NONE) {
    apaThirdBoundaryAppendEnvironmentBoundary(lnLeft, lnRight, obsInput, lineSet);
  }

  apaThirdBoundaryAppendLine(lineSet, seed.BoundaryBottom);

  apaThirdBoundaryCalcSlotPoseError(vehPos, slot->TarPos, lnLeft, lnRight, &yawErr, &distErr);
  crossLeft = apaMathCross2D(
      vehPos[0] - lnLeft[0], vehPos[1] - lnLeft[1], lnLeft[2] - lnLeft[0], lnLeft[3] - lnLeft[1]);
  crossRight = apaMathCross2D(vehPos[0] - lnRight[0],
                              vehPos[1] - lnRight[1],
                              lnRight[2] - lnRight[0],
                              lnRight[3] - lnRight[1]);
  isFrontOfSlot = apaMathLess(crossLeft * crossRight, 0.0f);

  yawSmall10 = apaMathLess(apaMathAbs(yawErr), 10.0f / 180.0f * APA_PI);
  distSmall05 = apaMathLess(apaMathAbs(distErr), 0.5f);
  yawSmall5 = apaMathLess(apaMathAbs(yawErr), 5.0f / 180.0f * APA_PI);
  distSmall1 = apaMathLess(apaMathAbs(distErr), 1.0f);
  yawPositiveSmall = false;
  distPositiveSmall = false;
  yawNegativeSmall = false;
  distNegativeSmall = false;
  if (apaMathGreater(yawErr, 0.0f)) {
    yawPositiveSmall = apaMathLess(yawErr, 10.0f / 180.0f * APA_PI);
  }
  if (apaMathGreater(distErr, 0.0f)) {
    distPositiveSmall = apaMathLess(distErr, 1.0f);
  }
  if (apaMathLess(yawErr, 0.0f)) {
    yawNegativeSmall = apaMathGreater(yawErr, -10.0f / 180.0f * APA_PI);
  }
  if (apaMathLess(distErr, 0.0f)) {
    distNegativeSmall = apaMathGreater(distErr, -1.0f);
  }

  errAllowable0 = false;
  if (yawSmall10) {
    if (distSmall05) {
      errAllowable0 = true;
    }
  }
  errAllowable1 = false;
  if (yawSmall5) {
    if (distSmall1) {
      errAllowable1 = true;
    }
  }
  errAllowable2 = false;
  if (yawPositiveSmall) {
    if (distPositiveSmall) {
      errAllowable2 = true;
    }
  }
  errAllowable3 = false;
  if (yawNegativeSmall) {
    if (distNegativeSmall) {
      errAllowable3 = true;
    }
  }
  errAllowable = false;
  if (errAllowable0) {
    errAllowable = true;
  }
  if (errAllowable1) {
    errAllowable = true;
  }
  if (errAllowable2) {
    errAllowable = true;
  }
  if (errAllowable3) {
    errAllowable = true;
  }
  isFrontAndAllowable = false;
  if (isFrontOfSlot) {
    if (errAllowable) {
      isFrontAndAllowable = true;
    }
  }

  if (isFrontAndAllowable) {
    apaThirdBoundaryGetVirtualBoundaryInSlot(
        vehPos, slot->TarPos, surroundingType, lnLeft, lnRight, isNormalScene, vehConfig, lineSet);
  } else {
    apaThirdBoundaryAppendVerticalSearchLines(lnLeft,
                                              lnRight,
                                              surroundingType,
                                              parkingType,
                                              slotSubStatus,
                                              useSideExtendFlag,
                                              obsInput,
                                              lineSet);
  }

  apaThirdBoundaryAppendLine(lineSet, seed.BoundaryUp);
  if (slotSubStatus == APA_SLOT_SUB_END_NONE) {
    apaThirdBoundaryAppendVerticalDownLines(&seed, vehPos, vehConfig, lineSet);
  }
}

static Real apaThirdBoundaryMaxReal(Real valueA, Real valueB) {
  Real valueOut;

  valueOut = valueA;
  if (apaMathGreater(valueB, valueOut)) {
    valueOut = valueB;
  }
  return valueOut;
}

static Real apaThirdBoundaryMinReal(Real valueA, Real valueB) {
  Real valueOut;

  valueOut = valueA;
  if (apaMathLess(valueB, valueOut)) {
    valueOut = valueB;
  }
  return valueOut;
}

static void
apaThirdBoundaryTranslatePoint(const Real pointIn[2], const Real shiftVec[2], Real pointOut[2]) {
  pointOut[0] = pointIn[0] + shiftVec[0];
  pointOut[1] = pointIn[1] + shiftVec[1];
}

static void
apaThirdBoundaryLineFromPoints(const Real pointA[2], const Real pointB[2], Real lineOut[4]) {
  lineOut[0] = pointA[0];
  lineOut[1] = pointA[1];
  lineOut[2] = pointB[0];
  lineOut[3] = pointB[1];
}

static void apaThirdBoundaryTranslateLineByLen(const Real lineIn[4],
                                               const Real unitVec[2],
                                               Real biasLen,
                                               Real lineOut[4]) {
  Real shiftVec[2];

  shiftVec[0] = unitVec[0] * biasLen;
  shiftVec[1] = unitVec[1] * biasLen;
  apaThirdBoundaryLineTranslate(lineIn, shiftVec, lineOut);
}

static void apaThirdBoundarySearchBottomClearWithPose(const Real baseLine[4],
                                                      const Real unitVec[2],
                                                      Real maxLen,
                                                      Real stepLen,
                                                      const Real pose[3],
                                                      const ApaPlannerVehConfig_t *vehConfig,
                                                      Real lineOut[4],
                                                      Boolean *foundClear) {
  Real biasLen;
  Real candidateLine[4];
  Boolean hitFlag;
  Boolean loopActive;

  biasLen = 0.0f;
  *foundClear = false;
  apaThirdBoundaryCopy4(baseLine, lineOut);
  loopActive = apaMathLess(biasLen, maxLen);
  while (loopActive) {
    apaThirdBoundaryTranslateLineByLen(baseLine, unitVec, biasLen, candidateLine);
    hitFlag =
        apaThirdBoundaryLineIntersectsVehicle(candidateLine, pose, vehConfig, 0.0f, 0.0f, 0.0f);
    if (!hitFlag) {
      apaThirdBoundaryCopy4(candidateLine, lineOut);
      *foundClear = true;
    } else {
      biasLen = biasLen + stepLen;
    }
    loopActive = apaMathLess(biasLen, maxLen);
    if (loopActive) {
      loopActive = !(*foundClear);
    }
  }
}

static void apaThirdBoundaryBuildVerticalBottomWithPose(const Real lnLeft[4],
                                                        const Real lnRight[4],
                                                        const Real vehPos[3],
                                                        const Real tarPos[3],
                                                        const ApaPlannerVehConfig_t *vehConfig,
                                                        Real boundaryBottom[4]) {
  Real frontVec[2];
  Real frontLen;
  Real searchUnit[2];
  Real candidateLine[4];
  Boolean foundVehClear;
  Boolean foundTarClear;

  apaThirdBoundaryBuildVerticalBottomSeed(lnLeft, lnRight, boundaryBottom);
  frontVec[0] = lnLeft[0] - lnLeft[2];
  frontVec[1] = lnLeft[1] - lnLeft[3];
  frontLen = apaMathNorm2D(frontVec[0], frontVec[1]);
  searchUnit[0] = -frontVec[0] / apaMathDivideZeroProcess(frontLen);
  searchUnit[1] = -frontVec[1] / apaMathDivideZeroProcess(frontLen);

  apaThirdBoundarySearchBottomClearWithPose(
      boundaryBottom, searchUnit, 5.0f, 0.1f, vehPos, vehConfig, candidateLine, &foundVehClear);
  if (foundVehClear) {
    apaThirdBoundaryCopy4(candidateLine, boundaryBottom);
    apaThirdBoundarySearchBottomClearWithPose(
        boundaryBottom, searchUnit, 5.0f, 0.1f, tarPos, vehConfig, candidateLine, &foundTarClear);
    if (foundTarClear) {
      apaThirdBoundaryTranslateLineByLen(candidateLine, searchUnit, 0.1f, boundaryBottom);
    }
  }
}

static void apaThirdBoundarySearchLineClearWithPose(const Real baseLine[4],
                                                    const Real unitVec[2],
                                                    Real maxLen,
                                                    Real stepLen,
                                                    Real extraClearLen,
                                                    const Real pose[3],
                                                    const ApaPlannerVehConfig_t *vehConfig,
                                                    Real lineOut[4]) {
  Real biasLen;
  Real candidateLine[4];
  Boolean foundClear;
  Boolean loopActive;
  Boolean hitFlag;

  apaThirdBoundaryCopy4(baseLine, lineOut);
  biasLen = 0.0f;
  foundClear = false;
  loopActive = apaMathLess(biasLen, maxLen);
  while (loopActive) {
    apaThirdBoundaryTranslateLineByLen(baseLine, unitVec, biasLen, candidateLine);
    hitFlag =
        apaThirdBoundaryLineIntersectsVehicle(candidateLine, pose, vehConfig, 0.0f, 0.0f, 0.0f);
    if (!hitFlag) {
      foundClear = true;
    } else {
      biasLen = biasLen + stepLen;
    }
    loopActive = apaMathLess(biasLen, maxLen);
    if (loopActive) {
      loopActive = !foundClear;
    }
  }
  if (foundClear) {
    if (apaMathGreater(biasLen, 0.0f)) {
      biasLen = biasLen + extraClearLen;
      apaThirdBoundaryTranslateLineByLen(baseLine, unitVec, biasLen, candidateLine);
    }
    apaThirdBoundaryCopy4(candidateLine, lineOut);
  }
}

static void apaThirdBoundaryBuildParallelFrontRect(const Real ptFEnter[2],
                                                   const Real ptFNotEnter[2],
                                                   const Real ptREnter[2],
                                                   const Real ptRNotEnter[2],
                                                   Real ptFInsideCollision[2],
                                                   Real ptFOutsideCollision[2],
                                                   Real ptRInsideCollision[2],
                                                   Real ptROutsideCollision[2],
                                                   Real rectPts[8]) {
  Real lonBias[2];

  ptFInsideCollision[0] = ptFEnter[0];
  ptFInsideCollision[1] = ptFEnter[1];
  ptFOutsideCollision[0] = 0.5f * (ptFEnter[0] + ptFNotEnter[0]);
  ptFOutsideCollision[1] = 0.5f * (ptFEnter[1] + ptFNotEnter[1]);
  ptRInsideCollision[0] = 0.5f * (ptFEnter[0] + ptREnter[0]);
  ptRInsideCollision[1] = 0.5f * (ptFEnter[1] + ptREnter[1]);
  ptROutsideCollision[0] = 0.5f * (ptFEnter[0] + ptRNotEnter[0]);
  ptROutsideCollision[1] = 0.5f * (ptFEnter[1] + ptRNotEnter[1]);

  lonBias[0] = ptFInsideCollision[0] - ptRInsideCollision[0];
  lonBias[1] = ptFInsideCollision[1] - ptRInsideCollision[1];
  apaThirdBoundaryTranslatePoint(ptFInsideCollision, lonBias, ptFInsideCollision);
  apaThirdBoundaryTranslatePoint(ptFOutsideCollision, lonBias, ptFOutsideCollision);
  apaThirdBoundaryTranslatePoint(ptRInsideCollision, lonBias, ptRInsideCollision);
  apaThirdBoundaryTranslatePoint(ptROutsideCollision, lonBias, ptROutsideCollision);

  rectPts[0] = ptFInsideCollision[0];
  rectPts[1] = ptFInsideCollision[1];
  rectPts[2] = ptFOutsideCollision[0];
  rectPts[3] = ptFOutsideCollision[1];
  rectPts[4] = ptROutsideCollision[0];
  rectPts[5] = ptROutsideCollision[1];
  rectPts[6] = ptRInsideCollision[0];
  rectPts[7] = ptRInsideCollision[1];
}

static Real apaThirdBoundaryFrontRearCollisionDetect(const Real ptFInside[2],
                                                     const Real ptFOutside[2],
                                                     const Real ptROutside[2],
                                                     const Real ptRInside[2],
                                                     const VehicleCollisionCheckInput_t *obsInput,
                                                     Integer dir) {
  Real stepLen;
  Real baseVec[2];
  Real baseUnit[2];
  Real maxLen;
  Real biasLen;
  Real stepSign;
  Real rectPts[8];
  Real rectMoved[8];
  Real shiftVec[2];
  Real ptFInsideCollision[2];
  Real ptFOutsideCollision[2];
  Real ptROutsideCollision[2];
  Real ptRInsideCollision[2];
  Boolean isCollision;
  Boolean loopActive;
  Real retValue;

  stepLen = 0.1f;
  baseVec[0] = ptFInside[0] - ptRInside[0];
  baseVec[1] = ptFInside[1] - ptRInside[1];
  apaMathNormalize2D(baseVec[0], baseVec[1], &baseUnit[0], &baseUnit[1]);

  if (dir == 0) {
    maxLen = 3.0f;
    biasLen = -0.5f;
    stepSign = 1.0f;
    ptFInsideCollision[0] = ptFInside[0];
    ptFInsideCollision[1] = ptFInside[1];
    ptFOutsideCollision[0] = 0.5f * (ptFInside[0] + ptFOutside[0]);
    ptFOutsideCollision[1] = 0.5f * (ptFInside[1] + ptFOutside[1]);
    ptROutsideCollision[0] =
        0.5f * (ptFOutsideCollision[0] + 0.5f * (ptROutside[0] + ptRInside[0]));
    ptROutsideCollision[1] =
        0.5f * (ptFOutsideCollision[1] + 0.5f * (ptROutside[1] + ptRInside[1]));
    ptRInsideCollision[0] = 0.5f * (ptFInside[0] + ptRInside[0]);
    ptRInsideCollision[1] = 0.5f * (ptFInside[1] + ptRInside[1]);
  } else {
    maxLen = -2.0f;
    biasLen = 0.5f;
    stepSign = -1.0f;
    ptFInsideCollision[0] = 0.5f * (ptFInside[0] + ptRInside[0]);
    ptFInsideCollision[1] = 0.5f * (ptFInside[1] + ptRInside[1]);
    ptRInsideCollision[0] = ptRInside[0];
    ptRInsideCollision[1] = ptRInside[1];
    ptROutsideCollision[0] = 0.5f * (ptROutside[0] + ptRInside[0]);
    ptROutsideCollision[1] = 0.5f * (ptROutside[1] + ptRInside[1]);
    ptFOutsideCollision[0] =
        0.5f * (ptROutsideCollision[0] + 0.5f * (ptFInside[0] + ptFOutside[0]));
    ptFOutsideCollision[1] =
        0.5f * (ptROutsideCollision[1] + 0.5f * (ptFInside[1] + ptFOutside[1]));
  }

  rectPts[0] = ptFInsideCollision[0];
  rectPts[1] = ptFInsideCollision[1];
  rectPts[2] = ptFOutsideCollision[0];
  rectPts[3] = ptFOutsideCollision[1];
  rectPts[4] = ptROutsideCollision[0];
  rectPts[5] = ptROutsideCollision[1];
  rectPts[6] = ptRInsideCollision[0];
  rectPts[7] = ptRInsideCollision[1];

  isCollision = apaThirdBoundaryRectCollisionWithObs(rectPts, obsInput);
  if (!isCollision) {
    biasLen = 0.0f;
  }

  isCollision = false;
  if (dir == 0) {
    loopActive = !apaMathGreater(biasLen, maxLen);
  } else {
    loopActive = !apaMathLess(biasLen, maxLen);
  }
  while (loopActive) {
    shiftVec[0] = baseUnit[0] * biasLen;
    shiftVec[1] = baseUnit[1] * biasLen;
    apaThirdBoundaryRectTranslate(rectPts, shiftVec, rectMoved);
    isCollision = apaThirdBoundaryRectCollisionWithObs(rectMoved, obsInput);
    if (!isCollision) {
      biasLen = biasLen + stepSign * stepLen;
    }
    if (dir == 0) {
      loopActive = !apaMathGreater(biasLen, maxLen);
    } else {
      loopActive = !apaMathLess(biasLen, maxLen);
    }
    if (loopActive) {
      loopActive = !isCollision;
    }
  }

  retValue = biasLen;
  if (isCollision) {
    retValue = biasLen - stepSign * stepLen;
  }
  return retValue;
}

static Real apaThirdBoundaryMaxFrontCollisionDetect(const Real ptFInside[2],
                                                    const Real ptFOutside[2],
                                                    const Real rectPtROutside[2],
                                                    const Real rectPtRInside[2],
                                                    const VehicleCollisionCheckInput_t *obsInput) {
  Real stepLen;
  Real baseVec[2];
  Real baseUnit[2];
  Real rectPts[8];
  Real rectMoved[8];
  Real shiftVec[2];
  Real biasLen;
  Real maxLen;
  Real retValue;
  Real ptFInsideCollision[2];
  Real ptFOutsideCollision[2];
  Real ptROutsideCollision[2];
  Real ptRInsideCollision[2];
  Boolean loopActive;
  Boolean isCollision;

  stepLen = 0.1f;
  maxLen = 4.0f;
  baseVec[0] = ptFInside[0] - rectPtRInside[0];
  baseVec[1] = ptFInside[1] - rectPtRInside[1];
  apaMathNormalize2D(baseVec[0], baseVec[1], &baseUnit[0], &baseUnit[1]);

  ptFInsideCollision[0] = ptFInside[0];
  ptFInsideCollision[1] = ptFInside[1];
  ptFOutsideCollision[0] = 0.5f * (ptFInside[0] + ptFOutside[0]);
  ptFOutsideCollision[1] = 0.5f * (ptFInside[1] + ptFOutside[1]);
  ptRInsideCollision[0] = 0.5f * (ptFInside[0] + rectPtRInside[0]);
  ptRInsideCollision[1] = 0.5f * (ptFInside[1] + rectPtRInside[1]);
  ptROutsideCollision[0] =
      0.5f * (ptFOutsideCollision[0] + 0.5f * (rectPtROutside[0] + rectPtRInside[0]));
  ptROutsideCollision[1] =
      0.5f * (ptFOutsideCollision[1] + 0.5f * (rectPtROutside[1] + rectPtRInside[1]));
  rectPts[0] = ptFInsideCollision[0];
  rectPts[1] = ptFInsideCollision[1];
  rectPts[2] = ptFOutsideCollision[0];
  rectPts[3] = ptFOutsideCollision[1];
  rectPts[4] = ptROutsideCollision[0];
  rectPts[5] = ptROutsideCollision[1];
  rectPts[6] = ptRInsideCollision[0];
  rectPts[7] = ptRInsideCollision[1];

  biasLen = -0.5f;
  isCollision = apaThirdBoundaryRectCollisionWithObs(rectPts, obsInput);
  if (!isCollision) {
    biasLen = 0.0f;
  }

  retValue = 0.0f;
  loopActive = !apaMathGreater(biasLen, maxLen);
  while (loopActive) {
    shiftVec[0] = baseUnit[0] * biasLen;
    shiftVec[1] = baseUnit[1] * biasLen;
    apaThirdBoundaryRectTranslate(rectPts, shiftVec, rectMoved);
    isCollision = apaThirdBoundaryRectCollisionWithObs(rectMoved, obsInput);
    if (isCollision) {
      retValue = biasLen;
    }
    biasLen = biasLen + stepLen;
    loopActive = !apaMathGreater(biasLen, maxLen);
  }
  return retValue;
}

static Boolean apaThirdBoundaryJudgeStopperInSlot(const Real slotPts[8],
                                                  const VehicleCollisionCheckInput_t *obsInput) {
  Integer idx;
  Boolean stopperInSlot;
  Boolean ptInSlot;

  stopperInSlot = false;
  idx = 0;
  while (idx < obsInput->SlotTrackStopperNum) {
    ptInSlot = apaMathIsPtInRect(
        obsInput->SlotTrackStopperX1[idx], obsInput->SlotTrackStopperY1[idx], slotPts);
    if (ptInSlot) {
      stopperInSlot = true;
    }
    ptInSlot = apaMathIsPtInRect(
        obsInput->SlotTrackStopperX2[idx], obsInput->SlotTrackStopperY2[idx], slotPts);
    if (ptInSlot) {
      stopperInSlot = true;
    }
    idx = idx + 1;
  }

  idx = 0;
  while (idx < obsInput->SeparateStopperNum) {
    ptInSlot = apaMathIsPtInRect(
        obsInput->SeparateStopperX[idx], obsInput->SeparateStopperY[idx], slotPts);
    if (ptInSlot) {
      stopperInSlot = true;
    }
    idx = idx + 1;
  }
  return stopperInSlot;
}

static void apaThirdBoundaryAppendParallelLines(const ApaSlot_t *slot,
                                                const Real vehPos[3],
                                                const VehicleCollisionCheckInput_t *obsInput,
                                                const ApaPlannerVehConfig_t *vehConfig,
                                                const Real lnFront[4],
                                                const Real lnRear[4],
                                                ApaThirdBoundaryLineSet_t *lineSet) {
  Real devOutBias[2];
  Real ptFEnter[2];
  Real ptFNotEnter[2];
  Real ptREnter[2];
  Real ptRNotEnter[2];
  Real longVec[2];
  Real longUnit[2];
  Real lateralVec[2];
  Real lateralUnit[2];
  Real bottomLine[4];
  Real bottomTemp[4];
  Real upLine[4];
  Real upTemp[4];
  Real rearLine[4];
  Real rearTemp[4];
  Real pointA[2];
  Real pointB[2];
  Real shiftVec[2];
  Real lineK;
  Real lineB;
  Real distToEnterLine;
  Real upForwardLen;
  Real yawAbs;
  Real lenEnter;
  Real ptFInsideCollision[2];
  Real ptFOutsideCollision[2];
  Real ptRInsideCollision[2];
  Real ptROutsideCollision[2];
  Real frontRiskRect[8];
  Real frontBiasLen;
  Real maxFrontBias;
  Real frontBoundaryRearLen;
  Real rearBiasLen;
  Real lateralShiftLen;
  Boolean vehInSlot;
  Boolean frontCollisionRisk;
  Boolean longSlot;
  Boolean stopperInSlot;
  Boolean appendInsideFront;

  ptFEnter[0] = lnFront[0];
  ptFEnter[1] = lnFront[1];
  ptFNotEnter[0] = lnFront[2];
  ptFNotEnter[1] = lnFront[3];
  ptREnter[0] = lnRear[0];
  ptREnter[1] = lnRear[1];
  ptRNotEnter[0] = lnRear[2];
  ptRNotEnter[1] = lnRear[3];

  longVec[0] = ptFEnter[0] - ptREnter[0];
  longVec[1] = ptFEnter[1] - ptREnter[1];
  apaMathNormalize2D(longVec[0], longVec[1], &longUnit[0], &longUnit[1]);
  lateralVec[0] = ptFNotEnter[0] - ptFEnter[0];
  lateralVec[1] = ptFNotEnter[1] - ptFEnter[1];
  apaMathNormalize2D(lateralVec[0], lateralVec[1], &lateralUnit[0], &lateralUnit[1]);
  lenEnter = apaMathNorm2D(longVec[0], longVec[1]);

  shiftVec[0] = ptFNotEnter[0] - ptRNotEnter[0];
  shiftVec[1] = ptFNotEnter[1] - ptRNotEnter[1];
  apaThirdBoundaryTranslatePoint(ptFNotEnter, shiftVec, pointA);
  shiftVec[0] = -shiftVec[0];
  shiftVec[1] = -shiftVec[1];
  apaThirdBoundaryTranslatePoint(ptRNotEnter, shiftVec, pointB);
  apaThirdBoundaryLineFromPoints(pointA, pointB, bottomLine);
  apaThirdBoundarySearchLineClearWithPose(
      bottomLine, lateralUnit, 1.0f, 0.05f, 0.0f, vehPos, vehConfig, bottomTemp);
  apaThirdBoundarySearchLineClearWithPose(
      bottomTemp, lateralUnit, 0.5f, 0.05f, 0.0f, slot->TarPos, vehConfig, bottomLine);
  apaThirdBoundaryAppendLine(lineSet, bottomLine);

  apaGeoLinearFuncSolve(ptFEnter[0], ptFEnter[1], ptREnter[0], ptREnter[1], &lineK, &lineB);
  distToEnterLine = apaGeoPointLineDistanceFromSlopeIntercept(vehPos[0], vehPos[1], lineK, lineB);
  vehInSlot = apaMathIsPtInRect(vehPos[0], vehPos[1], slot->SlotPts);
  yawAbs = apaMathAbs(vehPos[2] - slot->TarPos[2]);
  if (!vehInSlot) {
    upForwardLen = apaThirdBoundaryMaxReal(4.0f, distToEnterLine + 2.5f);
  } else {
    if (apaMathLess(yawAbs, 4.0f / 180.0f * APA_PI)) {
      upForwardLen = 1.0f;
    } else if (apaMathLess(yawAbs, 10.0f / 180.0f * APA_PI)) {
      upForwardLen = 1.5f;
    } else {
      upForwardLen = 2.0f;
    }
  }

  shiftVec[0] = longVec[0] * 3.0f;
  shiftVec[1] = longVec[1] * 3.0f;
  apaThirdBoundaryTranslatePoint(ptFEnter, shiftVec, pointA);
  shiftVec[0] = -shiftVec[0];
  shiftVec[1] = -shiftVec[1];
  apaThirdBoundaryTranslatePoint(ptREnter, shiftVec, pointB);
  shiftVec[0] = -lateralUnit[0] * upForwardLen;
  shiftVec[1] = -lateralUnit[1] * upForwardLen;
  apaThirdBoundaryTranslatePoint(pointA, shiftVec, pointA);
  apaThirdBoundaryTranslatePoint(pointB, shiftVec, pointB);
  apaThirdBoundaryLineFromPoints(pointA, pointB, upLine);
  shiftVec[0] = -lateralUnit[0];
  shiftVec[1] = -lateralUnit[1];
  apaThirdBoundarySearchLineClearWithPose(
      upLine, shiftVec, 2.5f, 0.1f, 0.5f, vehPos, vehConfig, upTemp);
  apaThirdBoundaryAppendLine(lineSet, upTemp);

  apaThirdBoundaryBuildParallelFrontRect(ptFEnter,
                                         ptFNotEnter,
                                         ptREnter,
                                         ptRNotEnter,
                                         ptFInsideCollision,
                                         ptFOutsideCollision,
                                         ptRInsideCollision,
                                         ptROutsideCollision,
                                         frontRiskRect);

  if (!vehInSlot) {
    frontCollisionRisk = apaThirdBoundaryRectCollisionWithObs(frontRiskRect, obsInput);
    longSlot = apaMathGreater(lenEnter, 5.7f);
    if (frontCollisionRisk) {
      frontBiasLen = apaThirdBoundaryFrontRearCollisionDetect(
          ptFEnter, ptFNotEnter, ptRNotEnter, ptREnter, obsInput, 0);
      frontBiasLen = apaThirdBoundaryMaxReal(apaThirdBoundaryMinReal(6.0f - lenEnter, 1.0f),
                                             frontBiasLen - 0.55f);
      frontBiasLen = apaThirdBoundaryMaxReal(0.0f, frontBiasLen);
      shiftVec[0] = longUnit[0] * frontBiasLen;
      shiftVec[1] = longUnit[1] * frontBiasLen;
      apaThirdBoundaryParallelSlotSideEnterDeviation(
          ptFEnter, ptFNotEnter, ptREnter, ptRNotEnter, obsInput, devOutBias);
      pointA[0] = ptFEnter[0] + shiftVec[0] + devOutBias[0];
      pointA[1] = ptFEnter[1] + shiftVec[1] + devOutBias[1];
      apaThirdBoundaryTranslatePoint(ptFNotEnter, shiftVec, pointB);
    } else {
      frontBiasLen = apaThirdBoundaryMaxReal(7.0f - lenEnter, 0.0f);
      shiftVec[0] = longUnit[0] * frontBiasLen;
      shiftVec[1] = longUnit[1] * frontBiasLen;
      apaThirdBoundaryTranslatePoint(ptFEnter, shiftVec, pointA);
      apaThirdBoundaryTranslatePoint(ptFNotEnter, shiftVec, pointB);
      lateralShiftLen = 1.8f;
      if (longSlot) {
        lateralShiftLen = 1.2f;
      }
      shiftVec[0] = lateralUnit[0] * lateralShiftLen;
      shiftVec[1] = lateralUnit[1] * lateralShiftLen;
      apaThirdBoundaryTranslatePoint(pointA, shiftVec, pointA);
    }
    apaThirdBoundaryLineFromPoints(pointA, pointB, bottomTemp);
    apaThirdBoundaryAppendLine(lineSet, bottomTemp);

    maxFrontBias = apaThirdBoundaryMaxReal(2.0f, 8.0f - lenEnter);
    if (frontCollisionRisk) {
      shiftVec[0] = -longUnit[0] * 0.1f;
      shiftVec[1] = -longUnit[1] * 0.1f;
      apaThirdBoundaryTranslatePoint(ptFEnter, shiftVec, pointA);
      apaThirdBoundaryTranslatePoint(ptFNotEnter, shiftVec, pointB);
      maxFrontBias =
          apaThirdBoundaryMaxFrontCollisionDetect(ptFEnter, ptFNotEnter, pointB, pointA, obsInput);
    }
    frontBoundaryRearLen = lenEnter + frontBiasLen + 1.5f;
    if (apaMathGreater(maxFrontBias + lenEnter, frontBoundaryRearLen)) {
      shiftVec[0] = longUnit[0] * maxFrontBias;
      shiftVec[1] = longUnit[1] * maxFrontBias;
      apaThirdBoundaryTranslatePoint(ptFEnter, shiftVec, pointA);
      shiftVec[0] = longUnit[0] * frontBoundaryRearLen;
      shiftVec[1] = longUnit[1] * frontBoundaryRearLen;
      apaThirdBoundaryTranslatePoint(ptREnter, shiftVec, pointB);
      shiftVec[0] = lateralUnit[0] * 0.5f;
      shiftVec[1] = lateralUnit[1] * 0.5f;
      apaThirdBoundaryTranslatePoint(pointA, shiftVec, pointA);
      apaThirdBoundaryTranslatePoint(pointB, shiftVec, pointB);
      apaThirdBoundaryLineFromPoints(pointA, pointB, bottomTemp);
      apaThirdBoundaryAppendLine(lineSet, bottomTemp);
    }

    shiftVec[0] = longUnit[0] * (maxFrontBias + lenEnter);
    shiftVec[1] = longUnit[1] * (maxFrontBias + lenEnter);
    apaThirdBoundaryTranslatePoint(ptFEnter, shiftVec, pointA);
    apaThirdBoundaryTranslatePoint(ptREnter, shiftVec, pointB);
    apaThirdBoundaryLineFromPoints(pointA, pointB, bottomTemp);
    apaThirdBoundaryAppendLine(lineSet, bottomTemp);
  } else {
    if (apaMathLess(yawAbs, 10.0f / 180.0f * APA_PI)) {
      frontBiasLen = apaThirdBoundaryMaxReal(7.1f - lenEnter, 0.0f);
    } else {
      frontBiasLen = apaThirdBoundaryMaxReal(6.8f - lenEnter, 0.0f);
    }
    shiftVec[0] = longUnit[0] * frontBiasLen;
    shiftVec[1] = longUnit[1] * frontBiasLen;
    apaThirdBoundaryTranslatePoint(ptFEnter, shiftVec, pointA);
    apaThirdBoundaryTranslatePoint(ptFNotEnter, shiftVec, pointB);
    apaThirdBoundaryLineFromPoints(pointA, pointB, bottomTemp);
    appendInsideFront =
        !apaThirdBoundaryLineIntersectsVehicle(bottomTemp, vehPos, vehConfig, 0.0f, 0.0f, 0.0f);
    if (appendInsideFront) {
      apaThirdBoundaryAppendLine(lineSet, bottomTemp);
    }
  }

  stopperInSlot = apaThirdBoundaryJudgeStopperInSlot(slot->SlotPts, obsInput);
  if (!stopperInSlot) {
    rearBiasLen = -apaThirdBoundaryFrontRearCollisionDetect(
        ptFEnter, ptFNotEnter, ptRNotEnter, ptREnter, obsInput, 1);
    rearBiasLen = apaThirdBoundaryMaxReal(rearBiasLen - 0.6f, 0.0f);
  } else {
    rearBiasLen = 0.5f;
  }
  shiftVec[0] = -longUnit[0] * rearBiasLen;
  shiftVec[1] = -longUnit[1] * rearBiasLen;
  apaThirdBoundaryTranslatePoint(ptREnter, shiftVec, pointA);
  apaThirdBoundaryTranslatePoint(ptRNotEnter, shiftVec, pointB);
  apaThirdBoundaryLineFromPoints(pointA, pointB, rearLine);
  shiftVec[0] = -longUnit[0];
  shiftVec[1] = -longUnit[1];
  apaThirdBoundarySearchLineClearWithPose(
      rearLine, shiftVec, 2.5f, 0.1f, 0.2f, vehPos, vehConfig, rearTemp);
  apaThirdBoundaryAppendLine(lineSet, rearTemp);
}

static Boolean apaThirdBoundaryValidateInput(const ApaThirdBoundaryGetBoundaryInput_t *input) {
  Boolean valid;
  Boolean slotPtrNonNegative;
  Boolean slotPtrInRange;

  slotPtrNonNegative = (input->TarSlotPtr >= 0);
  slotPtrInRange = (input->TarSlotPtr < input->SlotCount);
  valid = false;
  if (slotPtrNonNegative) {
    if (slotPtrInRange) {
      valid = true;
    }
  }
  return valid;
}

static void apaThirdBoundaryRunCore(const ApaThirdBoundaryGetBoundaryInput_t *input,
                                    ApaThirdBoundaryGetBoundaryOutput_t *output) {
  const ApaSlot_t *tarSlot;
  Real lnLeft[4];
  Real lnRight[4];
  Integer slotClass;
  Integer slotSubStatus;

  tarSlot = &input->SlotList[input->TarSlotPtr];
  apaThirdBoundarySelectSlotClass(tarSlot, input->SlotClass, &slotClass);
  apaThirdBoundarySelectSlotSubStatus(tarSlot, input->SlotSubStatus, &slotSubStatus);
  apaSlotCalcSideLine(
      tarSlot->SlotPts, tarSlot->EnterEdge, tarSlot->TarPos, slotClass, lnLeft, lnRight);

  if (slotClass == APA_SLOT_CLASS_VERTICAL) {
    apaThirdBoundaryAppendVerticalLines(tarSlot,
                                        input->SlotList,
                                        input->SlotCount,
                                        input->TarSlotPtr,
                                        input->VehPos,
                                        input->SurroundingType,
                                        input->ParkingType,
                                        slotSubStatus,
                                        input->IsNormalScene,
                                        input->UseSideExtendFlag,
                                        &input->Obs,
                                        &input->VehConfig,
                                        lnLeft,
                                        lnRight,
                                        &output->OutLines);
    apaThirdBoundaryDeleteCollisionWithVehDefault(
        input->VehPos, &input->VehConfig, &output->OutLines);
    apaThirdBoundaryDeleteCollisionWithVehDefault(
        tarSlot->TarPos, &input->VehConfig, &output->OutLines);
  } else if (slotClass == APA_SLOT_CLASS_PARALLEL) {
    apaThirdBoundaryAppendParallelLines(
        tarSlot, input->VehPos, &input->Obs, &input->VehConfig, lnLeft, lnRight, &output->OutLines);
    apaThirdBoundaryDeleteCollisionWithVehDefault(
        input->VehPos, &input->VehConfig, &output->OutLines);
    apaThirdBoundaryDeleteCollisionWithVehDefault(
        tarSlot->TarPos, &input->VehConfig, &output->OutLines);
  } else {
    apaThirdBoundaryAppendLine(&output->OutLines, lnLeft);
    apaThirdBoundaryAppendLine(&output->OutLines, lnRight);
  }
}

/**
 * @brief APA 第三边界生成器大编排壳
 * @en_name apaThirdBoundaryGetBoundaryAide
 * @cn_name APA 第三边界生成器大编排壳
 * @type module
 * @param[IN] ApaThirdBoundaryGetBoundaryInput_t Input 第三边界输入
 * @param[OUT] ApaThirdBoundaryGetBoundaryOutput_t Output 第三边界输出
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 综合处理
 * @source matlab_rewrite get_3rd_boundary.m
 * @version 1.1
 * @date 2026-05-02
 * @author aide
 */
void apaThirdBoundaryGetBoundaryAide(const ApaThirdBoundaryGetBoundaryInput_t *input,
                                     ApaThirdBoundaryGetBoundaryOutput_t *output) {
  Boolean validInput;

  apaThirdBoundaryResetLineSet(&output->OutLines);
  output->Success = true;
  validInput = apaThirdBoundaryValidateInput(input);
  if (validInput) {
    apaThirdBoundaryRunCore(input, output);
  } else {
    output->Success = false;
  }
}

void apaThirdBoundaryGetBoundary_aide(const ApaThirdBoundaryGetBoundaryInput_t *input,
                                      ApaThirdBoundaryGetBoundaryOutput_t *output) {
  apaThirdBoundaryGetBoundaryAide(input, output);
}

Boolean apaThirdBoundaryGetBoundary(const Real vehPos[3],
                                    Integer slotClass,
                                    Integer surroundingType,
                                    Integer parkingType,
                                    const ApaSlot_t *slotList,
                                    Integer slotCount,
                                    Integer tarSlotPtr,
                                    const VehicleCollisionCheckInput_t *obs,
                                    Integer slotSubStatus,
                                    Boolean isNormalScene,
                                    Boolean useSideExtendFlag,
                                    const ApaPlannerVehConfig_t *vehConfig,
                                    ApaThirdBoundaryLineSet_t *outLines) {
  ApaThirdBoundaryGetBoundaryInput_t input;
  ApaThirdBoundaryGetBoundaryOutput_t output;
  Integer idx;
  Boolean success;
  Boolean copyLoop;

  input = (ApaThirdBoundaryGetBoundaryInput_t){0};
  output = (ApaThirdBoundaryGetBoundaryOutput_t){0};
  idx = 0;
  while (idx < 3) {
    input.VehPos[idx] = vehPos[idx];
    idx = idx + 1;
  }
  input.SlotClass = slotClass;
  input.SurroundingType = surroundingType;
  input.ParkingType = parkingType;
  input.SlotCount = slotCount;
  input.TarSlotPtr = tarSlotPtr;
  input.SlotSubStatus = slotSubStatus;
  input.IsNormalScene = isNormalScene;
  input.UseSideExtendFlag = useSideExtendFlag;
  input.Obs = *obs;
  input.VehConfig = *vehConfig;

  idx = 0;
  copyLoop = (idx < slotCount);
  if (copyLoop) {
    copyLoop = (idx < 4);
  }
  while (copyLoop) {
    input.SlotList[idx] = slotList[idx];
    idx = idx + 1;
    copyLoop = (idx < slotCount);
    if (copyLoop) {
      copyLoop = (idx < 4);
    }
  }

  apaThirdBoundaryGetBoundaryAide(&input, &output);
  *outLines = output.OutLines;
  success = output.Success;
  return success;
}
