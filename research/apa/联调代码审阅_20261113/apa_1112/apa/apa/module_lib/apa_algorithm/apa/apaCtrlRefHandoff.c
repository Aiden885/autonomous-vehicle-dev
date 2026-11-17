#include "apaCoordUtils.h"
#include "apaCountUtils.h"
#include "apaDebugTrace.h"
#include "apaCtrlRefHandoffTypes.h"
#include "apaEnums.h"
#include "apaMathUtils.h"
#include <math.h>
#include <stdio.h>

#define PHASE_IDLE 0
#define PHASE_SEG_INIT 1
#define PHASE_GEAR_WAIT 2
#define PHASE_SEG_TRACK 3
#define PHASE_STOP_WAIT 4
#define PHASE_SEG_ADVANCE 5
#define PHASE_ARRIVED 6

#ifndef APA_CTRL_REF_HANDOFF_ODO_EPSILON
#define APA_CTRL_REF_HANDOFF_ODO_EPSILON 1.0e-4f
#endif

#ifndef APA_CTRL_REF_HANDOFF_GOAL_YAW_TOL
#define APA_CTRL_REF_HANDOFF_GOAL_YAW_TOL 0.12f
#endif

#ifndef APA_CTRL_REF_HANDOFF_ARRIVED_LONG_ERR_TOL
#define APA_CTRL_REF_HANDOFF_ARRIVED_LONG_ERR_TOL 0.005f
#endif

#ifndef APA_CTRL_REF_HANDOFF_LOOK_AHEAD
#define APA_CTRL_REF_HANDOFF_LOOK_AHEAD 3
#endif

static Boolean apaCtrlRefHandoffPathValid(const ApaCtrlRefHandoffInput_t *input) {
  Integer index0;
  Integer nextIndex0;
  Real pathLength0;
  Real stepLength0;
  Boolean condCount0;
  Boolean condFinite0;
  Boolean condLength0;

  condCount0 = (input->PtCount > 1) && (input->PtCount <= PATH_PT_MAX);
  condFinite0 = false;
  condLength0 = false;
  if (condCount0) {
    condFinite0 = true;
    pathLength0 = 0.0f;
    index0 = 0;
    while ((index0 < input->PtCount) && condFinite0) {
      condFinite0 = apaMathIsFinitePathValue(input->PthPtX[index0]) &&
                    apaMathIsFinitePathValue(input->PthPtY[index0]) &&
                    apaMathIsFinitePathValue(input->PthPtTht[index0]) &&
                    apaMathIsFinitePathValue(input->PthPtOdo[index0]);
      if (index0 < (input->PtCount - 1)) {
        nextIndex0 = index0 + 1;
        stepLength0 = apaMathDistance2D(input->PthPtX[index0],
                                        input->PthPtY[index0],
                                        input->PthPtX[nextIndex0],
                                        input->PthPtY[nextIndex0]);
        pathLength0 = pathLength0 + stepLength0;
      }
      index0 = index0 + 1;
    }
    condLength0 = condFinite0 && (pathLength0 > APA_CTRL_REF_HANDOFF_ODO_EPSILON);
  }

  return condCount0 && condFinite0 && condLength0;
}

static Integer apaCtrlRefHandoffCurrentGoDir(const ApaCtrlRefHandoffState_t *state) {
  Integer goDir0;
  Integer stpNow0;
  Boolean condSegValid0;

  goDir0 = 0;
  stpNow0 = state->StpNow;
  condSegValid0 = (state->SegTableBuilt == 1) && (stpNow0 >= 0) && (stpNow0 < state->StpNum);
  if (condSegValid0) {
    goDir0 = state->SegGoDir[stpNow0];
  }

  return goDir0;
}

static Integer apaCtrlRefHandoffResolveArrivedDir(const ApaCtrlRefHandoffInput_t *input,
                                                  const ApaCtrlRefHandoffState_t *state,
                                                  Integer maxIndex,
                                                  Real longErr) {
  Integer arrivedDir0;
  Integer lastSeg0;
  Integer scanIdx0;
  Boolean condLongForward0;
  Boolean condLongReverse0;
  Boolean condUseSegDir0;
  Boolean condFoundOdoDir0;

  arrivedDir0 = 1;
  condLongForward0 = (longErr > APA_CTRL_REF_HANDOFF_ARRIVED_LONG_ERR_TOL);
  condLongReverse0 = (longErr < -APA_CTRL_REF_HANDOFF_ARRIVED_LONG_ERR_TOL);

  if (condLongForward0) {
    arrivedDir0 = 1;
  } else if (condLongReverse0) {
    arrivedDir0 = -1;
  } else {
    lastSeg0 = state->StpNum - 1;
    condUseSegDir0 =
        (state->SegTableBuilt == 1) && (lastSeg0 >= 0) && (lastSeg0 < APA_CTRL_REF_HANDOFF_MAX_SEG);
    if (condUseSegDir0) {
      arrivedDir0 = state->SegGoDir[lastSeg0];
      if (arrivedDir0 == 0) {
        arrivedDir0 = 1;
      }
    } else {
      condFoundOdoDir0 = false;
      scanIdx0 = maxIndex - 1;
      while ((scanIdx0 >= 0) && (!condFoundOdoDir0)) {
        Real odoNow0;

        odoNow0 = input->PthPtOdo[scanIdx0];
        if (odoNow0 > APA_CTRL_REF_HANDOFF_ODO_EPSILON) {
          arrivedDir0 = 1;
          condFoundOdoDir0 = true;
        } else if (odoNow0 < -APA_CTRL_REF_HANDOFF_ODO_EPSILON) {
          arrivedDir0 = -1;
          condFoundOdoDir0 = true;
        }
        scanIdx0 = scanIdx0 - 1;
      }
    }
  }

  return arrivedDir0;
}

static Real apaCtrlRefHandoffCalcArrivedSpeedScale(const ApaCtrlRefHandoffParam_t *param,
                                                   Real goalDistSq,
                                                   Real goalYawErr) {
  Real goalDist0;
  Real goalDistTol0;
  Real distScale0;
  Real yawScale0;
  Real speedScale0;

  goalDist0 = sqrtf(goalDistSq);
  goalDistTol0 = sqrtf(param->PointReachedDistSq);
  if (goalDistTol0 < 0.05f) {
    goalDistTol0 = 0.05f;
  }

  distScale0 = goalDist0 / (goalDistTol0 * 3.0f);
  yawScale0 = goalYawErr / (APA_CTRL_REF_HANDOFF_GOAL_YAW_TOL * 3.0f);
  speedScale0 = distScale0;
  if (yawScale0 > speedScale0) {
    speedScale0 = yawScale0;
  }
  if (speedScale0 > 0.3f) {
    speedScale0 = 0.3f;
  }
  if (speedScale0 < 0.0f) {
    speedScale0 = 0.0f;
  }

  return speedScale0;
}

/**
 * @brief 计算参考点处路径曲率（仿MATLAB sin(dθ)/ds 中点欧拉法）
 *        以及车辆纵向/横向偏差和航向偏差
 */
static void apaCtrlRefHandoffCalcCurvatureAndErrors(const ApaCtrlRefHandoffInput_t *input,
                                                    Integer refIdx,
                                                    Integer segStart,
                                                    Integer segEnd,
                                                    Real *outCurvature,
                                                    Real *outLongError,
                                                    Real *outLatError,
                                                    Real *outHeadingError) {
  Real curvature0;
  Real longError0;
  Real latError0;
  Real headingError0;
  Real refYaw0;
  Real ds0;
  Real dTheta0;
  Integer ptCount0;

  ptCount0 = input->PtCount;
  curvature0 = 0.0f;

  if ((refIdx > segStart) && (refIdx < segEnd) && (refIdx < ptCount0 - 1)) {

    Real thtPrev0;
    Real thtNext0;
    Real dThetaCentral0;
    Real dsAvg0;

    thtPrev0 = input->PthPtTht[refIdx - 1];
    thtNext0 = input->PthPtTht[refIdx + 1];
    dThetaCentral0 = thtNext0 - thtPrev0;

    dsAvg0 = input->PthPtOdo[refIdx - 1] + input->PthPtOdo[refIdx];

    if (fabsf(dsAvg0) > APA_CTRL_REF_HANDOFF_ODO_EPSILON) {
      curvature0 = sinf(dThetaCentral0) / dsAvg0;
    }
  } else if ((refIdx < ptCount0 - 1) && (refIdx >= 0)) {

    dTheta0 = input->PthPtTht[refIdx + 1] - input->PthPtTht[refIdx];

    ds0 = input->PthPtOdo[refIdx];
    if (fabsf(ds0) > APA_CTRL_REF_HANDOFF_ODO_EPSILON) {
      curvature0 = sinf(dTheta0) / ds0;
    }
  }

  refYaw0 = input->PthPtTht[refIdx];
  {
    Real dxVeh0;
    Real dyVeh0;
    Real cosRef0;
    Real sinRef0;

    dxVeh0 = input->PthPtX[refIdx] - input->VehicleX;
    dyVeh0 = input->PthPtY[refIdx] - input->VehicleY;
    cosRef0 = cosf(refYaw0);
    sinRef0 = sinf(refYaw0);

    longError0 = cosRef0 * dxVeh0 + sinRef0 * dyVeh0;

    latError0 = -sinRef0 * dxVeh0 + cosRef0 * dyVeh0;
  }

  headingError0 = refYaw0 - input->VehicleYaw;
  headingError0 = apaCoordWrapToHalfOpenPi(headingError0);

  *outCurvature = curvature0;
  *outLongError = longError0;
  *outLatError = latError0;
  *outHeadingError = headingError0;
}

/**
 * @brief 构建段表: 扫描PthPtOdo,按odo符号翻转划分段边界
 */
static void apaCtrlRefHandoffBuildSegTable(const ApaCtrlRefHandoffInput_t *input,
                                           const ApaCtrlRefHandoffParam_t *param,
                                           ApaCtrlRefHandoffState_t *state) {
  Integer ptCount0;
  Integer maxIndex0;
  Integer segCount0;
  Integer scanIdx0;
  Integer maxSeg0;
  Real odoNow0;
  Real absOdoNow0;
  Integer lastSign0;
  Integer currentSign0;
  Boolean condFlip0;
  Boolean condOdoValid0;
  Boolean condSegFull0;
  Boolean condHasLastSign0;

  ptCount0 = input->PtCount;
  maxIndex0 = ptCount0 - 1;
  maxSeg0 = APA_CTRL_REF_HANDOFF_MAX_SEG;

  segCount0 = 0;
  state->SegStartIdx[0] = 0;
  state->SegGoDir[0] = 1;
  lastSign0 = 1;
  condHasLastSign0 = false;

  scanIdx0 = 0;
  while (scanIdx0 < maxIndex0) {
    odoNow0 = input->PthPtOdo[scanIdx0];
    absOdoNow0 = fabsf(odoNow0);
    condOdoValid0 = (absOdoNow0 > APA_CTRL_REF_HANDOFF_ODO_EPSILON);

    if (condOdoValid0) {
      if (odoNow0 > 0.0f) {
        currentSign0 = 1;
      } else {
        currentSign0 = -1;
      }

      if (condHasLastSign0) {
        condFlip0 = (currentSign0 != lastSign0);
        if (condFlip0) {

          state->SegEndIdx[segCount0] = scanIdx0;

          segCount0 = segCount0 + 1;
          condSegFull0 = (segCount0 >= maxSeg0);
          if (condSegFull0) {

            segCount0 = segCount0 - 1;
            state->SegEndIdx[segCount0] = maxIndex0;
            scanIdx0 = maxIndex0;
          } else {
            state->SegStartIdx[segCount0] = scanIdx0;
            state->SegGoDir[segCount0] = currentSign0;
            (void)fprintf(stderr,
                          "[APA_DIAG] frame_id=%lu stage=HANDOFF_ODO_TO_SEG "
                          "idx=%ld odo=%.9g sign=%ld seg=%ld SegGoDir=%ld\n",
                          (unsigned long)apaDebugTraceCurrentFrame(),
                          (long)scanIdx0,
                          (double)odoNow0,
                          (long)currentSign0,
                          (long)segCount0,
                          (long)state->SegGoDir[segCount0]);
          }
        }
      } else {
        state->SegGoDir[0] = currentSign0;
        condHasLastSign0 = true;
        (void)fprintf(stderr,
                      "[APA_DIAG] frame_id=%lu stage=HANDOFF_ODO_TO_SEG "
                      "idx=%ld odo=%.9g sign=%ld seg=0 SegGoDir=%ld\n",
                      (unsigned long)apaDebugTraceCurrentFrame(),
                      (long)scanIdx0,
                      (double)odoNow0,
                      (long)currentSign0,
                      (long)state->SegGoDir[0]);
      }
      lastSign0 = currentSign0;
    }
    scanIdx0 = scanIdx0 + 1;
  }

  state->SegEndIdx[segCount0] = maxIndex0;
  state->StpNum = segCount0 + 1;

  {
    Integer expectDir0;
    Real firstSegOdo0;
    Real expectThresh0;
    Boolean condExpectDirSet0;
    Boolean condFirstSegLong0;
    Boolean condUseExpect0;
    Boolean condNoOdoSign0;
    Boolean condHasEnoughPoints0;
    Boolean condUseGeometry0;

    expectDir0 = param->ExpectStartDir;
    expectThresh0 = param->ExpectDirOdoThresh;
    if (expectThresh0 < 0.01f) {
      expectThresh0 = 0.15f;
    }

    firstSegOdo0 = 0.0f;
    if (state->StpNum > 0) {
      Integer fsi;
      Integer fsEnd;
      fsEnd = state->SegEndIdx[0];
      for (fsi = state->SegStartIdx[0]; fsi < fsEnd && fsi < (ptCount0 - 1); fsi = fsi + 1) {
        firstSegOdo0 = firstSegOdo0 + fabsf(input->PthPtOdo[fsi]);
      }
    }

    condExpectDirSet0 = (expectDir0 != 0);
    condFirstSegLong0 = (firstSegOdo0 > expectThresh0);
    condUseExpect0 = condExpectDirSet0 && condFirstSegLong0;
    condNoOdoSign0 = (!condHasLastSign0);
    condHasEnoughPoints0 = (ptCount0 > 2);
    condUseGeometry0 = condNoOdoSign0 && condHasEnoughPoints0;
    if (condUseExpect0) {

      if (expectDir0 > 0) {
        state->SegGoDir[0] = 1;
      } else {
        state->SegGoDir[0] = -1;
      }
    } else if (condUseGeometry0) {

      Real cosYaw0;
      Real sinYaw0;
      Integer nearIdx0;
      Real pathDx0;
      Real pathDy0;
      Real dotPath0;
      Real distSq0;

      cosYaw0 = cosf(input->VehicleYaw);
      sinYaw0 = sinf(input->VehicleYaw);

      nearIdx0 = 3;
      if (nearIdx0 >= ptCount0) {
        nearIdx0 = ptCount0 - 1;
      }

      pathDx0 = input->PthPtX[nearIdx0] - input->PthPtX[0];
      pathDy0 = input->PthPtY[nearIdx0] - input->PthPtY[0];
      distSq0 = pathDx0 * pathDx0 + pathDy0 * pathDy0;

      if (distSq0 > 0.01f) {
        dotPath0 = cosYaw0 * pathDx0 + sinYaw0 * pathDy0;
        if (dotPath0 > APA_CTRL_REF_HANDOFF_ODO_EPSILON) {
          state->SegGoDir[0] = 1;
        } else if (dotPath0 < -APA_CTRL_REF_HANDOFF_ODO_EPSILON) {
          state->SegGoDir[0] = -1;
        }
      }
    }
  }

  state->SegTableBuilt = 1;
}

static Boolean
apaCtrlRefHandoffShouldUseGeneratedReversePoint(const ApaCtrlRefHandoffParam_t *param,
                                                const ApaCtrlRefHandoffState_t *state,
                                                Integer segIndex) {
  Boolean condSegValid0;
  Boolean condEnable0;
  Boolean condReverse0;
  Boolean condDist0;
  Boolean useReversePoint0;

  condSegValid0 = (segIndex >= 0) && (segIndex < state->StpNum);
  condEnable0 = (param->EnableGenRevPt != 0);
  condReverse0 = false;
  if (condSegValid0) {
    condReverse0 = (state->SegGoDir[segIndex] < 0);
  }
  condDist0 = (param->ReverseDist > APA_CTRL_REF_HANDOFF_ODO_EPSILON);
  useReversePoint0 = condSegValid0 && condEnable0 && condReverse0 && condDist0;

  return useReversePoint0;
}

static Integer apaCtrlRefHandoffResolveReversePointIndex(const ApaCtrlRefHandoffInput_t *input,
                                                         const ApaCtrlRefHandoffParam_t *param,
                                                         Integer segStart,
                                                         Integer segEnd) {
  Integer reversePointIdx0;
  Real reverseDist0;
  Real accumDist0;

  reversePointIdx0 = segStart;
  reverseDist0 = param->ReverseDist;
  accumDist0 = 0.0f;

  while ((reversePointIdx0 < segEnd) && (accumDist0 < reverseDist0)) {
    accumDist0 = accumDist0 + fabsf(input->PthPtOdo[reversePointIdx0]);
    reversePointIdx0 = reversePointIdx0 + 1;
  }

  if (reversePointIdx0 > segEnd) {
    reversePointIdx0 = segEnd;
  }

  return reversePointIdx0;
}

static Real apaCtrlRefHandoffCalcRemainingOdoToEnd(const ApaCtrlRefHandoffInput_t *input,
                                                   Integer startIdx,
                                                   Integer segEnd) {
  Integer scanIdx0;
  Real remainingOdo0;

  scanIdx0 = startIdx;
  remainingOdo0 = 0.0f;
  while (scanIdx0 < segEnd) {
    Real stepOdo0;
    Real stepAbs0;

    stepOdo0 = input->PthPtOdo[scanIdx0];
    stepAbs0 = fabsf(stepOdo0);
    remainingOdo0 = remainingOdo0 + stepAbs0;
    scanIdx0 = scanIdx0 + 1;
  }

  return remainingOdo0;
}

static Real apaCtrlRefHandoffCurrentDist2Go(const ApaCtrlRefHandoffInput_t *input,
                                            const ApaCtrlRefHandoffState_t *state) {
  Integer stpNow0;
  Integer curIdx0;
  Integer segEnd0;
  Real dist2Go0;
  Boolean condSegValid0;

  dist2Go0 = 0.0f;
  stpNow0 = state->StpNow;
  condSegValid0 = (state->SegTableBuilt == 1) && (stpNow0 >= 0) && (stpNow0 < state->StpNum);
  if (condSegValid0) {
    segEnd0 = state->SegEndIdx[stpNow0];
    curIdx0 = state->CurIndx;
    if (curIdx0 < state->SegStartIdx[stpNow0]) {
      curIdx0 = state->SegStartIdx[stpNow0];
    }
    if (curIdx0 > segEnd0) {
      curIdx0 = segEnd0;
    }
    dist2Go0 = apaCtrlRefHandoffCalcRemainingOdoToEnd(input, curIdx0, segEnd0);
  }

  return dist2Go0;
}

static inline void apaCtrlRefHandoffAdvanceSegTrack(const ApaCtrlRefHandoffInput_t *input,
                                                    const ApaCtrlRefHandoffParam_t *param,
                                                    ApaCtrlRefHandoffState_t *state,
                                                    Integer *phase) {
  Real distToEnd0;
  Real distOverStp0;
  Boolean condReachSegEnd0;
  Integer stpNow0;
  Integer segEnd0;

  stpNow0 = state->StpNow;
  segEnd0 = state->SegEndIdx[stpNow0];

  {
    Real stepDx0 = input->VehicleX - state->PrevVehX;
    Real stepDy0 = input->VehicleY - state->PrevVehY;
    Real stepDist0 = sqrtf(stepDx0 * stepDx0 + stepDy0 * stepDy0);
    state->PrevVehX = input->VehicleX;
    state->PrevVehY = input->VehicleY;
    state->OdoAccum = state->OdoAccum + stepDist0;

    Integer prevIdx0 = state->CurIndx;
    while ((state->CurIndx < segEnd0) && (state->PathOdoAccum < state->OdoAccum)) {
      state->PathOdoAccum = state->PathOdoAccum + fabsf(input->PthPtOdo[state->CurIndx]);
      state->CurIndx = state->CurIndx + 1;
    }

    if (state->CurIndx > prevIdx0) {
      state->StallCounter = 0;
    } else {
      Integer stallTimeout0;
      state->StallCounter = state->StallCounter + 1;
      stallTimeout0 = param->StallTimeout;
      if (stallTimeout0 < 10) {
        stallTimeout0 = 10;
      }
      if (state->StallCounter >= stallTimeout0) {
        state->StallCounter = 0;
        stpNow0 = state->StpNow;
        if ((stpNow0 + 1) >= state->StpNum) {
          *phase = PHASE_ARRIVED;
        } else {
          state->StopCounter = 0;
          *phase = PHASE_STOP_WAIT;
        }
      }
    }
  }

  distToEnd0 = apaCtrlRefHandoffCurrentDist2Go(input, state);
  distOverStp0 = param->DistOverStp;
  if (distOverStp0 < 0.05f) {
    distOverStp0 = 0.05f;
  }
  condReachSegEnd0 = (distToEnd0 < distOverStp0);
  if (condReachSegEnd0 && (*phase == PHASE_SEG_TRACK)) {
    stpNow0 = state->StpNow;
    if ((stpNow0 + 1) >= state->StpNum) {
      *phase = PHASE_ARRIVED;
    } else {
      state->StopCounter = 0;
      *phase = PHASE_STOP_WAIT;
    }
  }

  state->CurIndx = apaClampCount(state->CurIndx, segEnd0);
}

static void apaCtrlRefHandoffAdvanceState(const ApaCtrlRefHandoffInput_t *input,
                                          const ApaCtrlRefHandoffParam_t *param,
                                          ApaCtrlRefHandoffState_t *state,
                                          Integer *phase) {
  Integer phase0 = *phase;
  Integer stpNow0;

  if (phase0 == PHASE_IDLE) {
    if (state->SegTableBuilt == 0) {
      apaCtrlRefHandoffBuildSegTable(input, param, state);
    }
    state->StpNow = 0;
    state->CurIndx = 0;
    state->StopCounter = 0;
    state->GearCounter = 0;
    phase0 = PHASE_SEG_INIT;
  }

  if (phase0 == PHASE_SEG_INIT) {
    stpNow0 = state->StpNow;
    if (stpNow0 >= state->StpNum) {
      phase0 = PHASE_ARRIVED;
    } else {
      state->CurIndx = state->SegStartIdx[stpNow0];
      state->GearCounter = 0;
      state->OdoAccum = 0.0f;
      state->PathOdoAccum = 0.0f;
      phase0 = PHASE_GEAR_WAIT;
    }
  }

  if (phase0 == PHASE_GEAR_WAIT) {
    Integer gearWait0 = param->GearWaitCycles;
    Integer segGoDir0;
    if (gearWait0 < 0) {
      gearWait0 = 0;
    }
    Integer gearCounter0 = state->GearCounter;
    segGoDir0 = apaCtrlRefHandoffCurrentGoDir(state);
    Boolean condGearMatched0 = apaGearPosMatchesGoDir(input->GearPos, segGoDir0);
    Boolean condGearDone0 = (gearCounter0 >= gearWait0) && condGearMatched0;
    if (condGearDone0) {
      state->PrevVehX = input->VehicleX;
      state->PrevVehY = input->VehicleY;
      phase0 = PHASE_SEG_TRACK;
    } else {
      if (gearCounter0 < gearWait0) {
        state->GearCounter = gearCounter0 + 1;
      }
    }
  }

  if (phase0 == PHASE_SEG_TRACK) {
    apaCtrlRefHandoffAdvanceSegTrack(input, param, state, &phase0);
  }

  if (phase0 == PHASE_STOP_WAIT) {
    Integer stopDwell0 = param->StopDwellCycles;
    if (stopDwell0 < 1) {
      stopDwell0 = 1;
    }
    Integer stopCounter0 = state->StopCounter;
    Boolean condStopDone0 = (stopCounter0 >= stopDwell0);
    if (condStopDone0) {
      phase0 = PHASE_SEG_ADVANCE;
    } else {
      state->StopCounter = stopCounter0 + 1;
    }
  }

  if (phase0 == PHASE_SEG_ADVANCE) {
    stpNow0 = state->StpNow + 1;
    state->StpNow = stpNow0;
    if (stpNow0 >= state->StpNum) {
      phase0 = PHASE_ARRIVED;
    } else {
      state->CurIndx = state->SegStartIdx[stpNow0];
      state->GearCounter = 0;
      state->OdoAccum = 0.0f;
      state->PathOdoAccum = 0.0f;
      phase0 = PHASE_GEAR_WAIT;
    }
  }

  *phase = phase0;
}

static void apaCtrlRefHandoffCalcOutputsSegTrack(const ApaCtrlRefHandoffInput_t *input,
                                                 const ApaCtrlRefHandoffParam_t *param,
                                                 ApaCtrlRefHandoffState_t *state,
                                                 Real *outRefX,
                                                 Real *outRefY,
                                                 Real *outRefYaw,
                                                 Real *outRefSpeed,
                                                 Integer *outRefIndex) {
  Integer stpNow0 = state->StpNow;
  Integer segGoDir0 = state->SegGoDir[stpNow0];
  Integer segStart0 = state->SegStartIdx[stpNow0];
  Integer segEnd0 = state->SegEndIdx[stpNow0];
  Integer reversePointIdx0;
  Boolean condUseReversePoint0;
  Boolean condReferenceAtSwitch0;
  Integer refIdx1;

  refIdx1 = state->CurIndx + APA_CTRL_REF_HANDOFF_LOOK_AHEAD;
  if (refIdx1 > segEnd0) {
    refIdx1 = segEnd0;
  }

  if ((stpNow0 + 1) < state->StpNum) {
    Integer lastSameDirRefIdx0;
    Boolean condKeepRefInSameDir0;

    lastSameDirRefIdx0 = segEnd0 - 1;
    if (lastSameDirRefIdx0 < segStart0) {
      lastSameDirRefIdx0 = segStart0;
    }
    condKeepRefInSameDir0 = (state->CurIndx < lastSameDirRefIdx0) && (refIdx1 > lastSameDirRefIdx0);
    if (condKeepRefInSameDir0) {
      refIdx1 = lastSameDirRefIdx0;
    }
  }

  condUseReversePoint0 = apaCtrlRefHandoffShouldUseGeneratedReversePoint(param, state, stpNow0);
  if (condUseReversePoint0) {
    reversePointIdx0 = apaCtrlRefHandoffResolveReversePointIndex(input, param, segStart0, segEnd0);
    if ((state->CurIndx < reversePointIdx0) && (refIdx1 < reversePointIdx0)) {
      refIdx1 = reversePointIdx0;
    }
  }

  condReferenceAtSwitch0 = ((stpNow0 + 1) < state->StpNum) && (refIdx1 >= segEnd0);

  *outRefX = input->PthPtX[refIdx1];
  *outRefY = input->PthPtY[refIdx1];
  *outRefYaw = input->PthPtTht[refIdx1];
  *outRefIndex = refIdx1;

  {
    Real odoDistToEnd0;
    Real decelDist0;
    Boolean condNeedDecel0;
    Integer stepsRemain1;
    Real stepSize1;

    stepsRemain1 = segEnd0 - state->CurIndx;
    stepSize1 = fabsf(input->PthPtOdo[state->CurIndx]);
    if (stepSize1 < APA_CTRL_REF_HANDOFF_ODO_EPSILON) {
      stepSize1 = APA_CTRL_REF_HANDOFF_ODO_EPSILON;
    }
    odoDistToEnd0 = ((Real)stepsRemain1) * stepSize1;
    decelDist0 = param->DistOverStp * 3.0f;
    condNeedDecel0 = (odoDistToEnd0 < decelDist0);

    if (segGoDir0 > 0) {
      *outRefSpeed = param->ForwardRefSpeed;
    } else {
      *outRefSpeed = param->ReverseRefSpeed;
    }

    if (condNeedDecel0) {
      Real decelRatio0 = odoDistToEnd0 / decelDist0;
      if (decelRatio0 < 0.2f) {
        decelRatio0 = 0.2f;
      }
      *outRefSpeed = *outRefSpeed * decelRatio0;
    }
    if (condReferenceAtSwitch0) {
      *outRefSpeed = param->StopRefSpeed;
    }
  }
}

static void apaCtrlRefHandoffCalcOutputs(const ApaCtrlRefHandoffInput_t *input,
                                         const ApaCtrlRefHandoffParam_t *param,
                                         ApaCtrlRefHandoffState_t *state,
                                         Integer phase,
                                         Integer maxIndex,
                                         Real *outRefX,
                                         Real *outRefY,
                                         Real *outRefYaw,
                                         Real *outRefSpeed,
                                         Integer *outRefIndex) {
  Integer stpNow0 = state->StpNow;
  Integer segStart0;
  Integer segEnd0;

  if (phase == PHASE_ARRIVED) {
    *outRefX = input->PthPtX[maxIndex];
    *outRefY = input->PthPtY[maxIndex];
    *outRefYaw = input->PthPtTht[maxIndex];
    *outRefIndex = maxIndex;

    Real goalDistSq0 = apaMathDistanceSquared2D(
        input->VehicleX, input->VehicleY, input->PthPtX[maxIndex], input->PthPtY[maxIndex]);
    Real goalYawDiff0 = input->VehicleYaw - input->PthPtTht[maxIndex];
    Real goalYawWrapped0 = apaCoordWrapToHalfOpenPi(goalYawDiff0);
    Real goalYawErr0 = fabsf(goalYawWrapped0);
    Boolean condGoalDistReached0 = (goalDistSq0 <= param->PointReachedDistSq);
    Boolean condGoalYawReached0 = (goalYawErr0 <= APA_CTRL_REF_HANDOFF_GOAL_YAW_TOL);
    Boolean condGoalReached0 = condGoalDistReached0 && condGoalYawReached0;
    if (condGoalReached0) {
      *outRefSpeed = param->StopRefSpeed;
    } else {
      Real refYaw0;
      Real dxVeh0;
      Real dyVeh0;
      Real cosRef0;
      Real sinRef0;
      Real longErrRaw0;
      Real longErr0;
      Real speedScale0;
      Integer arrivedDir0;

      refYaw0 = input->PthPtTht[maxIndex];
      dxVeh0 = input->PthPtX[maxIndex] - input->VehicleX;
      dyVeh0 = input->PthPtY[maxIndex] - input->VehicleY;
      cosRef0 = cosf(refYaw0);
      sinRef0 = sinf(refYaw0);
      longErrRaw0 = cosRef0 * dxVeh0 + sinRef0 * dyVeh0;
      if (condGoalDistReached0) {
        longErr0 = 0.0f;
      } else {
        longErr0 = longErrRaw0;
      }
      arrivedDir0 = apaCtrlRefHandoffResolveArrivedDir(input, state, maxIndex, longErr0);
      speedScale0 = apaCtrlRefHandoffCalcArrivedSpeedScale(param, goalDistSq0, goalYawErr0);

      if (arrivedDir0 > 0) {
        *outRefSpeed = param->ForwardRefSpeed * speedScale0;
      } else {
        *outRefSpeed = param->ReverseRefSpeed * speedScale0;
      }
    }
  } else if (phase == PHASE_STOP_WAIT) {
    segEnd0 = state->SegEndIdx[stpNow0];
    *outRefX = input->PthPtX[segEnd0];
    *outRefY = input->PthPtY[segEnd0];
    *outRefYaw = input->PthPtTht[segEnd0];
    *outRefSpeed = param->StopRefSpeed;
    *outRefIndex = segEnd0;
  } else if (phase == PHASE_GEAR_WAIT) {
    segStart0 = state->SegStartIdx[stpNow0];
    *outRefX = input->PthPtX[segStart0];
    *outRefY = input->PthPtY[segStart0];
    *outRefYaw = input->PthPtTht[segStart0];
    *outRefSpeed = param->StopRefSpeed;
    *outRefIndex = segStart0;
  } else if (phase == PHASE_SEG_TRACK) {
    apaCtrlRefHandoffCalcOutputsSegTrack(
        input, param, state, outRefX, outRefY, outRefYaw, outRefSpeed, outRefIndex);
  } else {
    *outRefX = input->RawRefX;
    *outRefY = input->RawRefY;
    *outRefYaw = input->RawRefYaw;
    *outRefSpeed = param->StopRefSpeed;
    *outRefIndex = 0;
  }
}

static void apaCtrlRefHandoffComputeOutputsFinal(const ApaCtrlRefHandoffInput_t *input,
                                                 const ApaCtrlRefHandoffParam_t *param,
                                                 ApaCtrlRefHandoffState_t *state,
                                                 ApaCtrlRefHandoffOutput_t *output,
                                                 Boolean condHasPath0,
                                                 Integer outputRefIndex0) {
  {
    Real refCurvature0;
    Real longitudinalError0;
    Real lateralError0;
    Real headingError0;
    Integer curSegStart0;
    Integer curSegEnd0;
    Integer stpNow2;
    Boolean condSegValid1;

    refCurvature0 = 0.0f;
    longitudinalError0 = 0.0f;
    lateralError0 = 0.0f;
    headingError0 = 0.0f;

    stpNow2 = state->StpNow;
    condSegValid1 = (state->SegTableBuilt == 1) && (stpNow2 >= 0) && (stpNow2 < state->StpNum);
    if (condSegValid1) {
      curSegStart0 = state->SegStartIdx[stpNow2];
      curSegEnd0 = state->SegEndIdx[stpNow2];
    } else {
      curSegStart0 = 0;
      if (input->PtCount > 0) {
        curSegEnd0 = input->PtCount - 1;
      } else {
        curSegEnd0 = 0;
      }
    }

    if ((condHasPath0) && (outputRefIndex0 >= 0) && (outputRefIndex0 < input->PtCount)) {
      apaCtrlRefHandoffCalcCurvatureAndErrors(input,
                                              outputRefIndex0,
                                              curSegStart0,
                                              curSegEnd0,
                                              &refCurvature0,
                                              &longitudinalError0,
                                              &lateralError0,
                                              &headingError0);
    }

    output->RefCurvature = refCurvature0;
    output->LongitudinalError = longitudinalError0;
    output->LateralError = lateralError0;
    output->HeadingError = headingError0;
  }

  {
    Integer goDir0;
    Integer stpNow0;
    Integer apaStpNow0;

    goDir0 = apaCtrlRefHandoffCurrentGoDir(state);
    stpNow0 = state->StpNow;
    apaStpNow0 = 0;
    if ((state->SegTableBuilt == 1) && (stpNow0 >= 0) && (stpNow0 < state->StpNum)) {
      apaStpNow0 = stpNow0 + 1;
    }

    output->CtrlPhase = state->Phase;
    output->ApaStpNum = state->StpNum;
    output->ApaStpNow = apaStpNow0;
    output->ApaGoDir = goDir0;
    output->Dist2Go = apaCtrlRefHandoffCurrentDist2Go(input, state);
    if (goDir0 == 0) {
      output->TargetGearReq = 0;
    } else {
      output->TargetGearReq = apaGearPosForGoDir(goDir0);
    }
    {
      Integer pathIndex0;
      Real odo0;

      pathIndex0 = state->CurIndx;
      odo0 = 0.0f;
      if ((pathIndex0 >= 0) && (pathIndex0 < input->PtCount)) {
        odo0 = input->PthPtOdo[pathIndex0];
      }
      apaDebugTraceGear(apaDebugTraceCurrentFrame(),
                        "HANDOFF_FINAL_GEAR",
                        "HANDOFF",
                        pathIndex0,
                        odo0,
                        goDir0,
                        output->TargetGearReq);
      (void)fprintf(stderr,
                    "[APA_DIAG] frame_id=%lu stage=HANDOFF_GEAR_COMPARE "
                    "InputTargetGearReq=NA ActualGear=%ld "
                    "OutputTargetGearReq=%ld gear_changed=NA "
                    "reason=no_target_gear_field_in_handoff_input\n",
                    (unsigned long)apaDebugTraceCurrentFrame(),
                    (long)input->GearPos,
                    (long)output->TargetGearReq);
    }
  }

  {
    Integer maxIndex1;
    Integer segSearchStart0;
    Integer segSearchEnd0;
    Integer stpNow1;
    Boolean condSegValid0;

    if (condHasPath0) {
      maxIndex1 = input->PtCount - 1;
    } else {
      maxIndex1 = 0;
    }
    stpNow1 = state->StpNow;
    condSegValid0 = (state->SegTableBuilt == 1) && (stpNow1 >= 0) && (stpNow1 < state->StpNum);
    if (condSegValid0) {
      segSearchStart0 = state->SegStartIdx[stpNow1];
      segSearchEnd0 = state->SegEndIdx[stpNow1];
      if (apaCtrlRefHandoffShouldUseGeneratedReversePoint(param, state, stpNow1)) {
        Integer reversePointIdx0;

        reversePointIdx0 =
            apaCtrlRefHandoffResolveReversePointIndex(input, param, segSearchStart0, segSearchEnd0);
        if (state->CurIndx < reversePointIdx0) {
          segSearchEnd0 = reversePointIdx0;
        }
      }
    } else {

      segSearchStart0 = 0;
      segSearchEnd0 = maxIndex1;
    }
    output->SegSearchStart = segSearchStart0;
    output->SegSearchEnd = segSearchEnd0;
  }
}

/**
 * @brief 按段推进参考交接模块
 * @en_name apaCtrlRefHandoff
 * @cn_name 按段推进参考交接
 * @type widget_module
 * @config cn_name=前进参考速度, en_name=ForwardRefSpeed,
 * desc=前进段参考速度, unit=m/s, type=Real, range=[0,5], default=1.2
 * @config cn_name=倒车参考速度, en_name=ReverseRefSpeed,
 * desc=倒车段参考速度, unit=m/s, type=Real, range=[-5,0], default=-0.8
 * @config cn_name=停止参考速度, en_name=StopRefSpeed,
 * desc=停止参考速度, unit=m/s, type=Real, range=[-1,1], default=0.0
 * @config cn_name=段切换距离阈值, en_name=DistOverStp,
 * desc=车辆距段末点小于该距离时触发段切换(对应MATLAB Dist_OverStp), unit=m,
 * type=Real, range=[0.05,2.0], default=0.3
 * @config cn_name=目标到达距离平方, en_name=PointReachedDistSq,
 * desc=认为已到达目标的距离平方阈值, unit=m2,
 * type=Real, range=[0,4], default=0.25
 * @config cn_name=停车过渡周期, en_name=StopDwellCycles,
 * desc=换向段间零速度停车持续的周期数, unit=1,
 * type=Integer, range=[1,20], default=5
 * @config cn_name=档位等待周期, en_name=GearWaitCycles,
 * desc=模拟档位切换等待的周期数(对应MATLAB GearCheck), unit=1,
 * type=Integer, range=[0,10], default=3
 * @config cn_name=阻塞超时周期, en_name=StallTimeout,
 * desc=段内索引停滞超过该周期数则强制段切换(对应MATLAB MocoBlock), unit=1,
 * type=Integer, range=[10,200], default=100
 * @config cn_name=起步方向偏好, en_name=ExpectStartDir,
 * desc=期望起步方向(0自动推断 1前进 -1倒车), unit=1,
 * type=Integer, range=[-1,1], default=0
 * @config cn_name=起步方向阈值, en_name=ExpectDirOdoThresh,
 * desc=起步方向偏好生效的最小段长阈值, unit=m,
 * type=Real, range=[0.0,2.0], default=0.15
 * @config cn_name=启用倒车点生成, en_name=EnableGenRevPt,
 * desc=是否使能倒车点生成(0否 1是), unit=1,
 * type=Integer, range=[0,1], default=0
 * @config cn_name=倒车点距离, en_name=ReverseDist,
 * desc=倒车点距离, unit=m,
 * type=Real, range=[0.0,3.0], default=1.0
 * @param[IN] Real VehicleX 车辆当前位置X坐标
 * @param[IN] Real VehicleY 车辆当前位置Y坐标
 * @param[IN] Real VehicleYaw 车辆当前航向角
 * @param[IN] Integer GearPos 实际档位(0未接入 1D 2R)
 * @param[IN] Integer PathUpdateIdx 路径刷新序号
 * @param[IN] Integer PtCount 有效离散点数量
 * @param[IN] Real[100] PthPtX 路径点X数组
 * @param[IN] Real[100] PthPtY 路径点Y数组
 * @param[IN] Real[100] PthPtTht 路径点航向数组
 * @param[IN] Real[100] PthPtOdo 路径点里程数组
 * @param[IN] Real RawRefX 选择器原始参考点X
 * @param[IN] Real RawRefY 选择器原始参考点Y
 * @param[IN] Real RawRefYaw 选择器原始参考点航向
 * @param[IN] Real RawRefSpeed 选择器原始参考速度
 * @param[IN] Integer RawRefIndex 选择器原始参考点索引
 * @param[OUT] Real RefX 输出参考点X
 * @param[OUT] Real RefY 输出参考点Y
 * @param[OUT] Real RefYaw 输出参考点航向
 * @param[OUT] Real RefSpeed 输出参考速度
 * @param[OUT] Integer RefIndex 输出参考点索引
 * @param[OUT] Real RefCurvature 参考点路径曲率
 * @param[OUT] Real LongitudinalError 纵向偏差
 * @param[OUT] Real LateralError 横向偏差
 * @param[OUT] Real HeadingError 航向偏差
 * @param[OUT] Integer CtrlPhase APA_Ctrl_Chart阶段
 * @param[OUT] Integer ApaStpNum 路径总段数
 * @param[OUT] Integer ApaStpNow 当前段号1-based
 * @param[OUT] Integer ApaGoDir 当前段行驶方向
 * @param[OUT] Real Dist2Go 当前段剩余路径距离
 * @param[OUT] Integer TargetGearReq 目标档位请求(1D 2R)
 * @param[OUT] Integer SegSearchStart 当前段搜索起始索引
 * @param[OUT] Integer SegSearchEnd 当前段搜索结束索引
 * @var Integer Phase 状态机阶段
 * @var Integer StpNum 总段数
 * @var Integer StpNow 当前段号
 * @var Integer CurIndx 段内索引
 * @var Integer StopCounter 停车计数
 * @var Integer GearCounter 档位等待计数
 * @var Integer SegTableBuilt 段表已构建标志
 * @var Integer LastPtCount 上一周期路径点数(路径刷新检测)
 * @var Integer LastPathUpdateIdx 上一周期路径刷新序号
 * @var Integer StallCounter 段内索引停滞计数器
 * @var Real OdoAccum 段内车辆行驶里程累积(对应MATLAB LocationTrack)
 * @var Real PathOdoAccum 段内路径odo累积到CurIndx
 * @var Real PrevVehX 上一步车辆X
 * @var Real PrevVehY 上一步车辆Y
 * @granularity atomic
 * @tag_level1 泊车控制
 * @tag_level2 参考交接
 * @source slx_stateflow APA_Ctrl_Chart (Moco3/Stateflow)
 * @version 2.0
 * @date 2026-03-14
 * @author aide
 */

void apaCtrlRefHandoff(const ApaCtrlRefHandoffInput_t *input,
                       const ApaCtrlRefHandoffParam_t *param,
                       ApaCtrlRefHandoffState_t *state,
                       ApaCtrlRefHandoffOutput_t *output) {
  Integer ptCount0;
  Boolean condHasPath0;
  Integer phase0;
  Real outputRefX0;
  Real outputRefY0;
  Real outputRefYaw0;
  Real outputRefSpeed0;
  Integer outputRefIndex0;

  ptCount0 = input->PtCount;
  condHasPath0 = apaCtrlRefHandoffPathValid(input);

  if (condHasPath0) {
    Integer maxIndex0;

    maxIndex0 = ptCount0 - 1;
    phase0 = state->Phase;

    if (phase0 < PHASE_IDLE) {
      phase0 = PHASE_IDLE;
    }
    if (phase0 > PHASE_ARRIVED) {
      phase0 = PHASE_IDLE;
    }

    {
      Integer lastPtCount0;
      Integer lastPathUpdateIdx0;
      Boolean condPathChanged0;
      Boolean condPathUpdateChanged0;

      lastPtCount0 = state->LastPtCount;
      lastPathUpdateIdx0 = state->LastPathUpdateIdx;
      condPathChanged0 = (ptCount0 != lastPtCount0) && (lastPtCount0 > 0);
      condPathUpdateChanged0 = (input->PathUpdateIdx != lastPathUpdateIdx0) &&
                               (input->PathUpdateIdx != 0) && (lastPtCount0 > 0);
      if (condPathChanged0 || condPathUpdateChanged0) {
        state->SegTableBuilt = 0;
        state->StpNow = 0;
        state->CurIndx = 0;
        state->StopCounter = 0;
        state->GearCounter = 0;
        state->StallCounter = 0;
        state->OdoAccum = 0.0f;
        state->PathOdoAccum = 0.0f;
        phase0 = PHASE_IDLE;
      }
      state->LastPtCount = ptCount0;
      state->LastPathUpdateIdx = input->PathUpdateIdx;
    }

    apaCtrlRefHandoffAdvanceState(input, param, state, &phase0);

    state->Phase = phase0;

    apaCtrlRefHandoffCalcOutputs(input,
                                 param,
                                 state,
                                 phase0,
                                 maxIndex0,
                                 &outputRefX0,
                                 &outputRefY0,
                                 &outputRefYaw0,
                                 &outputRefSpeed0,
                                 &outputRefIndex0);
  } else {

    state->Phase = PHASE_IDLE;
    state->StpNum = 0;
    state->StpNow = 0;
    state->CurIndx = 0;
    state->StopCounter = 0;
    state->GearCounter = 0;
    state->SegTableBuilt = 0;
    state->LastPtCount = 0;
    state->LastPathUpdateIdx = 0;
    state->StallCounter = 0;
    state->OdoAccum = 0.0f;
    state->PathOdoAccum = 0.0f;
    state->PrevVehX = 0.0f;
    state->PrevVehY = 0.0f;
    outputRefX0 = input->RawRefX;
    outputRefY0 = input->RawRefY;
    outputRefYaw0 = input->RawRefYaw;
    outputRefSpeed0 = param->StopRefSpeed;
    outputRefIndex0 = 0;
  }

  output->RefX = outputRefX0;
  output->RefY = outputRefY0;
  output->RefYaw = outputRefYaw0;
  output->RefSpeed = outputRefSpeed0;
  output->RefIndex = outputRefIndex0;

  apaCtrlRefHandoffComputeOutputsFinal(input, param, state, output, condHasPath0, outputRefIndex0);
}
