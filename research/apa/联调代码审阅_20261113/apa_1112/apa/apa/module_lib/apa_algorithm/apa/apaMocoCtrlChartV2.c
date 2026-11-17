#include "apaCoordUtils.h"
#include "apaEnums.h"
#include "apaMathUtils.h"
#include "apaMocoCtrlChartV2Types.h"
#include "apaPathInterval.h"
#include <math.h>
#include <string.h>

#ifndef APA_MOCO_V2_ODO_EPSILON
#define APA_MOCO_V2_ODO_EPSILON 1.0e-4f
#endif

static Integer apaMocoCtrlChartV2SignFromOdo(Real odo) {
  Integer sign0;
  Real absOdo0;

  absOdo0 = fabsf(odo);
  if (absOdo0 <= APA_MOCO_V2_ODO_EPSILON) {
    sign0 = 0;
  } else if (odo > 0.0f) {
    sign0 = 1;
  } else {
    sign0 = -1;
  }

  return sign0;
}

static Boolean apaMocoCtrlChartV2PathValid(const ApaMocoCtrlChartV2Input_t *input) {
  Integer index0;
  Integer nextIndex0;
  Real totalLen0;
  Real stepLen0;
  Boolean condCount0;
  Boolean condFinite0;
  Boolean condLength0;

  condCount0 = (input->PtCount > 1) && (input->PtCount <= PATH_PT_MAX);
  condFinite0 = false;
  condLength0 = false;
  if (condCount0) {
    condFinite0 = true;
    totalLen0 = 0.0f;
    index0 = 0;
    while ((index0 < input->PtCount) && condFinite0) {
      condFinite0 = apaMathIsFinitePathValue(input->PthPtX[index0]) &&
                    apaMathIsFinitePathValue(input->PthPtY[index0]) &&
                    apaMathIsFinitePathValue(input->PthPtTht[index0]) &&
                    apaMathIsFinitePathValue(input->PthPtOdo[index0]);
      if (index0 < (input->PtCount - 1)) {
        nextIndex0 = index0 + 1;
        stepLen0 = apaMathDistance2D(input->PthPtX[index0],
                                     input->PthPtY[index0],
                                     input->PthPtX[nextIndex0],
                                     input->PthPtY[nextIndex0]);
        totalLen0 = totalLen0 + stepLen0;
        condFinite0 = condFinite0 && apaPathIntervalIsConsistent(input->PthPtX[index0],
                                                                 input->PthPtY[index0],
                                                                 input->PthPtTht[index0],
                                                                 input->PthPtOdo[index0],
                                                                 input->PthPtX[nextIndex0],
                                                                 input->PthPtY[nextIndex0],
                                                                 input->PthPtTht[nextIndex0]);
      }
      index0 = index0 + 1;
    }
    condLength0 = condFinite0 && (totalLen0 > APA_MOCO_V2_ODO_EPSILON);
  }

  return condCount0 && condFinite0 && condLength0;
}

static void apaMocoCtrlChartV2ClearLocal(ApaMocoCtrlChartV2Output_t *output) {
  Integer idx0;

  idx0 = 0;
  while (idx0 < PATH_PT_MAX) {
    output->LocalPathX[idx0] = 0.0f;
    output->LocalPathY[idx0] = 0.0f;
    output->LocalPathYaw[idx0] = 0.0f;
    output->LocalPathOdo[idx0] = 0.0f;
    idx0 = idx0 + 1;
  }
}

static void apaMocoCtrlChartV2ResetState(ApaMocoCtrlChartV2State_t *state) {
  Integer idx0;

  memset(state, 0, sizeof(*state));
  state->Phase = APA_MOCO_NOACT;
  state->StpNum = 0;
  state->StpNow = 0;
  state->StopCounter = 0;
  state->GearCounter = 0;
  state->LastPtCount = 0;
  state->LastPathUpdateIdx = 0;
  idx0 = 0;
  while (idx0 < APA_MOCO_V2_MAX_SEG) {
    state->SegStartIdx[idx0] = 0;
    state->SegEndIdx[idx0] = 0;
    state->SegGoDir[idx0] = 1;
    idx0 = idx0 + 1;
  }
}

static void apaMocoCtrlChartV2BuildSegTable(const ApaMocoCtrlChartV2Input_t *input,
                                            ApaMocoCtrlChartV2State_t *state) {
  Integer maxIndex0;
  Integer scanIdx0;
  Integer segCount0;
  Integer lastSign0;
  Integer currentSign0;
  Boolean condHasLastSign0;
  Boolean condFlip0;
  Boolean condCanAppend0;

  apaMocoCtrlChartV2ResetState(state);
  maxIndex0 = input->PtCount - 1;
  state->SegStartIdx[0] = 0;
  state->SegGoDir[0] = 1;
  segCount0 = 0;
  lastSign0 = 1;
  condHasLastSign0 = false;

  scanIdx0 = 0;
  while (scanIdx0 < maxIndex0) {
    currentSign0 = apaMocoCtrlChartV2SignFromOdo(input->PthPtOdo[scanIdx0]);
    if (currentSign0 != 0) {
      if (condHasLastSign0) {
        condFlip0 = (currentSign0 != lastSign0);
        if (condFlip0) {
          state->SegEndIdx[segCount0] = scanIdx0;
          condCanAppend0 = (segCount0 < (APA_MOCO_V2_MAX_SEG - 1));
          if (condCanAppend0) {
            segCount0 = segCount0 + 1;
            state->SegStartIdx[segCount0] = scanIdx0;
            state->SegGoDir[segCount0] = currentSign0;
          } else {
            state->StpNum = 0;
            state->Phase = APA_MOCO_FAULT;
            state->StopReason = APA_MOCO_STOP_INVALID;
            return;
          }
        }
      } else {
        state->SegGoDir[0] = currentSign0;
        condHasLastSign0 = true;
      }
      lastSign0 = currentSign0;
    }
    scanIdx0 = scanIdx0 + 1;
  }

  state->SegEndIdx[segCount0] = maxIndex0;
  state->StpNum = segCount0 + 1;
  state->Phase = APA_MOCO_INITIAL;
}

static void apaMocoCtrlChartV2CopyLocalPath(const ApaMocoCtrlChartV2Input_t *input,
                                            const ApaMocoCtrlChartV2State_t *state,
                                            ApaMocoCtrlChartV2Output_t *output) {
  apaMocoCtrlChartV2ClearLocal(output);
  output->LocalPtCount = 0;
  output->PathOverflowFlag = 0;
  if (state->StpNum <= 0)
    return;
  const Integer segment = state->StpNow;
  const Integer start = state->SegStartIdx[segment];
  const Integer end = state->SegEndIdx[segment];
  const Integer count = end - start + 1;
  output->SegmentStartIdx = start;
  output->SegmentEndIdx = end;
  if (start < 0 || end >= input->PtCount || count < 2 || count > PATH_PT_MAX) {
    output->PathOverflowFlag = 1;
    return;
  }

  for (Integer i = 0; i < count; ++i) {
    output->LocalPathX[i] = input->PthPtX[start + i];
    output->LocalPathY[i] = input->PthPtY[start + i];
    output->LocalPathYaw[i] = input->PthPtTht[start + i];
    output->LocalPathOdo[i] = i + 1 < count ? input->PthPtOdo[start + i] : 0.0f;
  }
  output->LocalPtCount = count;
}

static void apaMocoCtrlChartV2Fault(ApaMocoCtrlChartV2State_t *state, Integer reason) {
  state->Phase = APA_MOCO_FAULT;
  state->StopReason = reason;
  state->StopCounter = 0;
}

static void apaMocoCtrlChartV2Project(const ApaMocoCtrlChartV2Input_t *input,
                                      const ApaMocoCtrlChartV2Param_t *param,
                                      ApaMocoCtrlChartV2State_t *state,
                                      ApaMocoCtrlChartV2Output_t *output) {
  Integer end = output->LocalPtCount - 1;
  apaMocoProjectPath(output->LocalPathX,
                     output->LocalPathY,
                     output->LocalPathYaw,
                     output->LocalPtCount,
                     input->VehicleX,
                     input->VehicleY,
                     input->VehicleYaw,
                     state->ProjectionIndex,
                     state->Progress,
                     param->SearchBackSteps,
                     param->SearchAheadSteps,
                     param->MaxProgressAdvance,
                     param->MaxReferenceDistance,
                     param->MaxHeadingError,
                     &output->Projection);
  if (output->Projection.Valid) {
    state->ProjectionIndex = output->Projection.Index;
    state->Progress = fmaxf(state->Progress, output->Projection.Progress);
  }
  output->Dist2Go = output->Projection.Remaining;
  output->GoalDistance =
      hypotf(output->LocalPathX[end] - input->VehicleX, output->LocalPathY[end] - input->VehicleY);
  output->GoalYawError =
      fabsf(apaCoordWrapToHalfOpenPi(output->LocalPathYaw[end] - input->VehicleYaw));
  if (state->StpNow == state->StpNum - 1) {
    output->GoalDistance = hypotf(input->GoalX - input->VehicleX, input->GoalY - input->VehicleY);
    output->GoalYawError = fabsf(apaCoordWrapToHalfOpenPi(input->GoalYaw - input->VehicleYaw));
  }
}

static void apaMocoCtrlChartV2AdvanceState(const ApaMocoCtrlChartV2Input_t *input,
                                           const ApaMocoCtrlChartV2Param_t *param,
                                           ApaMocoCtrlChartV2State_t *state,
                                           ApaMocoCtrlChartV2Output_t *output) {
  Boolean last = state->StpNow == state->StpNum - 1;
  Boolean stopped = fabsf(input->VehicleSpeed) <= param->StandstillSpeed;
  Real posTolerance = last ? param->GoalPositionTolerance : param->CuspPositionTolerance;
  Real yawTolerance = last ? param->GoalYawTolerance : param->CuspYawTolerance;
  Boolean poseReached = output->GoalDistance <= posTolerance &&
                        output->GoalYawError <= yawTolerance && (!last || !input->GoalBlocked);
  const Boolean goalBrakingRoom = last && output->Dist2Go <= 0.04f &&
                                  input->BrakeDistance <= posTolerance - output->GoalDistance;
  Boolean brakeAtPose =
      poseReached &&
      (!last || stopped || output->GoalDistance <= 0.8f * posTolerance || goalBrakingRoom);
  Real approachTrigger = input->ApproachDistance;

  Real creepDistance =
      last ? input->CreepDistance
           : fminf(input->CreepDistance, fmaxf(0.08f, 0.25f * output->Projection.Length));
  if (state->Phase == APA_MOCO_FAULT || state->Phase == APA_MOCO_DONE)
    return;
  if (!output->Projection.Valid) {
    apaMocoCtrlChartV2Fault(state, APA_MOCO_STOP_REFERENCE);
    return;
  }
  if (state->Phase == APA_MOCO_INITIAL)
    state->Phase = APA_MOCO_GEAR_CHECK;
  if (state->Phase == APA_MOCO_GEAR_CHECK) {
    ++state->GearTimeoutCounter;
    if (stopped && apaGearPosMatchesGoDir(input->GearPos, state->SegGoDir[state->StpNow])) {
      ++state->GearCounter;
      if (state->GearCounter > param->GearWaitCycles && !input->SteeringPending) {
        state->Phase = APA_MOCO_TRACK;
        state->GearCounter = 0;
      }
    } else
      state->GearCounter = 0;
    if (state->GearTimeoutCounter > param->GearTimeoutCycles)
      apaMocoCtrlChartV2Fault(state, APA_MOCO_STOP_GEAR_TIMEOUT);
  }
  if (apaMocoMotionPhase(state->Phase)) {
    if (!apaGearPosMatchesGoDir(input->GearPos, state->SegGoDir[state->StpNow])) {
      state->Phase = APA_MOCO_GEAR_CHECK;
      state->GearCounter = state->GearTimeoutCounter = 0;
      return;
    }
    ++state->SegmentCounter;
    if (state->Progress > state->ProgressAnchor + 0.01f) {
      state->ProgressAnchor = state->Progress;
      state->StallCounter = 0;
    } else
      ++state->StallCounter;
    if (state->StallCounter > param->StallTimeoutCycles ||
        state->SegmentCounter > param->SegmentTimeoutCycles) {
      apaMocoCtrlChartV2Fault(state, APA_MOCO_STOP_STALLED);
      return;
    }
    if ((brakeAtPose && (last || output->Dist2Go <= 0.02f)) ||
        (stopped && output->Dist2Go <= 0.02f) || output->Dist2Go <= 0.005f) {
      state->Phase = APA_MOCO_STOP_WAIT;
      state->StopReason = last ? APA_MOCO_STOP_GOAL : APA_MOCO_STOP_CUSP;
      state->StopCounter = 0;
    } else if (output->Dist2Go < creepDistance) {
      state->Phase = APA_MOCO_CREEP;
    } else if (output->Dist2Go <= approachTrigger ||
               (state->Phase == APA_MOCO_DECEL && output->Dist2Go <= 2.0f * creepDistance)) {

      state->Phase = APA_MOCO_DECEL;
    } else
      state->Phase = APA_MOCO_TRACK;
  }
  if (state->Phase == APA_MOCO_STOP_WAIT) {
    if (!stopped) {
      state->StopCounter = 0;
    } else {
      ++state->StopCounter;

      if (state->StopCounter > (last ? param->GoalDwellCycles : param->StopDwellCycles)) {
        if (!poseReached) {
          apaMocoCtrlChartV2Fault(state,
                                  last && input->GoalBlocked ? APA_MOCO_STOP_GOAL_BOUNDARY
                                                             : APA_MOCO_STOP_ENDPOINT);
        } else if (last) {
          state->Phase = APA_MOCO_DONE;
          output->DoneReady = 1;
        } else {
          ++state->StpNow;
          state->Phase = APA_MOCO_INITIAL;
          state->ProjectionIndex = 0;
          state->Progress = state->ProgressAnchor = 0.0f;
          state->StallCounter = state->SegmentCounter = 0;
          state->StopCounter = state->GearCounter = state->GearTimeoutCounter = 0;
          state->StopReason = APA_MOCO_STOP_NONE;
          apaMocoCtrlChartV2CopyLocalPath(input, state, output);
          apaMocoCtrlChartV2Project(input, param, state, output);
        }
      }
    }
  }
}

void apaMocoCtrlChartV2(const ApaMocoCtrlChartV2Input_t *input,
                        const ApaMocoCtrlChartV2Param_t *param,
                        ApaMocoCtrlChartV2State_t *state,
                        ApaMocoCtrlChartV2Output_t *output) {
  Boolean refresh;
  *output = (ApaMocoCtrlChartV2Output_t){0};
  if (!apaMocoCtrlChartV2PathValid(input) || !isfinite(input->VehicleSpeed) ||
      !isfinite(input->VehicleX) || !isfinite(input->VehicleY) || !isfinite(input->VehicleYaw) ||
      !isfinite(input->BrakeDistance) || !isfinite(input->ApproachDistance) ||
      !isfinite(input->CreepDistance) || input->CreepDistance < 0.0f || !isfinite(input->GoalX) ||
      !isfinite(input->GoalY) || !isfinite(input->GoalYaw)) {

    apaMocoCtrlChartV2Fault(state, APA_MOCO_STOP_INVALID);
    output->CtrlPhase = state->Phase;
    output->StopReason = state->StopReason;
    return;
  }
  refresh = input->PtCount != state->LastPtCount ||
            (input->PathUpdateIdx != 0 && input->PathUpdateIdx != state->LastPathUpdateIdx);
  if (refresh) {
    apaMocoCtrlChartV2BuildSegTable(input, state);
    state->LastPtCount = input->PtCount;
    state->LastPathUpdateIdx = input->PathUpdateIdx;
    output->PathRefreshAccepted = 1;
  }
  if (state->StpNum <= 0 || state->StpNow < 0 || state->StpNow >= state->StpNum) {
    apaMocoCtrlChartV2Fault(state, APA_MOCO_STOP_INVALID);
    output->CtrlPhase = state->Phase;
    output->StopReason = state->StopReason;
    return;
  }
  apaMocoCtrlChartV2CopyLocalPath(input, state, output);
  apaMocoCtrlChartV2Project(input, param, state, output);
  apaMocoCtrlChartV2AdvanceState(input, param, state, output);
  output->CtrlPhase = state->Phase;
  output->ApaStpNum = state->StpNum;
  output->ApaStpNow = state->StpNow + 1;
  output->ApaGoDir = state->SegGoDir[state->StpNow];
  output->TargetGearReq = apaGearPosForGoDir(output->ApaGoDir);
  output->TerminalMode = state->StpNow == state->StpNum - 1 && output->Dist2Go < 1.5f;
  output->ParkingCate = input->TargetSlotKind;
  output->StopReason = state->StopReason;
}
