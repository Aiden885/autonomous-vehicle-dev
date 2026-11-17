#include "apaDebugTrace.h"

#include <math.h>
#include <stdio.h>

#ifndef APA_PARKOUT_TRACE_LEAVE_DISTANCE
#define APA_PARKOUT_TRACE_LEAVE_DISTANCE 0.05f
#endif

static UInteger ApaDebugTraceFrameId = 0;
static UInteger ApaDebugTraceFirstPFrameId = 0;
static Boolean ApaDebugTraceFirstPSeen = false;

UInteger apaDebugTraceBeginFrame(void) {
  ApaDebugTraceFrameId = ApaDebugTraceFrameId + 1;
  ApaDebugTraceFirstPFrameId = ApaDebugTraceFrameId;
  ApaDebugTraceFirstPSeen = false;
  return ApaDebugTraceFrameId;
}

UInteger apaDebugTraceCurrentFrame(void) {
  return ApaDebugTraceFrameId;
}

void apaDebugTracePlanningInput(UInteger frameId,
                                const ApaParkingPathPlanningInput_t *input,
                                const ApaParkingPathPlanningParam_t *param) {
  (void)fprintf(stderr,
                "[APA_DIAG] frame_id=%lu stage=APA_INPUT "
                "ParkingAction=%ld TargetSlotKind=%ld "
                "VehicleX=%.9g VehicleY=%.9g VehicleYaw=%.9g "
                "TarX=%.9g TarY=%.9g TarTheta=%.9g "
                "EntrPtX=%.9g EntrPtY=%.9g "
                "SltPt1X=%.9g SltPt1Y=%.9g SltPt2X=%.9g SltPt2Y=%.9g "
                "SltPt3X=%.9g SltPt3Y=%.9g SltPt4X=%.9g SltPt4Y=%.9g "
                "Rmin=%.9g NeedHybridAlgFusion=%ld "
                "BoundaryLineNum=%ld VirtualBoundaryLineNum=%ld "
                "BoundarySepPointNum=%ld StopperPointNum=%ld "
                "SlotTrackStopperNum=%ld\n",
                (unsigned long)frameId,
                (long)input->ParkingAction,
                (long)input->TargetSlotKind,
                (double)input->VehicleX,
                (double)input->VehicleY,
                (double)input->VehicleYaw,
                (double)input->TarX,
                (double)input->TarY,
                (double)input->TarTheta,
                (double)input->EntrPtX,
                (double)input->EntrPtY,
                (double)input->SltPt1X,
                (double)input->SltPt1Y,
                (double)input->SltPt2X,
                (double)input->SltPt2Y,
                (double)input->SltPt3X,
                (double)input->SltPt3Y,
                (double)input->SltPt4X,
                (double)input->SltPt4Y,
                (double)input->Rmin,
                (long)param->NeedHybridAlgFusion,
                (long)input->BoundaryLineNum,
                (long)input->VirtualBoundaryLineNum,
                (long)input->BoundarySepPtNum,
                (long)input->StopperPtNum,
                (long)input->SlotTrackStopperNum);
}

void apaDebugTracePlanningOutput(const char *stage,
                                 UInteger frameId,
                                 const ApaParkingPathPlanningOutput_t *output) {
  Integer pointCount0;
  Integer pointIndex0;
  Integer lastIndex0;
  Boolean hasNan0;
  Boolean hasInf0;
  Boolean endpointsSame0;
  Boolean pathValid0;
  Real maxPointJump0;
  Real firstX0;
  Real firstY0;
  Real firstTheta0;
  Real firstOdo0;
  Real lastX0;
  Real lastY0;
  Real lastTheta0;
  Real lastOdo0;

  pointCount0 = output->PtCount;
  if (pointCount0 < 0) {
    pointCount0 = 0;
  }
  if (pointCount0 > APA_PARKING_PATH_PT_MAX) {
    pointCount0 = APA_PARKING_PATH_PT_MAX;
  }
  hasNan0 = false;
  hasInf0 = false;
  endpointsSame0 = false;
  maxPointJump0 = 0.0f;
  firstX0 = 0.0f;
  firstY0 = 0.0f;
  firstTheta0 = 0.0f;
  firstOdo0 = 0.0f;
  lastX0 = 0.0f;
  lastY0 = 0.0f;
  lastTheta0 = 0.0f;
  lastOdo0 = 0.0f;
  pointIndex0 = 0;
  while (pointIndex0 < pointCount0) {
    Real x0;
    Real y0;
    Real theta0;
    Real odo0;

    x0 = output->PthPtX[pointIndex0];
    y0 = output->PthPtY[pointIndex0];
    theta0 = output->PthPtTht[pointIndex0];
    odo0 = output->PthPtOdo[pointIndex0];
    if (isnan(x0) || isnan(y0) || isnan(theta0) || isnan(odo0)) {
      hasNan0 = true;
    }
    if (isinf(x0) || isinf(y0) || isinf(theta0) || isinf(odo0)) {
      hasInf0 = true;
    }
    if (pointIndex0 > 0) {
      Real deltaX0;
      Real deltaY0;
      Real pointJump0;

      deltaX0 = x0 - output->PthPtX[pointIndex0 - 1];
      deltaY0 = y0 - output->PthPtY[pointIndex0 - 1];
      pointJump0 = sqrtf((deltaX0 * deltaX0) + (deltaY0 * deltaY0));
      if (pointJump0 > maxPointJump0) {
        maxPointJump0 = pointJump0;
      }
    }
    pointIndex0 = pointIndex0 + 1;
  }
  if (pointCount0 > 0) {
    lastIndex0 = pointCount0 - 1;
    firstX0 = output->PthPtX[0];
    firstY0 = output->PthPtY[0];
    firstTheta0 = output->PthPtTht[0];
    firstOdo0 = output->PthPtOdo[0];
    lastX0 = output->PthPtX[lastIndex0];
    lastY0 = output->PthPtY[lastIndex0];
    lastTheta0 = output->PthPtTht[lastIndex0];
    lastOdo0 = output->PthPtOdo[lastIndex0];
    endpointsSame0 = (firstX0 == lastX0) && (firstY0 == lastY0) && (firstTheta0 == lastTheta0);
  }
  pathValid0 = (output->PlanFlg != 0) && (pointCount0 > 1) && (output->SegNum > 0) &&
               (output->PathLen > 0.0f) && (!hasNan0) && (!hasInf0);
  (void)fprintf(stderr,
                "[APA_DIAG] frame_id=%lu stage=%s "
                "PlanFlg=%ld PlanOk=%ld ErrorType=%ld PlannerCase=%ld "
                "SegNum=%ld StpN=%ld PathLen=%.9g PtCount=%ld "
                "StepCount=%ld St2Idx=%ld "
                "first_x=%.9g first_y=%.9g first_theta=%.9g first_odo=%.9g "
                "last_x=%.9g last_y=%.9g last_theta=%.9g last_odo=%.9g "
                "path_valid=%ld has_nan=%ld has_inf=%ld point_count=%ld "
                "path_length=%.9g max_point_jump=%.9g endpoints_same=%ld\n",
                (unsigned long)frameId,
                stage,
                (long)output->PlanFlg,
                (long)(output->PlanFlg != 0),
                (long)output->ErrorType,
                (long)output->PlannerCase,
                (long)output->SegNum,
                (long)output->StpN,
                (double)output->PathLen,
                (long)output->PtCount,
                (long)output->StepCount,
                (long)output->St2Idx,
                (double)firstX0,
                (double)firstY0,
                (double)firstTheta0,
                (double)firstOdo0,
                (double)lastX0,
                (double)lastY0,
                (double)lastTheta0,
                (double)lastOdo0,
                (long)pathValid0,
                (long)hasNan0,
                (long)hasInf0,
                (long)pointCount0,
                (double)output->PathLen,
                (double)maxPointJump0,
                (long)endpointsSame0);
}

void apaDebugTraceGear(UInteger frameId,
                       const char *stage,
                       const char *source,
                       Integer pathIndex,
                       Real odo,
                       Integer segGoDir,
                       Integer targetGearReq) {
  Integer odoSign0;

  odoSign0 = 0;
  if (odo > 1.0e-4f) {
    odoSign0 = 1;
  } else if (odo < -1.0e-4f) {
    odoSign0 = -1;
  }
  (void)fprintf(stderr,
                "[APA_DIAG] frame_id=%lu stage=%s path_idx=%ld odo=%.9g "
                "odo_sign=%ld SegGoDir=%ld TargetGearReq=%ld "
                "gear_name=%s\n",
                (unsigned long)frameId,
                stage,
                (long)pathIndex,
                (double)odo,
                (long)odoSign0,
                (long)segGoDir,
                (long)targetGearReq,
                targetGearReq == 1
                    ? "D"
                    : (targetGearReq == 2 ? "R" : (targetGearReq == 3 ? "P" : "N_OR_INVALID")));
  if (ApaDebugTraceFirstPFrameId != frameId) {
    ApaDebugTraceFirstPFrameId = frameId;
    ApaDebugTraceFirstPSeen = false;
  }
  if ((targetGearReq == 3) && (!ApaDebugTraceFirstPSeen)) {
    ApaDebugTraceFirstPSeen = true;
    (void)fprintf(stderr,
                  "[APA_DIAG] frame_id=%lu stage=GEAR_FIRST_P_SOURCE "
                  "source=%s\n",
                  (unsigned long)frameId,
                  source);
  }
}

void apaDebugTraceParkoutInput(const char *stage,
                               UInteger frameId,
                               const ApaParkingPathPlanningInput_t *input) {
  if (input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKOUT) {
    (void)fprintf(stderr,
                  "[APA_PARKOUT_TRACE] frame_id=%lu stage=%s "
                  "VehicleX=%.9g VehicleY=%.9g VehicleYaw=%.9g "
                  "ParkingAction=%ld TargetSlotKind=%ld "
                  "TarX=%.9g TarY=%.9g TarTheta=%.9g "
                  "EntrPtX=%.9g EntrPtY=%.9g "
                  "SltPt1X=%.9g SltPt1Y=%.9g "
                  "SltPt2X=%.9g SltPt2Y=%.9g "
                  "SltPt3X=%.9g SltPt3Y=%.9g "
                  "SltPt4X=%.9g SltPt4Y=%.9g\n",
                  (unsigned long)frameId,
                  stage,
                  (double)input->VehicleX,
                  (double)input->VehicleY,
                  (double)input->VehicleYaw,
                  (long)input->ParkingAction,
                  (long)input->TargetSlotKind,
                  (double)input->TarX,
                  (double)input->TarY,
                  (double)input->TarTheta,
                  (double)input->EntrPtX,
                  (double)input->EntrPtY,
                  (double)input->SltPt1X,
                  (double)input->SltPt1Y,
                  (double)input->SltPt2X,
                  (double)input->SltPt2Y,
                  (double)input->SltPt3X,
                  (double)input->SltPt3Y,
                  (double)input->SltPt4X,
                  (double)input->SltPt4Y);
  }
}

void apaDebugTraceParkoutPath(const char *stage,
                              UInteger frameId,
                              const ApaParkingPathPlanningInput_t *input,
                              const ApaParkingPathPlanningOutput_t *output) {
  Integer pointCount0;
  Integer printCount0;
  Integer pointIndex0;
  Integer firstPathIndex0;
  Real deltaX0;
  Real deltaY0;
  Real distance0;
  Real forward0;
  Real firstNonzeroOdo0;
  Real lastNonzeroOdo0;
  Integer lastOdoSign0;
  Integer odoSign0;
  Integer forwardSegmentCount0;
  Integer reverseSegmentCount0;
  Integer directionChangeCount0;
  Boolean firstNonzeroOdoFound0;
  const char *direction0;

  if (input->ParkingAction != APA_SCENARIO_PARKING_ACTION_PARKOUT) {
    return;
  }
  apaDebugTracePlanningOutput(stage, frameId, output);
  pointCount0 = output->PtCount;
  if (pointCount0 < 0) {
    pointCount0 = 0;
  }
  if (pointCount0 > APA_PARKING_PATH_PT_MAX) {
    pointCount0 = APA_PARKING_PATH_PT_MAX;
  }
  (void)fprintf(stderr,
                "[APA_PARKOUT_TRACE] frame_id=%lu stage=%s_RESULT "
                "PlanFlg=%ld ErrorType=%ld PlannerCase=%ld PtCount=%ld\n",
                (unsigned long)frameId,
                stage,
                (long)output->PlanFlg,
                (long)output->ErrorType,
                (long)output->PlannerCase,
                (long)output->PtCount);

  printCount0 = pointCount0;
  if (printCount0 > 10) {
    printCount0 = 10;
  }
  pointIndex0 = 0;
  while (pointIndex0 < printCount0) {
    (void)fprintf(stderr,
                  "[APA_PARKOUT_TRACE] frame_id=%lu stage=%s_PATH "
                  "index=%ld PthPtX=%.9g PthPtY=%.9g "
                  "PthPtTht=%.9g PthPtOdo=%.9g\n",
                  (unsigned long)frameId,
                  stage,
                  (long)pointIndex0,
                  (double)output->PthPtX[pointIndex0],
                  (double)output->PthPtY[pointIndex0],
                  (double)output->PthPtTht[pointIndex0],
                  (double)output->PthPtOdo[pointIndex0]);
    pointIndex0 = pointIndex0 + 1;
  }

  firstPathIndex0 = -1;
  pointIndex0 = 0;
  while ((pointIndex0 < pointCount0) && (firstPathIndex0 < 0)) {
    deltaX0 = output->PthPtX[pointIndex0] - input->VehicleX;
    deltaY0 = output->PthPtY[pointIndex0] - input->VehicleY;
    distance0 = sqrtf((deltaX0 * deltaX0) + (deltaY0 * deltaY0));
    if (distance0 > APA_PARKOUT_TRACE_LEAVE_DISTANCE) {
      firstPathIndex0 = pointIndex0;
    }
    pointIndex0 = pointIndex0 + 1;
  }
  if (firstPathIndex0 >= 0) {
    deltaX0 = output->PthPtX[firstPathIndex0] - input->VehicleX;
    deltaY0 = output->PthPtY[firstPathIndex0] - input->VehicleY;
    forward0 = (cosf(input->VehicleYaw) * deltaX0) + (sinf(input->VehicleYaw) * deltaY0);
    direction0 = "FRONT";
    if (forward0 < 0.0f) {
      direction0 = "REAR";
    }
    (void)fprintf(stderr,
                  "[APA_PARKOUT_TRACE] frame_id=%lu stage=%s_DIRECTION "
                  "first_path_idx=%ld first_path_x=%.9g first_path_y=%.9g "
                  "first_path_theta=%.9g first_path_odo=%.9g "
                  "forward=%.9g direction=%s\n",
                  (unsigned long)frameId,
                  stage,
                  (long)firstPathIndex0,
                  (double)output->PthPtX[firstPathIndex0],
                  (double)output->PthPtY[firstPathIndex0],
                  (double)output->PthPtTht[firstPathIndex0],
                  (double)output->PthPtOdo[firstPathIndex0],
                  (double)forward0,
                  direction0);
  } else {
    (void)fprintf(stderr,
                  "[APA_PARKOUT_TRACE] frame_id=%lu stage=%s_DIRECTION "
                  "first_path_idx=-1 first_path_x=0 first_path_y=0 "
                  "first_path_theta=0 first_path_odo=0 "
                  "forward=0 direction=NONE\n",
                  (unsigned long)frameId,
                  stage);
  }

  firstNonzeroOdo0 = 0.0f;
  lastNonzeroOdo0 = 0.0f;
  lastOdoSign0 = 0;
  forwardSegmentCount0 = 0;
  reverseSegmentCount0 = 0;
  directionChangeCount0 = 0;
  firstNonzeroOdoFound0 = false;
  pointIndex0 = 0;
  while (pointIndex0 < pointCount0) {
    odoSign0 = 0;
    if (output->PthPtOdo[pointIndex0] > 1.0e-4f) {
      odoSign0 = 1;
    } else if (output->PthPtOdo[pointIndex0] < -1.0e-4f) {
      odoSign0 = -1;
    }
    if (odoSign0 != 0) {
      if (!firstNonzeroOdoFound0) {
        firstNonzeroOdo0 = output->PthPtOdo[pointIndex0];
        firstNonzeroOdoFound0 = true;
      }
      lastNonzeroOdo0 = output->PthPtOdo[pointIndex0];
      if (odoSign0 != lastOdoSign0) {
        if (lastOdoSign0 != 0) {
          directionChangeCount0 = directionChangeCount0 + 1;
        }
        if (odoSign0 > 0) {
          forwardSegmentCount0 = forwardSegmentCount0 + 1;
        } else {
          reverseSegmentCount0 = reverseSegmentCount0 + 1;
        }
        lastOdoSign0 = odoSign0;
      }
    }
    pointIndex0 = pointIndex0 + 1;
  }
  (void)fprintf(stderr,
                "[APA_PARKOUT_TRACE] frame_id=%lu stage=%s_ODO_SUMMARY "
                "first_nonzero_odo=%.9g last_nonzero_odo=%.9g "
                "forward_segment_count=%ld reverse_segment_count=%ld "
                "direction_change_count=%ld\n",
                (unsigned long)frameId,
                stage,
                (double)firstNonzeroOdo0,
                (double)lastNonzeroOdo0,
                (long)forwardSegmentCount0,
                (long)reverseSegmentCount0,
                (long)directionChangeCount0);
}
