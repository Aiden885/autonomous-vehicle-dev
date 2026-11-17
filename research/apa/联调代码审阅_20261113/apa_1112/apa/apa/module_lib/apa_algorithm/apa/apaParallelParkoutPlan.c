/**
 * @file apaParallelParkoutPlan.c
 * @brief APA 平行泊出规划器
 * @source geo_PkOutFt_PathPlanning.m, ClassParallelParkOut.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */

#include "apaParallelParkoutPlan.h"
#include "apaCountUtils.h"
#include "apaMathUtils.h"
#include "apaPathAlgo.h"
#include "apaVehicleCollisionCheck.h"
#include "geoAxisTransTypes.h"
#include "geoCircleIntrvCheckTypes.h"
#include "geoDivideTypes.h"
#include "geoLineIntrvCheckTypes.h"
#include "geoPkOutDangerPtTypes.h"
#include "obsSlightAdjustForSpecificPosTypes.h"
#include "vehicleCollisionCheckTypes.h"

#ifndef APA_MATH_PI
#define APA_MATH_PI 3.14159265f
#endif

#ifndef APA_PARKOUT_PATH_LEN_MIN
#define APA_PARKOUT_PATH_LEN_MIN 0.12f
#endif

#ifndef APA_PARKOUT_PATH_LEN_MAX
#define APA_PARKOUT_PATH_LEN_MAX 100.0f
#endif

#ifndef APA_PARKOUT_OUT_OF_TROUBLE_DISTANCE
#define APA_PARKOUT_OUT_OF_TROUBLE_DISTANCE 0.5f
#endif

#ifndef APA_PKOUT_FT_PLANNER_STAGE_MAX
#define APA_PKOUT_FT_PLANNER_STAGE_MAX 30
#endif

#ifndef APA_PKOUT_FT_PLANNER_SEG_LIMIT
#define APA_PKOUT_FT_PLANNER_SEG_LIMIT 9
#endif

#ifndef APA_PKOUT_FT_PLANNER_POINT_MAX
#define APA_PKOUT_FT_PLANNER_POINT_MAX (APA_PKOUT_FT_PLANNER_STAGE_MAX + 1)
#endif

#ifndef APA_PKOUT_FT_DISCRETIZE_SAMPLE_STEP_DEFAULT
#define APA_PKOUT_FT_DISCRETIZE_SAMPLE_STEP_DEFAULT 0.20f
#endif

#ifndef APA_PKOUT_FT_DISCRETIZE_SAMPLE_STEP_MIN
#define APA_PKOUT_FT_DISCRETIZE_SAMPLE_STEP_MIN 0.05f
#endif

#ifndef APA_PKOUT_FT_LATERAL_SAFE_OFFSET
#define APA_PKOUT_FT_LATERAL_SAFE_OFFSET 0.6f
#endif

#ifndef APA_PARKOUT_MATLAB_DIR_OUT_RIGHT
#define APA_PARKOUT_MATLAB_DIR_OUT_RIGHT 2
#endif

#ifndef APA_PARKOUT_MATLAB_DIR_OUT_LEFT
#define APA_PARKOUT_MATLAB_DIR_OUT_LEFT 1
#endif

#ifndef APA_PARKOUT_MATLAB_DIR_OUT_FRONT
#define APA_PARKOUT_MATLAB_DIR_OUT_FRONT 3
#endif

#ifndef APA_PARKOUT_MATLAB_DIR_OUT_REAR
#define APA_PARKOUT_MATLAB_DIR_OUT_REAR 4
#endif

typedef struct {
  Integer PlanCmd;
  Integer DirS;
  Integer PthPlnFlg;
  Integer SegNumOut;
  Integer StpN;
  Integer FailRs;
  Integer PlnChkCnt;
  Integer DangerActive;
  Real SafeOfst;
  Real SampleStep;
  Real PathLen;
  Real TarPt[3];
  Real Pt1[2];
  Real CurrentPose[3];
  Real CrashPos[3];
  Integer GoDirOut[APA_PKOUT_FT_PLANNER_STAGE_MAX];
  Integer TrnDirOut[APA_PKOUT_FT_PLANNER_STAGE_MAX];
  Real CrvOut[APA_PKOUT_FT_PLANNER_STAGE_MAX];
  Real ThtOut[APA_PKOUT_FT_PLANNER_STAGE_MAX];
  Real LengOut[APA_PKOUT_FT_PLANNER_STAGE_MAX];
  Real StxOut[APA_PKOUT_FT_PLANNER_POINT_MAX];
  Real StyOut[APA_PKOUT_FT_PLANNER_POINT_MAX];
  Real StThOut[APA_PKOUT_FT_PLANNER_POINT_MAX];
  Real CcOut[2 * APA_PKOUT_FT_PLANNER_STAGE_MAX];
  Integer FailRsOut[APA_PKOUT_FT_PLANNER_STAGE_MAX];
} ApaPkOutFtPlannerWork_t;

static inline void apaPkOutFtPlannerWorkInit(ApaPkOutFtPlannerWork_t *work) {
  Integer idx0;
  Integer pointIdx0;
  Integer ccIdx0;

  work->PlanCmd = 0;
  work->DirS = 1;
  work->PthPlnFlg = 0;
  work->SegNumOut = 0;
  work->StpN = 0;
  work->FailRs = APA_PARKOUT_FAIL_RS_NONE;
  work->PlnChkCnt = 0;
  work->DangerActive = 0;
  work->SafeOfst = 0.5f;
  work->SampleStep = APA_PKOUT_FT_DISCRETIZE_SAMPLE_STEP_DEFAULT;
  work->PathLen = 0.0f;
  work->TarPt[0] = 0.0f;
  work->TarPt[1] = 0.0f;
  work->TarPt[2] = 0.0f;
  work->Pt1[0] = 0.0f;
  work->Pt1[1] = 0.0f;
  work->CurrentPose[0] = 0.0f;
  work->CurrentPose[1] = 0.0f;
  work->CurrentPose[2] = 0.0f;
  work->CrashPos[0] = 0.0f;
  work->CrashPos[1] = 0.0f;
  work->CrashPos[2] = 0.0f;
  idx0 = 0;
  while (idx0 < APA_PKOUT_FT_PLANNER_STAGE_MAX) {
    work->GoDirOut[idx0] = 0;
    work->TrnDirOut[idx0] = 0;
    work->CrvOut[idx0] = 0.0f;
    work->ThtOut[idx0] = 0.0f;
    work->LengOut[idx0] = 0.0f;
    work->FailRsOut[idx0] = APA_PARKOUT_FAIL_RS_NONE;
    idx0 = idx0 + 1;
  }
  pointIdx0 = 0;
  while (pointIdx0 < APA_PKOUT_FT_PLANNER_POINT_MAX) {
    work->StxOut[pointIdx0] = 0.0f;
    work->StyOut[pointIdx0] = 0.0f;
    work->StThOut[pointIdx0] = 0.0f;
    pointIdx0 = pointIdx0 + 1;
  }
  ccIdx0 = 0;
  while (ccIdx0 < (2 * APA_PKOUT_FT_PLANNER_STAGE_MAX)) {
    work->CcOut[ccIdx0] = 0.0f;
    ccIdx0 = ccIdx0 + 1;
  }
}

static inline void apaPkOutFtPlannerApplyProbeTrace(ApaPkOutFtPlannerWork_t *work,
                                                    Integer probeCheckCount,
                                                    const Real crashPose[3]) {
  if (probeCheckCount > 0) {
    work->PlnChkCnt = work->PlnChkCnt + probeCheckCount;
  }
  if (crashPose != (const Real *)0) {
    work->CrashPos[0] = crashPose[0];
    work->CrashPos[1] = crashPose[1];
    work->CrashPos[2] = crashPose[2];
  }
}

static inline void apaPkOutFtPlannerApplyDangerSeed(ApaPkOutFtPlannerWork_t *work,
                                                    const GeoPkOutDangerAngleSearchOutput_t *danger,
                                                    Integer branchDir,
                                                    const Real startPose[3],
                                                    VehicleCollisionCheckInput_t *obsWork) {
  Real dangerPtX0;
  Real dangerPtY0;
  Real localX0;
  Real localY0;
  Real localTheta0;
  Real groundTheta0;
  Integer appendIdx0;
  Integer safeSpaceFlag0;
  Boolean noDanger0;
  Boolean isLeft0;
  Boolean isRight0;
  Boolean frontSpaceFail0;
  Boolean hasVirtualBoundary0;

  dangerPtX0 = 0.0f;
  dangerPtY0 = 0.0f;
  safeSpaceFlag0 = 1;
  if (danger != (const GeoPkOutDangerAngleSearchOutput_t *)0) {
    dangerPtX0 = danger->DangerPtX;
    dangerPtY0 = danger->DangerPtY;
    safeSpaceFlag0 = danger->SafeSpaceFlag;
  }

  work->CurrentPose[0] = startPose[0];
  work->CurrentPose[1] = startPose[1];
  work->CurrentPose[2] = startPose[2];
  work->DangerActive = 0;
  work->SafeOfst = 0.5f;

  isLeft0 = (branchDir == APA_PARKOUT_BRANCH_DIR_LEFT);
  isRight0 = (branchDir == APA_PARKOUT_BRANCH_DIR_RIGHT);
  noDanger0 = true;
  if (danger != (const GeoPkOutDangerAngleSearchOutput_t *)0) {
    noDanger0 = (danger->PtEx == 0);
  }
  hasVirtualBoundary0 = false;
  if (obsWork != (const VehicleCollisionCheckInput_t *)0) {
    hasVirtualBoundary0 = (obsWork->VirtualBoundaryLineNum > 0);
  }

  if (noDanger0) {
    localX0 = 6.2f;
    localY0 = 0.5f;
    if (isLeft0) {
      localY0 = -0.5f;
    }
    apaCoordVeh2Ground(startPose[0],
                       startPose[1],
                       startPose[2],
                       localX0,
                       localY0,
                       0.0f,
                       &dangerPtX0,
                       &dangerPtY0,
                       &groundTheta0);
    (void)groundTheta0;
    work->DangerActive = 1;
    work->SafeOfst = 2.5f;
  } else {
    dangerPtX0 = dangerPtX0 - 0.05f;
    if (isLeft0) {
      dangerPtY0 = dangerPtY0 - 0.1f;
    } else {
      if (isRight0) {
        dangerPtY0 = dangerPtY0 + 0.1f;
      }
    }
    if (danger != (const GeoPkOutDangerAngleSearchOutput_t *)0) {
      if ((danger->PtEx == 1) || ((danger->PtEx == 2) && hasVirtualBoundary0)) {
        work->DangerActive = 1;
      }
    }
  }

  work->Pt1[0] = dangerPtX0;
  work->Pt1[1] = dangerPtY0;
  if (obsWork != (VehicleCollisionCheckInput_t *)0) {
    appendIdx0 = obsWork->BoundarySepPtsNum;
    if ((appendIdx0 >= 0) && (appendIdx0 < VCC_MAX_SEPARATE_PTS)) {
      obsWork->BoundarySepPtsX[appendIdx0] = dangerPtX0;
      obsWork->BoundarySepPtsY[appendIdx0] = dangerPtY0;
      obsWork->BoundarySepPtsNum = appendIdx0 + 1;
    } else {
      work->PlanCmd = 5;
      work->FailRs = 9;
      work->FailRsOut[0] = 9;
    }
  }

  apaCoordGround2Veh(startPose[0],
                     startPose[1],
                     startPose[2],
                     dangerPtX0,
                     dangerPtY0,
                     0.0f,
                     &localX0,
                     &localY0,
                     &localTheta0);
  (void)localTheta0;
  frontSpaceFail0 = ((safeSpaceFlag0 == 0) || ((localX0 < 1.5f) && (localY0 < -1.0f)));
  if (frontSpaceFail0) {
    work->PlanCmd = 5;
    work->FailRs = APA_PARKOUT_FAIL_RS_FRONT_SPACE_INSUFFICIENT;
    work->FailRsOut[0] = APA_PARKOUT_FAIL_RS_FRONT_SPACE_INSUFFICIENT;
  }
}

static inline void
apaPkOutFtPlannerApplyFrontDefaultDangerSeed(ApaPkOutFtPlannerWork_t *work,
                                             const GeoPkOutDangerAngleSearchOutput_t *danger,
                                             Integer geometryBranchDir,
                                             const Real startPose[3],
                                             VehicleCollisionCheckInput_t *obsWork) {
  Real dangerPtX0;
  Real dangerPtY0;
  Real dangerPtTheta0;
  Integer seedIdx0;

  apaPkOutFtPlannerApplyDangerSeed(work, danger, geometryBranchDir, startPose, obsWork);
  if ((work != (ApaPkOutFtPlannerWork_t *)0) &&
      (danger != (const GeoPkOutDangerAngleSearchOutput_t *)0) && (startPose != (const Real *)0) &&
      (danger->PtEx == 0) && (work->FailRs != 9)) {
    dangerPtY0 = -0.8f;
    if (geometryBranchDir == APA_PARKOUT_BRANCH_DIR_LEFT) {
      dangerPtY0 = 0.8f;
    }
    apaCoordVeh2Ground(startPose[0],
                       startPose[1],
                       startPose[2],
                       4.5f,
                       dangerPtY0,
                       0.0f,
                       &dangerPtX0,
                       &dangerPtY0,
                       &dangerPtTheta0);
    (void)dangerPtTheta0;
    work->Pt1[0] = dangerPtX0;
    work->Pt1[1] = dangerPtY0;
    if (obsWork != (VehicleCollisionCheckInput_t *)0) {
      seedIdx0 = obsWork->BoundarySepPtsNum - 1;
      if ((seedIdx0 >= 0) && (seedIdx0 < VCC_MAX_SEPARATE_PTS)) {
        obsWork->BoundarySepPtsX[seedIdx0] = dangerPtX0;
        obsWork->BoundarySepPtsY[seedIdx0] = dangerPtY0;
      }
    }
  }
}

static inline Integer
apaPkOutFtPlannerResolveBranchDir(Integer dirOut, Real frontLeftFU, Real frontRightFU) {
  Boolean frontLeftWindow0;
  Boolean frontRightWindow0;
  Integer branchDir0;

  frontLeftWindow0 = ((frontLeftFU > 0.3f) && (frontLeftFU < 3.5f));
  frontRightWindow0 = ((frontRightFU > 0.3f) && (frontRightFU < 3.5f));
  branchDir0 = APA_PARKOUT_BRANCH_DIR_UNKNOWN;

  if ((dirOut == APA_PARKOUT_MATLAB_DIR_OUT_RIGHT) ||
      ((dirOut == APA_PARKOUT_MATLAB_DIR_OUT_FRONT) && frontLeftWindow0)) {
    branchDir0 = APA_PARKOUT_BRANCH_DIR_LEFT;
  } else {
    if ((dirOut == APA_PARKOUT_MATLAB_DIR_OUT_LEFT) ||
        ((dirOut == APA_PARKOUT_MATLAB_DIR_OUT_FRONT) && frontRightWindow0)) {
      branchDir0 = APA_PARKOUT_BRANCH_DIR_RIGHT;
    }
  }
  return branchDir0;
}

static inline Real apaPkOutResolveSideGateSpace(const ApaParallelParkoutContext_t *ctx,
                                                Integer branchDir) {
  Real sideSpace0;
  Boolean hasMatlabLeftUS0;

  sideSpace0 = 0.0f;
  if (ctx != (const ApaParallelParkoutContext_t *)0) {
    hasMatlabLeftUS0 = ((ctx->LeftUS[0] > 0.0f) || (ctx->LeftUS[1] > 0.0f));
    if (hasMatlabLeftUS0) {
      if (branchDir == APA_PARKOUT_BRANCH_DIR_LEFT) {
        sideSpace0 = ctx->LeftUS[1];
      } else {
        if (branchDir == APA_PARKOUT_BRANCH_DIR_RIGHT) {
          sideSpace0 = ctx->LeftUS[0];
        }
      }
    } else {
      if (branchDir == APA_PARKOUT_BRANCH_DIR_LEFT) {
        sideSpace0 = ctx->LeftSideSpace;
      } else {
        if (branchDir == APA_PARKOUT_BRANCH_DIR_RIGHT) {
          sideSpace0 = ctx->RightSideSpace;
        }
      }
    }
  }
  return sideSpace0;
}

static inline void apaPkOutFtPlannerApplySideSensorGate(ApaPkOutFtPlannerWork_t *work,
                                                        const ApaParallelParkoutContext_t *ctx,
                                                        Integer branchDir) {
  Real sideSpace0;
  Boolean gateHit0;

  sideSpace0 = apaPkOutResolveSideGateSpace(ctx, branchDir);
  gateHit0 = ((sideSpace0 > 0.2f) && (sideSpace0 < 1.0f));
  if (gateHit0) {
    work->PlanCmd = 5;
    work->FailRs = APA_PARKOUT_FAIL_RS_SIDE_SENSOR_CLOSE;
    work->FailRsOut[0] = APA_PARKOUT_FAIL_RS_SIDE_SENSOR_CLOSE;
  }
}

static inline void apaPkOutFtPlannerAdvancePlanCmdTail(ApaPkOutFtPlannerWork_t *work) {
  if (work->PlanCmd == 2) {
    work->DirS = -work->DirS;
  } else {
    if (work->PlanCmd == 5) {
      work->PthPlnFlg = 0;
    } else {
      if (work->PlanCmd == 1) {
        work->PthPlnFlg = 1;
      }
    }
  }
}

static inline void apaPkOutFtPlannerRecordLineSegment(ApaPkOutFtPlannerWork_t *work,
                                                      Integer goDir,
                                                      Real length,
                                                      Real startX,
                                                      Real startY,
                                                      Real startTheta) {
  Integer idx0;
  Boolean hasRoom0;

  idx0 = work->SegNumOut;
  hasRoom0 = (idx0 < APA_PKOUT_FT_PLANNER_STAGE_MAX);
  if (hasRoom0) {
    work->GoDirOut[idx0] = goDir;
    work->TrnDirOut[idx0] = 0;
    work->CrvOut[idx0] = 0.0f;
    work->ThtOut[idx0] = 0.0f;
    work->LengOut[idx0] = length;
    work->StxOut[idx0] = startX;
    work->StyOut[idx0] = startY;
    work->StThOut[idx0] = startTheta;
    work->CcOut[2 * idx0] = 0.0f;
    work->CcOut[(2 * idx0) + 1] = 0.0f;
    work->SegNumOut = idx0 + 1;
  } else {
    if (work->FailRs == APA_PARKOUT_FAIL_RS_NONE) {
      work->FailRs = 9;
    }
    if (work->FailRsOut[0] == APA_PARKOUT_FAIL_RS_NONE) {
      work->FailRsOut[0] = 9;
    }
  }
}

static inline void apaPkOutFtPlannerApplyLineProbeDecision(ApaPkOutFtPlannerWork_t *work,
                                                           Integer intrvFlg,
                                                           Integer goDir,
                                                           Real requestedLen,
                                                           Real safeLen,
                                                           Real shortenLen,
                                                           const Real startPose[3],
                                                           const Real endPose[3],
                                                           Integer probeCheckCount,
                                                           const Real crashPose[3]) {
  Real lineLen0;
  Real cosTheta0;
  Real sinTheta0;
  Real goDirReal0;

  apaPkOutFtPlannerApplyProbeTrace(work, probeCheckCount, crashPose);
  if (intrvFlg == 2) {
    work->PlanCmd = 5;
    work->FailRs = APA_PARKOUT_FAIL_RS_FORWARD_DANGER_BLOCKED;
    work->FailRsOut[0] = APA_PARKOUT_FAIL_RS_FORWARD_DANGER_BLOCKED;
  } else {
    if (intrvFlg == 0) {
      lineLen0 = requestedLen;
      work->TarPt[0] = endPose[0];
      work->TarPt[1] = endPose[1];
      work->TarPt[2] = endPose[2];
    } else {
      lineLen0 = safeLen - shortenLen;
      if (lineLen0 < 0.0f) {
        lineLen0 = 0.0f;
      }
      cosTheta0 = cosf(startPose[2]);
      sinTheta0 = sinf(startPose[2]);
      goDirReal0 = (Real)goDir;
      work->TarPt[0] = startPose[0] + (goDirReal0 * lineLen0 * cosTheta0);
      work->TarPt[1] = startPose[1] + (goDirReal0 * lineLen0 * sinTheta0);
      work->TarPt[2] = startPose[2];
    }
    apaPkOutFtPlannerRecordLineSegment(
        work, goDir, lineLen0, startPose[0], startPose[1], startPose[2]);
    if (work->FailRsOut[0] == APA_PARKOUT_FAIL_RS_NONE) {
      work->PlanCmd = 1;
      work->CurrentPose[0] = work->TarPt[0];
      work->CurrentPose[1] = work->TarPt[1];
      work->CurrentPose[2] = work->TarPt[2];
    } else {
      work->PlanCmd = 5;
    }
  }
}

static inline Boolean apaPkOutFtPlannerResolveForwardLineByDanger(Integer branchDir,
                                                                  const Real dangerPt[2],
                                                                  const Real startPose[3],
                                                                  const Real vehiclePose[3],
                                                                  Real safeOfst,
                                                                  Real vehWidth,
                                                                  Real latSafeOfst,
                                                                  Real *lineLen,
                                                                  Real endPose[3]) {
  GeoDivideInput_t divideInput0;
  GeoDivideOutput_t divideOutput0;
  Real dangerRelX0;
  Real dangerRelY0;
  Real dangerRelTheta0;
  Real startRelX0;
  Real startRelY0;
  Real startRelTheta0;
  Real vehHalfWidth0;
  Real lateralThreshold0;
  Real lenTmp1;
  Real lenTmp2;
  Real lineLen0;
  Real sinAbs0;
  Real divideCandidate0;
  Boolean activeLeft0;
  Boolean activeRight0;
  Boolean resolved0;

  resolved0 = false;
  if ((dangerPt != (const Real *)0) && (startPose != (const Real *)0) &&
      (vehiclePose != (const Real *)0) && (lineLen != (Real *)0) && (endPose != (Real *)0)) {
    apaCoordGround2Veh(startPose[0],
                       startPose[1],
                       startPose[2],
                       dangerPt[0],
                       dangerPt[1],
                       0.0f,
                       &dangerRelX0,
                       &dangerRelY0,
                       &dangerRelTheta0);
    (void)dangerRelTheta0;

    vehHalfWidth0 = vehWidth * 0.5f;
    lateralThreshold0 = vehHalfWidth0 + latSafeOfst;
    activeLeft0 = ((branchDir == APA_PARKOUT_BRANCH_DIR_LEFT) && (dangerRelY0 > lateralThreshold0));
    activeRight0 =
        ((branchDir == APA_PARKOUT_BRANCH_DIR_RIGHT) && (dangerRelY0 < -lateralThreshold0));
    if (activeLeft0 || activeRight0) {
      apaCoordGround2Veh(vehiclePose[0],
                         vehiclePose[1],
                         vehiclePose[2],
                         startPose[0],
                         startPose[1],
                         startPose[2],
                         &startRelX0,
                         &startRelY0,
                         &startRelTheta0);
      (void)startRelX0;
      (void)startRelTheta0;

      lenTmp1 = dangerRelX0 - safeOfst;
      sinAbs0 = fabsf(sinf(startPose[2]));
      if (activeLeft0) {
        if (startRelY0 > -vehHalfWidth0) {
          divideInput0.Numerator = startRelY0 + vehHalfWidth0;
          divideInput0.Denominator = sinAbs0;
          geoDivide(&divideInput0, &divideOutput0);
          divideCandidate0 = divideOutput0.Result;
          lenTmp2 = 2.0f;
          if (divideCandidate0 < lenTmp2) {
            lenTmp2 = divideCandidate0;
          }
        } else {
          lenTmp2 = startRelY0 + vehHalfWidth0;
        }
      } else {
        if (startRelY0 < vehHalfWidth0) {
          divideInput0.Numerator = vehHalfWidth0 - startRelY0;
          divideInput0.Denominator = sinAbs0;
          geoDivide(&divideInput0, &divideOutput0);
          divideCandidate0 = divideOutput0.Result;
          lenTmp2 = 2.0f;
          if (divideCandidate0 < lenTmp2) {
            lenTmp2 = divideCandidate0;
          }
        } else {
          lenTmp2 = vehHalfWidth0 - startRelY0;
        }
      }

      lineLen0 = lenTmp1;
      if (lenTmp2 < lineLen0) {
        lineLen0 = lenTmp2;
      }
      if (lineLen0 > 0.0f) {
        *lineLen = lineLen0;
        endPose[0] = startPose[0] + (lineLen0 * cosf(startPose[2]));
        endPose[1] = startPose[1] + (lineLen0 * sinf(startPose[2]));
        endPose[2] = startPose[2];
        resolved0 = true;
      }
    }
  }
  return resolved0;
}

static inline Boolean apaPkOutFtPlannerTryForwardLineByDanger(ApaPkOutFtPlannerWork_t *work,
                                                              Integer branchDir,
                                                              const Real dangerPt[2],
                                                              const Real startPose[3],
                                                              const Real vehiclePose[3],
                                                              Real safeOfst,
                                                              Real vehWidth,
                                                              Real latSafeOfst,
                                                              Integer intrvFlg,
                                                              Real safeLen,
                                                              Integer probeCheckCount,
                                                              const Real crashPose[3]) {
  Real lineLen0;
  Real endPose0[3];
  Boolean handled0;

  handled0 = false;
  if ((work != (ApaPkOutFtPlannerWork_t *)0) && (dangerPt != (const Real *)0) &&
      (startPose != (const Real *)0) && (vehiclePose != (const Real *)0) && (work->DirS == 1)) {
    if (apaPkOutFtPlannerResolveForwardLineByDanger(branchDir,
                                                    dangerPt,
                                                    startPose,
                                                    vehiclePose,
                                                    safeOfst,
                                                    vehWidth,
                                                    latSafeOfst,
                                                    &lineLen0,
                                                    endPose0)) {
      apaPkOutFtPlannerApplyLineProbeDecision(work,
                                              intrvFlg,
                                              work->DirS,
                                              lineLen0,
                                              safeLen,
                                              0.5f,
                                              startPose,
                                              endPose0,
                                              probeCheckCount,
                                              crashPose);
      handled0 = true;
    }
  }
  return handled0;
}

static inline Boolean
apaPkOutFtPlannerBuildForwardLineCheckInput(const ApaPkOutFtPlannerWork_t *work,
                                            Integer branchDir,
                                            const Real startPose[3],
                                            const Real vehiclePose[3],
                                            const ApaSlot_t *slot,
                                            const VehicleCollisionCheckInput_t *obsInput,
                                            const ApaPlannerVehConfig_t *vehConfig,
                                            Real safeOfst,
                                            Real latSafeOfst,
                                            Real checkStep,
                                            Integer dirOut,
                                            GeoLineIntrvCheckInput_t *lineInput) {
  Real dangerPt0[2];
  Real lineLen0;
  Real endPose0[3];
  Integer lineCount0;
  Integer lineIdx0;
  Integer virtualCount0;
  Integer virtualIdx0;
  Integer sepCount0;
  Integer sepIdx0;
  Integer stopperCount0;
  Integer stopperIdx0;
  Integer slotTrackCount0;
  Integer slotTrackIdx0;
  Boolean built0;

  built0 = false;
  if ((work != (const ApaPkOutFtPlannerWork_t *)0) && (startPose != (const Real *)0) &&
      (vehiclePose != (const Real *)0) && (slot != (const ApaSlot_t *)0) &&
      (obsInput != (const VehicleCollisionCheckInput_t *)0) &&
      (vehConfig != (const ApaPlannerVehConfig_t *)0) &&
      (lineInput != (GeoLineIntrvCheckInput_t *)0) && (work->DirS == 1)) {
    dangerPt0[0] = work->Pt1[0];
    dangerPt0[1] = work->Pt1[1];
    if (apaPkOutFtPlannerResolveForwardLineByDanger(branchDir,
                                                    dangerPt0,
                                                    startPose,
                                                    vehiclePose,
                                                    safeOfst,
                                                    vehConfig->W,
                                                    latSafeOfst,
                                                    &lineLen0,
                                                    endPose0)) {
      (void)lineLen0;
      *lineInput = (GeoLineIntrvCheckInput_t){0};
      lineInput->Pt1X = startPose[0];
      lineInput->Pt1Y = startPose[1];
      lineInput->Pt1Theta = startPose[2];
      lineInput->Pt2X = endPose0[0];
      lineInput->Pt2Y = endPose0[1];
      lineInput->Pt2Theta = endPose0[2];
      lineInput->CheckStep = checkStep;
      lineInput->SlotP1X = slot->SlotPts[0];
      lineInput->SlotP1Y = slot->SlotPts[1];
      lineInput->SlotP2X = slot->SlotPts[2];
      lineInput->SlotP2Y = slot->SlotPts[3];
      lineInput->SlotP3X = slot->SlotPts[4];
      lineInput->SlotP3Y = slot->SlotPts[5];
      lineInput->SlotP4X = slot->SlotPts[6];
      lineInput->SlotP4Y = slot->SlotPts[7];
      lineInput->VehWidth = vehConfig->W;
      lineInput->VehFrontOffset = vehConfig->LF;
      lineInput->VehRearOffset = vehConfig->LB;
      lineInput->UsePlanningCheck = true;
      lineInput->ParkingOutFlag = true;
      lineInput->DirOut = dirOut;
      lineCount0 = obsInput->BoundaryLineNum;
      if (lineCount0 > GEO_BOUNDARY_LINE_MAX) {
        lineCount0 = GEO_BOUNDARY_LINE_MAX;
      }
      if (lineCount0 < 0) {
        lineCount0 = 0;
      }
      lineInput->BoundaryLineNum = lineCount0;
      lineIdx0 = 0;
      while (lineIdx0 < lineCount0) {
        lineInput->BoundaryLineX1[lineIdx0] = obsInput->BoundaryLineX1[lineIdx0];
        lineInput->BoundaryLineY1[lineIdx0] = obsInput->BoundaryLineY1[lineIdx0];
        lineInput->BoundaryLineX2[lineIdx0] = obsInput->BoundaryLineX2[lineIdx0];
        lineInput->BoundaryLineY2[lineIdx0] = obsInput->BoundaryLineY2[lineIdx0];
        lineIdx0 = lineIdx0 + 1;
      }
      virtualCount0 = obsInput->VirtualBoundaryLineNum;
      if (virtualCount0 > VCC_MAX_VIRTUAL_LINES) {
        virtualCount0 = VCC_MAX_VIRTUAL_LINES;
      }
      if (virtualCount0 < 0) {
        virtualCount0 = 0;
      }
      lineInput->VirtualBoundaryLineNum = virtualCount0;
      virtualIdx0 = 0;
      while (virtualIdx0 < virtualCount0) {
        lineInput->VirtualBoundaryLineX1[virtualIdx0] =
            obsInput->VirtualBoundaryLineX1[virtualIdx0];
        lineInput->VirtualBoundaryLineY1[virtualIdx0] =
            obsInput->VirtualBoundaryLineY1[virtualIdx0];
        lineInput->VirtualBoundaryLineX2[virtualIdx0] =
            obsInput->VirtualBoundaryLineX2[virtualIdx0];
        lineInput->VirtualBoundaryLineY2[virtualIdx0] =
            obsInput->VirtualBoundaryLineY2[virtualIdx0];
        virtualIdx0 = virtualIdx0 + 1;
      }
      sepCount0 = obsInput->BoundarySepPtsNum;
      if (sepCount0 > VCC_MAX_SEPARATE_PTS) {
        sepCount0 = VCC_MAX_SEPARATE_PTS;
      }
      if (sepCount0 < 0) {
        sepCount0 = 0;
      }
      lineInput->BoundarySepPtNum = sepCount0;
      sepIdx0 = 0;
      while (sepIdx0 < sepCount0) {
        lineInput->BoundarySepPtX[sepIdx0] = obsInput->BoundarySepPtsX[sepIdx0];
        lineInput->BoundarySepPtY[sepIdx0] = obsInput->BoundarySepPtsY[sepIdx0];
        sepIdx0 = sepIdx0 + 1;
      }
      stopperCount0 = obsInput->SeparateStopperNum;
      if (stopperCount0 > VCC_MAX_SEPARATE_PTS) {
        stopperCount0 = VCC_MAX_SEPARATE_PTS;
      }
      if (stopperCount0 < 0) {
        stopperCount0 = 0;
      }
      lineInput->SeparateStopperNum = stopperCount0;
      stopperIdx0 = 0;
      while (stopperIdx0 < stopperCount0) {
        lineInput->SeparateStopperX[stopperIdx0] = obsInput->SeparateStopperX[stopperIdx0];
        lineInput->SeparateStopperY[stopperIdx0] = obsInput->SeparateStopperY[stopperIdx0];
        stopperIdx0 = stopperIdx0 + 1;
      }
      slotTrackCount0 = obsInput->SlotTrackStopperNum;
      if (slotTrackCount0 > VCC_MAX_TRACK_STOPPER) {
        slotTrackCount0 = VCC_MAX_TRACK_STOPPER;
      }
      if (slotTrackCount0 < 0) {
        slotTrackCount0 = 0;
      }
      lineInput->SlotTrackStopperNum = slotTrackCount0;
      slotTrackIdx0 = 0;
      while (slotTrackIdx0 < slotTrackCount0) {
        lineInput->SlotTrackStopperX1[slotTrackIdx0] = obsInput->SlotTrackStopperX1[slotTrackIdx0];
        lineInput->SlotTrackStopperY1[slotTrackIdx0] = obsInput->SlotTrackStopperY1[slotTrackIdx0];
        lineInput->SlotTrackStopperX2[slotTrackIdx0] = obsInput->SlotTrackStopperX2[slotTrackIdx0];
        lineInput->SlotTrackStopperY2[slotTrackIdx0] = obsInput->SlotTrackStopperY2[slotTrackIdx0];
        slotTrackIdx0 = slotTrackIdx0 + 1;
      }
      built0 = true;
    }
  }
  return built0;
}

static inline Boolean
apaPkOutFtPlannerTryForwardLineBranchEntry(ApaPkOutFtPlannerWork_t *work,
                                           Integer branchDir,
                                           const GeoPkOutDangerAngleSearchOutput_t *danger,
                                           const Real startPose[3],
                                           const Real vehiclePose[3],
                                           Real safeOfst,
                                           Real vehWidth,
                                           Real latSafeOfst,
                                           const GeoLineIntrvCheckOutput_t *lineProbe) {
  Real dangerPt0[2];
  Real crashPose0[3];
  Boolean canTry0;
  Boolean handled0;

  handled0 = false;
  if ((work != (ApaPkOutFtPlannerWork_t *)0) &&
      (danger != (const GeoPkOutDangerAngleSearchOutput_t *)0) && (startPose != (const Real *)0) &&
      (vehiclePose != (const Real *)0) && (lineProbe != (const GeoLineIntrvCheckOutput_t *)0)) {
    canTry0 = ((work->PlanCmd != 1) && (work->PlanCmd != 5) &&
               (work->SegNumOut < APA_PKOUT_FT_PLANNER_SEG_LIMIT) && (work->DangerActive != 0) &&
               (danger->SafeSpaceFlag != 0));
    if (canTry0) {
      dangerPt0[0] = work->Pt1[0];
      dangerPt0[1] = work->Pt1[1];
      crashPose0[0] = lineProbe->CrshPosX;
      crashPose0[1] = lineProbe->CrshPosY;
      crashPose0[2] = lineProbe->CrshPosTheta;
      handled0 = apaPkOutFtPlannerTryForwardLineByDanger(work,
                                                         branchDir,
                                                         dangerPt0,
                                                         startPose,
                                                         vehiclePose,
                                                         safeOfst,
                                                         vehWidth,
                                                         latSafeOfst,
                                                         lineProbe->IntrvFlg,
                                                         lineProbe->SafeLen,
                                                         lineProbe->CheckCount,
                                                         crashPose0);
    }
  }
  return handled0;
}

static inline Boolean
apaPkOutFtPlannerTryForwardLineBranchEntryChecked(ApaPkOutFtPlannerWork_t *work,
                                                  Integer branchDir,
                                                  const GeoPkOutDangerAngleSearchOutput_t *danger,
                                                  const Real startPose[3],
                                                  const Real vehiclePose[3],
                                                  const ApaSlot_t *slot,
                                                  const VehicleCollisionCheckInput_t *obsInput,
                                                  const ApaPlannerVehConfig_t *vehConfig,
                                                  Real safeOfst,
                                                  Real latSafeOfst,
                                                  Real checkStep,
                                                  Integer dirOut) {
  GeoLineIntrvCheckInput_t lineInput0;
  GeoLineIntrvCheckOutput_t lineOutput0;
  Boolean handled0;

  handled0 = false;
  if (apaPkOutFtPlannerBuildForwardLineCheckInput(work,
                                                  branchDir,
                                                  startPose,
                                                  vehiclePose,
                                                  slot,
                                                  obsInput,
                                                  vehConfig,
                                                  safeOfst,
                                                  latSafeOfst,
                                                  checkStep,
                                                  dirOut,
                                                  &lineInput0)) {
    lineOutput0 = (GeoLineIntrvCheckOutput_t){0};
    geoLineIntrvCheckPkOt(&lineInput0, &lineOutput0);
    handled0 = apaPkOutFtPlannerTryForwardLineBranchEntry(work,
                                                          branchDir,
                                                          danger,
                                                          startPose,
                                                          vehiclePose,
                                                          safeOfst,
                                                          vehConfig->W,
                                                          latSafeOfst,
                                                          &lineOutput0);
  }
  return handled0;
}

static inline void apaPkOutFtPlannerRecordArcSegment(ApaPkOutFtPlannerWork_t *work,
                                                     Integer goDir,
                                                     Integer trnDir,
                                                     Real radius,
                                                     Real theta,
                                                     Real length,
                                                     const Real startPose[3],
                                                     const Real centerPt[2]) {
  Integer idx0;
  Boolean hasRoom0;
  Real radiusSafe0;

  idx0 = work->SegNumOut;
  hasRoom0 = (idx0 < APA_PKOUT_FT_PLANNER_STAGE_MAX);
  if (hasRoom0) {
    radiusSafe0 = radius;
    if (radiusSafe0 < 0.001f) {
      radiusSafe0 = 0.001f;
    }
    work->GoDirOut[idx0] = goDir;
    work->TrnDirOut[idx0] = trnDir;
    work->CrvOut[idx0] = ((Real)trnDir) / radiusSafe0;
    work->ThtOut[idx0] = theta;
    work->LengOut[idx0] = length;
    work->StxOut[idx0] = startPose[0];
    work->StyOut[idx0] = startPose[1];
    work->StThOut[idx0] = startPose[2];
    work->CcOut[2 * idx0] = centerPt[0];
    work->CcOut[(2 * idx0) + 1] = centerPt[1];
    work->SegNumOut = idx0 + 1;
  } else {
    if (work->FailRs == APA_PARKOUT_FAIL_RS_NONE) {
      work->FailRs = 9;
    }
    if (work->FailRsOut[0] == APA_PARKOUT_FAIL_RS_NONE) {
      work->FailRsOut[0] = 9;
    }
  }
}

static inline void apaPkOutFtPlannerApplyArcProbeDecision(ApaPkOutFtPlannerWork_t *work,
                                                          Integer intrvFlg,
                                                          Integer goDir,
                                                          Integer trnDir,
                                                          Real radius,
                                                          Real theta,
                                                          Real availableLen,
                                                          const Real startPose[3],
                                                          const Real availablePose[3],
                                                          const Real centerPt[2],
                                                          Integer probeCheckCount,
                                                          const Real crashPose[3]) {
  Real theta0;

  apaPkOutFtPlannerApplyProbeTrace(work, probeCheckCount, crashPose);
  if (intrvFlg == 2) {
    work->PlanCmd = 5;
  } else {
    theta0 = theta;
    if (intrvFlg != 0) {
      theta0 = availablePose[2] - startPose[2];
    }
    apaPkOutFtPlannerRecordArcSegment(
        work, goDir, trnDir, radius, theta0, availableLen, startPose, centerPt);
    if (work->FailRsOut[0] == APA_PARKOUT_FAIL_RS_NONE) {
      work->CurrentPose[0] = availablePose[0];
      work->CurrentPose[1] = availablePose[1];
      work->CurrentPose[2] = availablePose[2];
      if (intrvFlg != 0) {
        work->PlanCmd = 2;
      }
    } else {
      work->PlanCmd = 5;
    }
  }
}

static inline Integer apaPkOutFtPlannerCountStpN(const ApaPkOutFtPlannerWork_t *work) {
  Integer idx0;
  Integer stpN0;
  Integer dirTmp0;

  stpN0 = 0;
  dirTmp0 = 0;
  if (work->SegNumOut > 0) {
    dirTmp0 = work->GoDirOut[0];
    if (dirTmp0 < 0) {
      stpN0 = 1;
    }
  }
  idx0 = 1;
  while (idx0 < work->SegNumOut) {
    if ((work->GoDirOut[idx0] * dirTmp0) < 0) {
      stpN0 = stpN0 + 1;
      dirTmp0 = work->GoDirOut[idx0];
    }
    idx0 = idx0 + 1;
  }
  return stpN0;
}

static inline void apaPkOutFtPlannerFinalizeOutput(ApaPkOutFtPlannerWork_t *work) {
  Integer idx0;
  Integer endPointIdx0;
  Boolean success0;

  success0 = ((work->PlanCmd == 1) && (work->FailRsOut[0] == APA_PARKOUT_FAIL_RS_NONE) &&
              (work->SegNumOut > 0) && (work->SegNumOut <= APA_PKOUT_FT_PLANNER_STAGE_MAX));
  if (success0) {
    endPointIdx0 = work->SegNumOut;
    work->StxOut[endPointIdx0] = work->TarPt[0];
    work->StyOut[endPointIdx0] = work->TarPt[1];
    work->StThOut[endPointIdx0] = work->TarPt[2];
    work->PathLen = 0.0f;
    idx0 = 0;
    while (idx0 < work->SegNumOut) {
      work->PathLen = work->PathLen + work->LengOut[idx0];
      idx0 = idx0 + 1;
    }
    work->PthPlnFlg = 1;
    work->StpN = apaPkOutFtPlannerCountStpN(work);
  } else {
    work->PathLen = 0.0f;
    work->PthPlnFlg = 0;
    work->StpN = 0;
    if (work->SegNumOut > 8) {
      work->FailRs = 9;
      work->FailRsOut[0] = 9;
    }
  }
}

static inline Real apaPkOutFtPlannerResolvePathSampleStep(const ApaPkOutFtPlannerWork_t *work) {
  Real sampleStep0;
  Real minFitStep0;

  sampleStep0 = APA_PKOUT_FT_DISCRETIZE_SAMPLE_STEP_DEFAULT;
  if (work != (const ApaPkOutFtPlannerWork_t *)0) {
    if (work->SampleStep >= APA_PKOUT_FT_DISCRETIZE_SAMPLE_STEP_MIN) {
      sampleStep0 = work->SampleStep;
    }
    minFitStep0 = work->PathLen / ((Real)(APA_PATH_MAX_POINTS - 2));
    if (minFitStep0 > sampleStep0) {
      sampleStep0 = minFitStep0;
    }
  }
  if (sampleStep0 < APA_PKOUT_FT_DISCRETIZE_SAMPLE_STEP_MIN) {
    sampleStep0 = APA_PKOUT_FT_DISCRETIZE_SAMPLE_STEP_MIN;
  }
  return sampleStep0;
}

static inline Boolean
apaPkOutFtPlannerAppendDiscretePoint(ApaPath_t *path, Real x, Real y, Real theta, Real radius) {
  Integer idx0;
  Boolean appended0;

  appended0 = false;
  if (path != (ApaPath_t *)0) {
    if (path->PathN < APA_PATH_MAX_POINTS) {
      idx0 = path->PathN;
      path->X[idx0] = x;
      path->Y[idx0] = y;
      path->Tht[idx0] = theta;
      path->OdoStep[idx0] = 0.0f;
      path->Radius[idx0] = radius;
      path->PathN = idx0 + 1;
      appended0 = true;
    }
  }
  return appended0;
}

static inline void apaPkOutFtPlannerSetLastOdoStep(ApaPath_t *path, Real odoStep) {
  Integer idx0;

  if (path != (ApaPath_t *)0) {
    if (path->PathN > 0) {
      idx0 = path->PathN - 1;
      path->OdoStep[idx0] = odoStep;
    }
  }
}

static inline void apaPkOutFtPlannerEvalSegmentPose(const ApaPkOutFtPlannerWork_t *work,
                                                    Integer segIdx,
                                                    Real progress,
                                                    Real *x,
                                                    Real *y,
                                                    Real *theta) {
  Real startTheta0;
  Real crv0;
  Real crvAbs0;
  Real goDir0;
  Real theta0;
  Real invCrv0;

  startTheta0 = work->StThOut[segIdx];
  crv0 = work->CrvOut[segIdx];
  crvAbs0 = fabsf(crv0);
  goDir0 = (Real)work->GoDirOut[segIdx];
  if (goDir0 == 0.0f) {
    goDir0 = 1.0f;
  }
  if (crvAbs0 <= APA_MATH_EPSILON) {
    *x = work->StxOut[segIdx] + (goDir0 * progress * cosf(startTheta0));
    *y = work->StyOut[segIdx] + (goDir0 * progress * sinf(startTheta0));
    *theta = startTheta0;
  } else {
    theta0 = startTheta0 + (goDir0 * crv0 * progress);
    invCrv0 = 1.0f / crv0;
    *x = work->CcOut[2 * segIdx] + (invCrv0 * sinf(theta0));
    *y = work->CcOut[(2 * segIdx) + 1] - (invCrv0 * cosf(theta0));
    *theta = theta0;
  }
}

static inline Boolean apaPkOutFtPlannerDiscretizeSegment(const ApaPkOutFtPlannerWork_t *work,
                                                         Integer segIdx,
                                                         Real sampleStep,
                                                         ApaPath_t *path) {
  Real segLen0;
  Real progress0;
  Real prevProgress0;
  Real delta0;
  Real radius0;
  Real crvAbs0;
  Real x0;
  Real y0;
  Real theta0;
  Real goDir0;
  Boolean success0;

  success0 = true;
  segLen0 = fabsf(work->LengOut[segIdx]);
  crvAbs0 = fabsf(work->CrvOut[segIdx]);
  radius0 = APA_MATH_INF;
  if (crvAbs0 > APA_MATH_EPSILON) {
    radius0 = 1.0f / crvAbs0;
  }
  goDir0 = (Real)work->GoDirOut[segIdx];
  if (goDir0 == 0.0f) {
    goDir0 = 1.0f;
  }
  if (segLen0 <= APA_MATH_EPSILON) {
    if (path->PathN > 0) {
      x0 = work->StxOut[segIdx + 1];
      y0 = work->StyOut[segIdx + 1];
      theta0 = work->StThOut[segIdx + 1];
      path->X[path->PathN - 1] = x0;
      path->Y[path->PathN - 1] = y0;
      path->Tht[path->PathN - 1] = theta0;
      path->Radius[path->PathN - 1] = radius0;
    }
  } else {
    progress0 = sampleStep;
    prevProgress0 = 0.0f;
    while (((progress0 + APA_MATH_EPSILON) < segLen0) && (path->PathN < APA_PATH_MAX_POINTS) &&
           success0) {
      delta0 = progress0 - prevProgress0;
      apaPkOutFtPlannerSetLastOdoStep(path, goDir0 * delta0);
      apaPkOutFtPlannerEvalSegmentPose(work, segIdx, progress0, &x0, &y0, &theta0);
      success0 = apaPkOutFtPlannerAppendDiscretePoint(path, x0, y0, theta0, radius0);
      if (success0) {
        prevProgress0 = progress0;
        progress0 = progress0 + sampleStep;
      }
    }
    if (success0) {
      if (path->PathN < APA_PATH_MAX_POINTS) {
        delta0 = segLen0 - prevProgress0;
        if (delta0 < 0.0f) {
          delta0 = 0.0f;
        }
        apaPkOutFtPlannerSetLastOdoStep(path, goDir0 * delta0);
        x0 = work->StxOut[segIdx + 1];
        y0 = work->StyOut[segIdx + 1];
        theta0 = work->StThOut[segIdx + 1];
        success0 = apaPkOutFtPlannerAppendDiscretePoint(path, x0, y0, theta0, radius0);
      } else {
        success0 = false;
      }
    }
  }
  return success0;
}

static inline Boolean apaPkOutFtPlannerDiscretizePathFromWork(const ApaPkOutFtPlannerWork_t *work,
                                                              ApaPath_t *path) {
  Integer idx0;
  Real sampleStep0;
  Boolean success0;

  sampleStep0 = apaPkOutFtPlannerResolvePathSampleStep(work);
  success0 = apaPkOutFtPlannerAppendDiscretePoint(
      path, work->StxOut[0], work->StyOut[0], work->StThOut[0], APA_MATH_INF);
  idx0 = 0;
  while (success0 && (idx0 < work->SegNumOut)) {
    success0 = apaPkOutFtPlannerDiscretizeSegment(work, idx0, sampleStep0, path);
    idx0 = idx0 + 1;
  }
  if (success0 && (path->PathN > 1)) {
    path->OdoStep[path->PathN - 1] = 0.0f;
  }
  return success0 && (path->PathN > 1);
}

static inline Boolean apaPkOutFtPlannerBuildPathFromWork(const ApaPkOutFtPlannerWork_t *work,
                                                         ApaPath_t *path) {
  Boolean success0;

  success0 = true;
  if (path != (ApaPath_t *)0) {
    apaPathInit(path);
  } else {
    success0 = false;
  }
  if (work == (const ApaPkOutFtPlannerWork_t *)0) {
    success0 = false;
  }
  if (success0) {
    if (work->PthPlnFlg != 1) {
      success0 = false;
    }
    if (work->FailRsOut[0] != APA_PARKOUT_FAIL_RS_NONE) {
      success0 = false;
    }
    if (work->SegNumOut <= 0) {
      success0 = false;
    }
  }
  if (success0) {
    path->ValidFlag = true;
    path->ErrorType = APA_FAULT_NO_ERR;
    path->PathType = APA_PATH_TYPE_ALL;
    path->PathSource = APA_PATH_SOURCE_SEARCH;
    path->CollisionFlag = APA_COLLISION_RISK_NOT_EXIST;
    path->RadiusFlag = true;
    success0 = apaPkOutFtPlannerDiscretizePathFromWork(work, path);
    if (success0) {
      path->SegNum = apaPathCalcSegNum(path);
      path->SegNumFlag = true;
    } else {
      apaPathInit(path);
    }
  }
  return success0;
}

static inline void apaParkoutDiagRecordCollision(ApaParallelParkoutDiag_t *diag,
                                                 Integer failReason,
                                                 Integer collisionStage,
                                                 const VehicleCollisionCheckInput_t *vccInput,
                                                 const VehicleCollisionCheckOutput_t *vccOutput) {
  if (diag != (ApaParallelParkoutDiag_t *)0) {
    if (failReason != APA_FAULT_NO_ERR) {
      diag->FailReason = failReason;
    }
    if (diag->CollisionStage == APA_PARKOUT_COLLISION_STAGE_NONE) {
      diag->CollisionType = vccOutput->CollisionType;
      diag->CollisionStage = collisionStage;
      diag->CollisionPoseX = vccInput->VehPosX;
      diag->CollisionPoseY = vccInput->VehPosY;
      diag->CollisionPoseTheta = vccInput->VehPosTheta;
    }
  }
}

static inline Integer apaPkOutResolveMatlabDirOut(const ApaParallelParkoutContext_t *ctx) {
  Integer dirOut0;

  dirOut0 = APA_PARKOUT_MATLAB_DIR_OUT_REAR;
  if (ctx != (const ApaParallelParkoutContext_t *)0) {
    if ((ctx->DirOut >= APA_PARKOUT_MATLAB_DIR_OUT_LEFT) &&
        (ctx->DirOut <= APA_PARKOUT_MATLAB_DIR_OUT_REAR)) {
      dirOut0 = ctx->DirOut;
    } else if (ctx->RequestedDir == APA_PARKOUT_BRANCH_DIR_RIGHT) {
      dirOut0 = APA_PARKOUT_MATLAB_DIR_OUT_RIGHT;
    } else if (ctx->RequestedDir == APA_PARKOUT_BRANCH_DIR_LEFT) {
      dirOut0 = APA_PARKOUT_MATLAB_DIR_OUT_LEFT;
    } else if (ctx->RequestedDir == APA_PARKOUT_BRANCH_DIR_FRONT) {
      dirOut0 = APA_PARKOUT_MATLAB_DIR_OUT_FRONT;
    } else if (ctx->SlotOnTheRight) {
      dirOut0 = APA_PARKOUT_MATLAB_DIR_OUT_LEFT;
    } else {
      dirOut0 = APA_PARKOUT_MATLAB_DIR_OUT_RIGHT;
    }
  }
  return dirOut0;
}

static inline Integer apaPkOutResolveSideBranch(const ApaParallelParkoutContext_t *ctx,
                                                Integer matlabDirOut) {
  Real frontLeft0;
  Real frontRight0;
  Integer branchDir0;

  frontLeft0 = 0.0f;
  frontRight0 = 0.0f;
  if (ctx != (const ApaParallelParkoutContext_t *)0) {
    frontLeft0 = ctx->FrontUS[0];
    frontRight0 = ctx->FrontUS[3];
    if ((frontLeft0 <= 0.0f) && (ctx->LeftSideSpace > 0.0f)) {
      frontLeft0 = ctx->LeftSideSpace;
    }
    if ((frontRight0 <= 0.0f) && (ctx->RightSideSpace > 0.0f)) {
      frontRight0 = ctx->RightSideSpace;
    }
  }
  branchDir0 = apaPkOutFtPlannerResolveBranchDir(matlabDirOut, frontLeft0, frontRight0);
  return branchDir0;
}

static inline Integer apaParkoutResolveBranchDir(const ApaParallelParkoutContext_t *ctx) {
  Integer branchDir;
  Integer dirOut;

  dirOut = apaPkOutResolveMatlabDirOut(ctx);
  if (dirOut == APA_PARKOUT_MATLAB_DIR_OUT_FRONT) {
    branchDir = APA_PARKOUT_BRANCH_DIR_FRONT;
  } else {
    branchDir = apaPkOutResolveSideBranch(ctx, dirOut);
  }
  if (branchDir == APA_PARKOUT_BRANCH_DIR_UNKNOWN) {
    branchDir = ctx->RequestedDir;
  }
  if ((branchDir != APA_PARKOUT_BRANCH_DIR_LEFT) && (branchDir != APA_PARKOUT_BRANCH_DIR_RIGHT) &&
      (branchDir != APA_PARKOUT_BRANCH_DIR_FRONT)) {
    branchDir = APA_PARKOUT_BRANCH_DIR_RIGHT;
    if (ctx->SlotOnTheRight) {
      branchDir = APA_PARKOUT_BRANCH_DIR_LEFT;
    }
  }
  return branchDir;
}

static inline Integer apaParkoutResolveGeometryBranchDir(const ApaParallelParkoutContext_t *ctx,
                                                         Integer requestedBranchDir) {
  Integer geometryBranchDir;
  Integer frontSensorBranchDir;
  Integer dirOut;

  geometryBranchDir = requestedBranchDir;
  if (geometryBranchDir == APA_PARKOUT_BRANCH_DIR_FRONT) {
    dirOut = apaPkOutResolveMatlabDirOut(ctx);
    frontSensorBranchDir = apaPkOutResolveSideBranch(ctx, dirOut);
    if ((frontSensorBranchDir == APA_PARKOUT_BRANCH_DIR_LEFT) ||
        (frontSensorBranchDir == APA_PARKOUT_BRANCH_DIR_RIGHT)) {
      geometryBranchDir = frontSensorBranchDir;
    } else {
      geometryBranchDir = APA_PARKOUT_BRANCH_DIR_UNKNOWN;
    }
  }
  return geometryBranchDir;
}

static inline Boolean apaParkoutBranchIsLeftOrRight(Integer branchDir) {
  return ((branchDir == APA_PARKOUT_BRANCH_DIR_LEFT) ||
          (branchDir == APA_PARKOUT_BRANCH_DIR_RIGHT));
}

static inline Integer apaParkoutResolveFtPlannerDirOut(Integer requestedBranchDir,
                                                       Integer geometryBranchDir) {
  Integer dirOut;

  dirOut = APA_PARKOUT_MATLAB_DIR_OUT_REAR;
  if (requestedBranchDir == APA_PARKOUT_BRANCH_DIR_FRONT) {
    dirOut = APA_PARKOUT_MATLAB_DIR_OUT_FRONT;
  } else if (geometryBranchDir == APA_PARKOUT_BRANCH_DIR_LEFT) {
    dirOut = APA_PARKOUT_MATLAB_DIR_OUT_RIGHT;
  } else if (geometryBranchDir == APA_PARKOUT_BRANCH_DIR_RIGHT) {
    dirOut = APA_PARKOUT_MATLAB_DIR_OUT_LEFT;
  }
  return dirOut;
}

static inline void apaParkoutDiagCaptureFrspSnapshot(ApaParallelParkoutDiag_t *diag,
                                                     const VehicleCollisionCheckInput_t *obsInput) {
  Boolean hasObs0;
  Integer lineRawCount0;
  Integer lineCount0;
  Integer lineIdx0;
  Integer lineIdx1;
  Integer sepRawCount0;
  Integer sepCount0;
  Integer sepIdx0;
  Integer sepIdx1;
  Integer virtualRawCount0;
  Integer virtualCount0;
  Integer virtualIdx0;
  Integer virtualIdx1;
  Integer stopperRawCount0;
  Integer stopperCount0;
  Integer stopperIdx0;
  Integer stopperIdx1;
  Integer slotTrackRawCount0;
  Integer slotTrackCount0;
  Integer slotTrackIdx0;
  Integer slotTrackIdx1;

  if (diag != (ApaParallelParkoutDiag_t *)0) {
    hasObs0 = (obsInput != (const VehicleCollisionCheckInput_t *)0);
    if (hasObs0) {
      lineRawCount0 = obsInput->BoundaryLineNum;
      sepRawCount0 = obsInput->BoundarySepPtsNum;
      virtualRawCount0 = obsInput->VirtualBoundaryLineNum;
      stopperRawCount0 = obsInput->SeparateStopperNum;
      slotTrackRawCount0 = obsInput->SlotTrackStopperNum;
    } else {
      lineRawCount0 = 0;
      sepRawCount0 = 0;
      virtualRawCount0 = 0;
      stopperRawCount0 = 0;
      slotTrackRawCount0 = 0;
    }
    lineCount0 = apaClampCount(lineRawCount0, VCC_MAX_BOUNDARY_LINES);
    sepCount0 = apaClampCount(sepRawCount0, VCC_MAX_SEPARATE_PTS);
    virtualCount0 = apaClampCount(virtualRawCount0, VCC_MAX_VIRTUAL_LINES);
    stopperCount0 = apaClampCount(stopperRawCount0, VCC_MAX_SEPARATE_PTS);
    slotTrackCount0 = apaClampCount(slotTrackRawCount0, VCC_MAX_TRACK_STOPPER);

    lineIdx0 = 0;
    while (lineIdx0 < VCC_MAX_BOUNDARY_LINES) {
      diag->FrspLineX1[lineIdx0] = 0.0f;
      diag->FrspLineY1[lineIdx0] = 0.0f;
      diag->FrspLineX2[lineIdx0] = 0.0f;
      diag->FrspLineY2[lineIdx0] = 0.0f;
      lineIdx0 = lineIdx0 + 1;
    }
    sepIdx0 = 0;
    while (sepIdx0 < VCC_MAX_SEPARATE_PTS) {
      diag->FrspSepPtX[sepIdx0] = 0.0f;
      diag->FrspSepPtY[sepIdx0] = 0.0f;
      sepIdx0 = sepIdx0 + 1;
    }
    virtualIdx0 = 0;
    while (virtualIdx0 < VCC_MAX_VIRTUAL_LINES) {
      diag->FrspVirtualLineX1[virtualIdx0] = 0.0f;
      diag->FrspVirtualLineY1[virtualIdx0] = 0.0f;
      diag->FrspVirtualLineX2[virtualIdx0] = 0.0f;
      diag->FrspVirtualLineY2[virtualIdx0] = 0.0f;
      virtualIdx0 = virtualIdx0 + 1;
    }
    stopperIdx0 = 0;
    while (stopperIdx0 < VCC_MAX_SEPARATE_PTS) {
      diag->FrspSeparateStopperX[stopperIdx0] = 0.0f;
      diag->FrspSeparateStopperY[stopperIdx0] = 0.0f;
      stopperIdx0 = stopperIdx0 + 1;
    }
    slotTrackIdx0 = 0;
    while (slotTrackIdx0 < VCC_MAX_TRACK_STOPPER) {
      diag->FrspSlotTrackStopperX1[slotTrackIdx0] = 0.0f;
      diag->FrspSlotTrackStopperY1[slotTrackIdx0] = 0.0f;
      diag->FrspSlotTrackStopperX2[slotTrackIdx0] = 0.0f;
      diag->FrspSlotTrackStopperY2[slotTrackIdx0] = 0.0f;
      slotTrackIdx0 = slotTrackIdx0 + 1;
    }
    diag->FrspLineNum = lineCount0;
    if (hasObs0) {
      lineIdx1 = 0;
      while (lineIdx1 < lineCount0) {
        diag->FrspLineX1[lineIdx1] = obsInput->BoundaryLineX1[lineIdx1];
        diag->FrspLineY1[lineIdx1] = obsInput->BoundaryLineY1[lineIdx1];
        diag->FrspLineX2[lineIdx1] = obsInput->BoundaryLineX2[lineIdx1];
        diag->FrspLineY2[lineIdx1] = obsInput->BoundaryLineY2[lineIdx1];
        lineIdx1 = lineIdx1 + 1;
      }
    }
    diag->FrspSepPtNum = sepCount0;
    if (hasObs0) {
      sepIdx1 = 0;
      while (sepIdx1 < sepCount0) {
        diag->FrspSepPtX[sepIdx1] = obsInput->BoundarySepPtsX[sepIdx1];
        diag->FrspSepPtY[sepIdx1] = obsInput->BoundarySepPtsY[sepIdx1];
        sepIdx1 = sepIdx1 + 1;
      }
    }
    diag->FrspVirtualLineNum = virtualCount0;
    if (hasObs0) {
      virtualIdx1 = 0;
      while (virtualIdx1 < virtualCount0) {
        diag->FrspVirtualLineX1[virtualIdx1] = obsInput->VirtualBoundaryLineX1[virtualIdx1];
        diag->FrspVirtualLineY1[virtualIdx1] = obsInput->VirtualBoundaryLineY1[virtualIdx1];
        diag->FrspVirtualLineX2[virtualIdx1] = obsInput->VirtualBoundaryLineX2[virtualIdx1];
        diag->FrspVirtualLineY2[virtualIdx1] = obsInput->VirtualBoundaryLineY2[virtualIdx1];
        virtualIdx1 = virtualIdx1 + 1;
      }
    }
    diag->FrspSeparateStopperNum = stopperCount0;
    if (hasObs0) {
      stopperIdx1 = 0;
      while (stopperIdx1 < stopperCount0) {
        diag->FrspSeparateStopperX[stopperIdx1] = obsInput->SeparateStopperX[stopperIdx1];
        diag->FrspSeparateStopperY[stopperIdx1] = obsInput->SeparateStopperY[stopperIdx1];
        stopperIdx1 = stopperIdx1 + 1;
      }
    }
    diag->FrspSlotTrackStopperNum = slotTrackCount0;
    if (hasObs0) {
      slotTrackIdx1 = 0;
      while (slotTrackIdx1 < slotTrackCount0) {
        diag->FrspSlotTrackStopperX1[slotTrackIdx1] = obsInput->SlotTrackStopperX1[slotTrackIdx1];
        diag->FrspSlotTrackStopperY1[slotTrackIdx1] = obsInput->SlotTrackStopperY1[slotTrackIdx1];
        diag->FrspSlotTrackStopperX2[slotTrackIdx1] = obsInput->SlotTrackStopperX2[slotTrackIdx1];
        diag->FrspSlotTrackStopperY2[slotTrackIdx1] = obsInput->SlotTrackStopperY2[slotTrackIdx1];
        slotTrackIdx1 = slotTrackIdx1 + 1;
      }
    }
  }
}

static inline void apaParkoutDiagSetFailRs(ApaParallelParkoutDiag_t *diag, Integer failRs) {
  if (diag != (ApaParallelParkoutDiag_t *)0) {
    if (diag->PkOutFailRs == APA_PARKOUT_FAIL_RS_NONE) {
      diag->PkOutFailRs = failRs;
    }
  }
}

static inline Real apaParkoutPathAbsLength(const ApaPath_t *path) {
  Real totalLen;
  Integer idx;

  totalLen = 0.0f;
  for (idx = 0; idx < path->PathN; idx = idx + 1) {
    Real odoAbs;

    odoAbs = path->OdoStep[idx];
    if (odoAbs < 0.0f) {
      odoAbs = -odoAbs;
    }
    totalLen = totalLen + odoAbs;
  }
  return totalLen;
}

static inline Real apaParkoutCalcStraightLengthToDanger(Real dngPtX,
                                                        Real dngPtY,
                                                        Real stPtX,
                                                        Real stPtY,
                                                        Real stPtTheta,
                                                        Real vehX,
                                                        Real vehY,
                                                        Real vehTheta,
                                                        Real safeOfst,
                                                        Real vehWidth,
                                                        Integer dir) {
  GeoAxisTransInput_t axisInput0;
  GeoAxisTransOutput_t dngPtRel0;
  GeoAxisTransOutput_t stPtRel0;
  GeoDivideInput_t divideInput0;
  GeoDivideOutput_t divideOutput0;
  Real lenTmp1;
  Real lenTmp2;
  Real lenTmp0;
  Real vehHalfWidth0;
  Real sinAbs0;
  Real divideCandidate0;
  Boolean isLeftDir0;
  Boolean useDivide0;

  axisInput0.PtX = dngPtX;
  axisInput0.PtY = dngPtY;
  axisInput0.PtTheta = 0.0f;
  axisInput0.Ori1X = 0.0f;
  axisInput0.Ori1Y = 0.0f;
  axisInput0.Ori1Theta = 0.0f;
  axisInput0.Ori2X = stPtX;
  axisInput0.Ori2Y = stPtY;
  axisInput0.Ori2Theta = stPtTheta;
  geoAxisTrans(&axisInput0, &dngPtRel0);

  axisInput0.PtX = stPtX;
  axisInput0.PtY = stPtY;
  axisInput0.PtTheta = stPtTheta;
  axisInput0.Ori1X = 0.0f;
  axisInput0.Ori1Y = 0.0f;
  axisInput0.Ori1Theta = 0.0f;
  axisInput0.Ori2X = vehX;
  axisInput0.Ori2Y = vehY;
  axisInput0.Ori2Theta = vehTheta;
  geoAxisTrans(&axisInput0, &stPtRel0);

  lenTmp1 = dngPtRel0.ResultX - safeOfst;
  vehHalfWidth0 = vehWidth * 0.5f;
  sinAbs0 = fabsf(sinf(stPtTheta));
  isLeftDir0 = (dir == 2);
  if (isLeftDir0) {
    useDivide0 = (stPtRel0.ResultY > -vehHalfWidth0);
    if (useDivide0) {
      divideInput0.Numerator = stPtRel0.ResultY + vehHalfWidth0;
      divideInput0.Denominator = sinAbs0;
      geoDivide(&divideInput0, &divideOutput0);
      divideCandidate0 = divideOutput0.Result;
      lenTmp2 = 2.0f;
      if (divideCandidate0 < lenTmp2) {
        lenTmp2 = divideCandidate0;
      }
    } else {
      lenTmp2 = stPtRel0.ResultY + vehHalfWidth0;
    }
  } else {
    useDivide0 = (stPtRel0.ResultY < vehHalfWidth0);
    if (useDivide0) {
      divideInput0.Numerator = vehHalfWidth0 - stPtRel0.ResultY;
      divideInput0.Denominator = sinAbs0;
      geoDivide(&divideInput0, &divideOutput0);
      divideCandidate0 = divideOutput0.Result;
      lenTmp2 = 2.0f;
      if (divideCandidate0 < lenTmp2) {
        lenTmp2 = divideCandidate0;
      }
    } else {
      lenTmp2 = vehHalfWidth0 - stPtRel0.ResultY;
    }
  }

  lenTmp0 = lenTmp1;
  if (lenTmp2 < lenTmp0) {
    lenTmp0 = lenTmp2;
  }

  return lenTmp0;
}

static inline Boolean apaParkoutPathAppendEndpointMatches(const ApaPath_t *totalPath,
                                                          const ApaPath_t *segPath) {
  Integer totalLastIdx;
  Real deltaX0;
  Real deltaY0;
  Real pointDistance0;
  Boolean endpointMatches;

  endpointMatches = false;
  if ((totalPath->PathN > 0) && (segPath->PathN > 0)) {
    totalLastIdx = totalPath->PathN - 1;
    deltaX0 = segPath->X[0] - totalPath->X[totalLastIdx];
    deltaY0 = segPath->Y[0] - totalPath->Y[totalLastIdx];
    pointDistance0 = apaMathNorm2D(deltaX0, deltaY0);
    endpointMatches = apaMathEqual(pointDistance0, 0.0f);
  }

  return endpointMatches;
}

static inline void apaParkoutAppendPathMatlab(ApaPath_t *totalPath, const ApaPath_t *segPath) {
  Integer segIdx;
  Integer dstIdx;

  segIdx = 1;
  while ((segIdx < segPath->PathN) && (totalPath->PathN < APA_PATH_MAX_POINTS)) {
    dstIdx = totalPath->PathN;
    totalPath->X[dstIdx] = segPath->X[segIdx];
    totalPath->Y[dstIdx] = segPath->Y[segIdx];
    totalPath->Tht[dstIdx] = segPath->Tht[segIdx];
    totalPath->OdoStep[dstIdx - 1] = segPath->OdoStep[segIdx - 1];
    totalPath->PathN = totalPath->PathN + 1;
    segIdx = segIdx + 1;
  }

  if (totalPath->PathN > 0) {
    dstIdx = totalPath->PathN - 1;
    totalPath->OdoStep[dstIdx] = 0.0f;
  }
  apaPathResetFlags(totalPath);
}

static inline Boolean apaParkoutAppendPathChecked(ApaPath_t *totalPath, const ApaPath_t *segPath) {
  Boolean appendable;
  Boolean endpointMatches;
  Integer rawCount;

  appendable = false;
  if (segPath->PathN == 0) {
    appendable = true;
  } else if (totalPath->PathN <= 0) {
    apaPathCopy(totalPath, segPath);
    appendable = true;
  } else {
    rawCount = totalPath->PathN + segPath->PathN;
    if (rawCount < APA_PATH_MAX_POINTS) {
      endpointMatches = apaParkoutPathAppendEndpointMatches(totalPath, segPath);
      if (endpointMatches) {
        apaParkoutAppendPathMatlab(totalPath, segPath);
        appendable = true;
      }
    }
  }

  return appendable;
}

static inline void apaParkoutPreparePlanningVccInput(VehicleCollisionCheckInput_t *vccInput,
                                                     Integer collisionModel,
                                                     Integer collisionDemand) {
  vccInput->CheckType = VCC_CHECK_TYPE_PLANNING;
  vccInput->CollisionModel = collisionModel;
  vccInput->CollisionDemand = collisionDemand;
  vccInput->ParkingOutFlag = true;
  vccInput->SlotClass = VCC_SLOT_CLASS_PARALLEL;
  vccInput->UseAutoMargin = true;
}

static inline void apaParkoutApplyObsSlightAdjustment(const ApaParallelParkoutContext_t *ctx,
                                                      const Real startPose[3],
                                                      VehicleCollisionCheckInput_t *obsWork) {
  ObsSlightAdjustForSpecificPosInput_t slightInput;
  ObsSlightAdjustForSpecificPosOutput_t slightOutput;
  Integer slotIndex;

  slightInput = (ObsSlightAdjustForSpecificPosInput_t){0};
  slightOutput = (ObsSlightAdjustForSpecificPosOutput_t){0};

  slightInput.CheckType = VCC_CHECK_TYPE_PLANNING;
  slightInput.PosX = startPose[0];
  slightInput.PosY = startPose[1];
  slightInput.PosTheta = startPose[2];
  slightInput.SlotPtsValid = true;
  slightInput.SlotClass = VCC_SLOT_CLASS_PARALLEL;
  slightInput.ConsiderCollisionDepthFlag = false;
  slightInput.ParkingOutFlag = false;
  slightInput.StObs = *obsWork;
  slightInput.StObs.CheckType = VCC_CHECK_TYPE_PLANNING;
  slightInput.StObs.SlotClass = VCC_SLOT_CLASS_PARALLEL;
  slightInput.StObs.SlotPtsValid = true;
  slightInput.StObs.UseAutoMargin = true;

  slotIndex = 0;
  while (slotIndex < 8) {
    slightInput.SlotPts[slotIndex] = ctx->Slot.SlotPts[slotIndex];
    slightInput.StObs.SlotPts[slotIndex] = ctx->Slot.SlotPts[slotIndex];
    slotIndex = slotIndex + 1;
  }

  obsSlightAdjustForSpecificPos(&slightInput, &slightOutput);
  if (slightOutput.ObsValid) {
    *obsWork = slightOutput.StObs;
  }
}

/**
 * @brief 直行探查器 — 对应 generateStraightSegment
 *
 * 沿初始方向步进测试，遇到碰撞时退出并构建直线端点路径。用于腾出安全转弯余量。
 */
static inline Boolean apaParkoutGenStraightSegment(const ApaParallelParkoutContext_t *ctx,
                                                   const VehicleCollisionCheckInput_t *obsInput,
                                                   const ApaPlannerVehConfig_t *vehConfig,
                                                   const Real startPose[3],
                                                   Integer direction,
                                                   ApaPath_t *outSeg,
                                                   Real stopPose[3],
                                                   ApaParallelParkoutDiag_t *diag) {
  VehicleCollisionCheckInput_t vccInput;
  VehicleCollisionCheckOutput_t vccOutput;
  Real firstStep;
  Real s;
  Real sLimit;
  Real tmpPos[3];
  Integer pathN;
  Boolean success0;
  Boolean pathOpen;

  (void)vehConfig;

  apaPathInit(outSeg);

  firstStep = 0.04f;

  pathN = 0;
  outSeg->X[pathN] = startPose[0];
  outSeg->Y[pathN] = startPose[1];
  outSeg->Tht[pathN] = startPose[2];
  outSeg->OdoStep[pathN] = direction * firstStep;
  outSeg->PathSource = APA_PATH_SOURCE_SEARCH;
  outSeg->PathType = APA_PATH_TYPE_ALL;
  pathN = pathN + 1;

  stopPose[0] = startPose[0];
  stopPose[1] = startPose[1];
  stopPose[2] = startPose[2];

  vccInput = *obsInput;
  apaParkoutPreparePlanningVccInput(
      &vccInput, VCC_COLLISION_MODEL_EIGHT_EDGES, VCC_COLLISION_DEMAND_SIMPLIFY);
  vccInput.VehPosX = startPose[0];
  vccInput.VehPosY = startPose[1];
  vccInput.VehPosTheta = startPose[2];
  vehicleCollisionCheck(&vccInput, &vccOutput);
  success0 = false;
  if (vccOutput.IsCollision) {
    apaParkoutDiagRecordCollision(diag,
                                  APA_FAULT_COLLISION_AT_START,
                                  APA_PARKOUT_COLLISION_STAGE_INITIAL_STRAIGHT,
                                  &vccInput,
                                  &vccOutput);
  } else {
    s = firstStep;
    sLimit = 2.0f;
    if (ctx->Config.BackoutDistanceLimit > 0.0f) {
      sLimit = ctx->Config.BackoutDistanceLimit;
    }

    pathOpen = true;
    while (apaMathLess(s, sLimit) && (pathN < APA_PATH_MAX_POINTS) && pathOpen) {
      tmpPos[0] = startPose[0] + s * cosf(startPose[2]) * direction;
      tmpPos[1] = startPose[1] + s * sinf(startPose[2]) * direction;
      tmpPos[2] = startPose[2];

      vccInput.VehPosX = tmpPos[0];
      vccInput.VehPosY = tmpPos[1];
      vccInput.VehPosTheta = tmpPos[2];
      vehicleCollisionCheck(&vccInput, &vccOutput);

      if (vccOutput.IsCollision) {
        apaParkoutDiagRecordCollision(diag,
                                      APA_FAULT_GEO_PATH_REVERSE_COLLISION,
                                      APA_PARKOUT_COLLISION_STAGE_INITIAL_STRAIGHT,
                                      &vccInput,
                                      &vccOutput);
        pathOpen = false;
      } else {
        outSeg->X[pathN] = tmpPos[0];
        outSeg->Y[pathN] = tmpPos[1];
        outSeg->Tht[pathN] = tmpPos[2];
        outSeg->OdoStep[pathN] = direction * ctx->Config.SampleStep;
        pathN = pathN + 1;

        s = s + ctx->Config.SampleStep;
        stopPose[0] = tmpPos[0];
        stopPose[1] = tmpPos[1];
        stopPose[2] = tmpPos[2];
      }
    }

    if (pathN >= 2) {
      outSeg->PathN = pathN;
      outSeg->OdoStep[pathN - 1] = 0.0f;
      outSeg->ValidFlag = true;
      outSeg->ErrorType = APA_FAULT_NO_ERR;
      success0 = apaPathGetLengthValid(outSeg, APA_PARKOUT_PATH_LEN_MIN, APA_PARKOUT_PATH_LEN_MAX);
    }
  }
  return success0;
}

static inline Boolean apaParkoutGenManeuverSegment(const ApaParallelParkoutContext_t *ctx,
                                                   const VehicleCollisionCheckInput_t *obsInput,
                                                   const ApaPlannerVehConfig_t *vehConfig,
                                                   const Real startPose[3],
                                                   Integer direction,
                                                   Boolean turnLeft,
                                                   ApaPath_t *outSeg,
                                                   ApaParallelParkoutDiag_t *diag,
                                                   Boolean *isVertexOut) {
  VehicleCollisionCheckInput_t vccInput;
  VehicleCollisionCheckOutput_t vccOutput;
  Real firstStep;
  Real s;
  Real sLimit;
  Integer pathN;
  Real turnRadius;
  Boolean success;
  Boolean maneuverOpen;
  (void)vehConfig;
  *isVertexOut = false;
  success = false;
  apaPathInit(outSeg);

  firstStep = 0.04f;
  pathN = 0;
  outSeg->X[pathN] = startPose[0];
  outSeg->Y[pathN] = startPose[1];
  outSeg->Tht[pathN] = startPose[2];
  outSeg->OdoStep[pathN] = direction * firstStep;
  outSeg->PathSource = APA_PATH_SOURCE_SEARCH;
  outSeg->PathType = APA_PATH_TYPE_ALL;
  pathN = pathN + 1;

  s = firstStep;
  sLimit = 4.0f;
  turnRadius = ctx->Config.TurnRadius;
  vccInput = *obsInput;
  apaParkoutPreparePlanningVccInput(
      &vccInput, VCC_COLLISION_MODEL_EIGHT_EDGES, VCC_COLLISION_DEMAND_SIMPLIFY);

  maneuverOpen = true;
  while (apaMathLess(s, sLimit) && (pathN < APA_PATH_MAX_POINTS) && maneuverOpen) {
    Real turnAngle;
    Real relX;
    Real relY;
    Real relTht;
    Real tmpPos[3];

    turnAngle = s / apaMathDivideZeroProcess(turnRadius);

    if (direction == 1) {
      if (turnLeft) {
        relX = turnRadius * sinf(turnAngle);
        relY = turnRadius * (1.0f - cosf(turnAngle));
        relTht = turnAngle;
      } else {
        relX = turnRadius * sinf(turnAngle);
        relY = -turnRadius * (1.0f - cosf(turnAngle));
        relTht = -turnAngle;
      }
    } else {
      if (turnLeft) {
        relX = -turnRadius * sinf(turnAngle);
        relY = turnRadius * (1.0f - cosf(turnAngle));
        relTht = -turnAngle;
      } else {
        relX = -turnRadius * sinf(turnAngle);
        relY = -turnRadius * (1.0f - cosf(turnAngle));
        relTht = turnAngle;
      }
    }

    apaCoordVeh2Ground(startPose[0],
                       startPose[1],
                       startPose[2],
                       relX,
                       relY,
                       relTht,
                       &tmpPos[0],
                       &tmpPos[1],
                       &tmpPos[2]);

    vccInput.VehPosX = tmpPos[0];
    vccInput.VehPosY = tmpPos[1];
    vccInput.VehPosTheta = tmpPos[2];
    vehicleCollisionCheck(&vccInput, &vccOutput);

    if (vccOutput.IsCollision) {
      Integer collisionStage;

      collisionStage = APA_PARKOUT_COLLISION_STAGE_MANEUVER_BACKWARD;
      if (direction == 1) {
        collisionStage = APA_PARKOUT_COLLISION_STAGE_MANEUVER_FORWARD;
      }
      if (collisionStage == APA_PARKOUT_COLLISION_STAGE_MANEUVER_FORWARD) {
        apaParkoutDiagSetFailRs(diag, APA_PARKOUT_FAIL_RS_FORWARD_DANGER_BLOCKED);
      } else {
        apaParkoutDiagSetFailRs(diag, APA_PARKOUT_FAIL_RS_REVERSE_GEOMETRY_INFEASIBLE);
      }
      apaParkoutDiagRecordCollision(diag, APA_FAULT_NO_ERR, collisionStage, &vccInput, &vccOutput);
      maneuverOpen = false;
    } else {
      outSeg->X[pathN] = tmpPos[0];
      outSeg->Y[pathN] = tmpPos[1];
      outSeg->Tht[pathN] = tmpPos[2];
      outSeg->OdoStep[pathN] = direction * ctx->Config.SampleStep;
      pathN = pathN + 1;

      if (direction == 1) {
        Real poAngleThresh;
        Real cosThresh;
        Real cornerX;
        Real cornerY;
        Real tarTht;
        Real vec1[2], vec2[2];
        Real dot, norm;
        Real cosVal;

        poAngleThresh = 94.0f * APA_MATH_PI / 180.0f;
        cosThresh = cosf(poAngleThresh);

        if (ctx->SlotOnTheRight) {

          cornerX =
              tmpPos[0] + vehConfig->LF * cosf(tmpPos[2]) + (vehConfig->W / 2.0f) * sinf(tmpPos[2]);
          cornerY =
              tmpPos[1] + vehConfig->LF * sinf(tmpPos[2]) - (vehConfig->W / 2.0f) * cosf(tmpPos[2]);
          tarTht = apaCoordWrapToPi(ctx->Slot.TarPos[2] - APA_MATH_PI / 2.0f);
        } else {

          cornerX =
              tmpPos[0] + vehConfig->LF * cosf(tmpPos[2]) - (vehConfig->W / 2.0f) * sinf(tmpPos[2]);
          cornerY =
              tmpPos[1] + vehConfig->LF * sinf(tmpPos[2]) + (vehConfig->W / 2.0f) * cosf(tmpPos[2]);
          tarTht = apaCoordWrapToPi(ctx->Slot.TarPos[2] + APA_MATH_PI / 2.0f);
        }

        vec1[0] = cornerX - ctx->BackSlotPtNear[0];
        vec1[1] = cornerY - ctx->BackSlotPtNear[1];
        vec2[0] = cosf(tarTht);
        vec2[1] = sinf(tarTht);

        dot = vec1[0] * vec2[0] + vec1[1] * vec2[1];
        norm = apaMathNorm2D(vec1[0], vec1[1]) * apaMathNorm2D(vec2[0], vec2[1]);
        cosVal = dot / apaMathDivideZeroProcess(norm);

        if (cosVal < cosThresh) {
          *isVertexOut = true;
          maneuverOpen = false;
        }
      }
    }

    s = s + ctx->Config.SampleStep;
  }

  if (pathN >= 2) {
    outSeg->PathN = pathN;
    outSeg->OdoStep[pathN - 1] = 0.0f;
    outSeg->ValidFlag = true;
    outSeg->ErrorType = APA_FAULT_NO_ERR;
    if (!*isVertexOut) {
      success = apaPathGetLengthValid(outSeg, APA_PARKOUT_PATH_LEN_MIN, APA_PARKOUT_PATH_LEN_MAX);
    } else {
      success = true;
    }
  }

  return success;
}

static inline Boolean apaParkoutGenArcSegment(const ApaParallelParkoutContext_t *ctx,
                                              const VehicleCollisionCheckInput_t *obsInput,
                                              const Real startPose[3],
                                              Integer direction,
                                              Boolean turnLeft,
                                              Real arcLength,
                                              ApaPath_t *outSeg,
                                              ApaParallelParkoutDiag_t *diag) {
  VehicleCollisionCheckInput_t vccInput;
  VehicleCollisionCheckOutput_t vccOutput;
  Real firstStep;
  Real s;
  Integer pathN;
  Real turnRadius;
  Boolean collisionFree0;
  Boolean success0;

  apaPathInit(outSeg);

  firstStep = 0.04f;
  pathN = 0;
  outSeg->X[pathN] = startPose[0];
  outSeg->Y[pathN] = startPose[1];
  outSeg->Tht[pathN] = startPose[2];
  outSeg->OdoStep[pathN] = direction * firstStep;
  outSeg->PathSource = APA_PATH_SOURCE_SEARCH;
  outSeg->PathType = APA_PATH_TYPE_ALL;
  pathN = pathN + 1;

  s = firstStep;
  turnRadius = 10.0f * ctx->Config.TurnRadius;
  vccInput = *obsInput;
  apaParkoutPreparePlanningVccInput(
      &vccInput, VCC_COLLISION_MODEL_FOUR_EDGES, VCC_COLLISION_DEMAND_SIMPLIFY);
  collisionFree0 = true;
  success0 = false;

  while (apaMathLess(s, arcLength) && (pathN < APA_PATH_MAX_POINTS) && collisionFree0) {
    Real turnAngle;
    Real relX;
    Real relY;
    Real relTht;
    Real tmpPos[3];

    turnAngle = s / apaMathDivideZeroProcess(turnRadius);

    if (direction == 1) {
      if (turnLeft) {
        relX = turnRadius * sinf(turnAngle);
        relY = turnRadius * (1.0f - cosf(turnAngle));
        relTht = turnAngle;
      } else {
        relX = turnRadius * sinf(turnAngle);
        relY = -turnRadius * (1.0f - cosf(turnAngle));
        relTht = -turnAngle;
      }
    } else {
      if (turnLeft) {
        relX = -turnRadius * sinf(turnAngle);
        relY = turnRadius * (1.0f - cosf(turnAngle));
        relTht = -turnAngle;
      } else {
        relX = -turnRadius * sinf(turnAngle);
        relY = -turnRadius * (1.0f - cosf(turnAngle));
        relTht = turnAngle;
      }
    }

    apaCoordVeh2Ground(startPose[0],
                       startPose[1],
                       startPose[2],
                       relX,
                       relY,
                       relTht,
                       &tmpPos[0],
                       &tmpPos[1],
                       &tmpPos[2]);

    vccInput.VehPosX = tmpPos[0];
    vccInput.VehPosY = tmpPos[1];
    vccInput.VehPosTheta = tmpPos[2];
    vehicleCollisionCheck(&vccInput, &vccOutput);

    if (vccOutput.IsCollision) {
      apaParkoutDiagRecordCollision(
          diag, APA_FAULT_NO_ERR, APA_PARKOUT_COLLISION_STAGE_FINAL_ARC, &vccInput, &vccOutput);
      collisionFree0 = false;
    } else {
      outSeg->X[pathN] = tmpPos[0];
      outSeg->Y[pathN] = tmpPos[1];
      outSeg->Tht[pathN] = tmpPos[2];
      outSeg->OdoStep[pathN] = direction * ctx->Config.SampleStep;
      pathN = pathN + 1;

      s = s + ctx->Config.SampleStep;
    }
  }

  if (collisionFree0 && (pathN >= 2)) {
    outSeg->PathN = pathN;
    outSeg->OdoStep[pathN - 1] = 0.0f;
    outSeg->ValidFlag = true;
    outSeg->ErrorType = APA_FAULT_NO_ERR;
    success0 = apaPathGetLengthValid(outSeg, APA_PARKOUT_PATH_LEN_MIN, APA_PARKOUT_PATH_LEN_MAX);
  }
  return success0;
}

static inline Boolean
apaParkoutCheckInitialSafeSpaceWithFtPlanner(const ApaParallelParkoutContext_t *ctx,
                                             VehicleCollisionCheckInput_t *obsWork,
                                             const ApaPlannerVehConfig_t *vehConfig,
                                             const Real startPose[3],
                                             ApaParallelParkoutDiag_t *diag,
                                             GeoPkOutDangerAngleSearchOutput_t *ftPlannerDanger,
                                             ApaPkOutFtPlannerWork_t *ftPlannerSeedWork) {
  GeoPkOutDangerAngleSearchInput_t dangerInput;
  GeoPkOutDangerAngleSearchOutput_t dangerOutput;
  Real dangerSearchPose[3];
  Integer matlabDirOut;
  Integer branchDir;
  Integer seedBranchDir;
  Integer sideGateBranchDir;
  Boolean safeSpace;
  ApaPkOutFtPlannerWork_t ftPlannerWork;

  matlabDirOut = apaPkOutResolveMatlabDirOut(ctx);
  branchDir = apaParkoutResolveBranchDir(ctx);
  seedBranchDir = branchDir;
  if (branchDir == APA_PARKOUT_BRANCH_DIR_FRONT) {
    seedBranchDir = apaParkoutResolveGeometryBranchDir(ctx, branchDir);
  }
  sideGateBranchDir = branchDir;
  if ((branchDir == APA_PARKOUT_BRANCH_DIR_FRONT) && apaParkoutBranchIsLeftOrRight(seedBranchDir)) {
    sideGateBranchDir = seedBranchDir;
  }

  if (diag != (ApaParallelParkoutDiag_t *)0) {
    diag->BranchDir = branchDir;
  }

  apaPkOutFtPlannerWorkInit(&ftPlannerWork);
  apaPkOutFtPlannerApplySideSensorGate(&ftPlannerWork, ctx, sideGateBranchDir);
  if (ftPlannerWork.PlanCmd == 5) {
    apaParkoutDiagCaptureFrspSnapshot(diag, obsWork);
    if (diag != (ApaParallelParkoutDiag_t *)0) {
      diag->FailReason = APA_FAULT_PKING_OUT_UNENABLE;
      diag->PkOutFailRs = ftPlannerWork.FailRsOut[0];
    }
    safeSpace = false;
  } else {
    dangerSearchPose[0] = startPose[0];
    dangerSearchPose[1] = startPose[1];
    dangerSearchPose[2] = startPose[2];
    if (ctx->RplnFlag == 1) {
      dangerSearchPose[0] = 0.0f;
      dangerSearchPose[1] = 0.0f;
      dangerSearchPose[2] = 0.0f;
    }
    dangerInput = (GeoPkOutDangerAngleSearchInput_t){0};
    dangerInput.VehX = dangerSearchPose[0];
    dangerInput.VehY = dangerSearchPose[1];
    dangerInput.VehTheta = dangerSearchPose[2];
    dangerInput.Dir = matlabDirOut;
    dangerInput.CheckStep = ctx->Config.SampleStep;
    dangerInput.FrontCheckLen = 4.0f;
    dangerInput.FrontUs[0] = ctx->FrontUS[0];
    dangerInput.FrontUs[1] = ctx->FrontUS[1];
    dangerInput.FrontUs[2] = ctx->FrontUS[2];
    dangerInput.FrontUs[3] = ctx->FrontUS[3];
    dangerInput.VehConfig = *vehConfig;
    dangerInput.Obstacles = *obsWork;

    geoPkOutDangerAngleSearch(&dangerInput, &dangerOutput);
    if ((branchDir == APA_PARKOUT_BRANCH_DIR_FRONT) && (dangerOutput.PtEx == 0)) {
      apaPkOutFtPlannerApplyFrontDefaultDangerSeed(
          &ftPlannerWork, &dangerOutput, seedBranchDir, startPose, obsWork);
    } else {
      apaPkOutFtPlannerApplyDangerSeed(
          &ftPlannerWork, &dangerOutput, seedBranchDir, startPose, obsWork);
    }
    apaParkoutDiagCaptureFrspSnapshot(diag, obsWork);
    if (ftPlannerDanger != (GeoPkOutDangerAngleSearchOutput_t *)0) {
      *ftPlannerDanger = dangerOutput;
    }
    if (ftPlannerSeedWork != (ApaPkOutFtPlannerWork_t *)0) {
      *ftPlannerSeedWork = ftPlannerWork;
    }
    safeSpace = (ftPlannerWork.PlanCmd != 5);
    if (diag != (ApaParallelParkoutDiag_t *)0) {
      diag->SafeSpaceFlag = dangerOutput.SafeSpaceFlag;
      diag->DangerPtType = dangerOutput.PtEx;
      diag->CheckCount = dangerOutput.CheckCount;
      diag->DangerPtX = dangerOutput.DangerPtX;
      diag->DangerPtY = dangerOutput.DangerPtY;
      diag->SelectedPoseX = dangerOutput.SelectedPoseX;
      diag->SelectedPoseY = dangerOutput.SelectedPoseY;
      diag->SelectedPoseTheta = dangerOutput.SelectedPoseTheta;
      diag->ForwardSafeLen = dangerOutput.ForwardSafeLen;
      if (safeSpace) {
        diag->FailReason = APA_FAULT_NO_ERR;
      } else {
        diag->FailReason = APA_FAULT_PKING_OUT_UNENABLE;
        if (ftPlannerWork.FailRsOut[0] != APA_PARKOUT_FAIL_RS_NONE) {
          apaParkoutDiagSetFailRs(diag, ftPlannerWork.FailRsOut[0]);
        } else {
          apaParkoutDiagSetFailRs(diag, APA_PARKOUT_FAIL_RS_FRONT_SPACE_INSUFFICIENT);
        }
      }
    }
  }
  return safeSpace;
}

static inline void apaParkoutBuildRetryStart(const ApaParallelParkoutContext_t *ctx,
                                             const Real basePose[3],
                                             Real retryPose[3]) {
  Real localOffset[3];

  localOffset[0] = 0.0f;
  localOffset[1] = 0.2f;
  localOffset[2] = 0.0f;
  if (!ctx->SlotOnTheRight) {
    localOffset[1] = -0.2f;
  }

  apaCoordVehToGroundPose(basePose, localOffset, retryPose);
}

void apaParkoutGetOutOfTroublePoint(const Real vehPos[3], Integer mebDir, Real point[3]) {
  Real ds;

  ds = -((Real)mebDir) * APA_PARKOUT_OUT_OF_TROUBLE_DISTANCE;
  point[0] = vehPos[0] + ds * cosf(vehPos[2]);
  point[1] = vehPos[1] + ds * sinf(vehPos[2]);
  point[2] = vehPos[2];
}

static inline Real apaParkoutCalcBackEnableDistance(const ApaParallelParkoutContext_t *ctx,
                                                    const ApaPlannerVehConfig_t *vehConfig,
                                                    const Real startPose[3],
                                                    const ApaParallelParkoutDiag_t *diag) {
  Real distance;
  Integer branchDir;

  distance = 0.0f;
  branchDir = apaParkoutResolveBranchDir(ctx);
  if ((branchDir != APA_PARKOUT_BRANCH_DIR_FRONT) && (diag->DangerPtType > 0)) {
    Real dangerLocalX;
    Real dangerLocalY;
    Real dangerLocalTheta;
    Real turnRadius;
    Real frontRadius;
    Real frontLen;
    Real reachRadius;
    Real lateralDelta;
    Real rootTerm;
    Real rawDistance;
    Real maxDistance;

    apaCoordGround2Veh(startPose[0],
                       startPose[1],
                       startPose[2],
                       diag->DangerPtX,
                       diag->DangerPtY,
                       0.0f,
                       &dangerLocalX,
                       &dangerLocalY,
                       &dangerLocalTheta);
    (void)dangerLocalTheta;

    turnRadius = ctx->Config.TurnRadius;
    frontRadius = vehConfig->MinCircle;
    if (frontRadius < 1.0f) {
      frontRadius = turnRadius;
    }
    frontLen = vehConfig->Lf;
    if (frontLen < 0.1f) {
      frontLen = 0.6f;
    }
    reachRadius = frontRadius + frontLen + 0.6f;
    if (branchDir == APA_PARKOUT_BRANCH_DIR_RIGHT) {
      lateralDelta = -turnRadius - dangerLocalY;
      maxDistance = 2.0f;
    } else {
      lateralDelta = turnRadius - dangerLocalY;
      maxDistance = 3.5f;
    }

    rootTerm = reachRadius * reachRadius - lateralDelta * lateralDelta;
    if (rootTerm > 0.0f) {
      rawDistance = sqrtf(rootTerm) - dangerLocalX;
      if (rawDistance > APA_PARKOUT_PATH_LEN_MIN) {
        distance = apaMathClamp(rawDistance, 0.0f, maxDistance);
      }
    }
  }
  return distance;
}

static inline Real apaParkoutResolvePreBackDistance(const ApaParallelParkoutContext_t *ctx,
                                                    const ApaPlannerVehConfig_t *vehConfig,
                                                    const Real startPose[3],
                                                    const ApaParallelParkoutDiag_t *diag,
                                                    Integer *backStrategy,
                                                    Integer *moveDirection) {
  Real distance;

  distance = 0.0f;
  *backStrategy = APA_PARKOUT_BACK_STRATEGY_NONE;
  *moveDirection = -1;
  if (ctx->MebDir != 0) {
    distance = 2.0f;
    *backStrategy = APA_PARKOUT_BACK_STRATEGY_MEB;
    if (ctx->MebDir < 0) {
      *moveDirection = 1;
    }
  } else if (ctx->BackEnable != 0) {
    distance = apaParkoutCalcBackEnableDistance(ctx, vehConfig, startPose, diag);
    if (distance > APA_PARKOUT_PATH_LEN_MIN) {
      *backStrategy = APA_PARKOUT_BACK_STRATEGY_BACK_ENABLE;
    }
  }
  return distance;
}

static inline void apaParkoutTryPreBackSegment(const ApaParallelParkoutContext_t *ctx,
                                               const VehicleCollisionCheckInput_t *obsInput,
                                               const ApaPlannerVehConfig_t *vehConfig,
                                               const Real startPose[3],
                                               ApaPath_t *totalPath,
                                               ApaPath_t *segPath,
                                               Real currentPose[3],
                                               ApaParallelParkoutDiag_t *diag,
                                               Boolean *success) {
  ApaParallelParkoutContext_t backCtx;
  Real stopPose[3];
  Real backDistance;
  Integer backStrategy;
  Integer moveDirection;
  Boolean successInit;
  Boolean appendOk;

  backDistance = apaParkoutResolvePreBackDistance(
      ctx, vehConfig, startPose, diag, &backStrategy, &moveDirection);
  if ((*success) && (backDistance > APA_PARKOUT_PATH_LEN_MIN)) {
    backCtx = *ctx;
    backCtx.Config.BackoutDistanceLimit = backDistance;
    successInit = apaParkoutGenStraightSegment(
        &backCtx, obsInput, vehConfig, currentPose, moveDirection, segPath, stopPose, diag);
    if (successInit) {
      appendOk = apaParkoutAppendPathChecked(totalPath, segPath);
      if (appendOk) {
        currentPose[0] = segPath->X[segPath->PathN - 1];
        currentPose[1] = segPath->Y[segPath->PathN - 1];
        currentPose[2] = segPath->Tht[segPath->PathN - 1];
        diag->BackStrategy = backStrategy;
        diag->BackoutDistance = apaParkoutPathAbsLength(segPath);
      } else {
        *success = false;
      }
    } else {
      currentPose[0] = stopPose[0];
      currentPose[1] = stopPose[1];
      currentPose[2] = stopPose[2];
      if (backStrategy == APA_PARKOUT_BACK_STRATEGY_MEB) {
        *success = false;
        diag->BackStrategy = backStrategy;
        diag->BackoutDistance = 0.0f;
      }
    }
  }
}

static inline Boolean apaPkOutFtPlannerResolveArcByDanger(Integer branchDir,
                                                          Integer goDir,
                                                          const Real dangerPt[2],
                                                          const Real startPose[3],
                                                          Real radius,
                                                          Real vehWidth,
                                                          Real latSafeOfst,
                                                          Integer *trnDir,
                                                          Real centerPt[2],
                                                          Real endPose[3],
                                                          Real *theta,
                                                          Real *arcLen) {
  Real dangerRelX0;
  Real dangerRelY0;
  Real dangerRelTheta0;
  Real ltmp0;
  Real trnOutLen0;
  Real dangerLat0;
  Real acosArg10;
  Real acosArg20;
  Real theta0;
  Real reverseInnerRadius0;
  Real reverseThetaAbs0;
  Integer trnDir0;
  Boolean ok0;

  ok0 = false;
  if ((dangerPt != (const Real *)0) && (startPose != (const Real *)0) && (trnDir != (Integer *)0) &&
      (centerPt != (Real *)0) && (endPose != (Real *)0) && (theta != (Real *)0) &&
      (arcLen != (Real *)0) && (radius > 0.001f)) {

    trnDir0 = goDir;
    if (branchDir == APA_PARKOUT_BRANCH_DIR_LEFT) {
      trnDir0 = -goDir;
    }
    centerPt[0] = startPose[0] + radius * cosf(startPose[2] + ((Real)trnDir0) * APA_HALF_PI);
    centerPt[1] = startPose[1] + radius * sinf(startPose[2] + ((Real)trnDir0) * APA_HALF_PI);
    apaCoordGround2Veh(startPose[0],
                       startPose[1],
                       startPose[2],
                       dangerPt[0],
                       dangerPt[1],
                       0.0f,
                       &dangerRelX0,
                       &dangerRelY0,
                       &dangerRelTheta0);
    (void)dangerRelTheta0;
    ltmp0 = sqrtf((centerPt[0] - dangerPt[0]) * (centerPt[0] - dangerPt[0]) +
                  (centerPt[1] - dangerPt[1]) * (centerPt[1] - dangerPt[1]));
    if (ltmp0 > 0.001f) {
      if (goDir >= 0) {
        trnOutLen0 = radius + (vehWidth * 0.5f) + latSafeOfst;
        if (trnOutLen0 < radius) {
          trnOutLen0 = radius;
        }
        if (branchDir == APA_PARKOUT_BRANCH_DIR_RIGHT) {
          dangerLat0 = fabsf(radius - dangerRelY0);
        } else {
          dangerLat0 = fabsf(radius + dangerRelY0);
        }
        if (trnOutLen0 > ltmp0) {
          if (branchDir == APA_PARKOUT_BRANCH_DIR_RIGHT) {
            theta0 = ((Real)trnDir0) * (APA_MATH_PI / 3.6f);
          } else {
            theta0 = ((Real)trnDir0) * (APA_MATH_PI / 3.0f);
          }
        } else {
          acosArg10 = apaMathClampUnit(dangerLat0 / ltmp0);
          acosArg20 = apaMathClampUnit(trnOutLen0 / ltmp0);
          theta0 = ((Real)trnDir0) * (acosf(acosArg10) - acosf(acosArg20));
        }
      } else {

        reverseInnerRadius0 = radius - (vehWidth * 0.5f) - (latSafeOfst * 1.5f);
        if (reverseInnerRadius0 < 0.0f) {
          reverseInnerRadius0 = 0.0f;
        }
        reverseThetaAbs0 = asinf(apaMathClampUnit(dangerRelX0 / ltmp0)) -
                           acosf(apaMathClampUnit(reverseInnerRadius0 / ltmp0));
        theta0 = ((Real)trnDir0) * reverseThetaAbs0;
      }
      endPose[2] = startPose[2] + theta0;
      endPose[0] = centerPt[0] + radius * cosf(endPose[2] - ((Real)trnDir0) * APA_HALF_PI);
      endPose[1] = centerPt[1] + radius * sinf(endPose[2] - ((Real)trnDir0) * APA_HALF_PI);
      *trnDir = trnDir0;
      *theta = theta0;
      *arcLen = fabsf(theta0) * radius;
      ok0 = ((*arcLen) > APA_PARKOUT_PATH_LEN_MIN);
    }
  }
  return ok0;
}

static inline void
apaPkOutFtPlannerFillCircleCheckInput(const ApaSlot_t *slot,
                                      const VehicleCollisionCheckInput_t *obsInput,
                                      const ApaPlannerVehConfig_t *vehConfig,
                                      const Real startPose[3],
                                      const Real endPose[3],
                                      const Real centerPt[2],
                                      Real radius,
                                      Integer goDir,
                                      Integer trnDir,
                                      Real checkStep,
                                      Integer dirOut,
                                      GeoCircleIntrvCheckInput_t *circleInput) {
  Integer idx0;
  Integer count0;

  *circleInput = (GeoCircleIntrvCheckInput_t){0};
  circleInput->CptX = centerPt[0];
  circleInput->CptY = centerPt[1];
  circleInput->Radius = radius;
  circleInput->Pt1X = startPose[0];
  circleInput->Pt1Y = startPose[1];
  circleInput->Pt1Theta = startPose[2];
  circleInput->Pt2X = endPose[0];
  circleInput->Pt2Y = endPose[1];
  circleInput->Pt2Theta = endPose[2];
  circleInput->GoDir = goDir;
  circleInput->TrnDir = trnDir;
  circleInput->CheckStep = checkStep;
  circleInput->SlotP1X = slot->SlotPts[0];
  circleInput->SlotP1Y = slot->SlotPts[1];
  circleInput->SlotP2X = slot->SlotPts[2];
  circleInput->SlotP2Y = slot->SlotPts[3];
  circleInput->SlotP3X = slot->SlotPts[4];
  circleInput->SlotP3Y = slot->SlotPts[5];
  circleInput->SlotP4X = slot->SlotPts[6];
  circleInput->SlotP4Y = slot->SlotPts[7];
  circleInput->SlotClass = VCC_SLOT_CLASS_PARALLEL;
  circleInput->VehWidth = vehConfig->W;
  circleInput->VehFrontOffset = vehConfig->LF;
  circleInput->VehRearOffset = vehConfig->LB;
  circleInput->UseCollisionContract = true;
  circleInput->CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  circleInput->CollisionDemand = VCC_COLLISION_DEMAND_ALL;
  circleInput->UsePlanningCheck = true;
  circleInput->ParkingOutFlag = true;
  circleInput->DirOut = dirOut;

  count0 = obsInput->BoundaryLineNum;
  if (count0 > GEO_BOUNDARY_LINE_MAX) {
    count0 = GEO_BOUNDARY_LINE_MAX;
  }
  if (count0 < 0) {
    count0 = 0;
  }
  circleInput->BoundaryLineNum = count0;
  idx0 = 0;
  while (idx0 < count0) {
    circleInput->BoundaryLineX1[idx0] = obsInput->BoundaryLineX1[idx0];
    circleInput->BoundaryLineY1[idx0] = obsInput->BoundaryLineY1[idx0];
    circleInput->BoundaryLineX2[idx0] = obsInput->BoundaryLineX2[idx0];
    circleInput->BoundaryLineY2[idx0] = obsInput->BoundaryLineY2[idx0];
    idx0 = idx0 + 1;
  }

  count0 = obsInput->VirtualBoundaryLineNum;
  if (count0 > VCC_MAX_VIRTUAL_LINES) {
    count0 = VCC_MAX_VIRTUAL_LINES;
  }
  if (count0 < 0) {
    count0 = 0;
  }
  circleInput->VirtualBoundaryLineNum = count0;
  idx0 = 0;
  while (idx0 < count0) {
    circleInput->VirtualBoundaryLineX1[idx0] = obsInput->VirtualBoundaryLineX1[idx0];
    circleInput->VirtualBoundaryLineY1[idx0] = obsInput->VirtualBoundaryLineY1[idx0];
    circleInput->VirtualBoundaryLineX2[idx0] = obsInput->VirtualBoundaryLineX2[idx0];
    circleInput->VirtualBoundaryLineY2[idx0] = obsInput->VirtualBoundaryLineY2[idx0];
    idx0 = idx0 + 1;
  }

  count0 = obsInput->BoundarySepPtsNum;
  if (count0 > VCC_MAX_SEPARATE_PTS) {
    count0 = VCC_MAX_SEPARATE_PTS;
  }
  if (count0 < 0) {
    count0 = 0;
  }
  circleInput->BoundarySepPtNum = count0;
  idx0 = 0;
  while (idx0 < count0) {
    circleInput->BoundarySepPtX[idx0] = obsInput->BoundarySepPtsX[idx0];
    circleInput->BoundarySepPtY[idx0] = obsInput->BoundarySepPtsY[idx0];
    idx0 = idx0 + 1;
  }

  count0 = obsInput->SeparateStopperNum;
  if (count0 > VCC_MAX_SEPARATE_PTS) {
    count0 = VCC_MAX_SEPARATE_PTS;
  }
  if (count0 < 0) {
    count0 = 0;
  }
  circleInput->StopperPtNum = count0;
  idx0 = 0;
  while (idx0 < count0) {
    circleInput->StopperPtX[idx0] = obsInput->SeparateStopperX[idx0];
    circleInput->StopperPtY[idx0] = obsInput->SeparateStopperY[idx0];
    idx0 = idx0 + 1;
  }

  count0 = obsInput->SlotTrackStopperNum;
  if (count0 > VCC_MAX_TRACK_STOPPER) {
    count0 = VCC_MAX_TRACK_STOPPER;
  }
  if (count0 < 0) {
    count0 = 0;
  }
  circleInput->SlotTrackStopperNum = count0;
  idx0 = 0;
  while (idx0 < count0) {
    circleInput->SlotTrackStopperX1[idx0] = obsInput->SlotTrackStopperX1[idx0];
    circleInput->SlotTrackStopperY1[idx0] = obsInput->SlotTrackStopperY1[idx0];
    circleInput->SlotTrackStopperX2[idx0] = obsInput->SlotTrackStopperX2[idx0];
    circleInput->SlotTrackStopperY2[idx0] = obsInput->SlotTrackStopperY2[idx0];
    idx0 = idx0 + 1;
  }
}

static inline Boolean
apaPkOutFtPlannerBuildLineLengthCheckInput(const ApaPkOutFtPlannerWork_t *work,
                                           const ApaSlot_t *slot,
                                           const VehicleCollisionCheckInput_t *obsInput,
                                           const ApaPlannerVehConfig_t *vehConfig,
                                           Real lineLen,
                                           Real checkStep,
                                           Integer dirOut,
                                           GeoLineIntrvCheckInput_t *lineInput,
                                           Real endPose[3]) {
  Integer lineCount0;
  Integer lineIdx0;
  Integer virtualCount0;
  Integer virtualIdx0;
  Integer sepCount0;
  Integer sepIdx0;
  Integer stopperCount0;
  Integer stopperIdx0;
  Integer slotTrackCount0;
  Integer slotTrackIdx0;
  Real dirReal0;
  Boolean built0;

  built0 = false;
  if ((work != (const ApaPkOutFtPlannerWork_t *)0) && (slot != (const ApaSlot_t *)0) &&
      (obsInput != (const VehicleCollisionCheckInput_t *)0) &&
      (vehConfig != (const ApaPlannerVehConfig_t *)0) &&
      (lineInput != (GeoLineIntrvCheckInput_t *)0) && (endPose != (Real *)0) &&
      (lineLen > APA_PARKOUT_PATH_LEN_MIN)) {
    dirReal0 = (Real)work->DirS;
    endPose[0] = work->CurrentPose[0] + dirReal0 * lineLen * cosf(work->CurrentPose[2]);
    endPose[1] = work->CurrentPose[1] + dirReal0 * lineLen * sinf(work->CurrentPose[2]);
    endPose[2] = work->CurrentPose[2];
    *lineInput = (GeoLineIntrvCheckInput_t){0};
    lineInput->Pt1X = work->CurrentPose[0];
    lineInput->Pt1Y = work->CurrentPose[1];
    lineInput->Pt1Theta = work->CurrentPose[2];
    lineInput->Pt2X = endPose[0];
    lineInput->Pt2Y = endPose[1];
    lineInput->Pt2Theta = endPose[2];
    lineInput->CheckStep = checkStep;
    lineInput->SlotP1X = slot->SlotPts[0];
    lineInput->SlotP1Y = slot->SlotPts[1];
    lineInput->SlotP2X = slot->SlotPts[2];
    lineInput->SlotP2Y = slot->SlotPts[3];
    lineInput->SlotP3X = slot->SlotPts[4];
    lineInput->SlotP3Y = slot->SlotPts[5];
    lineInput->SlotP4X = slot->SlotPts[6];
    lineInput->SlotP4Y = slot->SlotPts[7];
    lineInput->VehWidth = vehConfig->W;
    lineInput->VehFrontOffset = vehConfig->LF;
    lineInput->VehRearOffset = vehConfig->LB;
    lineInput->UsePlanningCheck = true;
    lineInput->ParkingOutFlag = true;
    lineInput->DirOut = dirOut;
    lineCount0 = obsInput->BoundaryLineNum;
    if (lineCount0 > GEO_BOUNDARY_LINE_MAX) {
      lineCount0 = GEO_BOUNDARY_LINE_MAX;
    }
    if (lineCount0 < 0) {
      lineCount0 = 0;
    }
    lineInput->BoundaryLineNum = lineCount0;
    lineIdx0 = 0;
    while (lineIdx0 < lineCount0) {
      lineInput->BoundaryLineX1[lineIdx0] = obsInput->BoundaryLineX1[lineIdx0];
      lineInput->BoundaryLineY1[lineIdx0] = obsInput->BoundaryLineY1[lineIdx0];
      lineInput->BoundaryLineX2[lineIdx0] = obsInput->BoundaryLineX2[lineIdx0];
      lineInput->BoundaryLineY2[lineIdx0] = obsInput->BoundaryLineY2[lineIdx0];
      lineIdx0 = lineIdx0 + 1;
    }
    virtualCount0 = obsInput->VirtualBoundaryLineNum;
    if (virtualCount0 > VCC_MAX_VIRTUAL_LINES) {
      virtualCount0 = VCC_MAX_VIRTUAL_LINES;
    }
    if (virtualCount0 < 0) {
      virtualCount0 = 0;
    }
    lineInput->VirtualBoundaryLineNum = virtualCount0;
    virtualIdx0 = 0;
    while (virtualIdx0 < virtualCount0) {
      lineInput->VirtualBoundaryLineX1[virtualIdx0] = obsInput->VirtualBoundaryLineX1[virtualIdx0];
      lineInput->VirtualBoundaryLineY1[virtualIdx0] = obsInput->VirtualBoundaryLineY1[virtualIdx0];
      lineInput->VirtualBoundaryLineX2[virtualIdx0] = obsInput->VirtualBoundaryLineX2[virtualIdx0];
      lineInput->VirtualBoundaryLineY2[virtualIdx0] = obsInput->VirtualBoundaryLineY2[virtualIdx0];
      virtualIdx0 = virtualIdx0 + 1;
    }
    sepCount0 = obsInput->BoundarySepPtsNum;
    if (sepCount0 > VCC_MAX_SEPARATE_PTS) {
      sepCount0 = VCC_MAX_SEPARATE_PTS;
    }
    if (sepCount0 < 0) {
      sepCount0 = 0;
    }
    lineInput->BoundarySepPtNum = sepCount0;
    sepIdx0 = 0;
    while (sepIdx0 < sepCount0) {
      lineInput->BoundarySepPtX[sepIdx0] = obsInput->BoundarySepPtsX[sepIdx0];
      lineInput->BoundarySepPtY[sepIdx0] = obsInput->BoundarySepPtsY[sepIdx0];
      sepIdx0 = sepIdx0 + 1;
    }
    stopperCount0 = obsInput->SeparateStopperNum;
    if (stopperCount0 > VCC_MAX_SEPARATE_PTS) {
      stopperCount0 = VCC_MAX_SEPARATE_PTS;
    }
    if (stopperCount0 < 0) {
      stopperCount0 = 0;
    }
    lineInput->SeparateStopperNum = stopperCount0;
    stopperIdx0 = 0;
    while (stopperIdx0 < stopperCount0) {
      lineInput->SeparateStopperX[stopperIdx0] = obsInput->SeparateStopperX[stopperIdx0];
      lineInput->SeparateStopperY[stopperIdx0] = obsInput->SeparateStopperY[stopperIdx0];
      stopperIdx0 = stopperIdx0 + 1;
    }
    slotTrackCount0 = obsInput->SlotTrackStopperNum;
    if (slotTrackCount0 > VCC_MAX_TRACK_STOPPER) {
      slotTrackCount0 = VCC_MAX_TRACK_STOPPER;
    }
    if (slotTrackCount0 < 0) {
      slotTrackCount0 = 0;
    }
    lineInput->SlotTrackStopperNum = slotTrackCount0;
    slotTrackIdx0 = 0;
    while (slotTrackIdx0 < slotTrackCount0) {
      lineInput->SlotTrackStopperX1[slotTrackIdx0] = obsInput->SlotTrackStopperX1[slotTrackIdx0];
      lineInput->SlotTrackStopperY1[slotTrackIdx0] = obsInput->SlotTrackStopperY1[slotTrackIdx0];
      lineInput->SlotTrackStopperX2[slotTrackIdx0] = obsInput->SlotTrackStopperX2[slotTrackIdx0];
      lineInput->SlotTrackStopperY2[slotTrackIdx0] = obsInput->SlotTrackStopperY2[slotTrackIdx0];
      slotTrackIdx0 = slotTrackIdx0 + 1;
    }
    built0 = true;
  }
  return built0;
}

static inline Real apaPkOutFtPlannerResolveArcTailLineCheckStep(Integer branchDir,
                                                                Integer circleIntrvFlg,
                                                                Real defaultStep) {
  Real step0;

  step0 = defaultStep;
  if ((circleIntrvFlg == 0) && (branchDir == APA_PARKOUT_BRANCH_DIR_RIGHT)) {
    step0 = 0.1f;
  }
  return step0;
}

static inline Real apaPkOutFtPlannerResolveArcTailLineShorten(Integer circleIntrvFlg,
                                                              const Real vehiclePose[3],
                                                              const Real lineStartPose[3],
                                                              Real defaultShorten) {
  Real localX0;
  Real localY0;
  Real localTheta0;
  Real shorten0;

  shorten0 = defaultShorten;
  if ((circleIntrvFlg == 0) && (vehiclePose != (const Real *)0) &&
      (lineStartPose != (const Real *)0)) {
    apaCoordGround2Veh(vehiclePose[0],
                       vehiclePose[1],
                       vehiclePose[2],
                       lineStartPose[0],
                       lineStartPose[1],
                       lineStartPose[2],
                       &localX0,
                       &localY0,
                       &localTheta0);
    (void)localX0;
    (void)localTheta0;
    shorten0 = 0.1f;
    if (fabsf(localY0) > 0.6f) {
      shorten0 = 0.6f;
    }
  }
  return shorten0;
}

static inline Boolean
apaPkOutFtPlannerTryArcProbeFromCurrent(ApaPkOutFtPlannerWork_t *work,
                                        Integer branchDir,
                                        const ApaSlot_t *slot,
                                        const VehicleCollisionCheckInput_t *obsInput,
                                        const ApaPlannerVehConfig_t *vehConfig,
                                        Real latSafeOfst,
                                        Real circleCheckStep,
                                        Integer dirOut,
                                        Integer *circleIntrvFlg) {
  GeoCircleIntrvCheckInput_t circleInput0;
  GeoCircleIntrvCheckOutput_t circleOutput0;
  Real dangerPt0[2];
  Real startPose0[3];
  Real centerPt0[2];
  Real arcEndPose0[3];
  Real avlPose0[3];
  Real arcCrashPose0[3];
  Real theta0;
  Real arcLen0;
  Integer trnDir0;
  Boolean handled0;

  handled0 = false;
  if ((work != (ApaPkOutFtPlannerWork_t *)0) && (slot != (const ApaSlot_t *)0) &&
      (obsInput != (const VehicleCollisionCheckInput_t *)0) &&
      (vehConfig != (const ApaPlannerVehConfig_t *)0)) {
    dangerPt0[0] = work->Pt1[0];
    dangerPt0[1] = work->Pt1[1];
    startPose0[0] = work->CurrentPose[0];
    startPose0[1] = work->CurrentPose[1];
    startPose0[2] = work->CurrentPose[2];
    if (apaPkOutFtPlannerResolveArcByDanger(branchDir,
                                            work->DirS,
                                            dangerPt0,
                                            startPose0,
                                            vehConfig->MinCircle,
                                            vehConfig->W,
                                            latSafeOfst,
                                            &trnDir0,
                                            centerPt0,
                                            arcEndPose0,
                                            &theta0,
                                            &arcLen0)) {
      apaPkOutFtPlannerFillCircleCheckInput(slot,
                                            obsInput,
                                            vehConfig,
                                            startPose0,
                                            arcEndPose0,
                                            centerPt0,
                                            vehConfig->MinCircle,
                                            work->DirS,
                                            trnDir0,
                                            circleCheckStep,
                                            dirOut,
                                            &circleInput0);
      circleOutput0 = (GeoCircleIntrvCheckOutput_t){0};
      geoCircleIntrvCheckPkOt(&circleInput0, &circleOutput0);
      avlPose0[0] = circleOutput0.AvlPosX;
      avlPose0[1] = circleOutput0.AvlPosY;
      avlPose0[2] = circleOutput0.AvlPosTheta;
      if (circleOutput0.IntrvFlg == 0) {
        avlPose0[0] = arcEndPose0[0];
        avlPose0[1] = arcEndPose0[1];
        avlPose0[2] = arcEndPose0[2];
        circleOutput0.SafeLen = arcLen0;
      }
      arcCrashPose0[0] = circleOutput0.CrshPosX;
      arcCrashPose0[1] = circleOutput0.CrshPosY;
      arcCrashPose0[2] = circleOutput0.CrshPosTheta;
      apaPkOutFtPlannerApplyArcProbeDecision(work,
                                             circleOutput0.IntrvFlg,
                                             work->DirS,
                                             trnDir0,
                                             vehConfig->MinCircle,
                                             theta0,
                                             circleOutput0.SafeLen,
                                             startPose0,
                                             avlPose0,
                                             centerPt0,
                                             circleOutput0.CheckCount,
                                             arcCrashPose0);
      if (circleIntrvFlg != (Integer *)0) {
        *circleIntrvFlg = circleOutput0.IntrvFlg;
      }
      handled0 = true;
    }
  }
  return handled0;
}

static inline Boolean
apaPkOutFtPlannerTryArcTailLineBranchEntryChecked(ApaPkOutFtPlannerWork_t *work,
                                                  Integer branchDir,
                                                  const GeoPkOutDangerAngleSearchOutput_t *danger,
                                                  const Real startPose[3],
                                                  const Real vehiclePose[3],
                                                  const ApaSlot_t *slot,
                                                  const VehicleCollisionCheckInput_t *obsInput,
                                                  const ApaPlannerVehConfig_t *vehConfig,
                                                  Real safeOfst,
                                                  Real latSafeOfst,
                                                  Real circleCheckStep,
                                                  Real lineCheckStep,
                                                  Integer dirOut) {
  GeoCircleIntrvCheckInput_t circleInput0;
  GeoCircleIntrvCheckOutput_t circleOutput0;
  GeoLineIntrvCheckInput_t lineInput0;
  GeoLineIntrvCheckOutput_t lineOutput0;
  Real dangerPt0[2];
  Real centerPt0[2];
  Real arcEndPose0[3];
  Real avlPose0[3];
  Real arcCrashPose0[3];
  Real lineCrashPose0[3];
  Real lineEndPose0[3];
  Real theta0;
  Real arcLen0;
  Real lineLen0;
  Real resolvedLineCheckStep0;
  Real resolvedLineShorten0;
  Integer trnDir0;
  Integer straightLenDir0;
  Integer circleIntrvFlgForLine0;
  Integer followupIntrvFlg0;
  Boolean handled0;
  Boolean canTry0;
  Boolean lineBuilt0;
  Boolean followupHandled0;
  Boolean followupLoop0;
  Boolean lineAllowed0;

  handled0 = false;
  followupIntrvFlg0 = 2;
  followupLoop0 = false;
  if ((work != (ApaPkOutFtPlannerWork_t *)0) &&
      (danger != (const GeoPkOutDangerAngleSearchOutput_t *)0) && (startPose != (const Real *)0) &&
      (vehiclePose != (const Real *)0) && (slot != (const ApaSlot_t *)0) &&
      (obsInput != (const VehicleCollisionCheckInput_t *)0) &&
      (vehConfig != (const ApaPlannerVehConfig_t *)0)) {
    canTry0 = ((work->PlanCmd != 1) && (work->PlanCmd != 5) &&
               (work->SegNumOut < APA_PKOUT_FT_PLANNER_SEG_LIMIT) && (work->DangerActive != 0) &&
               (danger->SafeSpaceFlag != 0) && (work->DirS == 1));
    if (canTry0) {
      dangerPt0[0] = work->Pt1[0];
      dangerPt0[1] = work->Pt1[1];
      if (apaPkOutFtPlannerResolveArcByDanger(branchDir,
                                              work->DirS,
                                              dangerPt0,
                                              startPose,
                                              vehConfig->MinCircle,
                                              vehConfig->W,
                                              latSafeOfst,
                                              &trnDir0,
                                              centerPt0,
                                              arcEndPose0,
                                              &theta0,
                                              &arcLen0)) {
        apaPkOutFtPlannerFillCircleCheckInput(slot,
                                              obsInput,
                                              vehConfig,
                                              startPose,
                                              arcEndPose0,
                                              centerPt0,
                                              vehConfig->MinCircle,
                                              work->DirS,
                                              trnDir0,
                                              circleCheckStep,
                                              dirOut,
                                              &circleInput0);
        circleOutput0 = (GeoCircleIntrvCheckOutput_t){0};
        geoCircleIntrvCheckPkOt(&circleInput0, &circleOutput0);
        avlPose0[0] = circleOutput0.AvlPosX;
        avlPose0[1] = circleOutput0.AvlPosY;
        avlPose0[2] = circleOutput0.AvlPosTheta;
        if (circleOutput0.IntrvFlg == 0) {
          avlPose0[0] = arcEndPose0[0];
          avlPose0[1] = arcEndPose0[1];
          avlPose0[2] = arcEndPose0[2];
          circleOutput0.SafeLen = arcLen0;
        }
        arcCrashPose0[0] = circleOutput0.CrshPosX;
        arcCrashPose0[1] = circleOutput0.CrshPosY;
        arcCrashPose0[2] = circleOutput0.CrshPosTheta;
        apaPkOutFtPlannerApplyArcProbeDecision(work,
                                               circleOutput0.IntrvFlg,
                                               work->DirS,
                                               trnDir0,
                                               vehConfig->MinCircle,
                                               theta0,
                                               circleOutput0.SafeLen,
                                               startPose,
                                               avlPose0,
                                               centerPt0,
                                               circleOutput0.CheckCount,
                                               arcCrashPose0);
        handled0 = true;
        circleIntrvFlgForLine0 = circleOutput0.IntrvFlg;
        if (work->PlanCmd == 2) {
          if (dirOut == APA_PARKOUT_BRANCH_DIR_FRONT) {
            apaPkOutFtPlannerAdvancePlanCmdTail(work);
          } else {
            followupLoop0 = true;
          }
        }
        while (followupLoop0) {
          followupLoop0 = false;
          if (work->PlanCmd == 2) {
            if (work->SegNumOut < APA_PKOUT_FT_PLANNER_SEG_LIMIT) {
              apaPkOutFtPlannerAdvancePlanCmdTail(work);
              work->PlanCmd = 0;
              followupHandled0 = apaPkOutFtPlannerTryArcProbeFromCurrent(work,
                                                                         branchDir,
                                                                         slot,
                                                                         obsInput,
                                                                         vehConfig,
                                                                         latSafeOfst,
                                                                         circleCheckStep,
                                                                         dirOut,
                                                                         &followupIntrvFlg0);
              if (followupHandled0) {
                circleIntrvFlgForLine0 = followupIntrvFlg0;
                if (work->PlanCmd == 2) {
                  followupLoop0 = true;
                }
              } else {
                work->PlanCmd = 5;
                work->FailRs = APA_PARKOUT_FAIL_RS_REVERSE_GEOMETRY_INFEASIBLE;
                work->FailRsOut[0] = APA_PARKOUT_FAIL_RS_REVERSE_GEOMETRY_INFEASIBLE;
              }
            } else {
              work->PlanCmd = 5;
              if (work->FailRs == APA_PARKOUT_FAIL_RS_NONE) {
                work->FailRs = 9;
              }
              if (work->FailRsOut[0] == APA_PARKOUT_FAIL_RS_NONE) {
                work->FailRsOut[0] = 9;
              }
            }
          }
        }
        lineAllowed0 = (work->PlanCmd != 5);
        if ((lineAllowed0) && (work->PlanCmd == 2) && (dirOut != APA_PARKOUT_BRANCH_DIR_FRONT)) {
          lineAllowed0 = false;
        }
        if (lineAllowed0) {
          straightLenDir0 = 2;
          if (branchDir == APA_PARKOUT_BRANCH_DIR_RIGHT) {
            straightLenDir0 = 1;
          }
          lineLen0 = apaParkoutCalcStraightLengthToDanger(work->Pt1[0],
                                                          work->Pt1[1],
                                                          work->CurrentPose[0],
                                                          work->CurrentPose[1],
                                                          work->CurrentPose[2],
                                                          vehiclePose[0],
                                                          vehiclePose[1],
                                                          vehiclePose[2],
                                                          safeOfst,
                                                          vehConfig->W,
                                                          straightLenDir0);
          resolvedLineCheckStep0 = apaPkOutFtPlannerResolveArcTailLineCheckStep(
              branchDir, circleIntrvFlgForLine0, lineCheckStep);
          lineBuilt0 = apaPkOutFtPlannerBuildLineLengthCheckInput(work,
                                                                  slot,
                                                                  obsInput,
                                                                  vehConfig,
                                                                  lineLen0,
                                                                  resolvedLineCheckStep0,
                                                                  dirOut,
                                                                  &lineInput0,
                                                                  lineEndPose0);
          if (lineBuilt0) {
            lineOutput0 = (GeoLineIntrvCheckOutput_t){0};
            geoLineIntrvCheckPkOt(&lineInput0, &lineOutput0);
            lineCrashPose0[0] = lineOutput0.CrshPosX;
            lineCrashPose0[1] = lineOutput0.CrshPosY;
            lineCrashPose0[2] = lineOutput0.CrshPosTheta;
            resolvedLineShorten0 = apaPkOutFtPlannerResolveArcTailLineShorten(
                circleIntrvFlgForLine0, vehiclePose, work->CurrentPose, 0.5f);
            apaPkOutFtPlannerApplyLineProbeDecision(work,
                                                    lineOutput0.IntrvFlg,
                                                    work->DirS,
                                                    lineLen0,
                                                    lineOutput0.SafeLen,
                                                    resolvedLineShorten0,
                                                    work->CurrentPose,
                                                    lineEndPose0,
                                                    lineOutput0.CheckCount,
                                                    lineCrashPose0);
          } else {
            work->PlanCmd = 1;
            work->TarPt[0] = work->CurrentPose[0];
            work->TarPt[1] = work->CurrentPose[1];
            work->TarPt[2] = work->CurrentPose[2];
          }
        }
      }
    }
  }
  return handled0;
}

static inline Boolean
apaParkoutCanUseFtPlannerDangerCandidate(const ApaParallelParkoutContext_t *ctx,
                                         const VehicleCollisionCheckInput_t *obsWork,
                                         const GeoPkOutDangerAngleSearchOutput_t *danger,
                                         const ApaPkOutFtPlannerWork_t *seedWork) {
  Integer branchDir0;
  Integer geometryBranchDir0;
  Boolean canUse0;
  Boolean noExplicitObstacle0;
  Boolean explicitObstacle0;
  Boolean noBackAdapter0;
  Boolean branchLeftOrRight0;
  Boolean forwardLineObstacleSupported0;
  Boolean dangerPointDirect0;
  Boolean dangerPointDefault0;
  Boolean dangerPointSupported0;
  Boolean seedStatusValid0;
  Boolean seedDangerActive0;

  canUse0 = false;
  if ((ctx != (const ApaParallelParkoutContext_t *)0) &&
      (obsWork != (const VehicleCollisionCheckInput_t *)0) &&
      (danger != (const GeoPkOutDangerAngleSearchOutput_t *)0)) {
    noExplicitObstacle0 =
        ((obsWork->BoundaryLineNum == 0) && (obsWork->BoundarySepPtsNum <= 1) &&
         (obsWork->SlotTrackStopperNum == 0) && (obsWork->SeparateStopperNum == 0) &&
         (obsWork->VirtualBoundaryLineNum == 0));
    explicitObstacle0 = !noExplicitObstacle0;
    noBackAdapter0 = ((ctx->BackEnable == 0) && (ctx->MebDir == 0));
    branchDir0 = apaParkoutResolveBranchDir(ctx);
    geometryBranchDir0 = apaParkoutResolveGeometryBranchDir(ctx, branchDir0);
    branchLeftOrRight0 = apaParkoutBranchIsLeftOrRight(geometryBranchDir0);
    forwardLineObstacleSupported0 =
        ((obsWork->BoundaryLineNum >= 0) && (obsWork->BoundaryLineNum <= GEO_BOUNDARY_LINE_MAX) &&
         (obsWork->BoundarySepPtsNum >= 0) && (obsWork->BoundarySepPtsNum <= 2) &&
         (obsWork->SlotTrackStopperNum >= 0) &&
         (obsWork->SlotTrackStopperNum <= VCC_MAX_TRACK_STOPPER) &&
         (obsWork->SeparateStopperNum >= 0) &&
         (obsWork->SeparateStopperNum <= VCC_MAX_SEPARATE_PTS) &&
         (obsWork->VirtualBoundaryLineNum >= 0) &&
         (obsWork->VirtualBoundaryLineNum <= VCC_MAX_VIRTUAL_LINES) &&
         ((obsWork->BoundaryLineNum > 0) || (obsWork->SlotTrackStopperNum > 0) ||
          (obsWork->VirtualBoundaryLineNum > 0)));
    dangerPointDirect0 =
        ((danger->PtEx == 1) || ((danger->PtEx == 2) && (obsWork->VirtualBoundaryLineNum > 0)));
    seedStatusValid0 = ((seedWork != (const ApaPkOutFtPlannerWork_t *)0) &&
                        (seedWork->PlanCmd != 5) && (seedWork->FailRs != 9));
    seedDangerActive0 =
        ((seedWork != (const ApaPkOutFtPlannerWork_t *)0) && (seedWork->DangerActive != 0));
    dangerPointDefault0 =
        ((danger->PtEx == 0) && seedDangerActive0 && (branchDir0 == APA_PARKOUT_BRANCH_DIR_FRONT));
    dangerPointSupported0 = (dangerPointDirect0 || dangerPointDefault0);
    if (noExplicitObstacle0) {
      if (seedStatusValid0 && (danger->SafeSpaceFlag != 0) &&
          ((danger->PtEx == 1) || (danger->PtEx == 0))) {
        canUse0 = true;
      }
    } else if (noBackAdapter0 && explicitObstacle0 && branchLeftOrRight0 &&
               forwardLineObstacleSupported0 && seedStatusValid0 && dangerPointSupported0 &&
               (danger->SafeSpaceFlag != 0)) {
      canUse0 = true;
    }
  }
  return canUse0;
}

static inline void
apaParkoutApplyForwardLineDefaultDanger(ApaPkOutFtPlannerWork_t *work,
                                        Integer geometryBranchDir,
                                        const Real startPose[3],
                                        const ApaPlannerVehConfig_t *vehConfig,
                                        const GeoPkOutDangerAngleSearchOutput_t *danger,
                                        VehicleCollisionCheckInput_t *obsWork) {
  Real dangerPtX0;
  Real dangerPtY0;
  Real dangerPtTheta0;
  Real dangerPtLocalY0;
  Integer seedIdx0;
  Boolean noExplicitObstacle0;
  Boolean canApply0;

  if ((obsWork != (VehicleCollisionCheckInput_t *)0)) {
    noExplicitObstacle0 =
        ((obsWork->BoundaryLineNum == 0) && (obsWork->BoundarySepPtsNum <= 1) &&
         (obsWork->SlotTrackStopperNum == 0) && (obsWork->SeparateStopperNum == 0) &&
         (obsWork->VirtualBoundaryLineNum == 0));
  } else {
    noExplicitObstacle0 = true;
  }
  canApply0 = ((work != (ApaPkOutFtPlannerWork_t *)0) && (startPose != (const Real *)0) &&
               (vehConfig != (const ApaPlannerVehConfig_t *)0) &&
               (danger != (const GeoPkOutDangerAngleSearchOutput_t *)0) &&
               (obsWork != (VehicleCollisionCheckInput_t *)0) && (!noExplicitObstacle0) &&
               (danger->PtEx == 0) && (work->DangerActive != 0) &&
               ((geometryBranchDir == APA_PARKOUT_BRANCH_DIR_LEFT) ||
                (geometryBranchDir == APA_PARKOUT_BRANCH_DIR_RIGHT)));
  if (canApply0) {
    (void)vehConfig;
    dangerPtLocalY0 = 0.8f;
    if (geometryBranchDir == APA_PARKOUT_BRANCH_DIR_RIGHT) {
      dangerPtLocalY0 = -0.8f;
    }
    apaCoordVeh2Ground(startPose[0],
                       startPose[1],
                       startPose[2],
                       4.5f,
                       dangerPtLocalY0,
                       0.0f,
                       &dangerPtX0,
                       &dangerPtY0,
                       &dangerPtTheta0);
    (void)dangerPtTheta0;
    work->Pt1[0] = dangerPtX0;
    work->Pt1[1] = dangerPtY0;
    seedIdx0 = obsWork->BoundarySepPtsNum - 1;
    if ((seedIdx0 >= 0) && (seedIdx0 < VCC_MAX_SEPARATE_PTS)) {
      obsWork->BoundarySepPtsX[seedIdx0] = dangerPtX0;
      obsWork->BoundarySepPtsY[seedIdx0] = dangerPtY0;
    }
  }
}

static inline Boolean
apaParkoutCanUseFtPlannerArcTailCandidate(const ApaParallelParkoutContext_t *ctx,
                                          const VehicleCollisionCheckInput_t *obsWork,
                                          const GeoPkOutDangerAngleSearchOutput_t *danger,
                                          const ApaPkOutFtPlannerWork_t *seedWork) {
  Integer branchDir0;
  Integer geometryBranchDir0;
  Boolean canUse0;
  Boolean noExplicitObstacle0;
  Boolean explicitObstacle0;
  Boolean noBackAdapter0;
  Boolean backAdapterCompatible0;
  Boolean branchLeftOrRight0;
  Boolean seededCountsValid0;
  Boolean seedDangerActive0;
  Boolean directDangerSupported0;
  Boolean defaultDangerSupported0;
  Boolean arcTailDangerSupported0;

  canUse0 = false;
  if ((ctx != (const ApaParallelParkoutContext_t *)0) &&
      (obsWork != (const VehicleCollisionCheckInput_t *)0) &&
      (danger != (const GeoPkOutDangerAngleSearchOutput_t *)0)) {
    noExplicitObstacle0 =
        ((obsWork->BoundaryLineNum == 0) && (obsWork->BoundarySepPtsNum <= 1) &&
         (obsWork->SlotTrackStopperNum == 0) && (obsWork->SeparateStopperNum == 0) &&
         (obsWork->VirtualBoundaryLineNum == 0));
    explicitObstacle0 = !noExplicitObstacle0;
    noBackAdapter0 = ((ctx->BackEnable == 0) && (ctx->MebDir == 0));
    branchDir0 = apaParkoutResolveBranchDir(ctx);
    geometryBranchDir0 = apaParkoutResolveGeometryBranchDir(ctx, branchDir0);
    branchLeftOrRight0 = apaParkoutBranchIsLeftOrRight(geometryBranchDir0);
    backAdapterCompatible0 = (noBackAdapter0 || (branchDir0 != APA_PARKOUT_BRANCH_DIR_FRONT));
    seededCountsValid0 =
        ((obsWork->BoundaryLineNum >= 0) && (obsWork->BoundaryLineNum <= VCC_MAX_BOUNDARY_LINES) &&
         (obsWork->VirtualBoundaryLineNum >= 0) &&
         (obsWork->VirtualBoundaryLineNum <= VCC_MAX_VIRTUAL_LINES) &&
         (obsWork->BoundarySepPtsNum > 0) && (obsWork->BoundarySepPtsNum <= VCC_MAX_SEPARATE_PTS) &&
         (obsWork->SeparateStopperNum >= 0) &&
         (obsWork->SeparateStopperNum <= VCC_MAX_SEPARATE_PTS) &&
         (obsWork->SlotTrackStopperNum >= 0) &&
         (obsWork->SlotTrackStopperNum <= VCC_MAX_TRACK_STOPPER));
    seedDangerActive0 =
        ((seedWork != (const ApaPkOutFtPlannerWork_t *)0) && (seedWork->PlanCmd != 5) &&
         (seedWork->FailRs != 9) && (seedWork->DangerActive != 0));
    directDangerSupported0 =
        (seedDangerActive0 &&
         ((danger->PtEx == 1) || ((danger->PtEx == 2) && (obsWork->VirtualBoundaryLineNum > 0))));
    defaultDangerSupported0 = (seedDangerActive0 && (danger->PtEx == 0));
    arcTailDangerSupported0 = (directDangerSupported0 || defaultDangerSupported0);

    if (noExplicitObstacle0 &&
        apaParkoutCanUseFtPlannerDangerCandidate(ctx, obsWork, danger, seedWork)) {
      canUse0 = true;
    } else if (explicitObstacle0 && backAdapterCompatible0 && branchLeftOrRight0 &&
               seededCountsValid0 && arcTailDangerSupported0 && (danger->SafeSpaceFlag != 0)) {
      canUse0 = true;
    }
  }
  return canUse0;
}

static inline Boolean
apaParkoutTryFtPlannerForwardLineCandidate(const ApaParallelParkoutContext_t *ctx,
                                           const VehicleCollisionCheckInput_t *obsWork,
                                           const ApaPlannerVehConfig_t *vehConfig,
                                           const Real startPose[3],
                                           const GeoPkOutDangerAngleSearchOutput_t *danger,
                                           const ApaPkOutFtPlannerWork_t *seedWork,
                                           ApaParallelParkoutDiag_t *diag,
                                           ApaPath_t *candidatePath,
                                           Boolean *handledFailure) {
  ApaPkOutFtPlannerWork_t work;
  VehicleCollisionCheckInput_t candidateObsWork;
  Integer requestedBranchDir;
  Integer branchDir;
  Integer dirOut;
  Boolean handled;
  Boolean success;

  success = false;
  if (handledFailure != (Boolean *)0) {
    *handledFailure = false;
  }
  apaPathInit(candidatePath);
  if ((ctx != (const ApaParallelParkoutContext_t *)0) &&
      (obsWork != (const VehicleCollisionCheckInput_t *)0) &&
      (vehConfig != (const ApaPlannerVehConfig_t *)0) && (startPose != (const Real *)0) &&
      (danger != (const GeoPkOutDangerAngleSearchOutput_t *)0) &&
      (seedWork != (const ApaPkOutFtPlannerWork_t *)0) &&
      apaParkoutCanUseFtPlannerDangerCandidate(ctx, obsWork, danger, seedWork)) {
    work = *seedWork;
    work.SampleStep = ctx->Config.SampleStep;
    candidateObsWork = *obsWork;
    requestedBranchDir = apaParkoutResolveBranchDir(ctx);
    branchDir = apaParkoutResolveGeometryBranchDir(ctx, requestedBranchDir);
    dirOut = apaParkoutResolveFtPlannerDirOut(requestedBranchDir, branchDir);
    if (requestedBranchDir == APA_PARKOUT_BRANCH_DIR_FRONT) {
      apaParkoutApplyForwardLineDefaultDanger(
          &work, branchDir, startPose, vehConfig, danger, &candidateObsWork);
    }
    handled = apaPkOutFtPlannerTryForwardLineBranchEntryChecked(&work,
                                                                branchDir,
                                                                danger,
                                                                startPose,
                                                                ctx->StartPos,
                                                                &ctx->Slot,
                                                                &candidateObsWork,
                                                                vehConfig,
                                                                work.SafeOfst,
                                                                APA_PKOUT_FT_LATERAL_SAFE_OFFSET,
                                                                0.8f,
                                                                dirOut);
    if (handled && (work.PlanCmd == 1)) {
      apaPkOutFtPlannerFinalizeOutput(&work);
      success = apaPkOutFtPlannerBuildPathFromWork(&work, candidatePath);
      if (success) {
        candidatePath->PathSource = APA_PATH_SOURCE_GEO;
        candidatePath->SegNum = apaPathCalcSegNum(candidatePath);
        candidatePath->SegNumFlag = true;
        if (diag != (ApaParallelParkoutDiag_t *)0) {
          diag->CheckCount = diag->CheckCount + work.PlnChkCnt;
          diag->FailReason = APA_FAULT_NO_ERR;
          diag->PkOutFailRs = APA_PARKOUT_FAIL_RS_NONE;
        }
      }
    } else if (handled && (work.PlanCmd == 5)) {
      if (diag != (ApaParallelParkoutDiag_t *)0) {
        diag->CheckCount = diag->CheckCount + work.PlnChkCnt;
        if (work.FailRsOut[0] != APA_PARKOUT_FAIL_RS_NONE) {
          apaParkoutDiagSetFailRs(diag, work.FailRsOut[0]);
        }
        if (diag->FailReason == APA_FAULT_NO_ERR) {
          diag->FailReason = APA_FAULT_PKING_OUT_UNENABLE;
        }
        if (diag->PkOutFailRs == APA_PARKOUT_FAIL_RS_NONE) {
          apaParkoutDiagSetFailRs(diag, APA_PARKOUT_FAIL_RS_FORWARD_DANGER_BLOCKED);
        }
      }
      if (handledFailure != (Boolean *)0) {
        *handledFailure = true;
      }
    }
  }
  return success;
}

static inline Boolean
apaParkoutTryFtPlannerArcTailLineCandidate(const ApaParallelParkoutContext_t *ctx,
                                           const VehicleCollisionCheckInput_t *obsWork,
                                           const ApaPlannerVehConfig_t *vehConfig,
                                           const Real startPose[3],
                                           const GeoPkOutDangerAngleSearchOutput_t *danger,
                                           const ApaPkOutFtPlannerWork_t *seedWork,
                                           ApaParallelParkoutDiag_t *diag,
                                           ApaPath_t *candidatePath,
                                           Boolean *handledFailure,
                                           Boolean *allowRuntimeFallback) {
  ApaPkOutFtPlannerWork_t work;
  Integer requestedBranchDir;
  Integer branchDir;
  Integer dirOut;
  Boolean handled;
  Boolean success;

  success = false;
  if (handledFailure != (Boolean *)0) {
    *handledFailure = false;
  }
  if (allowRuntimeFallback != (Boolean *)0) {
    *allowRuntimeFallback = false;
  }
  apaPathInit(candidatePath);
  if ((ctx != (const ApaParallelParkoutContext_t *)0) &&
      (obsWork != (const VehicleCollisionCheckInput_t *)0) &&
      (vehConfig != (const ApaPlannerVehConfig_t *)0) && (startPose != (const Real *)0) &&
      (danger != (const GeoPkOutDangerAngleSearchOutput_t *)0) &&
      (seedWork != (const ApaPkOutFtPlannerWork_t *)0) &&
      apaParkoutCanUseFtPlannerArcTailCandidate(ctx, obsWork, danger, seedWork)) {
    work = *seedWork;
    work.SampleStep = ctx->Config.SampleStep;
    requestedBranchDir = apaParkoutResolveBranchDir(ctx);
    branchDir = apaParkoutResolveGeometryBranchDir(ctx, requestedBranchDir);
    dirOut = apaParkoutResolveFtPlannerDirOut(requestedBranchDir, branchDir);
    handled = apaPkOutFtPlannerTryArcTailLineBranchEntryChecked(&work,
                                                                branchDir,
                                                                danger,
                                                                startPose,
                                                                ctx->StartPos,
                                                                &ctx->Slot,
                                                                obsWork,
                                                                vehConfig,
                                                                work.SafeOfst,
                                                                APA_PKOUT_FT_LATERAL_SAFE_OFFSET,
                                                                0.15f,
                                                                0.8f,
                                                                dirOut);
    if (handled && (work.PlanCmd == 1)) {
      apaPkOutFtPlannerFinalizeOutput(&work);
      success = apaPkOutFtPlannerBuildPathFromWork(&work, candidatePath);
      if (success) {
        candidatePath->PathSource = APA_PATH_SOURCE_GEO;
        candidatePath->SegNum = apaPathCalcSegNum(candidatePath);
        candidatePath->SegNumFlag = true;
        if (diag != (ApaParallelParkoutDiag_t *)0) {
          diag->CheckCount = diag->CheckCount + work.PlnChkCnt;
          diag->FailReason = APA_FAULT_NO_ERR;
          diag->PkOutFailRs = APA_PARKOUT_FAIL_RS_NONE;
        }
      }
    } else if (handled && (work.PlanCmd == 5) && (diag != (ApaParallelParkoutDiag_t *)0)) {
      diag->CheckCount = diag->CheckCount + work.PlnChkCnt;
      if (work.FailRsOut[0] != APA_PARKOUT_FAIL_RS_NONE) {
        apaParkoutDiagSetFailRs(diag, work.FailRsOut[0]);
      }
      if (handledFailure != (Boolean *)0) {
        *handledFailure = true;
      }
      if (allowRuntimeFallback != (Boolean *)0) {
        *allowRuntimeFallback = false;
      }
    }
  }
  return success;
}

static inline Boolean apaParkoutShouldAdjustPath(const ApaParallelParkoutContext_t *ctx,
                                                 const ApaPlannerVehConfig_t *vehConfig,
                                                 const Real startPose[3],
                                                 const ApaPath_t *totalPath) {
  ApaVehicleCollisionCheckContext_t slotContext;
  Boolean vehInSlotFlag;
  Boolean adjustPath0;

  if (totalPath->PathN <= 0) {
    adjustPath0 = false;
  } else {
    apaVehicleCollisionCheckContextInit(&slotContext);
    apaVehicleCollisionCheckContextSetSlot(&slotContext, &ctx->Slot);
    vehInSlotFlag = apaVehicleCollisionCalcVehInSlotFlag(
        vehConfig, &slotContext, startPose[0], startPose[1], startPose[2]);
    adjustPath0 = !vehInSlotFlag;
  }
  return adjustPath0;
}

static inline void apaParkoutApplyFinalAdjustments(const ApaParallelParkoutContext_t *ctx,
                                                   const VehicleCollisionCheckInput_t *obsInput,
                                                   const ApaPlannerVehConfig_t *vehConfig,
                                                   const Real startPose[3],
                                                   ApaPath_t *totalPath,
                                                   ApaPath_t *segPath,
                                                   Real currentPose[3],
                                                   ApaParallelParkoutDiag_t *diag) {
  ApaPath_t preAdjustPath;
  Boolean adjustPath;
  Boolean successArc;
  Integer lastIdx;

  apaPathCopy(&preAdjustPath, totalPath);
  adjustPath = apaParkoutShouldAdjustPath(ctx, vehConfig, startPose, totalPath);

  if (adjustPath) {
    lastIdx = totalPath->PathN - 1;
    currentPose[0] = totalPath->X[lastIdx];
    currentPose[1] = totalPath->Y[lastIdx];
    currentPose[2] = totalPath->Tht[lastIdx];

    successArc = apaParkoutGenArcSegment(
        ctx, obsInput, currentPose, 1, ctx->SlotOnTheRight, 1.5f, segPath, diag);
    if (successArc) {
      successArc = apaParkoutAppendPathChecked(totalPath, segPath);
    }
    if (!successArc) {
      adjustPath = false;
    }
  }

  if ((adjustPath) && (totalPath->PathN <= 0)) {
    adjustPath = false;
  }

  if (adjustPath) {
    lastIdx = totalPath->PathN - 1;
    currentPose[0] = totalPath->X[lastIdx];
    currentPose[1] = totalPath->Y[lastIdx];
    currentPose[2] = totalPath->Tht[lastIdx];

    successArc = apaParkoutGenArcSegment(
        ctx, obsInput, currentPose, 1, !ctx->SlotOnTheRight, 1.0f, segPath, diag);
    if (successArc) {
      successArc = apaParkoutAppendPathChecked(totalPath, segPath);
    }
    if (!successArc) {
      adjustPath = false;
    }
  }

  if (!adjustPath) {
    apaPathCopy(totalPath, &preAdjustPath);
  }
}

static inline void apaParallelParkoutPlanLoopBody(const ApaParallelParkoutContext_t *ctx,
                                                  const VehicleCollisionCheckInput_t *obsInput,
                                                  const ApaPlannerVehConfig_t *vehConfig,
                                                  Integer loop,
                                                  Integer loopLimit,
                                                  Real *currentPose,
                                                  ApaPath_t *totalPath,
                                                  ApaPath_t *segPath,
                                                  ApaParallelParkoutDiag_t *diag,
                                                  Boolean *isVertexOut,
                                                  Boolean *success) {
  Boolean successFwd;
  Boolean turnLeftFwd;
  Boolean vertexOutFound;
  Boolean appendOk;

  if (loop > loopLimit) {
    *success = false;
  }

  if (*success) {
    if (totalPath->PathN > 0) {
      Integer lastIdx = totalPath->PathN - 1;
      currentPose[0] = totalPath->X[lastIdx];
      currentPose[1] = totalPath->Y[lastIdx];
      currentPose[2] = totalPath->Tht[lastIdx];
    }

    turnLeftFwd = ctx->SlotOnTheRight;
    successFwd = apaParkoutGenManeuverSegment(
        ctx, obsInput, vehConfig, currentPose, 1, turnLeftFwd, segPath, diag, &vertexOutFound);
    if (successFwd) {
      appendOk = apaParkoutAppendPathChecked(totalPath, segPath);
      if (appendOk) {
        *isVertexOut = vertexOutFound;
      } else {
        *success = false;
        *isVertexOut = false;
      }
    }

    if (!(*isVertexOut)) {
      Boolean successBwd;
      Boolean turnLeftBwd;
      Boolean dummyVertexOut;

      if (totalPath->PathN > 0) {
        Integer lastIdx = totalPath->PathN - 1;
        currentPose[0] = totalPath->X[lastIdx];
        currentPose[1] = totalPath->Y[lastIdx];
        currentPose[2] = totalPath->Tht[lastIdx];
      }

      turnLeftBwd = !ctx->SlotOnTheRight;
      successBwd = apaParkoutGenManeuverSegment(
          ctx, obsInput, vehConfig, currentPose, -1, turnLeftBwd, segPath, diag, &dummyVertexOut);
      if (successBwd) {
        appendOk = apaParkoutAppendPathChecked(totalPath, segPath);
        if (!appendOk) {
          *success = false;
        }
      }

      if (!successFwd && !successBwd) {
        *success = false;
      }
    }
  }
}

static inline Boolean apaParallelParkoutPlanAttempt(const ApaParallelParkoutContext_t *ctx,
                                                    const VehicleCollisionCheckInput_t *obsInput,
                                                    const ApaPlannerVehConfig_t *vehConfig,
                                                    const Real startPose[3],
                                                    ApaParallelParkoutDiag_t *diag,
                                                    ApaPath_t *attemptPath,
                                                    Boolean *handledFailure) {
  ApaPath_t totalPath;
  ApaPath_t segPath;
  VehicleCollisionCheckInput_t obsWork;
  Real currentPose[3];
  Boolean isVertexOut;
  Integer loop;
  Integer loopLimit;
  Boolean success;
  GeoPkOutDangerAngleSearchOutput_t ftPlannerDanger;
  ApaPkOutFtPlannerWork_t ftPlannerSeedWork;
  Boolean ft_plannerSuccess;
  Boolean ft_plannerEnabled;
  Boolean ft_plannerArcTailHandledFailure;
  Boolean ft_plannerArcTailAllowFallback;
  Boolean ft_plannerForwardLineHandledFailure;
  Integer ft_plannerFailReasonBaseline;
  Integer ft_plannerPkOutFailRsBaseline;
  Boolean loopStarted;
  Boolean terminal0;

  if (handledFailure != (Boolean *)0) {
    *handledFailure = false;
  }
  apaParallelParkoutDiagInit(diag);
  apaPathInit(&totalPath);
  apaPathInit(attemptPath);
  obsWork = *obsInput;
  apaParkoutApplyObsSlightAdjustment(ctx, startPose, &obsWork);
  ftPlannerDanger = (GeoPkOutDangerAngleSearchOutput_t){0};
  apaPkOutFtPlannerWorkInit(&ftPlannerSeedWork);
  ft_plannerEnabled = (ctx->Config.DisableFtPlannerCandidates == 0);
  success = apaParkoutCheckInitialSafeSpaceWithFtPlanner(
      ctx, &obsWork, vehConfig, startPose, diag, &ftPlannerDanger, &ftPlannerSeedWork);

  currentPose[0] = startPose[0];
  currentPose[1] = startPose[1];
  currentPose[2] = startPose[2];

  ft_plannerSuccess = false;
  ft_plannerArcTailHandledFailure = false;
  ft_plannerArcTailAllowFallback = false;
  ft_plannerForwardLineHandledFailure = false;
  ft_plannerFailReasonBaseline = APA_FAULT_NO_ERR;
  ft_plannerPkOutFailRsBaseline = APA_PARKOUT_FAIL_RS_NONE;
  terminal0 = false;
  if (success && (diag != (ApaParallelParkoutDiag_t *)0)) {
    ft_plannerFailReasonBaseline = diag->FailReason;
    ft_plannerPkOutFailRsBaseline = diag->PkOutFailRs;
  }
  if (success && ft_plannerEnabled) {
    ft_plannerSuccess =
        apaParkoutTryFtPlannerForwardLineCandidate(ctx,
                                                   &obsWork,
                                                   vehConfig,
                                                   startPose,
                                                   &ftPlannerDanger,
                                                   &ftPlannerSeedWork,
                                                   diag,
                                                   attemptPath,
                                                   &ft_plannerForwardLineHandledFailure);
    if ((!ft_plannerSuccess) && (!ft_plannerForwardLineHandledFailure)) {
      ft_plannerSuccess =
          apaParkoutTryFtPlannerArcTailLineCandidate(ctx,
                                                     &obsWork,
                                                     vehConfig,
                                                     startPose,
                                                     &ftPlannerDanger,
                                                     &ftPlannerSeedWork,
                                                     diag,
                                                     attemptPath,
                                                     &ft_plannerArcTailHandledFailure,
                                                     &ft_plannerArcTailAllowFallback);
    }
  }
  if (ft_plannerSuccess) {
    success = true;
    terminal0 = true;
  } else if (ft_plannerForwardLineHandledFailure) {
    if (handledFailure != (Boolean *)0) {
      *handledFailure = true;
    }
    success = false;
    terminal0 = true;
  }
  if ((!terminal0) && ft_plannerArcTailHandledFailure) {
    ft_plannerArcTailAllowFallback =
        (ft_plannerArcTailAllowFallback &&
         (apaParkoutResolveBranchDir(ctx) == APA_PARKOUT_BRANCH_DIR_FRONT));
    if (ft_plannerArcTailAllowFallback) {
      ft_plannerArcTailHandledFailure = false;
      if (handledFailure != (Boolean *)0) {
        *handledFailure = false;
      }
      if (diag != (ApaParallelParkoutDiag_t *)0) {
        diag->FailReason = ft_plannerFailReasonBaseline;
        diag->PkOutFailRs = ft_plannerPkOutFailRsBaseline;
      }
    } else {
      if (handledFailure != (Boolean *)0) {
        *handledFailure = true;
      }
      if (diag != (ApaParallelParkoutDiag_t *)0) {
        if (diag->FailReason == APA_FAULT_NO_ERR) {
          diag->FailReason = APA_FAULT_PKING_OUT_UNENABLE;
        }
        if (diag->PkOutFailRs == APA_PARKOUT_FAIL_RS_NONE) {
          apaParkoutDiagSetFailRs(diag, APA_PARKOUT_FAIL_RS_FORWARD_DANGER_BLOCKED);
        }
      }
      success = false;
      terminal0 = true;
    }
  }

  if (!terminal0) {

    apaParkoutTryPreBackSegment(
        ctx, &obsWork, vehConfig, startPose, &totalPath, &segPath, currentPose, diag, &success);

    isVertexOut = false;
    loop = 0;
    loopLimit = 6;
    loopStarted = success;
    while ((!isVertexOut) && success) {
      loop = loop + 1;
      apaParallelParkoutPlanLoopBody(ctx,
                                     &obsWork,
                                     vehConfig,
                                     loop,
                                     loopLimit,
                                     currentPose,
                                     &totalPath,
                                     &segPath,
                                     diag,
                                     &isVertexOut,
                                     &success);
    }

    if (!isVertexOut) {
      success = false;
      apaParkoutDiagSetFailRs(diag, APA_PARKOUT_FAIL_RS_REVERSE_GEOMETRY_INFEASIBLE);
      if (loopStarted && (diag->FailReason == APA_FAULT_NO_ERR) &&
          (diag->CollisionStage != APA_PARKOUT_COLLISION_STAGE_INITIAL_STRAIGHT)) {
        diag->FailReason = APA_FAULT_PKING_OUT_UNENABLE;
      }
    }

    apaParkoutApplyFinalAdjustments(
        ctx, &obsWork, vehConfig, startPose, &totalPath, &segPath, currentPose, diag);

    if (success && totalPath.ValidFlag && (totalPath.PathN >= 2)) {

      totalPath.PathType = APA_PATH_TYPE_ALL;
      totalPath.PathSource = APA_PATH_SOURCE_SEARCH;
      totalPath.SegNum = apaPathCalcSegNum(&totalPath);
      totalPath.SegNumFlag = true;

      apaPathCopy(attemptPath, &totalPath);
    } else {
      if ((diag->FailReason == APA_FAULT_NO_ERR) && (diag->CollisionType != VCC_COLLISION_NONE)) {
        diag->FailReason = APA_FAULT_GEO_PATH_REVERSE_COLLISION;
      }
      if (diag->FailReason == APA_FAULT_NO_ERR) {
        diag->FailReason = APA_FAULT_PKING_OUT_UNENABLE;
      }
      success = false;
    }
  }
  return success;
}

Boolean apaParallelParkoutPlanWithDiag(const ApaParallelParkoutContext_t *ctx,
                                       const VehicleCollisionCheckInput_t *obsInput,
                                       const ApaPlannerVehConfig_t *vehConfig,
                                       ApaParallelParkoutDiag_t *diag,
                                       ApaComplexPath_t *outPath) {
  ApaParallelParkoutDiag_t primaryDiag;
  ApaParallelParkoutDiag_t retryDiag;
  ApaPath_t primaryPath;
  ApaPath_t retryPath;
  Real primaryStart[3];
  Real retryStart[3];
  Boolean success;
  Boolean retrySuccess;
  Boolean primaryHandledFailure;
  Boolean retryHandledFailure;

  apaCPathInit(outPath);
  apaParallelParkoutDiagInit(diag);

  primaryStart[0] = ctx->StartPos[0];
  primaryStart[1] = ctx->StartPos[1];
  primaryStart[2] = ctx->StartPos[2];

  success = apaParallelParkoutPlanAttempt(
      ctx, obsInput, vehConfig, primaryStart, &primaryDiag, &primaryPath, &primaryHandledFailure);
  *diag = primaryDiag;
  if ((!success) && (!primaryHandledFailure)) {
    apaParkoutBuildRetryStart(ctx, primaryStart, retryStart);
    retrySuccess = apaParallelParkoutPlanAttempt(
        ctx, obsInput, vehConfig, retryStart, &retryDiag, &retryPath, &retryHandledFailure);
    if (retrySuccess) {
      success = true;
      *diag = retryDiag;
      apaPathCopy(&primaryPath, &retryPath);
    }
  }

  if (success) {
    apaPathCopy(&outPath->Path, &primaryPath);
    outPath->SlotId = 1;
    outPath->SlotStatus = APA_SLOT_STATUS_FINISH_PLAN;
  } else {
    apaCPathInit(outPath);
    outPath->Path.ErrorType = diag->FailReason;
    if ((diag->FailReason == APA_FAULT_PKING_OUT_UNENABLE) ||
        (diag->CollisionType != VCC_COLLISION_NONE)) {
      outPath->Path.CollisionFlag = APA_COLLISION_RISK_EXIST;
      outPath->CollisionCnt = diag->CheckCount;
      if (outPath->CollisionCnt <= 0) {
        outPath->CollisionCnt = 1;
      }
    }
  }

  return success;
}

/**
 * @brief APA 平行泊出搜索的主控制器
 * @en_name apaParallelParkoutPlan
 * @cn_name APA 平行泊出搜索的主控制器
 * @type module
 * @param[IN] ApaParallelParkoutContext_t Ctx 平行泊出规划上下文
 * @param[IN] VehicleCollisionCheckInput_t ObsInput 障碍与碰撞检测输入
 * @param[IN] ApaPlannerVehConfig_t VehConfig 车辆规划参数
 * @param[OUT] ApaComplexPath_t OutPath 平行泊出路径输出
 * @retval Boolean Success 平行泊出路径生成是否成功
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 综合处理
 * @source matlab_rewrite geo_PkOutFt_PathPlanning.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
Boolean apaParallelParkoutPlan(const ApaParallelParkoutContext_t *ctx,
                               const VehicleCollisionCheckInput_t *obsInput,
                               const ApaPlannerVehConfig_t *vehConfig,
                               ApaComplexPath_t *outPath) {
  ApaParallelParkoutDiag_t diag;
  Boolean success;

  success = apaParallelParkoutPlanWithDiag(ctx, obsInput, vehConfig, &diag, outPath);
  return success;
}
