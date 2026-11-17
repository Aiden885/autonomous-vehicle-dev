#include "apaCoordUtils.h"
#include "apaVehicleDefaults.h"
#include "geoCircleIntrvCheckTypes.h"
#include <math.h>

#ifndef GEO_CIRCLE_DEFAULT_WHEELBASE
#define GEO_CIRCLE_DEFAULT_WHEELBASE APA_VEHICLE_WHEELBASE_M
#endif

typedef struct {
  const ApaPlanningSceneQuery_t *Scene;
  Integer VccCheckType;
  Integer CollisionModel;
  Integer CollisionDemand;
  Integer BoundaryLineNum;
  Real FallbackX1[3];
  Real FallbackY1[3];
  Real FallbackX2[3];
  Real FallbackY2[3];
  const Real *UseX1;
  const Real *UseY1;
  const Real *UseX2;
  const Real *UseY2;
  Boolean HaveEnvelope;
  Real ThtStp;
  Real ArcLength;
  Real StepLocal;
} GeoCircleIntrvCheckWork_t;

typedef struct {
  Integer CheckCount;
  Real SafeLen;
  Real AvlX;
  Real AvlY;
  Real AvlTheta;
} GeoCircleIntrvCheckScanState_t;

typedef struct {
  Real X;
  Real Y;
  Real Theta;
} GeoCircleIntrvCheckPose_t;

static Real geoCircleIntrvCheckCalcArcLength(
    Real R, Real pt1Theta, Real pt2Theta, Integer trnGoProduct, Real *outThtStp) {
  Real arcLength;
  Boolean condPositive;
  condPositive = (trnGoProduct > 0);

  if (condPositive) {

    Boolean condWrap;
    condWrap = (pt2Theta < pt1Theta - 1e-4f);
    if (condWrap) {
      *outThtStp = pt2Theta + APA_TWO_PI - pt1Theta;
    } else {
      *outThtStp = pt2Theta - pt1Theta;
    }
    arcLength = R * (*outThtStp);
  } else {

    Boolean condWrap;
    condWrap = (pt1Theta < pt2Theta - 1e-4f);
    if (condWrap) {
      Real negTht;
      negTht = pt1Theta + APA_TWO_PI - pt2Theta;
      *outThtStp = -negTht;
    } else {
      Real negTht;
      negTht = pt1Theta - pt2Theta;
      *outThtStp = -negTht;
    }
    arcLength = -R * (*outThtStp);
  }
  return arcLength;
}

static void geoCircleIntrvCheckCalcPose(
    Real cptX, Real cptY, Real R, Integer trnDir, Real chkTheta, Real *chkX, Real *chkY) {
  Real angOff;

  if (trnDir > 0) {
    angOff = chkTheta - APA_HALF_PI;
  } else {
    angOff = chkTheta + APA_HALF_PI;
  }
  *chkX = cptX + R * cosf(angOff);
  *chkY = cptY + R * sinf(angOff);
}

static void geoCircleIntrvCheckEvalPoseCollision(const GeoCircleIntrvCheckInput_t *input,
                                                 const ApaPlanningSceneQuery_t *scene,
                                                 Boolean haveEnvelope,
                                                 Integer vccCheckType,
                                                 Integer collisionModel,
                                                 Integer collisionDemand,
                                                 Real chkX,
                                                 Real chkY,
                                                 Real chkTheta,
                                                 Integer boundaryLineNum,
                                                 const Real *useX1,
                                                 const Real *useY1,
                                                 const Real *useX2,
                                                 const Real *useY2,
                                                 Boolean *collisionFlg,
                                                 Integer *collisionType) {
  *collisionFlg = false;
  *collisionType = VCC_COLLISION_NONE;

  if (haveEnvelope) {
    VehicleCollisionCheckInput_t vccIn;
    VehicleCollisionCheckOutput_t vccOut;
    Integer bi;
    Integer pi;

    vccIn = (VehicleCollisionCheckInput_t){0};
    vccIn.CheckType = vccCheckType;
    vccIn.CollisionModel = collisionModel;
    vccIn.CollisionDemand = collisionDemand;
    vccIn.ParkingOutFlag = input->ParkingOutFlag;
    vccIn.SlotClass = input->SlotClass;
    vccIn.SlotPtsValid = true;
    vccIn.SlotPts[0] = input->SlotP1X;
    vccIn.SlotPts[1] = input->SlotP1Y;
    vccIn.SlotPts[2] = input->SlotP2X;
    vccIn.SlotPts[3] = input->SlotP2Y;
    vccIn.SlotPts[4] = input->SlotP3X;
    vccIn.SlotPts[5] = input->SlotP3Y;
    vccIn.SlotPts[6] = input->SlotP4X;
    vccIn.SlotPts[7] = input->SlotP4Y;
    if (vccCheckType == VCC_CHECK_TYPE_PLANNING) {
      vehicleCollisionCheckLoadDefaultMargins(&vccIn);
    }
    vccIn.VehPosX = chkX;
    vccIn.VehPosY = chkY;
    vccIn.VehPosTheta = chkTheta;
    vccIn.VehLF = input->VehFrontOffset;
    vccIn.VehLB = input->VehRearOffset;
    vccIn.VehW = input->VehWidth;
    vccIn.VehWB = GEO_CIRCLE_DEFAULT_WHEELBASE;
    vccIn.BoundaryLineNum = boundaryLineNum;
    if (boundaryLineNum > VCC_MAX_BOUNDARY_LINES) {
      vccIn.BoundaryLineNum = VCC_MAX_BOUNDARY_LINES;
    }
    if (vccIn.BoundaryLineNum < 0) {
      vccIn.BoundaryLineNum = 0;
    }
    for (bi = 0; bi < vccIn.BoundaryLineNum; bi = bi + 1) {
      vccIn.BoundaryLineX1[bi] = useX1[bi];
      vccIn.BoundaryLineY1[bi] = useY1[bi];
      vccIn.BoundaryLineX2[bi] = useX2[bi];
      vccIn.BoundaryLineY2[bi] = useY2[bi];
    }
    vccIn.VirtualBoundaryLineNum = input->VirtualBoundaryLineNum;
    if (vccIn.VirtualBoundaryLineNum > VCC_MAX_VIRTUAL_LINES) {
      vccIn.VirtualBoundaryLineNum = VCC_MAX_VIRTUAL_LINES;
    }
    if (vccIn.VirtualBoundaryLineNum < 0) {
      vccIn.VirtualBoundaryLineNum = 0;
    }
    bi = 0;
    while (bi < vccIn.VirtualBoundaryLineNum) {
      vccIn.VirtualBoundaryLineX1[bi] = input->VirtualBoundaryLineX1[bi];
      vccIn.VirtualBoundaryLineY1[bi] = input->VirtualBoundaryLineY1[bi];
      vccIn.VirtualBoundaryLineX2[bi] = input->VirtualBoundaryLineX2[bi];
      vccIn.VirtualBoundaryLineY2[bi] = input->VirtualBoundaryLineY2[bi];
      bi = bi + 1;
    }
    vccIn.BoundarySepPtsNum = input->BoundarySepPtNum;
    if (vccIn.BoundarySepPtsNum > VCC_MAX_SEPARATE_PTS) {
      vccIn.BoundarySepPtsNum = VCC_MAX_SEPARATE_PTS;
    }
    if (vccIn.BoundarySepPtsNum < 0) {
      vccIn.BoundarySepPtsNum = 0;
    }
    pi = 0;
    while (pi < vccIn.BoundarySepPtsNum) {
      vccIn.BoundarySepPtsX[pi] = input->BoundarySepPtX[pi];
      vccIn.BoundarySepPtsY[pi] = input->BoundarySepPtY[pi];
      pi = pi + 1;
    }
    vccIn.SeparateStopperNum = input->StopperPtNum;
    if (vccIn.SeparateStopperNum > VCC_MAX_SEPARATE_PTS) {
      vccIn.SeparateStopperNum = VCC_MAX_SEPARATE_PTS;
    }
    if (vccIn.SeparateStopperNum < 0) {
      vccIn.SeparateStopperNum = 0;
    }
    pi = 0;
    while (pi < vccIn.SeparateStopperNum) {
      vccIn.SeparateStopperX[pi] = input->StopperPtX[pi];
      vccIn.SeparateStopperY[pi] = input->StopperPtY[pi];
      pi = pi + 1;
    }
    vccIn.SlotTrackStopperNum = input->SlotTrackStopperNum;
    if (vccIn.SlotTrackStopperNum > VCC_MAX_TRACK_STOPPER) {
      vccIn.SlotTrackStopperNum = VCC_MAX_TRACK_STOPPER;
    }
    if (vccIn.SlotTrackStopperNum < 0) {
      vccIn.SlotTrackStopperNum = 0;
    }
    pi = 0;
    while (pi < vccIn.SlotTrackStopperNum) {
      vccIn.SlotTrackStopperX1[pi] = input->SlotTrackStopperX1[pi];
      vccIn.SlotTrackStopperY1[pi] = input->SlotTrackStopperY1[pi];
      vccIn.SlotTrackStopperX2[pi] = input->SlotTrackStopperX2[pi];
      vccIn.SlotTrackStopperY2[pi] = input->SlotTrackStopperY2[pi];
      pi = pi + 1;
    }

    apaPlanningSceneCollisionCheck(&vccIn, scene, &vccOut);
    *collisionFlg = vccOut.IsCollision;
    *collisionType = vccOut.CollisionType;
  }
}

static void geoCircleIntrvCheckInitOutput(const GeoCircleIntrvCheckInput_t *input,
                                          GeoCircleIntrvCheckOutput_t *output) {
  (void)input;
  output->IntrvFlg = 0;
  output->IntrvRS = VCC_COLLISION_NONE;
  output->SafeLen = 0.0f;
  output->CheckCount = 0;
  output->AvlPosX = 0.0f;
  output->AvlPosY = 0.0f;
  output->AvlPosTheta = 0.0f;
  output->CrshPosX = 0.0f;
  output->CrshPosY = 0.0f;
  output->CrshPosTheta = 0.0f;
}

static void geoCircleIntrvCheckInitScanState(const GeoCircleIntrvCheckInput_t *input,
                                             GeoCircleIntrvCheckScanState_t *state) {
  state->CheckCount = 0;
  state->SafeLen = 0.0f;
  state->AvlX = 0.0f;
  state->AvlY = 0.0f;
  state->AvlTheta = 0.0f;
  (void)input;
}

static void geoCircleIntrvCheckResolveCollisionSettings(const GeoCircleIntrvCheckInput_t *input,
                                                        GeoCircleIntrvCheckWork_t *work) {
  work->VccCheckType = VCC_CHECK_TYPE_GENERAL;
  if (input->UsePlanningCheck) {
    work->VccCheckType = VCC_CHECK_TYPE_PLANNING;
  }

  work->CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  work->CollisionDemand = VCC_COLLISION_DEMAND_SIMPLIFY;
  if (input->SlotClass == VCC_SLOT_CLASS_PARALLEL) {
    work->CollisionDemand = VCC_COLLISION_DEMAND_ALL;
  }
  if (input->ParkingOutFlag) {
    work->CollisionDemand = VCC_COLLISION_DEMAND_ALL;
  }
  if (input->UseCollisionContract) {
    work->CollisionModel = input->CollisionModel;
    work->CollisionDemand = input->CollisionDemand;
  }
}

static void geoCircleIntrvCheckResolveBoundary(const GeoCircleIntrvCheckInput_t *input,
                                               GeoCircleIntrvCheckWork_t *work) {
  work->BoundaryLineNum = input->BoundaryLineNum;
  if (work->BoundaryLineNum > 0) {
    work->UseX1 = input->BoundaryLineX1;
    work->UseY1 = input->BoundaryLineY1;
    work->UseX2 = input->BoundaryLineX2;
    work->UseY2 = input->BoundaryLineY2;
  } else if (!input->DisableSlotBoundaryFallback) {
    Real p1x;
    Real p1y;
    Real p2x;
    Real p2y;
    Real p3x;
    Real p3y;
    Real p4x;
    Real p4y;
    Real dx12;
    Real dy12;
    Real dxSq;
    Real dySq;
    Real wCheck;

    p1x = input->SlotP1X;
    p1y = input->SlotP1Y;
    p2x = input->SlotP2X;
    p2y = input->SlotP2Y;
    p3x = input->SlotP3X;
    p3y = input->SlotP3Y;
    p4x = input->SlotP4X;
    p4y = input->SlotP4Y;
    dx12 = p1x - p2x;
    dy12 = p1y - p2y;
    dxSq = dx12 * dx12;
    dySq = dy12 * dy12;
    wCheck = dxSq + dySq;
    if (wCheck > GEO_EPSILON) {
      work->FallbackX1[0] = p1x;
      work->FallbackY1[0] = p1y;
      work->FallbackX2[0] = p4x;
      work->FallbackY2[0] = p4y;
      work->FallbackX1[1] = p4x;
      work->FallbackY1[1] = p4y;
      work->FallbackX2[1] = p3x;
      work->FallbackY2[1] = p3y;
      work->FallbackX1[2] = p3x;
      work->FallbackY1[2] = p3y;
      work->FallbackX2[2] = p2x;
      work->FallbackY2[2] = p2y;
      work->BoundaryLineNum = 3;
    }
    work->UseX1 = work->FallbackX1;
    work->UseY1 = work->FallbackY1;
    work->UseX2 = work->FallbackX2;
    work->UseY2 = work->FallbackY2;
  } else {
    work->UseX1 = work->FallbackX1;
    work->UseY1 = work->FallbackY1;
    work->UseX2 = work->FallbackX2;
    work->UseY2 = work->FallbackY2;
  }
}

static void geoCircleIntrvCheckResolveEnvelope(const GeoCircleIntrvCheckInput_t *input,
                                               GeoCircleIntrvCheckWork_t *work) {
  work->HaveEnvelope = (work->BoundaryLineNum > 0);
  if (input->VirtualBoundaryLineNum > 0) {
    work->HaveEnvelope = true;
  }
  if (input->BoundarySepPtNum > 0) {
    work->HaveEnvelope = true;
  }
  if (input->StopperPtNum > 0) {
    work->HaveEnvelope = true;
  }
  if (input->SlotTrackStopperNum > 0) {
    work->HaveEnvelope = true;
  }
  if (input->VehWidth <= GEO_EPSILON) {
    work->HaveEnvelope = false;
  }
  if (input->VehFrontOffset <= GEO_EPSILON) {
    work->HaveEnvelope = false;
  }
}

static void geoCircleIntrvCheckPrepareWork(const GeoCircleIntrvCheckInput_t *input,
                                           GeoCircleIntrvCheckWork_t *work) {
  Integer trnGoProduct;

  geoCircleIntrvCheckResolveCollisionSettings(input, work);
  geoCircleIntrvCheckResolveBoundary(input, work);
  geoCircleIntrvCheckResolveEnvelope(input, work);

  trnGoProduct = input->TrnDir;
  trnGoProduct = trnGoProduct * input->GoDir;
  work->ArcLength = geoCircleIntrvCheckCalcArcLength(
      input->Radius, input->Pt1Theta, input->Pt2Theta, trnGoProduct, &work->ThtStp);
  work->StepLocal = input->CheckStep;
  if (work->StepLocal < 0.05f) {
    work->StepLocal = 0.05f;
  }
}

static void geoCircleIntrvCheckEvalWorkCollision(const GeoCircleIntrvCheckInput_t *input,
                                                 const GeoCircleIntrvCheckWork_t *work,
                                                 Real chkX,
                                                 Real chkY,
                                                 Real chkTheta,
                                                 Boolean *collisionFlg,
                                                 Integer *collisionType) {
  geoCircleIntrvCheckEvalPoseCollision(input,
                                       work->Scene,
                                       work->HaveEnvelope,
                                       work->VccCheckType,
                                       work->CollisionModel,
                                       work->CollisionDemand,
                                       chkX,
                                       chkY,
                                       chkTheta,
                                       work->BoundaryLineNum,
                                       work->UseX1,
                                       work->UseY1,
                                       work->UseX2,
                                       work->UseY2,
                                       collisionFlg,
                                       collisionType);
}

static void geoCircleIntrvCheckSetStartCollision(const GeoCircleIntrvCheckScanState_t *state,
                                                 const GeoCircleIntrvCheckPose_t *crashPose,
                                                 Integer collisionType,
                                                 GeoCircleIntrvCheckOutput_t *output) {
  output->IntrvFlg = 2;
  output->SafeLen = state->SafeLen;
  output->IntrvRS = collisionType;
  output->CheckCount = state->CheckCount;
  output->AvlPosX = state->AvlX;
  output->AvlPosY = state->AvlY;
  output->AvlPosTheta = state->AvlTheta;
  output->CrshPosX = crashPose->X;
  output->CrshPosY = crashPose->Y;
  output->CrshPosTheta = crashPose->Theta;
}

static void geoCircleIntrvCheckSetSimpleCollision(const GeoCircleIntrvCheckScanState_t *state,
                                                  const GeoCircleIntrvCheckPose_t *crashPose,
                                                  Integer collisionFlag,
                                                  Integer collisionType,
                                                  GeoCircleIntrvCheckOutput_t *output) {
  output->IntrvFlg = collisionFlag;
  output->SafeLen = state->SafeLen;
  output->IntrvRS = collisionType;
  output->CheckCount = state->CheckCount;
  output->AvlPosX = state->AvlX;
  output->AvlPosY = state->AvlY;
  output->AvlPosTheta = state->AvlTheta;
  output->CrshPosX = crashPose->X;
  output->CrshPosY = crashPose->Y;
  output->CrshPosTheta = crashPose->Theta;
}

static void geoCircleIntrvCheckSetNoCollision(const GeoCircleIntrvCheckInput_t *input,
                                              const GeoCircleIntrvCheckWork_t *work,
                                              const GeoCircleIntrvCheckScanState_t *state,
                                              GeoCircleIntrvCheckOutput_t *output) {
  output->IntrvRS = VCC_COLLISION_NONE;
  output->SafeLen = work->ArcLength;
  output->CheckCount = state->CheckCount;
  output->AvlPosX = input->Pt2X;
  output->AvlPosY = input->Pt2Y;
  output->AvlPosTheta = input->Pt2Theta;
  output->CrshPosX = 0.0f;
  output->CrshPosY = 0.0f;
  output->CrshPosTheta = 0.0f;
}

static void geoCircleIntrvCheckRefineCollision(const GeoCircleIntrvCheckInput_t *input,
                                               const GeoCircleIntrvCheckWork_t *work,
                                               GeoCircleIntrvCheckScanState_t *state,
                                               const GeoCircleIntrvCheckPose_t *crashPose,
                                               Integer collisionType,
                                               GeoCircleIntrvCheckOutput_t *output) {
  Real stX;
  Real stY;
  Real stTheta;
  Real edX;
  Real edY;
  Real edTheta;
  Real refineLen;
  Real lenPrcs;
  Real refineThreshold;
  Integer refinedCollisionType;

  stX = state->AvlX;
  stY = state->AvlY;
  stTheta = state->AvlTheta;
  edX = crashPose->X;
  edY = crashPose->Y;
  edTheta = crashPose->Theta;
  refineLen = 0.0f;
  lenPrcs = work->StepLocal / 2.0f;
  refineThreshold = 0.01f;
  if (input->ParkingOutFlag) {
    refineThreshold = 0.02f;
  }
  refinedCollisionType = collisionType;

  while (lenPrcs > refineThreshold) {
    Real midX;
    Real midY;
    Real midTheta;
    Boolean midCollisionFlg;
    Integer midCollisionType;

    midTheta = (stTheta + edTheta) / 2.0f;
    geoCircleIntrvCheckCalcPose(
        input->CptX, input->CptY, input->Radius, input->TrnDir, midTheta, &midX, &midY);
    geoCircleIntrvCheckEvalWorkCollision(
        input, work, midX, midY, midTheta, &midCollisionFlg, &midCollisionType);
    state->CheckCount = state->CheckCount + 1;
    if (midCollisionFlg) {
      edX = midX;
      edY = midY;
      edTheta = midTheta;
      refinedCollisionType = midCollisionType;
    } else {
      stX = midX;
      stY = midY;
      stTheta = midTheta;
      refineLen = refineLen + lenPrcs;
    }
    lenPrcs = lenPrcs / 2.0f;
  }

  output->SafeLen = state->SafeLen + refineLen;
  output->IntrvRS = refinedCollisionType;
  output->CheckCount = state->CheckCount;
  output->AvlPosX = stX;
  output->AvlPosY = stY;
  output->AvlPosTheta = stTheta;
  output->CrshPosX = edX;
  output->CrshPosY = edY;
  output->CrshPosTheta = edTheta;
}

static void geoCircleIntrvCheckRunSamples(const GeoCircleIntrvCheckInput_t *input,
                                          const GeoCircleIntrvCheckWork_t *work,
                                          GeoCircleIntrvCheckScanState_t *state,
                                          GeoCircleIntrvCheckOutput_t *output) {
  Real lenTmp;
  Boolean condLoop;

  lenTmp = 0.0f;
  condLoop = (lenTmp <= work->ArcLength + 1e-9f);
  while (condLoop) {
    Real ratio;
    Real absArcL;
    Real thtOffset;
    GeoCircleIntrvCheckPose_t chkPose;
    Boolean collisionFlg;
    Boolean isStartSample;
    Boolean ignorePkOtStartCollision;
    Integer collisionType;

    absArcL = fabsf(work->ArcLength);
    if (absArcL < GEO_EPSILON) {
      ratio = 0.0f;
    } else {
      ratio = lenTmp / work->ArcLength;
    }

    thtOffset = ratio * work->ThtStp;
    chkPose.Theta = input->Pt1Theta + thtOffset;
    geoCircleIntrvCheckCalcPose(input->CptX,
                                input->CptY,
                                input->Radius,
                                input->TrnDir,
                                chkPose.Theta,
                                &chkPose.X,
                                &chkPose.Y);

    geoCircleIntrvCheckEvalWorkCollision(
        input, work, chkPose.X, chkPose.Y, chkPose.Theta, &collisionFlg, &collisionType);
    state->CheckCount = state->CheckCount + 1;
    isStartSample = (lenTmp <= GEO_EPSILON);
    ignorePkOtStartCollision = collisionFlg && isStartSample;
    ignorePkOtStartCollision = ignorePkOtStartCollision && input->ParkingOutFlag;
    if (ignorePkOtStartCollision) {
      collisionFlg = false;
    }

    if (collisionFlg) {
      if (isStartSample) {
        geoCircleIntrvCheckSetStartCollision(state, &chkPose, collisionType, output);
      } else {
        output->IntrvFlg = 1;
        if (input->UsePlanningCheck) {
          geoCircleIntrvCheckRefineCollision(input, work, state, &chkPose, collisionType, output);
        } else {
          geoCircleIntrvCheckSetSimpleCollision(state, &chkPose, 1, collisionType, output);
        }
      }
      condLoop = false;
    } else {
      state->SafeLen = lenTmp;
      state->AvlX = chkPose.X;
      state->AvlY = chkPose.Y;
      state->AvlTheta = chkPose.Theta;
      lenTmp = lenTmp + work->StepLocal;
      condLoop = (lenTmp <= work->ArcLength + 1e-9f);
      if (state->CheckCount >= 200) {
        condLoop = false;
      }
    }
  }
}

static void geoCircleIntrvCheckHandleEndPose(const GeoCircleIntrvCheckInput_t *input,
                                             const GeoCircleIntrvCheckWork_t *work,
                                             GeoCircleIntrvCheckScanState_t *state,
                                             GeoCircleIntrvCheckOutput_t *output) {
  if (output->IntrvFlg == 0) {
    Boolean endCollisionFlg;
    Integer endCollisionType;
    GeoCircleIntrvCheckPose_t endPose;

    endPose.X = input->Pt2X;
    endPose.Y = input->Pt2Y;
    endPose.Theta = input->Pt2Theta;
    geoCircleIntrvCheckEvalWorkCollision(
        input, work, endPose.X, endPose.Y, endPose.Theta, &endCollisionFlg, &endCollisionType);
    state->CheckCount = state->CheckCount + 1;

    if (endCollisionFlg) {
      Boolean shouldRefine;
      Real absArcLength;

      output->IntrvFlg = 1;
      absArcLength = fabsf(work->ArcLength);
      if (absArcLength <= GEO_EPSILON) {
        output->IntrvFlg = 2;
      }

      shouldRefine = input->UsePlanningCheck;
      if (output->IntrvFlg == 2) {
        shouldRefine = false;
      }
      if (shouldRefine) {
        geoCircleIntrvCheckRefineCollision(input, work, state, &endPose, endCollisionType, output);
      } else {
        geoCircleIntrvCheckSetSimpleCollision(
            state, &endPose, output->IntrvFlg, endCollisionType, output);
      }
    } else {
      geoCircleIntrvCheckSetNoCollision(input, work, state, output);
    }
  }
}

/**
 * @brief 沿圆弧路径按固定步长采样检测位姿，输出全程安全长度和采样点数量（碰撞判定预留外部模块）
 * @en_name geoCircleIntrvCheck
 * @cn_name 圆弧区间检测
 * @type module
 * @param[IN] Real CptX 圆心X
 * @param[IN] Real CptY 圆心Y
 * @param[IN] Real Radius 半径
 * @param[IN] Real Pt1X 弧线起点X
 * @param[IN] Real Pt1Y 弧线起点Y
 * @param[IN] Real Pt1Theta 弧线起点航向
 * @param[IN] Real Pt2X 弧线终点X
 * @param[IN] Real Pt2Y 弧线终点Y
 * @param[IN] Real Pt2Theta 弧线终点航向
 * @param[IN] Integer GoDir 行驶方向1前进负1后退
 * @param[IN] Integer TrnDir 转弯方向1左转负1右转
 * @param[IN] Real CheckStep 检测步长
 * @param[IN] Real GoalX 目标点X
 * @param[IN] Real GoalY 目标点Y
 * @param[IN] Real GoalYaw 目标航向
 * @param[IN] Real SlotP1X 车位入口角点1X
 * @param[IN] Real SlotP1Y 车位入口角点1Y
 * @param[IN] Real SlotP2X 车位入口角点2X
 * @param[IN] Real SlotP2Y 车位入口角点2Y
 * @param[IN] Real SlotP3X 车位后角点3X
 * @param[IN] Real SlotP3Y 车位后角点3Y
 * @param[IN] Real SlotP4X 车位后角点4X
 * @param[IN] Real SlotP4Y 车位后角点4Y
 * @param[IN] Real VehWidth 车辆宽度
 * @param[IN] Real VehFrontOffset 车辆前包络偏置
 * @param[OUT] Integer IntrvFlg 碰撞标志0无碰撞
 * @param[OUT] Integer IntrvRS 碰撞类型
 * @param[OUT] Real SafeLen 安全长度
 * @param[OUT] Integer CheckCount 检测次数
 * @param[OUT] Real AvlPosX 安全点X
 * @param[OUT] Real AvlPosY 安全点Y
 * @param[OUT] Real AvlPosTheta 安全点航向
 * @param[OUT] Real CrshPosX 碰撞点X
 * @param[OUT] Real CrshPosY 碰撞点Y
 * @param[OUT] Real CrshPosTheta 碰撞点航向
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 碰撞检测
 * @source matlab_rewrite geo_circle_Intrv_check.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */

void geoCircleIntrvCheck(const GeoCircleIntrvCheckInput_t *input,
                         GeoCircleIntrvCheckOutput_t *output) {
  geoCircleIntrvCheckWithScene(input, (const ApaPlanningSceneQuery_t *)0, output);
}

void geoCircleIntrvCheckWithScene(const GeoCircleIntrvCheckInput_t *input,
                                  const ApaPlanningSceneQuery_t *scene,
                                  GeoCircleIntrvCheckOutput_t *output) {
  GeoCircleIntrvCheckWork_t work;
  GeoCircleIntrvCheckScanState_t state;

  work = (GeoCircleIntrvCheckWork_t){0};
  state = (GeoCircleIntrvCheckScanState_t){0};
  geoCircleIntrvCheckInitOutput(input, output);
  geoCircleIntrvCheckPrepareWork(input, &work);
  work.Scene = scene;
  if (scene && scene->CheckPose) {
    if (input->VehWidth > GEO_EPSILON && input->VehFrontOffset > GEO_EPSILON)
      work.HaveEnvelope = true;

    const Boolean htParIn = input->UsePlanningCheck &&
                            input->SlotClass == VCC_SLOT_CLASS_PARALLEL && !input->ParkingOutFlag;
    if (!htParIn && work.StepLocal > 0.05f)
      work.StepLocal = 0.05f;
  }
  geoCircleIntrvCheckInitScanState(input, &state);
  geoCircleIntrvCheckRunSamples(input, &work, &state, output);
  geoCircleIntrvCheckHandleEndPose(input, &work, &state, output);

  output->AvlPosTheta = apaCoordWrapToPi(output->AvlPosTheta);
  output->CrshPosTheta = apaCoordWrapToPi(output->CrshPosTheta);
}

/**
 * @brief 泊出圆弧区间碰撞检测包装，启用规划模式以覆盖虚拟边界并输出碰撞点
 * @en_name geoCircleIntrvCheckPkOt
 * @cn_name 泊出圆弧区间检测
 * @type module
 * @param[IN] GeoCircleIntrvCheckInput_t input 圆弧检测输入
 * @param[OUT] GeoCircleIntrvCheckOutput_t output 圆弧检测输出
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 泊出碰撞检测
 * @source matlab_rewrite geo_circle_Intrv_check_PkOt.m
 * @version 1.0
 * @date 2026-04-30
 * @author aide
 */
void geoCircleIntrvCheckPkOt(const GeoCircleIntrvCheckInput_t *input,
                             GeoCircleIntrvCheckOutput_t *output) {
  GeoCircleIntrvCheckInput_t pkOtInput;

  pkOtInput = *input;
  pkOtInput.UsePlanningCheck = true;
  pkOtInput.ParkingOutFlag = true;
  geoCircleIntrvCheck(&pkOtInput, output);
}
