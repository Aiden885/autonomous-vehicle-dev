#include "geoLineIntrvCheckTypes.h"
#include "apaVehicleDefaults.h"
#include <math.h>

#ifndef GEO_LINE_DEFAULT_WHEELBASE
#define GEO_LINE_DEFAULT_WHEELBASE APA_VEHICLE_WHEELBASE_M
#endif

typedef struct {
  const ApaPlanningSceneQuery_t *Scene;
  Integer BoundaryLineNum;
  Integer VirtualBoundaryLineNum;
  Integer BoundarySepPtNum;
  Integer SeparateStopperNum;
  Integer SlotTrackStopperNum;
  Boolean HaveEnvelope;
  Integer VccCheckType;
  Boolean ParkingOutFlag;
  Integer SlotClass;
  Integer SampleCollisionModel;
  Integer SampleCollisionDemand;
  Integer RefineCollisionDemand;
  Real SlotPts[8];
  Real FallbackX1[3];
  Real FallbackY1[3];
  Real FallbackX2[3];
  Real FallbackY2[3];
  const Real *UseX1;
  const Real *UseY1;
  const Real *UseX2;
  const Real *UseY2;
  Real Dx;
  Real Dy;
  Real LineLen;
  Real StepLocal;
} GeoLineIntrvCheckWork_t;

typedef struct {
  Integer CheckCount;
  Real SafeLen;
  Real AvlX;
  Real AvlY;
  Real AvlTheta;
} GeoLineIntrvCheckScanState_t;

typedef struct {
  Real X;
  Real Y;
  Real Theta;
} GeoLineIntrvCheckPose_t;

static void geoLineIntrvCheckEvalPoseCollision(Boolean haveEnvelope,
                                               const ApaPlanningSceneQuery_t *scene,
                                               Integer vccCheckType,
                                               Integer collisionModel,
                                               Integer collisionDemand,
                                               Boolean parkingOutFlag,
                                               Integer slotClass,
                                               const Real slotPts[8],
                                               Real chkX,
                                               Real chkY,
                                               Real chkTheta,
                                               Real vehWidth,
                                               Real vehFrontOffset,
                                               Real vehRearOffset,
                                               Integer boundaryLineNum,
                                               const Real *useX1,
                                               const Real *useY1,
                                               const Real *useX2,
                                               const Real *useY2,
                                               Integer virtualBoundaryLineNum,
                                               const Real *virtualBoundaryLineX1,
                                               const Real *virtualBoundaryLineY1,
                                               const Real *virtualBoundaryLineX2,
                                               const Real *virtualBoundaryLineY2,
                                               Integer boundarySepPtNum,
                                               const Real *boundarySepPtX,
                                               const Real *boundarySepPtY,
                                               Integer separateStopperNum,
                                               const Real *separateStopperX,
                                               const Real *separateStopperY,
                                               Integer slotTrackStopperNum,
                                               const Real *slotTrackStopperX1,
                                               const Real *slotTrackStopperY1,
                                               const Real *slotTrackStopperX2,
                                               const Real *slotTrackStopperY2,
                                               Boolean *collisionFlg,
                                               Integer *collisionType,
                                               VehicleCollisionCheckOutput_t *collisionDetails) {
  *collisionFlg = false;
  *collisionType = VCC_COLLISION_NONE;
  *collisionDetails = (VehicleCollisionCheckOutput_t){0};

  if (haveEnvelope) {
    VehicleCollisionCheckInput_t vccIn;
    VehicleCollisionCheckOutput_t vccOut;
    Integer bi;
    Integer si;
    Integer sti;
    Integer tki;
    Integer vbi;

    vccIn = (VehicleCollisionCheckInput_t){0};
    vccIn.CheckType = vccCheckType;
    vccIn.CollisionModel = collisionModel;
    vccIn.CollisionDemand = collisionDemand;
    vccIn.ParkingOutFlag = parkingOutFlag;
    vccIn.SlotClass = slotClass;
    vccIn.SlotPtsValid = true;
    vccIn.SlotPts[0] = slotPts[0];
    vccIn.SlotPts[1] = slotPts[1];
    vccIn.SlotPts[2] = slotPts[2];
    vccIn.SlotPts[3] = slotPts[3];
    vccIn.SlotPts[4] = slotPts[4];
    vccIn.SlotPts[5] = slotPts[5];
    vccIn.SlotPts[6] = slotPts[6];
    vccIn.SlotPts[7] = slotPts[7];
    if (vccCheckType == VCC_CHECK_TYPE_PLANNING) {
      vehicleCollisionCheckLoadDefaultMargins(&vccIn);
    }
    vccIn.VehPosX = chkX;
    vccIn.VehPosY = chkY;
    vccIn.VehPosTheta = chkTheta;
    vccIn.VehLF = vehFrontOffset;
    vccIn.VehLB = vehRearOffset;
    vccIn.VehW = vehWidth;
    vccIn.VehWB = GEO_LINE_DEFAULT_WHEELBASE;
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
    vccIn.VirtualBoundaryLineNum = virtualBoundaryLineNum;
    if (vccIn.VirtualBoundaryLineNum > VCC_MAX_VIRTUAL_LINES) {
      vccIn.VirtualBoundaryLineNum = VCC_MAX_VIRTUAL_LINES;
    }
    if (vccIn.VirtualBoundaryLineNum < 0) {
      vccIn.VirtualBoundaryLineNum = 0;
    }
    for (vbi = 0; vbi < vccIn.VirtualBoundaryLineNum; vbi = vbi + 1) {
      vccIn.VirtualBoundaryLineX1[vbi] = virtualBoundaryLineX1[vbi];
      vccIn.VirtualBoundaryLineY1[vbi] = virtualBoundaryLineY1[vbi];
      vccIn.VirtualBoundaryLineX2[vbi] = virtualBoundaryLineX2[vbi];
      vccIn.VirtualBoundaryLineY2[vbi] = virtualBoundaryLineY2[vbi];
    }
    vccIn.BoundarySepPtsNum = boundarySepPtNum;
    if (vccIn.BoundarySepPtsNum > VCC_MAX_SEPARATE_PTS) {
      vccIn.BoundarySepPtsNum = VCC_MAX_SEPARATE_PTS;
    }
    if (vccIn.BoundarySepPtsNum < 0) {
      vccIn.BoundarySepPtsNum = 0;
    }
    for (si = 0; si < vccIn.BoundarySepPtsNum; si = si + 1) {
      vccIn.BoundarySepPtsX[si] = boundarySepPtX[si];
      vccIn.BoundarySepPtsY[si] = boundarySepPtY[si];
    }
    vccIn.SeparateStopperNum = separateStopperNum;
    if (vccIn.SeparateStopperNum > VCC_MAX_SEPARATE_PTS) {
      vccIn.SeparateStopperNum = VCC_MAX_SEPARATE_PTS;
    }
    if (vccIn.SeparateStopperNum < 0) {
      vccIn.SeparateStopperNum = 0;
    }
    for (sti = 0; sti < vccIn.SeparateStopperNum; sti = sti + 1) {
      vccIn.SeparateStopperX[sti] = separateStopperX[sti];
      vccIn.SeparateStopperY[sti] = separateStopperY[sti];
    }
    vccIn.SlotTrackStopperNum = slotTrackStopperNum;
    if (vccIn.SlotTrackStopperNum > VCC_MAX_TRACK_STOPPER) {
      vccIn.SlotTrackStopperNum = VCC_MAX_TRACK_STOPPER;
    }
    if (vccIn.SlotTrackStopperNum < 0) {
      vccIn.SlotTrackStopperNum = 0;
    }
    for (tki = 0; tki < vccIn.SlotTrackStopperNum; tki = tki + 1) {
      vccIn.SlotTrackStopperX1[tki] = slotTrackStopperX1[tki];
      vccIn.SlotTrackStopperY1[tki] = slotTrackStopperY1[tki];
      vccIn.SlotTrackStopperX2[tki] = slotTrackStopperX2[tki];
      vccIn.SlotTrackStopperY2[tki] = slotTrackStopperY2[tki];
    }

    apaPlanningSceneCollisionCheck(&vccIn, scene, &vccOut);
    *collisionDetails = vccOut;
    *collisionFlg = vccOut.IsCollision;
    *collisionType = vccOut.CollisionType;
  }
}

static Boolean
geoLineIntrvCheckShouldIgnorePkOtSideScrape(Integer dirOut,
                                            const VehicleCollisionCheckOutput_t *collisionDetails) {
  Boolean result;
  Boolean sideArea;
  Boolean shallowDepth;
  Boolean hasCollision;

  result = false;
  sideArea = false;
  shallowDepth = false;
  hasCollision = collisionDetails->IsCollision;

  if (hasCollision) {
    if (dirOut == 1) {
      Boolean area9;
      Boolean area6;
      Boolean area3;
      Boolean sideArea96;
      Boolean baseDepth;
      Boolean crossLine4;
      Boolean crossDepth;

      area9 = (collisionDetails->CollisionArea[8] != 0U);
      area6 = (collisionDetails->CollisionArea[5] != 0U);
      area3 = (collisionDetails->CollisionArea[2] != 0U);
      sideArea96 = area9 || area6;
      sideArea = sideArea96 || area3;
      baseDepth = (collisionDetails->CollisionDepth < 0.07f);
      crossLine4 = (collisionDetails->CrossLine == 4U);
      crossDepth = crossLine4 && (collisionDetails->CollisionDepth < 0.10f);
      shallowDepth = baseDepth || crossDepth;
    } else if (dirOut == 2) {
      Boolean area1;
      Boolean area4;
      Boolean area7;
      Boolean sideArea14;
      Boolean baseDepth;
      Boolean crossLine6;
      Boolean crossDepth;

      area1 = (collisionDetails->CollisionArea[0] != 0U);
      area4 = (collisionDetails->CollisionArea[3] != 0U);
      area7 = (collisionDetails->CollisionArea[6] != 0U);
      sideArea14 = area1 || area4;
      sideArea = sideArea14 || area7;
      baseDepth = (collisionDetails->CollisionDepth < 0.07f);
      crossLine6 = (collisionDetails->CrossLine == 6U);
      crossDepth = crossLine6 && (collisionDetails->CollisionDepth < 0.10f);
      shallowDepth = baseDepth || crossDepth;
    }

    result = sideArea && shallowDepth;
  }

  return result;
}

static void geoLineIntrvCheckInitOutput(const GeoLineIntrvCheckInput_t *input,
                                        GeoLineIntrvCheckOutput_t *output) {
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

static void geoLineIntrvCheckInitScanState(const GeoLineIntrvCheckInput_t *input,
                                           GeoLineIntrvCheckScanState_t *state) {
  state->CheckCount = 0;
  state->SafeLen = 0.0f;
  state->AvlX = 0.0f;
  state->AvlY = 0.0f;
  state->AvlTheta = 0.0f;
  (void)input;
}

static void geoLineIntrvCheckResolveCollisionSettings(const GeoLineIntrvCheckInput_t *input,
                                                      GeoLineIntrvCheckWork_t *work) {
  work->ParkingOutFlag = input->ParkingOutFlag;
  work->SlotClass = input->SlotClass;
  work->VccCheckType = VCC_CHECK_TYPE_GENERAL;
  if (input->UsePlanningCheck) {
    work->VccCheckType = VCC_CHECK_TYPE_PLANNING;
  }

  work->SampleCollisionModel =
      input->UsePlanningCheck ? VCC_COLLISION_MODEL_EIGHT_EDGES : VCC_COLLISION_MODEL_FOUR_EDGES;
  work->SampleCollisionDemand = VCC_COLLISION_DEMAND_SIMPLIFY;
  work->RefineCollisionDemand = VCC_COLLISION_DEMAND_SIMPLIFY;
  if (work->ParkingOutFlag) {
    work->SampleCollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
    work->SampleCollisionDemand = VCC_COLLISION_DEMAND_ALL;
    work->RefineCollisionDemand = VCC_COLLISION_DEMAND_ALL;
  }
}

static void geoLineIntrvCheckLoadSlotPts(const GeoLineIntrvCheckInput_t *input,
                                         GeoLineIntrvCheckWork_t *work) {
  work->SlotPts[0] = input->SlotP1X;
  work->SlotPts[1] = input->SlotP1Y;
  work->SlotPts[2] = input->SlotP2X;
  work->SlotPts[3] = input->SlotP2Y;
  work->SlotPts[4] = input->SlotP3X;
  work->SlotPts[5] = input->SlotP3Y;
  work->SlotPts[6] = input->SlotP4X;
  work->SlotPts[7] = input->SlotP4Y;
}

static void geoLineIntrvCheckResolveBoundary(const GeoLineIntrvCheckInput_t *input,
                                             GeoLineIntrvCheckWork_t *work) {
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

static void geoLineIntrvCheckResolveEnvelope(const GeoLineIntrvCheckInput_t *input,
                                             GeoLineIntrvCheckWork_t *work) {
  work->VirtualBoundaryLineNum = input->VirtualBoundaryLineNum;
  work->BoundarySepPtNum = input->BoundarySepPtNum;
  work->SeparateStopperNum = input->SeparateStopperNum;
  work->SlotTrackStopperNum = input->SlotTrackStopperNum;
  work->HaveEnvelope = (work->BoundaryLineNum > 0);
  if (work->BoundarySepPtNum > 0) {
    work->HaveEnvelope = true;
  }
  if (work->SeparateStopperNum > 0) {
    work->HaveEnvelope = true;
  }
  if (work->SlotTrackStopperNum > 0) {
    work->HaveEnvelope = true;
  }
  if (work->VirtualBoundaryLineNum > 0) {
    work->HaveEnvelope = true;
  }
  if (input->VehWidth <= GEO_EPSILON) {
    work->HaveEnvelope = false;
  }
  if (input->VehFrontOffset <= GEO_EPSILON) {
    work->HaveEnvelope = false;
  }
}

static void geoLineIntrvCheckPrepareWork(const GeoLineIntrvCheckInput_t *input,
                                         GeoLineIntrvCheckWork_t *work) {
  Real dx2;
  Real dy2;
  Real sum2;

  geoLineIntrvCheckResolveCollisionSettings(input, work);
  geoLineIntrvCheckLoadSlotPts(input, work);
  geoLineIntrvCheckResolveBoundary(input, work);
  geoLineIntrvCheckResolveEnvelope(input, work);

  work->Dx = input->Pt2X - input->Pt1X;
  work->Dy = input->Pt2Y - input->Pt1Y;
  dx2 = work->Dx * work->Dx;
  dy2 = work->Dy * work->Dy;
  sum2 = dx2 + dy2;
  work->LineLen = sqrtf(sum2);
  if (input->CheckStep < 0.1f) {
    work->StepLocal = 0.1f;
  } else {
    work->StepLocal = input->CheckStep;
  }
}

static void geoLineIntrvCheckCalcPose(const GeoLineIntrvCheckInput_t *input,
                                      const GeoLineIntrvCheckWork_t *work,
                                      Real ratio,
                                      GeoLineIntrvCheckPose_t *pose) {
  Real dxR;
  Real dyR;
  Real dtR;

  dxR = ratio * work->Dx;
  dyR = ratio * work->Dy;
  pose->X = input->Pt1X + dxR;
  pose->Y = input->Pt1Y + dyR;
  dtR = ratio * (input->Pt2Theta - input->Pt1Theta);
  pose->Theta = input->Pt1Theta + dtR;
}

static void geoLineIntrvCheckEvalWorkCollision(const GeoLineIntrvCheckInput_t *input,
                                               const GeoLineIntrvCheckWork_t *work,
                                               Integer collisionModel,
                                               Integer collisionDemand,
                                               const GeoLineIntrvCheckPose_t *pose,
                                               Boolean *collisionFlg,
                                               Integer *collisionType,
                                               VehicleCollisionCheckOutput_t *collisionDetails) {
  geoLineIntrvCheckEvalPoseCollision(work->HaveEnvelope,
                                     work->Scene,
                                     work->VccCheckType,
                                     collisionModel,
                                     collisionDemand,
                                     work->ParkingOutFlag,
                                     work->SlotClass,
                                     work->SlotPts,
                                     pose->X,
                                     pose->Y,
                                     pose->Theta,
                                     input->VehWidth,
                                     input->VehFrontOffset,
                                     input->VehRearOffset,
                                     work->BoundaryLineNum,
                                     work->UseX1,
                                     work->UseY1,
                                     work->UseX2,
                                     work->UseY2,
                                     work->VirtualBoundaryLineNum,
                                     input->VirtualBoundaryLineX1,
                                     input->VirtualBoundaryLineY1,
                                     input->VirtualBoundaryLineX2,
                                     input->VirtualBoundaryLineY2,
                                     work->BoundarySepPtNum,
                                     input->BoundarySepPtX,
                                     input->BoundarySepPtY,
                                     work->SeparateStopperNum,
                                     input->SeparateStopperX,
                                     input->SeparateStopperY,
                                     work->SlotTrackStopperNum,
                                     input->SlotTrackStopperX1,
                                     input->SlotTrackStopperY1,
                                     input->SlotTrackStopperX2,
                                     input->SlotTrackStopperY2,
                                     collisionFlg,
                                     collisionType,
                                     collisionDetails);
}

static void geoLineIntrvCheckSetStartCollision(const GeoLineIntrvCheckScanState_t *state,
                                               const GeoLineIntrvCheckPose_t *crashPose,
                                               Integer collisionType,
                                               GeoLineIntrvCheckOutput_t *output) {
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

static void geoLineIntrvCheckSetSimpleCollision(const GeoLineIntrvCheckScanState_t *state,
                                                const GeoLineIntrvCheckPose_t *crashPose,
                                                Integer collisionFlag,
                                                Integer collisionType,
                                                GeoLineIntrvCheckOutput_t *output) {
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

static void geoLineIntrvCheckSetNoCollision(const GeoLineIntrvCheckInput_t *input,
                                            const GeoLineIntrvCheckWork_t *work,
                                            const GeoLineIntrvCheckScanState_t *state,
                                            GeoLineIntrvCheckOutput_t *output) {
  output->IntrvRS = VCC_COLLISION_NONE;
  output->SafeLen = work->LineLen;
  output->CheckCount = state->CheckCount;
  output->AvlPosX = input->Pt2X;
  output->AvlPosY = input->Pt2Y;
  output->AvlPosTheta = input->Pt2Theta;
  output->CrshPosX = 0.0f;
  output->CrshPosY = 0.0f;
  output->CrshPosTheta = 0.0f;
}

static void geoLineIntrvCheckRefineCollision(const GeoLineIntrvCheckInput_t *input,
                                             const GeoLineIntrvCheckWork_t *work,
                                             GeoLineIntrvCheckScanState_t *state,
                                             const GeoLineIntrvCheckPose_t *crashPose,
                                             Integer collisionType,
                                             GeoLineIntrvCheckOutput_t *output) {
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
  refineThreshold = 0.02f;
  if (input->ParkingOutFlag) {
    refineThreshold = 0.01f;
  }
  refinedCollisionType = collisionType;

  while (lenPrcs > refineThreshold) {
    GeoLineIntrvCheckPose_t midPose;
    Boolean midCollisionFlg;
    Integer midCollisionType;
    VehicleCollisionCheckOutput_t midCollisionDetails;

    midPose.X = (stX + edX) / 2.0f;
    midPose.Y = (stY + edY) / 2.0f;
    midPose.Theta = stTheta;
    geoLineIntrvCheckEvalWorkCollision(input,
                                       work,
                                       VCC_COLLISION_MODEL_EIGHT_EDGES,
                                       work->RefineCollisionDemand,
                                       &midPose,
                                       &midCollisionFlg,
                                       &midCollisionType,
                                       &midCollisionDetails);
    state->CheckCount = state->CheckCount + 1;
    if (midCollisionFlg) {
      edX = midPose.X;
      edY = midPose.Y;
      edTheta = midPose.Theta;
      refinedCollisionType = midCollisionType;
    } else {
      stX = midPose.X;
      stY = midPose.Y;
      stTheta = midPose.Theta;
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

static void geoLineIntrvCheckRunSamples(const GeoLineIntrvCheckInput_t *input,
                                        const GeoLineIntrvCheckWork_t *work,
                                        GeoLineIntrvCheckScanState_t *state,
                                        GeoLineIntrvCheckOutput_t *output) {
  Real lenTmp;
  Boolean condLoop;

  lenTmp = 0.0f;
  condLoop = (lenTmp <= work->LineLen + 1e-9f);
  while (condLoop) {
    Real ratio;
    GeoLineIntrvCheckPose_t chkPose;
    Boolean condLenTiny;
    Boolean collisionFlg;
    Boolean isStartSample;
    Boolean canApplyPkOtExemption;
    Boolean ignorePkOtStartCollision;
    Integer collisionType;
    VehicleCollisionCheckOutput_t collisionDetails;

    condLenTiny = (work->LineLen < GEO_EPSILON);
    if (condLenTiny) {
      ratio = 0.0f;
    } else {
      ratio = lenTmp / work->LineLen;
    }
    geoLineIntrvCheckCalcPose(input, work, ratio, &chkPose);
    state->CheckCount = state->CheckCount + 1;
    geoLineIntrvCheckEvalWorkCollision(input,
                                       work,
                                       work->SampleCollisionModel,
                                       work->SampleCollisionDemand,
                                       &chkPose,
                                       &collisionFlg,
                                       &collisionType,
                                       &collisionDetails);
    isStartSample = (lenTmp <= GEO_EPSILON);
    canApplyPkOtExemption = collisionFlg;
    if (isStartSample) {
      canApplyPkOtExemption = false;
    }
    if (!work->ParkingOutFlag) {
      canApplyPkOtExemption = false;
    }
    if (canApplyPkOtExemption) {
      if (geoLineIntrvCheckShouldIgnorePkOtSideScrape(input->DirOut, &collisionDetails)) {
        collisionFlg = false;
      }
    }
    ignorePkOtStartCollision = collisionFlg;
    if (!isStartSample) {
      ignorePkOtStartCollision = false;
    }
    if (!work->ParkingOutFlag) {
      ignorePkOtStartCollision = false;
    }
    if (ignorePkOtStartCollision) {
      collisionFlg = false;
    }

    if (collisionFlg) {
      if (isStartSample) {
        geoLineIntrvCheckSetStartCollision(state, &chkPose, collisionType, output);
      } else {
        output->IntrvFlg = 1;
        if (input->UsePlanningCheck) {
          geoLineIntrvCheckRefineCollision(input, work, state, &chkPose, collisionType, output);
        } else {
          geoLineIntrvCheckSetSimpleCollision(state, &chkPose, 1, collisionType, output);
        }
      }
      condLoop = false;
    } else {
      state->SafeLen = lenTmp;
      state->AvlX = chkPose.X;
      state->AvlY = chkPose.Y;
      state->AvlTheta = chkPose.Theta;
      lenTmp = lenTmp + work->StepLocal;
      condLoop = (lenTmp <= work->LineLen + 1e-9f);
      if (state->CheckCount >= 200) {
        condLoop = false;
      }
    }
  }
}

static void geoLineIntrvCheckHandleEndPose(const GeoLineIntrvCheckInput_t *input,
                                           const GeoLineIntrvCheckWork_t *work,
                                           GeoLineIntrvCheckScanState_t *state,
                                           GeoLineIntrvCheckOutput_t *output) {
  if (output->IntrvFlg == 0) {
    GeoLineIntrvCheckPose_t endPose;
    Boolean endCollisionFlg;
    Integer endCollisionType;
    VehicleCollisionCheckOutput_t endCollisionDetails;

    endPose.X = input->Pt2X;
    endPose.Y = input->Pt2Y;
    endPose.Theta = input->Pt2Theta;
    geoLineIntrvCheckEvalWorkCollision(input,
                                       work,
                                       VCC_COLLISION_MODEL_EIGHT_EDGES,
                                       work->RefineCollisionDemand,
                                       &endPose,
                                       &endCollisionFlg,
                                       &endCollisionType,
                                       &endCollisionDetails);
    state->CheckCount = state->CheckCount + 1;

    if (endCollisionFlg) {
      Boolean shouldRefine;

      output->IntrvFlg = 1;
      if (work->LineLen <= GEO_EPSILON) {
        output->IntrvFlg = 2;
      }

      shouldRefine = input->UsePlanningCheck;
      if (output->IntrvFlg == 2) {
        shouldRefine = false;
      }
      if (shouldRefine) {
        geoLineIntrvCheckRefineCollision(input, work, state, &endPose, endCollisionType, output);
      } else {
        geoLineIntrvCheckSetSimpleCollision(
            state, &endPose, output->IntrvFlg, endCollisionType, output);
      }
    } else {
      geoLineIntrvCheckSetNoCollision(input, work, state, output);
    }
  }
}

/**
 * @brief 沿直线路径按固定步长采样检测位姿，输出全程安全长度和采样点数量（碰撞判定预留外部模块）
 * @en_name geoLineIntrvCheck
 * @cn_name 直线区间检测
 * @type module
 * @param[IN] Real Pt1X 直线起点X
 * @param[IN] Real Pt1Y 直线起点Y
 * @param[IN] Real Pt1Theta 直线起点航向
 * @param[IN] Real Pt2X 直线终点X
 * @param[IN] Real Pt2Y 直线终点Y
 * @param[IN] Real Pt2Theta 直线终点航向
 * @param[IN] Real CheckStep 检测步长
 * @param[IN] Real GoalX 目标点X
 * @param[IN] Real GoalY 目标点Y
 * @param[IN] Real GoalYaw 目标航向
 * @param[IN] Real SlotP1X 车位角点1X
 * @param[IN] Real SlotP1Y 车位角点1Y
 * @param[IN] Real SlotP2X 车位角点2X
 * @param[IN] Real SlotP2Y 车位角点2Y
 * @param[IN] Real SlotP3X 车位角点3X
 * @param[IN] Real SlotP3Y 车位角点3Y
 * @param[IN] Real SlotP4X 车位角点4X
 * @param[IN] Real SlotP4Y 车位角点4Y
 * @param[IN] Real VehWidth 车宽
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
 * @source matlab_rewrite geo_line_Intrv_check.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
void geoLineIntrvCheck(const GeoLineIntrvCheckInput_t *input, GeoLineIntrvCheckOutput_t *output) {
  geoLineIntrvCheckWithScene(input, (const ApaPlanningSceneQuery_t *)0, output);
}

void geoLineIntrvCheckWithScene(const GeoLineIntrvCheckInput_t *input,
                                const ApaPlanningSceneQuery_t *scene,
                                GeoLineIntrvCheckOutput_t *output) {
  GeoLineIntrvCheckWork_t work;
  GeoLineIntrvCheckScanState_t state;

  work = (GeoLineIntrvCheckWork_t){0};
  state = (GeoLineIntrvCheckScanState_t){0};
  geoLineIntrvCheckInitOutput(input, output);
  geoLineIntrvCheckPrepareWork(input, &work);
  work.Scene = scene;
  if (scene && scene->CheckPose) {
    if (input->VehWidth > GEO_EPSILON && input->VehFrontOffset > GEO_EPSILON)
      work.HaveEnvelope = true;
    const Boolean htParIn = input->UsePlanningCheck &&
                            input->SlotClass == VCC_SLOT_CLASS_PARALLEL && !input->ParkingOutFlag;
    if (!htParIn && work.StepLocal > 0.05f)
      work.StepLocal = 0.05f;
  }
  geoLineIntrvCheckInitScanState(input, &state);
  geoLineIntrvCheckRunSamples(input, &work, &state, output);
  geoLineIntrvCheckHandleEndPose(input, &work, &state, output);
}

/**
 * @brief 泊出直线区间碰撞检测包装，输出碰撞类型、可用点和碰撞点
 * @en_name geoLineIntrvCheckPkOt
 * @cn_name 泊出直线区间检测
 * @type module
 * @param[IN] GeoLineIntrvCheckInput_t input 直线检测输入
 * @param[OUT] GeoLineIntrvCheckOutput_t output 直线检测输出
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 泊出碰撞检测
 * @source matlab_rewrite geo_line_Intrv_check_PkOt.m
 * @version 1.0
 * @date 2026-04-30
 * @author aide
 */
void geoLineIntrvCheckPkOt(const GeoLineIntrvCheckInput_t *input,
                           GeoLineIntrvCheckOutput_t *output) {
  GeoLineIntrvCheckInput_t pkOtInput;

  pkOtInput = *input;
  pkOtInput.UsePlanningCheck = true;
  pkOtInput.ParkingOutFlag = true;
  geoLineIntrvCheck(&pkOtInput, output);
}
