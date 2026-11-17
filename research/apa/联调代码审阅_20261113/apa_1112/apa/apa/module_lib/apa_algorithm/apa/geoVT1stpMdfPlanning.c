#include "apaSlotTypes.h"
#include "apaCountUtils.h"
#include "apaMathUtils.h"
#include "geoAxisTransTypes.h"
#include "geoCircleIntrvCheckTypes.h"
#include "geoDivideTypes.h"
#include "geoRev2CircleLinkTypes.h"
#include "geoStPtSpcVtTypes.h"
#include "geoVT1stpMdfPlanningTypes.h"
#include <math.h>

#ifndef GEO_VT_1STP_SAFE_MARGIN
#define GEO_VT_1STP_SAFE_MARGIN 0.4f
#endif

#ifndef GEO_VT_1STP_CHECK_STEP
#define GEO_VT_1STP_CHECK_STEP 0.2f
#endif

#ifndef GEO_VT_1STP_FALLBACK_SPACE
#define GEO_VT_1STP_FALLBACK_SPACE 4.1f
#endif

typedef struct {
  Real X;
  Real Y;
  Real Theta;
} GeoVT1stpPose_t;

static void geoVT1stpMdfPlanningResetOutput(GeoVT1stpMdfPlanningOutput_t *output) {
  Integer idx0;

  output->PthPlnFlg = 0;
  output->StpPthTypLN = 0;
  output->Pt1X = 0.0f;
  output->Pt1Y = 0.0f;
  output->PlnChkCnt = 0;
  output->PathLen = 0.0f;
  output->StpN = 0;
  output->PathSegNum = 0;

  idx0 = 0;
  while (idx0 < APA_PATH_SEG_MAX) {
    output->StpPthTypList[idx0] = 0;
    output->FailRsOut[idx0] = 0;
    output->PathGoDir[idx0] = 0;
    output->PathTrnDir[idx0] = 0;
    output->PathCrv[idx0] = 0.0f;
    output->PathTht[idx0] = 0.0f;
    output->PathLenArr[idx0] = 0.0f;
    output->PathStx[idx0] = 0.0f;
    output->PathSty[idx0] = 0.0f;
    output->PathStTh[idx0] = 0.0f;
    idx0 = idx0 + 1;
  }

  idx0 = 0;
  while (idx0 < APA_PATH_CC_MAX) {
    output->PathCC[idx0] = 0.0f;
    idx0 = idx0 + 1;
  }
}

static Real geoVT1stpMdfPlanningResolveRmin(const GeoVT1stpMdfPlanningInput_t *input) {
  Real rmin0;

  if (input->SltState == 1) {
    rmin0 = input->RminSuccSlot;
  } else {
    if (input->SltState == 3) {
      rmin0 = input->RminSuccSlot;
    } else {
      rmin0 = input->RminDefault;
    }
  }

  return rmin0;
}

static void geoVT1stpMdfPlanningAxisTrans(Real ptX,
                                          Real ptY,
                                          Real ptTheta,
                                          Real ori1X,
                                          Real ori1Y,
                                          Real ori1Theta,
                                          Real ori2X,
                                          Real ori2Y,
                                          Real ori2Theta,
                                          GeoVT1stpPose_t *pose) {
  GeoAxisTransInput_t transIn;
  GeoAxisTransOutput_t transOut;

  transIn = (GeoAxisTransInput_t){0};
  transOut = (GeoAxisTransOutput_t){0};
  transIn.PtX = ptX;
  transIn.PtY = ptY;
  transIn.PtTheta = ptTheta;
  transIn.Ori1X = ori1X;
  transIn.Ori1Y = ori1Y;
  transIn.Ori1Theta = ori1Theta;
  transIn.Ori2X = ori2X;
  transIn.Ori2Y = ori2Y;
  transIn.Ori2Theta = ori2Theta;
  geoAxisTrans(&transIn, &transOut);

  pose->X = transOut.ResultX;
  pose->Y = transOut.ResultY;
  pose->Theta = transOut.ResultTheta;
}

static void geoVT1stpMdfPlanningFillVccTemplate(const GeoVT1stpMdfPlanningInput_t *input,
                                                Real vehX,
                                                Real vehY,
                                                Real vehTheta,
                                                VehicleCollisionCheckInput_t *vccInput) {
  Integer idx0;
  Integer boundaryCount0;

  *vccInput = (VehicleCollisionCheckInput_t){0};
  vccInput->CheckType = VCC_CHECK_TYPE_PLANNING;
  vccInput->CollisionModel = VCC_COLLISION_MODEL_FOUR_EDGES;
  vccInput->CollisionDemand = VCC_COLLISION_DEMAND_SIMPLIFY;
  vccInput->SlotClass = VCC_SLOT_CLASS_VERTICAL;
  vccInput->SlotPtsValid = true;
  vccInput->SlotPts[0] = input->SltPt1X;
  vccInput->SlotPts[1] = input->SltPt1Y;
  vccInput->SlotPts[2] = input->SltPt2X;
  vccInput->SlotPts[3] = input->SltPt2Y;
  vccInput->SlotPts[4] = input->SltPt3X;
  vccInput->SlotPts[5] = input->SltPt3Y;
  vccInput->SlotPts[6] = input->SltPt4X;
  vccInput->SlotPts[7] = input->SltPt4Y;
  apaSlotOrderFromEntrance(input->SltEntr,
                           &vccInput->SlotPts[0],
                           &vccInput->SlotPts[1],
                           &vccInput->SlotPts[2],
                           &vccInput->SlotPts[3],
                           &vccInput->SlotPts[4],
                           &vccInput->SlotPts[5],
                           &vccInput->SlotPts[6],
                           &vccInput->SlotPts[7]);
  vccInput->TarPosValid = true;
  vccInput->TarPosX = input->TarX;
  vccInput->TarPosY = input->TarY;
  vccInput->TarPosTheta = input->TarTheta;
  vccInput->VehPosX = vehX;
  vccInput->VehPosY = vehY;
  vccInput->VehPosTheta = vehTheta;
  vccInput->VehLF = input->VehLlF + input->VehLfMin;
  vccInput->VehLB = input->VehLr;
  vccInput->VehLL = input->VehLlF + input->VehLfMin + input->VehLr;
  vccInput->VehW = input->VehW;
  vccInput->VehWB = input->VehLlF;

  boundaryCount0 = apaClampCount(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX);
  vccInput->BoundaryLineNum = boundaryCount0;
  idx0 = 0;
  while (idx0 < boundaryCount0) {
    vccInput->BoundaryLineX1[idx0] = input->BoundaryLineX1[idx0];
    vccInput->BoundaryLineY1[idx0] = input->BoundaryLineY1[idx0];
    vccInput->BoundaryLineX2[idx0] = input->BoundaryLineX2[idx0];
    vccInput->BoundaryLineY2[idx0] = input->BoundaryLineY2[idx0];
    idx0 = idx0 + 1;
  }
}

static void geoVT1stpMdfPlanningFillStSpace(const GeoVT1stpMdfPlanningInput_t *input,
                                            GeoStPtSpcVtInput_t *spcIn) {
  VehicleCollisionCheckInput_t vccTemplate;

  *spcIn = (GeoStPtSpcVtInput_t){0};
  spcIn->VehPosX = input->EntrPtX;
  spcIn->VehPosY = input->EntrPtY;
  spcIn->VehPosTheta = input->VehTheta;
  spcIn->TarPtX = input->TarX;
  spcIn->TarPtY = input->TarY;
  spcIn->TarPtTheta = input->TarTheta;
  spcIn->SltPt1X = input->SltPt1X;
  spcIn->SltPt1Y = input->SltPt1Y;
  spcIn->SltPt2X = input->SltPt2X;
  spcIn->SltPt2Y = input->SltPt2Y;
  spcIn->SltPt3X = input->SltPt3X;
  spcIn->SltPt3Y = input->SltPt3Y;
  spcIn->SltPt4X = input->SltPt4X;
  spcIn->SltPt4Y = input->SltPt4Y;
  spcIn->SltEntr = input->SltEntr;
  spcIn->CheckLen = 0.0f;
  spcIn->SltType = VCC_SLOT_CLASS_VERTICAL;
  spcIn->FallbackSpace = GEO_VT_1STP_FALLBACK_SPACE;
  spcIn->UseCollisionCheck = true;

  geoVT1stpMdfPlanningFillVccTemplate(
      input, input->EntrPtX, input->EntrPtY, input->VehTheta, &vccTemplate);
  spcIn->VccTemplate = vccTemplate;
}

static void geoVT1stpMdfPlanningFillCircleCheck(const GeoVT1stpMdfPlanningInput_t *input,
                                                Real cptX,
                                                Real cptY,
                                                Real radius,
                                                Real pt1X,
                                                Real pt1Y,
                                                Real pt1Theta,
                                                Real pt2X,
                                                Real pt2Y,
                                                Real pt2Theta,
                                                Integer goDir,
                                                Integer trnDir,
                                                GeoCircleIntrvCheckInput_t *chkIn) {
  Integer idx0;
  Integer boundaryCount0;

  *chkIn = (GeoCircleIntrvCheckInput_t){0};
  chkIn->CptX = cptX;
  chkIn->CptY = cptY;
  chkIn->Radius = radius;
  chkIn->Pt1X = pt1X;
  chkIn->Pt1Y = pt1Y;
  chkIn->Pt1Theta = pt1Theta;
  chkIn->Pt2X = pt2X;
  chkIn->Pt2Y = pt2Y;
  chkIn->Pt2Theta = pt2Theta;
  chkIn->GoDir = goDir;
  chkIn->TrnDir = trnDir;
  chkIn->CheckStep = GEO_VT_1STP_CHECK_STEP;
  chkIn->GoalX = input->TarX;
  chkIn->GoalY = input->TarY;
  chkIn->GoalYaw = input->TarTheta;
  chkIn->SlotP1X = input->SltPt1X;
  chkIn->SlotP1Y = input->SltPt1Y;
  chkIn->SlotP2X = input->SltPt2X;
  chkIn->SlotP2Y = input->SltPt2Y;
  chkIn->SlotP3X = input->SltPt3X;
  chkIn->SlotP3Y = input->SltPt3Y;
  chkIn->SlotP4X = input->SltPt4X;
  chkIn->SlotP4Y = input->SltPt4Y;
  apaSlotOrderFromEntrance(input->SltEntr,
                           &chkIn->SlotP1X,
                           &chkIn->SlotP1Y,
                           &chkIn->SlotP2X,
                           &chkIn->SlotP2Y,
                           &chkIn->SlotP3X,
                           &chkIn->SlotP3Y,
                           &chkIn->SlotP4X,
                           &chkIn->SlotP4Y);
  chkIn->SlotClass = VCC_SLOT_CLASS_VERTICAL;
  chkIn->VehWidth = input->VehW;
  chkIn->VehFrontOffset = input->VehLlF + input->VehLfMin * 0.5f;
  chkIn->VehRearOffset = input->VehLr;
  chkIn->UsePlanningCheck = true;
  chkIn->ParkingOutFlag = false;
  chkIn->DirOut = 0;

  boundaryCount0 = apaClampCount(input->BoundaryLineNum, GEO_BOUNDARY_LINE_MAX);
  chkIn->BoundaryLineNum = boundaryCount0;
  idx0 = 0;
  while (idx0 < boundaryCount0) {
    chkIn->BoundaryLineX1[idx0] = input->BoundaryLineX1[idx0];
    chkIn->BoundaryLineY1[idx0] = input->BoundaryLineY1[idx0];
    chkIn->BoundaryLineX2[idx0] = input->BoundaryLineX2[idx0];
    chkIn->BoundaryLineY2[idx0] = input->BoundaryLineY2[idx0];
    idx0 = idx0 + 1;
  }
}

static Boolean geoVT1stpMdfPlanningCircleIsSafe(const GeoVT1stpMdfPlanningInput_t *input,
                                                Real cptX,
                                                Real cptY,
                                                Real radius,
                                                const GeoVT1stpPose_t *pt1,
                                                const GeoVT1stpPose_t *pt2,
                                                Integer goDir,
                                                Integer trnDir,
                                                Integer *checkCount) {
  GeoCircleIntrvCheckInput_t chkIn;
  GeoCircleIntrvCheckOutput_t chkOut;
  Boolean safe0;

  chkIn = (GeoCircleIntrvCheckInput_t){0};
  chkOut = (GeoCircleIntrvCheckOutput_t){0};
  geoVT1stpMdfPlanningFillCircleCheck(input,
                                      cptX,
                                      cptY,
                                      radius,
                                      pt1->X,
                                      pt1->Y,
                                      pt1->Theta,
                                      pt2->X,
                                      pt2->Y,
                                      pt2->Theta,
                                      goDir,
                                      trnDir,
                                      &chkIn);
  geoCircleIntrvCheck(&chkIn, &chkOut);
  *checkCount = *checkCount + chkOut.CheckCount;

  if (chkOut.IntrvFlg > 0) {
    safe0 = false;
  } else {
    safe0 = true;
  }

  return safe0;
}

static void geoVT1stpMdfPlanningStorePathLenAndStp(GeoVT1stpMdfPlanningOutput_t *output) {
  Integer idx0;
  Integer dirTmp0;
  Integer stpN0;

  output->PathLen = 0.0f;
  idx0 = 0;
  while (idx0 < output->PathSegNum) {
    output->PathLen = output->PathLen + output->PathLenArr[idx0];
    idx0 = idx0 + 1;
  }

  stpN0 = 0;
  if (output->PathSegNum > 0) {
    dirTmp0 = output->PathGoDir[0];
    if (dirTmp0 < 0) {
      stpN0 = 1;
    }
    idx0 = 1;
    while (idx0 < output->PathSegNum) {
      if ((output->PathGoDir[idx0] * dirTmp0) < 0) {
        stpN0 = stpN0 + 1;
        dirTmp0 = output->PathGoDir[idx0];
      }
      idx0 = idx0 + 1;
    }
  }

  output->StpN = stpN0;
}

static void geoVT1stpMdfPlanningWriteFourSeg(const GeoVT1stpMdfPlanningInput_t *input,
                                             Real radius,
                                             const GeoVT1stpPose_t *st2,
                                             const GeoVT1stpPose_t *st3,
                                             const GeoVT1stpPose_t *st4,
                                             Real ct1X,
                                             Real ct1Y,
                                             Real ct2X,
                                             Real ct2Y,
                                             Real ct1RelX,
                                             Real lenArc1,
                                             Real lenArc2,
                                             Integer trnDir1,
                                             Integer trnDir2,
                                             GeoVT1stpMdfPlanningOutput_t *output) {
  Real dxTail0;
  Real dyTail0;
  Real tailLen0;

  dxTail0 = st4->X - input->TarX;
  dyTail0 = st4->Y - input->TarY;
  tailLen0 = sqrtf((dxTail0 * dxTail0) + (dyTail0 * dyTail0));

  output->PathSegNum = 4;
  output->PathGoDir[0] = 1;
  output->PathGoDir[1] = 1;
  output->PathGoDir[2] = -1;
  output->PathGoDir[3] = -1;
  output->PathTrnDir[0] = 0;
  output->PathTrnDir[1] = trnDir1;
  output->PathTrnDir[2] = trnDir2;
  output->PathTrnDir[3] = 0;
  output->PathCrv[0] = 0.0f;
  output->PathCrv[1] = (Real)trnDir1 / radius;
  output->PathCrv[2] = (Real)trnDir2 / radius;
  output->PathCrv[3] = 0.0f;
  output->PathTht[0] = 0.0f;
  output->PathTht[1] = st3->Theta - st2->Theta;
  output->PathTht[2] = st4->Theta - st3->Theta;
  output->PathTht[3] = 0.0f;
  output->PathLenArr[0] = ct1RelX;
  output->PathLenArr[1] = lenArc1;
  output->PathLenArr[2] = lenArc2;
  output->PathLenArr[3] = tailLen0;
  output->PathStx[0] = input->VehX;
  output->PathStx[1] = st2->X;
  output->PathStx[2] = st3->X;
  output->PathStx[3] = st4->X;
  output->PathSty[0] = input->VehY;
  output->PathSty[1] = st2->Y;
  output->PathSty[2] = st3->Y;
  output->PathSty[3] = st4->Y;
  output->PathStTh[0] = input->VehTheta;
  output->PathStTh[1] = st2->Theta;
  output->PathStTh[2] = st3->Theta;
  output->PathStTh[3] = st4->Theta;
  output->PathCC[2] = ct1X;
  output->PathCC[3] = ct1Y;
  output->PathCC[4] = ct2X;
  output->PathCC[5] = ct2Y;
  output->PthPlnFlg = 1;

  geoVT1stpMdfPlanningStorePathLenAndStp(output);
}

static Boolean geoVT1stpMdfPlanningTryFourSeg(const GeoVT1stpMdfPlanningInput_t *input,
                                              const GeoVT1stpPose_t *targetRel,
                                              Real selectedSpace,
                                              Real radius,
                                              Boolean targetOnLeft,
                                              Boolean *fourBranchUsed,
                                              Integer *checkCount,
                                              GeoVT1stpMdfPlanningOutput_t *output) {
  Real wdHalf0;
  Real frontTerm0;
  Real circleBase0;
  Real circleDen0;
  Real spaceFree0;
  Real ratio0;
  Real thetaLimit0;
  Real thetaFront0;
  Real thetaOne0;
  Real thetaTwo0;
  Real ct1RelX0;
  Real ct1RelY0;
  Real ct2RelX0;
  Real ct2RelY0;
  Real sinTr0;
  Real cosTr0;
  Real numerator0;
  Integer trnDir1;
  Integer trnDir2;
  GeoVT1stpPose_t ct1;
  GeoVT1stpPose_t ct2;
  GeoVT1stpPose_t st2;
  GeoVT1stpPose_t st3;
  GeoVT1stpPose_t st4;
  GeoVT1stpPose_t st3Rel;
  GeoVT1stpPose_t st4Rel;
  Boolean safeArc1;
  Boolean safeArc2;
  Boolean planned0;

  wdHalf0 = input->VehW * 0.5f;
  frontTerm0 = input->VehLlF + input->VehLfMin;
  circleBase0 = radius - wdHalf0;
  circleDen0 = sqrtf((circleBase0 * circleBase0) + (frontTerm0 * frontTerm0));
  spaceFree0 = selectedSpace - GEO_VT_1STP_SAFE_MARGIN;
  ratio0 = (circleBase0 - spaceFree0) / circleDen0;
  ratio0 = apaMathClampUnit(ratio0);
  thetaLimit0 = acosf(ratio0);
  thetaFront0 = atanf(geoDivideScalar(frontTerm0, circleBase0));

  if (targetOnLeft) {
    thetaOne0 = thetaFront0 - thetaLimit0;
    trnDir1 = -1;
    trnDir2 = 1;
    ct1RelY0 = -radius;
    ct2RelY0 = ct1RelY0 + ((radius + radius) * cosf(thetaOne0));
  } else {
    thetaOne0 = thetaLimit0 - thetaFront0;
    trnDir1 = 1;
    trnDir2 = -1;
    ct1RelY0 = radius;
    ct2RelY0 = ct1RelY0 - ((radius + radius) * cosf(thetaOne0));
  }

  thetaTwo0 = targetRel->Theta - thetaOne0;
  sinTr0 = sinf(targetRel->Theta);
  cosTr0 = cosf(targetRel->Theta);
  if (targetOnLeft) {
    numerator0 = -radius + (targetRel->X * sinTr0) - (targetRel->Y * cosTr0) + (ct2RelY0 * cosTr0);
    ct2RelX0 = geoDivideScalar(numerator0, sinTr0);
    ct1RelX0 = ct2RelX0 + ((radius + radius) * sinf(thetaOne0));
  } else {
    numerator0 = radius + (targetRel->X * sinTr0) - (targetRel->Y * cosTr0) + (ct2RelY0 * cosTr0);
    ct2RelX0 = geoDivideScalar(numerator0, sinTr0);
    ct1RelX0 = ct2RelX0 - ((radius + radius) * sinf(thetaOne0));
  }

  planned0 = false;
  *fourBranchUsed = false;
  if (ct1RelX0 > 0.0f) {
    *fourBranchUsed = true;
    geoVT1stpMdfPlanningAxisTrans(ct1RelX0,
                                  ct1RelY0,
                                  0.0f,
                                  input->VehX,
                                  input->VehY,
                                  input->VehTheta,
                                  0.0f,
                                  0.0f,
                                  0.0f,
                                  &ct1);
    geoVT1stpMdfPlanningAxisTrans(ct2RelX0,
                                  ct2RelY0,
                                  0.0f,
                                  input->VehX,
                                  input->VehY,
                                  input->VehTheta,
                                  0.0f,
                                  0.0f,
                                  0.0f,
                                  &ct2);
    geoVT1stpMdfPlanningAxisTrans(
        ct1RelX0, 0.0f, 0.0f, input->VehX, input->VehY, input->VehTheta, 0.0f, 0.0f, 0.0f, &st2);

    st3Rel.Theta = thetaOne0;
    if (targetOnLeft) {
      st3Rel.X = ct1RelX0 + (radius * cosf(thetaOne0 + APA_HALF_PI));
      st3Rel.Y = ct1RelY0 + (radius * sinf(thetaOne0 + APA_HALF_PI));
      st4Rel.X = ct2RelX0 + (radius * cosf(targetRel->Theta - APA_HALF_PI));
      st4Rel.Y = ct2RelY0 + (radius * sinf(targetRel->Theta - APA_HALF_PI));
    } else {
      st3Rel.X = ct1RelX0 + (radius * cosf(thetaOne0 - APA_HALF_PI));
      st3Rel.Y = ct1RelY0 + (radius * sinf(thetaOne0 - APA_HALF_PI));
      st4Rel.X = ct2RelX0 + (radius * cosf(targetRel->Theta + APA_HALF_PI));
      st4Rel.Y = ct2RelY0 + (radius * sinf(targetRel->Theta + APA_HALF_PI));
    }
    st4Rel.Theta = targetRel->Theta;

    geoVT1stpMdfPlanningAxisTrans(st3Rel.X,
                                  st3Rel.Y,
                                  st3Rel.Theta,
                                  input->VehX,
                                  input->VehY,
                                  input->VehTheta,
                                  0.0f,
                                  0.0f,
                                  0.0f,
                                  &st3);
    geoVT1stpMdfPlanningAxisTrans(st4Rel.X,
                                  st4Rel.Y,
                                  st4Rel.Theta,
                                  input->VehX,
                                  input->VehY,
                                  input->VehTheta,
                                  0.0f,
                                  0.0f,
                                  0.0f,
                                  &st4);

    safeArc1 = geoVT1stpMdfPlanningCircleIsSafe(
        input, ct1.X, ct1.Y, radius, &st2, &st3, 1, trnDir1, checkCount);
    if (safeArc1) {
      safeArc2 = geoVT1stpMdfPlanningCircleIsSafe(
          input, ct2.X, ct2.Y, radius, &st3, &st4, -1, trnDir2, checkCount);
      if (safeArc2) {
        geoVT1stpMdfPlanningWriteFourSeg(input,
                                         radius,
                                         &st2,
                                         &st3,
                                         &st4,
                                         ct1.X,
                                         ct1.Y,
                                         ct2.X,
                                         ct2.Y,
                                         ct1RelX0,
                                         fabsf(thetaOne0 * radius),
                                         fabsf(thetaTwo0 * radius),
                                         trnDir1,
                                         trnDir2,
                                         output);
        planned0 = true;
      }
    }
  }

  return planned0;
}

static Boolean geoVT1stpMdfPlanningTryThreeSeg(const GeoVT1stpMdfPlanningInput_t *input,
                                               Real radius,
                                               Integer *checkCount,
                                               GeoVT1stpMdfPlanningOutput_t *output) {
  GeoRev2CircleLinkInput_t revIn;
  GeoRev2CircleLinkOutput_t revOut;
  GeoVT1stpPose_t st1;
  GeoVT1stpPose_t st2;
  GeoVT1stpPose_t st3;
  Boolean safeArc1;
  Boolean safeArc2;
  Real dxTail0;
  Real dyTail0;
  Real tailLen0;
  Boolean planned0;

  revIn = (GeoRev2CircleLinkInput_t){0};
  revOut = (GeoRev2CircleLinkOutput_t){0};
  revIn.Pt1X = input->VehX;
  revIn.Pt1Y = input->VehY;
  revIn.Pt1Theta = input->VehTheta;
  revIn.Pt2X = input->TarX;
  revIn.Pt2Y = input->TarY;
  revIn.Pt2Theta = input->TarTheta;
  revIn.Dir = 1;
  revIn.R1 = radius;
  revIn.R2 = radius;
  geoRev2CircleLink(&revIn, &revOut);

  planned0 = false;
  if (revOut.GeoOk == 1) {
    st1.X = input->VehX;
    st1.Y = input->VehY;
    st1.Theta = input->VehTheta;
    st2.X = revOut.St2X;
    st2.Y = revOut.St2Y;
    st2.Theta = revOut.St2Theta;
    st3.X = revOut.St3X;
    st3.Y = revOut.St3Y;
    st3.Theta = revOut.St3Theta;

    safeArc1 = geoVT1stpMdfPlanningCircleIsSafe(
        input, revOut.Ct1X, revOut.Ct1Y, radius, &st1, &st2, 1, revOut.TrnD1, checkCount);
    if (safeArc1) {
      safeArc2 = geoVT1stpMdfPlanningCircleIsSafe(
          input, revOut.Ct2X, revOut.Ct2Y, radius, &st2, &st3, -1, revOut.TrnD2, checkCount);
      if (safeArc2) {
        dxTail0 = st3.X - input->TarX;
        dyTail0 = st3.Y - input->TarY;
        tailLen0 = sqrtf((dxTail0 * dxTail0) + (dyTail0 * dyTail0));

        output->PathSegNum = 3;
        output->PathGoDir[0] = 1;
        output->PathGoDir[1] = -1;
        output->PathGoDir[2] = -1;
        output->PathTrnDir[0] = revOut.TrnD1;
        output->PathTrnDir[1] = revOut.TrnD2;
        output->PathTrnDir[2] = 0;
        output->PathCrv[0] = (Real)revOut.TrnD1 / radius;
        output->PathCrv[1] = (Real)revOut.TrnD2 / radius;
        output->PathCrv[2] = 0.0f;
        output->PathTht[0] = st2.Theta - st1.Theta;
        output->PathTht[1] = st3.Theta - st2.Theta;
        output->PathTht[2] = 0.0f;
        output->PathLenArr[0] = revOut.Len1;
        output->PathLenArr[1] = revOut.Len2;
        output->PathLenArr[2] = tailLen0;
        output->PathStx[0] = st1.X;
        output->PathStx[1] = st2.X;
        output->PathStx[2] = st3.X;
        output->PathSty[0] = st1.Y;
        output->PathSty[1] = st2.Y;
        output->PathSty[2] = st3.Y;
        output->PathStTh[0] = st1.Theta;
        output->PathStTh[1] = st2.Theta;
        output->PathStTh[2] = st3.Theta;
        output->PathCC[0] = revOut.Ct1X;
        output->PathCC[1] = revOut.Ct1Y;
        output->PathCC[2] = revOut.Ct2X;
        output->PathCC[3] = revOut.Ct2Y;
        output->PthPlnFlg = 1;

        geoVT1stpMdfPlanningStorePathLenAndStp(output);
        planned0 = true;
      }
    }
  }

  return planned0;
}

/**
 * @brief VT单步修正主规划，按 MATLAB geo_VT_1stpMdfPlanning.m 的空间判定、
 *        4段预留安全距离构型和3段反双圆fallback生成路径。
 * @en_name geoVT1stpMdfPlanning
 * @cn_name VT单步修正规划
 * @type module
 * @param[IN] Real VehX 车辆X
 * @param[IN] Real VehY 车辆Y
 * @param[IN] Real VehTheta 车辆航向
 * @param[IN] Real TarX 目标X
 * @param[IN] Real TarY 目标Y
 * @param[IN] Real TarTheta 目标航向
 * @param[IN] Real EntrPtX 修正点X
 * @param[IN] Real EntrPtY 修正点Y
 * @param[IN] Real MdfPtTheta 修正点航向
 * @param[IN] Real SltPt1X 车位角点1X
 * @param[IN] Real SltPt1Y 车位角点1Y
 * @param[IN] Real SltPt2X 车位角点2X
 * @param[IN] Real SltPt2Y 车位角点2Y
 * @param[IN] Real SltPt3X 车位角点3X
 * @param[IN] Real SltPt3Y 车位角点3Y
 * @param[IN] Real SltPt4X 车位角点4X
 * @param[IN] Real SltPt4Y 车位角点4Y
 * @param[IN] Integer SltEntr 车位入口边编号
 * @param[IN] Integer SltState 车位状态
 * @param[IN] Real RminDefault 默认最小半径
 * @param[IN] Real RminSuccSlot 成功车位最小半径
 * @param[IN] Real VehW 车宽
 * @param[IN] Real VehLlF 前悬到参考点长度
 * @param[IN] Real VehLfMin 最小前向安全长度
 * @param[OUT] Integer PthPlnFlg 规划成功标志
 * @param[OUT] Integer StpPthTypLN 路径类型序列长度
 * @param[OUT] Integer[30] StpPthTypList 路径类型序列
 * @param[OUT] Integer[30] FailRsOut 失败原因序列
 * @param[OUT] Real Pt1X 保留输出X
 * @param[OUT] Real Pt1Y 保留输出Y
 * @param[OUT] Integer PlnChkCnt 规划检查次数
 * @param[OUT] Real PathLen 路径总长
 * @param[OUT] Integer StpN 换向次数
 * @param[OUT] Integer PathSegNum 路径段数
 * @param[OUT] Integer[30] PathGoDir 行驶方向数组
 * @param[OUT] Integer[30] PathTrnDir 转向数组
 * @param[OUT] Real[30] PathCrv 曲率数组
 * @param[OUT] Real[30] PathTht 转角数组
 * @param[OUT] Real[30] PathLenArr 长度数组
 * @param[OUT] Real[30] PathStx 起点X数组
 * @param[OUT] Real[30] PathSty 起点Y数组
 * @param[OUT] Real[30] PathStTh 起点航向数组
 * @param[OUT] Real[60] PathCC 圆心数组
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 路径规划
 * @source matlab_rewrite geo_VT_1stpMdfPlanning.m
 * @version 1.1
 * @date 2026-05-06
 * @author aide
 */
void geoVT1stpMdfPlanning(const GeoVT1stpMdfPlanningInput_t *input,
                          GeoVT1stpMdfPlanningOutput_t *output) {
  GeoVT1stpPose_t targetRel;
  GeoVT1stpPose_t mdfRel;
  GeoStPtSpcVtInput_t spcIn;
  GeoStPtSpcVtOutput_t spcOut;
  Real radius0;
  Real requiredSpace0;
  Real selectedSpace0;
  Integer checkCount0;
  Boolean targetOnLeft0;
  Boolean spaceEnough0;
  Boolean fourBranchUsed0;
  Boolean planned0;

  geoVT1stpMdfPlanningResetOutput(output);
  radius0 = geoVT1stpMdfPlanningResolveRmin(input);
  checkCount0 = 0;
  planned0 = false;

  if (fabsf(radius0) >= GEO_EPSILON) {
    geoVT1stpMdfPlanningAxisTrans(input->TarX,
                                  input->TarY,
                                  input->TarTheta,
                                  0.0f,
                                  0.0f,
                                  0.0f,
                                  input->VehX,
                                  input->VehY,
                                  input->VehTheta,
                                  &targetRel);
    geoVT1stpMdfPlanningAxisTrans(input->EntrPtX,
                                  input->EntrPtY,
                                  input->MdfPtTheta,
                                  0.0f,
                                  0.0f,
                                  0.0f,
                                  input->VehX,
                                  input->VehY,
                                  input->VehTheta,
                                  &mdfRel);

    spcIn = (GeoStPtSpcVtInput_t){0};
    spcOut = (GeoStPtSpcVtOutput_t){0};
    geoVT1stpMdfPlanningFillStSpace(input, &spcIn);
    geoStPtSpcVt(&spcIn, &spcOut);
    checkCount0 = checkCount0 + spcOut.CheckCount;

    requiredSpace0 =
        ((input->VehLlF + input->VehLfMin) * fabsf(sinf(mdfRel.Theta))) + GEO_VT_1STP_SAFE_MARGIN;
    targetOnLeft0 = (targetRel.Y > 0.0f);
    if (targetOnLeft0) {
      selectedSpace0 = spcOut.RightSpace;
    } else {
      selectedSpace0 = spcOut.LeftSpace;
    }
    spaceEnough0 = (selectedSpace0 >= requiredSpace0);

    if (spaceEnough0) {
      fourBranchUsed0 = false;
      planned0 = geoVT1stpMdfPlanningTryFourSeg(input,
                                                &targetRel,
                                                selectedSpace0,
                                                radius0,
                                                targetOnLeft0,
                                                &fourBranchUsed0,
                                                &checkCount0,
                                                output);
      if (!planned0) {
        if (!fourBranchUsed0) {
          planned0 = geoVT1stpMdfPlanningTryThreeSeg(input, radius0, &checkCount0, output);
        }
      }
    }
  }

  output->PlnChkCnt = checkCount0;
  if (!planned0) {
    output->PthPlnFlg = 0;
    output->PathLen = 0.0f;
    output->StpN = 0;
    output->PathSegNum = 0;
  }
}
