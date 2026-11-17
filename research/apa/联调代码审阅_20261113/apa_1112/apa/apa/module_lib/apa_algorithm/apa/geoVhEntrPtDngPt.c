#include "geoVhEntrPtDngPtTypes.h"
#include "geoLineCrossPtTypes.h"
#include "geoAxisTransTypes.h"
#include "geoDivideTypes.h"
#include <math.h>

#ifndef APA_PI
#define APA_PI 3.14159265f
#endif
#ifndef APA_HALF_PI
#define APA_HALF_PI 1.57079632f
#endif

static inline void geoVhEntrPtDngPtCheckBoundaryLineSegment(Real pt1X,
                                                            Real pt1Y,
                                                            Real pt2X,
                                                            Real pt2Y,
                                                            Real ct1X,
                                                            Real ct1Y,
                                                            Real rOutDng,
                                                            Real lnTr1,
                                                            Real lnTr2,
                                                            Real lnTr3,
                                                            Real lnHtr1,
                                                            Real lnHtr2,
                                                            Real lnHtr3,
                                                            Real lnCtr1,
                                                            Real lnCtr2,
                                                            Real lnCtr3,
                                                            Real lnSbr1,
                                                            Real lnSbr2,
                                                            Real lnSbr3,
                                                            Integer *ptEx) {
  Real lnB1, lnB2, lnB3, ptH1, ptH2, den;
  lnB1 = pt2Y - pt1Y;
  lnB2 = pt1X - pt2X;
  lnB3 = pt2X * pt1Y - pt1X * pt2Y;

  den = lnB1 * lnB1 + lnB2 * lnB2;
  if (den > 0.1f) {
    ptH1 = geoDivideScalar(lnB2 * lnB2 * ct1X - lnB1 * lnB2 * ct1Y - lnB1 * lnB3, den);
    ptH2 = geoDivideScalar(lnB1 * lnB1 * ct1Y - lnB1 * lnB2 * ct1X - lnB2 * lnB3, den);
    Boolean condBetween =
        (((pt1X - ptH1) * (pt2X - ptH1) <= 0.0f) && ((pt1Y - ptH2) * (pt2Y - ptH2) <= 0.0f));
    if (condBetween) {
      Real valT = lnTr1 * ptH1 + lnTr2 * ptH2 + lnTr3;
      Real valSB = lnSbr1 * ptH1 + lnSbr2 * ptH2 + lnSbr3;
      Real valH = lnHtr1 * ptH1 + lnHtr2 * ptH2 + lnHtr3;
      Real valC = lnCtr1 * ptH1 + lnCtr2 * ptH2 + lnCtr3;
      if (((valT * valSB) < 0.0f) && ((valH * valC) < 0.0f)) {
        Real dSqr = (ct1X - ptH1) * (ct1X - ptH1) + (ct1Y - ptH2) * (ct1Y - ptH2);
        if (dSqr < (rOutDng * rOutDng)) {
          *ptEx = 1;
        }
      }
    } else {
      Real d1Sqr = (ct1X - pt1X) * (ct1X - pt1X) + (ct1Y - pt1Y) * (ct1Y - pt1Y);
      Real d2Sqr = (ct1X - pt2X) * (ct1X - pt2X) + (ct1Y - pt2Y) * (ct1Y - pt2Y);
      if (d1Sqr < d2Sqr) {
        Real valT = lnTr1 * pt1X + lnTr2 * pt1Y + lnTr3;
        Real valSB = lnSbr1 * pt1X + lnSbr2 * pt1Y + lnSbr3;
        Real valH = lnHtr1 * pt1X + lnHtr2 * pt1Y + lnHtr3;
        Real valC = lnCtr1 * pt1X + lnCtr2 * pt1Y + lnCtr3;
        if (((valT * valSB) < 0.0f) && ((valH * valC) < 0.0f) && (d1Sqr < (rOutDng * rOutDng))) {
          *ptEx = 1;
        }
      } else {
        Real valT = lnTr1 * pt2X + lnTr2 * pt2Y + lnTr3;
        Real valSB = lnSbr1 * pt2X + lnSbr2 * pt2Y + lnSbr3;
        Real valH = lnHtr1 * pt2X + lnHtr2 * pt2Y + lnHtr3;
        Real valC = lnCtr1 * pt2X + lnCtr2 * pt2Y + lnCtr3;
        if (((valT * valSB) < 0.0f) && ((valH * valC) < 0.0f) && (d2Sqr < (rOutDng * rOutDng))) {
          *ptEx = 1;
        }
      }
    }
  }
}

static Integer geoVhEntrPtDngPtCheckBoundaryInterference(const GeoVhEntrPtDngPtInput_t *pIn,
                                                         Real ct1X,
                                                         Real ct1Y,
                                                         Real rOutDng,
                                                         Real lnTr1,
                                                         Real lnTr2,
                                                         Real lnTr3,
                                                         Real lnHtr1,
                                                         Real lnHtr2,
                                                         Real lnHtr3,
                                                         Real lnCtr1,
                                                         Real lnCtr2,
                                                         Real lnCtr3,
                                                         Real lnSbr1,
                                                         Real lnSbr2,
                                                         Real lnSbr3,
                                                         Real lnHWh1,
                                                         Real lnHWh2,
                                                         Real lnHWh3) {
  Integer ptEx = 0;
  Integer ii;
  Real pt1X, pt1Y, pt2X, pt2Y;

  for (ii = 0; ii < pIn->BoundaryLineNum && ii < DGR_LINE_MAX && ptEx == 0; ii = ii + 1) {
    pt1X = pIn->BoundaryP1X[ii];
    pt1Y = pIn->BoundaryP1Y[ii];
    pt2X = pIn->BoundaryP2X[ii];
    pt2Y = pIn->BoundaryP2Y[ii];
    geoVhEntrPtDngPtCheckBoundaryLineSegment(pt1X,
                                             pt1Y,
                                             pt2X,
                                             pt2Y,
                                             ct1X,
                                             ct1Y,
                                             rOutDng,
                                             lnTr1,
                                             lnTr2,
                                             lnTr3,
                                             lnHtr1,
                                             lnHtr2,
                                             lnHtr3,
                                             lnCtr1,
                                             lnCtr2,
                                             lnCtr3,
                                             lnSbr1,
                                             lnSbr2,
                                             lnSbr3,
                                             &ptEx);
  }

  for (ii = 0; ii < pIn->VirtualBoundaryLineNum && ii < DGR_LINE_MAX && ptEx == 0; ii = ii + 1) {
    pt1X = pIn->VirtualBoundaryP1X[ii];
    pt1Y = pIn->VirtualBoundaryP1Y[ii];
    pt2X = pIn->VirtualBoundaryP2X[ii];
    pt2Y = pIn->VirtualBoundaryP2Y[ii];
    geoVhEntrPtDngPtCheckBoundaryLineSegment(pt1X,
                                             pt1Y,
                                             pt2X,
                                             pt2Y,
                                             ct1X,
                                             ct1Y,
                                             rOutDng,
                                             lnTr1,
                                             lnTr2,
                                             lnTr3,
                                             lnHtr1,
                                             lnHtr2,
                                             lnHtr3,
                                             lnCtr1,
                                             lnCtr2,
                                             lnCtr3,
                                             lnSbr1,
                                             lnSbr2,
                                             lnSbr3,
                                             &ptEx);
  }

  for (ii = 0; ii < pIn->BoundarySepPtNum && ii < DGR_PT_MAX && ptEx == 0; ii = ii + 1) {
    Real spX = pIn->BoundarySepPtX[ii];
    Real spY = pIn->BoundarySepPtY[ii];
    Boolean condValidSp = (fabsf(spX) > 1e-3f || fabsf(spY) > 1e-3f);
    if (condValidSp) {
      Real valT = lnTr1 * spX + lnTr2 * spY + lnTr3;
      Real valSB = lnSbr1 * spX + lnSbr2 * spY + lnSbr3;
      Real valH = lnHtr1 * spX + lnHtr2 * spY + lnHtr3;
      Real valC = lnCtr1 * spX + lnCtr2 * spY + lnCtr3;
      if ((valT * valSB) < 0.0f) {
        if ((valH * valC) < 0.0f) {
          Real dSqr = (ct1X - spX) * (ct1X - spX) + (ct1Y - spY) * (ct1Y - spY);
          if (dSqr < (rOutDng * rOutDng)) {
            ptEx = 1;
          }
        }
      }
    }
  }

  for (ii = 0; ii < pIn->StopperPtNum && ii < DGR_PT_MAX && ptEx == 0; ii = ii + 1) {
    Real spX = pIn->StopperPtX[ii];
    Real spY = pIn->StopperPtY[ii];
    Boolean condValidSp = (fabsf(spX) > 1e-3f || fabsf(spY) > 1e-3f);
    if (condValidSp) {
      Real valT = lnTr1 * spX + lnTr2 * spY + lnTr3;
      Real valSB = lnSbr1 * spX + lnSbr2 * spY + lnSbr3;
      Real valHW = lnHWh1 * spX + lnHWh2 * spY + lnHWh3;
      Real valC = lnCtr1 * spX + lnCtr2 * spY + lnCtr3;
      if ((valT * valSB) < 0.0f) {
        if ((valHW * valC) < 0.0f) {
          Real dSqr = (ct1X - spX) * (ct1X - spX) + (ct1Y - spY) * (ct1Y - spY);
          Real tRad = rOutDng - 0.9f;
          if (dSqr < (tRad * tRad)) {
            ptEx = 1;
          }
        }
      }
    }
  }

  return ptEx;
}

/**
 * @brief VH入口点及危险点检测模块
 * @en_name geoVhEntrPtDngPt
 * @cn_name VH入口点及危险点检测
 * @type module
 * @param[IN] GeoVhEntrPtDngPtInput_t Input VH入口点及危险点输入
 * @param[OUT] GeoVhEntrPtDngPtOutput_t Output VH入口点及危险点输出
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 VH垂直头入
 * @source matlab_rewrite geo_vh_entrPt_dngPt.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
void geoVhEntrPtDngPt(const GeoVhEntrPtDngPtInput_t *input, GeoVhEntrPtDngPtOutput_t *output) {
  Real kApaWhlBse = 2.88f;
  Real kApaLf = 3.85f;
  Real kApaWd = 1.9f;
  Real latOfst = 0.35f;
  Real kEnLLim = 0.8f;
  Real chckStp = 0.4f;

  Real rOutDng;
  Real halfWd;
  Real outDngSqr;
  Real chckLTmp = 0.0f;
  Real x1Tmp, y1Tmp;

  Real sinTTgt, cosTTgt;
  Real lnT1, lnT2, lnT3;

  Real hPtX, hPtY, hPtTh;
  Real sinHT, cosHT;
  Real lnHTr1, lnHTr2, lnHTr3;

  Real hwPtX, hwPtY, hwPtTh;
  Real sinHW, cosHW;
  Real lnHWh1, lnHWh2, lnHWh3;

  Real sltEnX1, sltEnY1, sltEnX2, sltEnY2;

  *output = (GeoVhEntrPtDngPtOutput_t){0};

  halfWd = kApaWd / 2.0f;
  outDngSqr = (input->Radius + halfWd) * (input->Radius + halfWd) + kApaLf * kApaLf;
  rOutDng = sqrtf(outDngSqr) + latOfst;

  sinTTgt = sinf(input->TgtTheta);
  cosTTgt = cosf(input->TgtTheta);
  lnT1 = sinTTgt;
  lnT2 = -cosTTgt;
  lnT3 = input->TgtY * cosTTgt - input->TgtX * sinTTgt;

  hPtX = input->TgtX + kApaLf * cosTTgt;
  hPtY = input->TgtY + kApaLf * sinTTgt;
  hPtTh = input->TgtTheta + APA_HALF_PI;
  sinHT = sinf(hPtTh);
  cosHT = cosf(hPtTh);
  lnHTr1 = sinHT;
  lnHTr2 = -cosHT;
  lnHTr3 = hPtY * cosHT - hPtX * sinHT;

  hwPtX = input->TgtX + kApaWhlBse * cosTTgt;
  hwPtY = input->TgtY + kApaWhlBse * sinTTgt;
  hwPtTh = input->TgtTheta + APA_HALF_PI;
  sinHW = sinf(hwPtTh);
  cosHW = cosf(hwPtTh);
  lnHWh1 = sinHW;
  lnHWh2 = -cosHW;
  lnHWh3 = hwPtY * cosHW - hwPtX * sinHW;

  if (input->SltEntr == 1) {
    sltEnX1 = input->SltPt1X;
    sltEnY1 = input->SltPt1Y;
    sltEnX2 = input->SltPt2X;
    sltEnY2 = input->SltPt2Y;
  } else if (input->SltEntr == 2) {
    sltEnX1 = input->SltPt2X;
    sltEnY1 = input->SltPt2Y;
    sltEnX2 = input->SltPt3X;
    sltEnY2 = input->SltPt3Y;
  } else if (input->SltEntr == 3) {
    sltEnX1 = input->SltPt3X;
    sltEnY1 = input->SltPt3Y;
    sltEnX2 = input->SltPt4X;
    sltEnY2 = input->SltPt4Y;
  } else {
    sltEnX1 = input->SltPt4X;
    sltEnY1 = input->SltPt4Y;
    sltEnX2 = input->SltPt1X;
    sltEnY2 = input->SltPt1Y;
  }

  GeoLineCrossPtInput_t crsIn;
  GeoLineCrossPtOutput_t crsOut;
  crsIn.Ln1X0 = sltEnX1;
  crsIn.Ln1Y0 = sltEnY1;
  crsIn.Ln1X1 = sltEnX2;
  crsIn.Ln1Y1 = sltEnY2;
  crsIn.Ln2X0 = input->TgtX;
  crsIn.Ln2Y0 = input->TgtY;
  crsIn.Ln2X1 = input->TgtX + cosTTgt;
  crsIn.Ln2Y1 = input->TgtY + sinTTgt;
  geoLineCrossPt(&crsIn, &crsOut);

  x1Tmp = crsOut.CrossX;
  y1Tmp = crsOut.CrossY;

  GeoAxisTransInput_t axIn;
  GeoAxisTransOutput_t axOut;
  axIn.PtX = x1Tmp;
  axIn.PtY = y1Tmp;
  axIn.PtTheta = 0.0f;
  axIn.Ori1X = 0.0f;
  axIn.Ori1Y = 0.0f;
  axIn.Ori1Theta = 0.0f;
  axIn.Ori2X = input->TgtX;
  axIn.Ori2Y = input->TgtY;
  axIn.Ori2Theta = input->TgtTheta;
  geoAxisTrans(&axIn, &axOut);

  Real sltDpth;
  sltDpth = -axOut.ResultX;

  axIn.PtX = input->TgtX;
  axIn.PtY = input->TgtY;
  axIn.PtTheta = input->TgtTheta;
  axIn.Ori2X = input->VehX;
  axIn.Ori2Y = input->VehY;
  axIn.Ori2Theta = input->VehTheta;
  geoAxisTrans(&axIn, &axOut);
  Real ptR2;
  ptR2 = axOut.ResultY;

  if (input->PrkType == 1) {
    if (input->SurdType == 3 || input->SurdType == 1) {
      chckLTmp = -kApaWhlBse;
    } else {
      chckLTmp = -kApaLf;
    }
  } else if (input->PrkType == 2) {
    if (input->SurdType == 3 || input->SurdType == 2) {
      chckLTmp = -kApaWhlBse;
    } else {
      chckLTmp = -kApaLf;
    }
  } else {
    chckLTmp = -kApaLf;
  }

  Real xOut1, yOut1;
  xOut1 = x1Tmp + chckLTmp * cosTTgt;
  yOut1 = y1Tmp + chckLTmp * sinTTgt;

  Boolean condStartCheck;
  Real limitDpth;
  limitDpth = sltDpth - kEnLLim;
  condStartCheck = (chckLTmp < limitDpth);

  if (ptR2 > 0.0f) {
    while (condStartCheck) {
      Real ctPt1, ctPt2;
      Real sinCt, cosCt;
      Real lnCt1, lnCt2, lnCt3;
      Real sinSB, cosSB;
      Real lnSb1, lnSb2, lnSb3;
      Integer intrvFlg;

      ctPt1 = x1Tmp + chckLTmp * cosTTgt + input->Radius * cosf(input->TgtTheta + APA_HALF_PI);
      ctPt2 = y1Tmp + chckLTmp * sinTTgt + input->Radius * sinf(input->TgtTheta + APA_HALF_PI);

      sinCt = sinf(input->TgtTheta + APA_HALF_PI);
      cosCt = cosf(input->TgtTheta + APA_HALF_PI);
      lnCt1 = sinCt;
      lnCt2 = -cosCt;
      lnCt3 = ctPt2 * cosCt - ctPt1 * sinCt;

      sinSB = sinTTgt;
      cosSB = cosTTgt;
      lnSb1 = sinSB;
      lnSb2 = -cosSB;
      lnSb3 = (ctPt2 - rOutDng * cosSB) * cosSB - (ctPt1 + rOutDng * sinSB) * sinSB;

      intrvFlg = geoVhEntrPtDngPtCheckBoundaryInterference(input,
                                                           ctPt1,
                                                           ctPt2,
                                                           rOutDng,
                                                           lnT1,
                                                           lnT2,
                                                           lnT3,
                                                           lnHTr1,
                                                           lnHTr2,
                                                           lnHTr3,
                                                           lnCt1,
                                                           lnCt2,
                                                           lnCt3,
                                                           lnSb1,
                                                           lnSb2,
                                                           lnSb3,
                                                           lnHWh1,
                                                           lnHWh2,
                                                           lnHWh3);

      if (intrvFlg == 1) {
        condStartCheck = 0;
      } else {
        xOut1 = x1Tmp + chckLTmp * cosTTgt;
        yOut1 = y1Tmp + chckLTmp * sinTTgt;
        chckLTmp = chckLTmp + chckStp;
        condStartCheck = (chckLTmp < limitDpth);
      }
    }
  } else {
    while (condStartCheck) {
      Real ctPt1, ctPt2;
      Real sinCt, cosCt;
      Real lnCt1, lnCt2, lnCt3;
      Real sinSB, cosSB;
      Real lnSb1, lnSb2, lnSb3;
      Integer intrvFlg;

      ctPt1 = x1Tmp + chckLTmp * cosTTgt + input->Radius * cosf(input->TgtTheta - APA_HALF_PI);
      ctPt2 = y1Tmp + chckLTmp * sinTTgt + input->Radius * sinf(input->TgtTheta - APA_HALF_PI);

      sinCt = sinf(input->TgtTheta + APA_HALF_PI);
      cosCt = cosf(input->TgtTheta + APA_HALF_PI);
      lnCt1 = sinCt;
      lnCt2 = -cosCt;
      lnCt3 = ctPt2 * cosCt - ctPt1 * sinCt;

      sinSB = sinTTgt;
      cosSB = cosTTgt;
      lnSb1 = sinSB;
      lnSb2 = -cosSB;
      lnSb3 = (ctPt2 + rOutDng * cosSB) * cosSB - (ctPt1 - rOutDng * sinSB) * sinSB;

      intrvFlg = geoVhEntrPtDngPtCheckBoundaryInterference(input,
                                                           ctPt1,
                                                           ctPt2,
                                                           rOutDng,
                                                           lnT1,
                                                           lnT2,
                                                           lnT3,
                                                           lnHTr1,
                                                           lnHTr2,
                                                           lnHTr3,
                                                           lnCt1,
                                                           lnCt2,
                                                           lnCt3,
                                                           lnSb1,
                                                           lnSb2,
                                                           lnSb3,
                                                           lnHWh1,
                                                           lnHWh2,
                                                           lnHWh3);

      if (intrvFlg == 1) {
        condStartCheck = 0;
      } else {
        xOut1 = x1Tmp + chckLTmp * cosTTgt;
        yOut1 = y1Tmp + chckLTmp * sinTTgt;
        chckLTmp = chckLTmp + chckStp;
        condStartCheck = (chckLTmp < limitDpth);
      }
    }
  }

  output->X1 = xOut1;
  output->Y1 = yOut1;
}
