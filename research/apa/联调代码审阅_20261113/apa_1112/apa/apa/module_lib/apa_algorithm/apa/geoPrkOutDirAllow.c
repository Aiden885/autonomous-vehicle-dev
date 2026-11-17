#include "apaCountUtils.h"
#include "geoPrkOutDirAllowTypes.h"
#include "geoAxisTransTypes.h"
#include <math.h>

#ifndef APA_MATH_EPSILON
#define APA_MATH_EPSILON 1e-6f
#endif

#ifndef APA_HALF_PI
#define APA_HALF_PI 1.5707963f
#endif

static inline void geoPrkOutDirAllowProcessLineInfo(const GeoPrkOutDirAllowInput_t *input,
                                                    Integer idx,
                                                    Real ptX,
                                                    Real ptY,
                                                    Real x1,
                                                    Real x2,
                                                    Real y1,
                                                    Real y2,
                                                    Real dx,
                                                    Real dy,
                                                    Boolean isLeft,
                                                    Boolean *allow,
                                                    GeoAxisTransInput_t *transIn,
                                                    GeoAxisTransOutput_t *transOut) {
  if (isLeft) {
    if ((fabsf(input->BoundaryLineX1[idx] - ptX) < 3.0f) &&
        (fabsf(input->BoundaryLineY1[idx] - ptY) < 3.0f)) {
      Real ptrX;
      Real ptrY;
      transIn->PtX = input->BoundaryLineX1[idx];
      transIn->PtY = input->BoundaryLineY1[idx];
      transIn->PtTheta = 0.0f;
      geoAxisTrans(transIn, transOut);
      ptrX = transOut->ResultX;
      ptrY = transOut->ResultY;
      if ((ptrX > x1) && (ptrX < x2) && (ptrY < y2) && (ptrY > y1)) {
        *allow = false;
      }
      if (*allow && (ptrX > (x1 + 1.5f * dx)) && (ptrX < (x2 + dx)) && (ptrY < (y2 + 2.0f * dy)) &&
          (ptrY > (y1 + dy))) {
        *allow = false;
      }
    }
    if (*allow && (fabsf(input->BoundaryLineX2[idx] - ptX) < 3.0f) &&
        (fabsf(input->BoundaryLineY2[idx] - ptY) < 3.0f)) {
      Real ptrX;
      Real ptrY;
      transIn->PtX = input->BoundaryLineX2[idx];
      transIn->PtY = input->BoundaryLineY2[idx];
      transIn->PtTheta = 0.0f;
      geoAxisTrans(transIn, transOut);
      ptrX = transOut->ResultX;
      ptrY = transOut->ResultY;
      if ((ptrX > x1) && (ptrX < x2) && (ptrY < y2) && (ptrY > y1)) {
        *allow = false;
      }
      if (*allow && (ptrX > (x1 + 1.5f * dx)) && (ptrX < (x2 + dx)) && (ptrY < (y2 + 2.0f * dy)) &&
          (ptrY > (y1 + dy))) {
        *allow = false;
      }
    }
  } else {
    if ((fabsf(input->BoundaryLineX1[idx] - ptX) < 3.0f) &&
        (fabsf(input->BoundaryLineY1[idx] - ptY) < 3.0f)) {
      Real ptrX;
      Real ptrY;
      transIn->PtX = input->BoundaryLineX1[idx];
      transIn->PtY = input->BoundaryLineY1[idx];
      transIn->PtTheta = 0.0f;
      geoAxisTrans(transIn, transOut);
      ptrX = transOut->ResultX;
      ptrY = transOut->ResultY;
      if ((ptrX > x1) && (ptrX < x2) && (ptrY < -y1) && (ptrY > -y2)) {
        *allow = false;
      }
      if (*allow && (ptrX > (x1 + 1.5f * dx)) && (ptrX < (x2 + dx)) && (ptrY < (-y1 - dy)) &&
          (ptrY > (-y2 - 2.0f * dy))) {
        *allow = false;
      }
    }
    if (*allow && (fabsf(input->BoundaryLineX2[idx] - ptX) < 3.0f) &&
        (fabsf(input->BoundaryLineY2[idx] - ptY) < 3.0f)) {
      Real ptrX;
      Real ptrY;
      transIn->PtX = input->BoundaryLineX2[idx];
      transIn->PtY = input->BoundaryLineY2[idx];
      transIn->PtTheta = 0.0f;
      geoAxisTrans(transIn, transOut);
      ptrX = transOut->ResultX;
      ptrY = transOut->ResultY;
      if ((ptrX > x1) && (ptrX < x2) && (ptrY < -y1) && (ptrY > -y2)) {
        *allow = false;
      }
      if (*allow && (ptrX > (x1 + 1.5f * dx)) && (ptrX < (x2 + dx)) && (ptrY < (-y1 - dy)) &&
          (ptrY > (-y2 - 2.0f * dy))) {
        *allow = false;
      }
    }
  }
}

static inline void geoPrkOutDirAllowProcessSepPts(const GeoPrkOutDirAllowInput_t *input,
                                                  Integer idx,
                                                  Real ptX,
                                                  Real ptY,
                                                  Real x1,
                                                  Real x2,
                                                  Real y1,
                                                  Real y2,
                                                  Real dx,
                                                  Real dy,
                                                  Boolean *allow,
                                                  GeoAxisTransInput_t *transIn,
                                                  GeoAxisTransOutput_t *transOut) {
  if ((fabsf(input->BoundarySepPtsX[idx] - ptX) < 3.0f) &&
      (fabsf(input->BoundarySepPtsY[idx] - ptY) < 3.0f)) {
    Real ptrX;
    Real ptrY;
    transIn->PtX = input->BoundarySepPtsX[idx];
    transIn->PtY = input->BoundarySepPtsY[idx];
    transIn->PtTheta = 0.0f;
    geoAxisTrans(transIn, transOut);
    ptrX = transOut->ResultX;
    ptrY = transOut->ResultY;
    if ((ptrX > x1) && (ptrX < x2) && (ptrY < y2) && (ptrY > y1)) {
      *allow = false;
    }
    if (*allow && (ptrX > (x1 + 1.5f * dx)) && (ptrX < (x2 + dx)) && (ptrY < (y2 + 2.0f * dy)) &&
        (ptrY > (y1 + dy))) {
      *allow = false;
    }
  }
}

static inline void geoPrkOutDirAllowProcessSepPtsRight(const GeoPrkOutDirAllowInput_t *input,
                                                       Integer idx,
                                                       Real ptX,
                                                       Real ptY,
                                                       Real x1,
                                                       Real x2,
                                                       Real y1,
                                                       Real y2,
                                                       Real dx,
                                                       Real dy,
                                                       Boolean *allow,
                                                       GeoAxisTransInput_t *transIn,
                                                       GeoAxisTransOutput_t *transOut) {
  if ((fabsf(input->BoundarySepPtsX[idx] - ptX) < 3.0f) &&
      (fabsf(input->BoundarySepPtsY[idx] - ptY) < 3.0f)) {
    Real ptrX;
    Real ptrY;
    transIn->PtX = input->BoundarySepPtsX[idx];
    transIn->PtY = input->BoundarySepPtsY[idx];
    transIn->PtTheta = 0.0f;
    geoAxisTrans(transIn, transOut);
    ptrX = transOut->ResultX;
    ptrY = transOut->ResultY;
    if ((ptrX > x1) && (ptrX < x2) && (ptrY < -y1) && (ptrY > -y2)) {
      *allow = false;
    }
    if (*allow && (ptrX > (x1 + 1.5f * dx)) && (ptrX < (x2 + dx)) && (ptrY < (-y1 - dy)) &&
        (ptrY > (-y2 - 2.0f * dy))) {
      *allow = false;
    }
  }
}
static inline void geoPrkOutDirAllowCheckLras(const GeoPrkOutDirAllowInput_t *input,
                                              Boolean prkOutDir[4],
                                              GeoAxisTransInput_t *transIn,
                                              GeoAxisTransOutput_t *transOut) {
  Real ptFar = 0.0f;
  Real lenTmp = 0.0f;
  Integer lrasCount;
  Integer i;
  lrasCount = apaClampCount(input->LrasPtN, MAX_LRAS_PTS);
  if (input->LrasValid && input->LrasCfSw && (lrasCount > 3)) {
    Boolean loopCond1 = true;
    i = 0;
    while ((i < lrasCount) && loopCond1) {
      Real ldx = input->LrasPtsX[i] - input->LrasVehPosX;
      Real ldy = input->LrasPtsY[i] - input->LrasVehPosY;
      lenTmp = ldx * ldx + ldy * ldy;
      if (lenTmp > ptFar) {
        ptFar = lenTmp;
      }
      if (ptFar > 4.0f) {
        loopCond1 = false;
      }
      if (loopCond1) {
        i = i + 1;
      }
    }
  }

  if (input->LrasValid && input->LrasCfSw && (lrasCount > 3) && (ptFar > 4.0f)) {
    Real d0X = input->LrasPtsX[0] - input->LrasVehPosX;
    Real d0Y = input->LrasPtsY[0] - input->LrasVehPosY;
    if ((d0X * d0X + d0Y * d0Y) < 0.25f) {
      Integer ftCnt = 0;
      Integer rrCnt = 0;
      Integer itNum;
      itNum = lrasCount;

      Boolean loopCond2 = true;
      i = 0;
      while ((i < itNum) && loopCond2) {
        Real ptrX;
        Real ptrY;

        transIn->PtX = input->LrasPtsX[i];
        transIn->PtY = input->LrasPtsY[i];
        transIn->PtTheta = 0.0f;
        transIn->Ori2X = input->LrasVehPosX;
        transIn->Ori2Y = input->LrasVehPosY;
        transIn->Ori2Theta = input->LrasVehPosTheta;
        geoAxisTrans(transIn, transOut);

        ptrX = transOut->ResultX;
        ptrY = transOut->ResultY;

        if ((ptrX > 1.2f) && (fabsf(ptrY) < 2.0f)) {
          ftCnt = ftCnt + 1;
        }
        if ((ptrX < -1.2f) && (fabsf(ptrY) < 2.0f)) {
          rrCnt = rrCnt + 1;
        }
        if ((ftCnt > 5) && (rrCnt > 5)) {
          loopCond2 = false;
        }
        if (loopCond2) {
          i = i + 1;
        }
      }
      if (ftCnt > 2) {
        prkOutDir[2] = true;
      }
      if (rrCnt > 2) {
        prkOutDir[3] = true;
      }
    } else {
      prkOutDir[2] = true;
      prkOutDir[3] = true;
    }
  } else {
    prkOutDir[2] = true;
    prkOutDir[3] = true;
  }
}

static inline void geoPrkOutDirAllowCheckStpbp(const GeoPrkOutDirAllowInput_t *input,
                                               Boolean *ltAllow,
                                               Boolean *rtAllow,
                                               Boolean prkOutDir[4],
                                               GeoAxisTransInput_t *transIn,
                                               GeoAxisTransOutput_t *transOut) {
  Integer stopperCount;
  Integer i;
  stopperCount = apaClampCount(input->StpbpN, MAX_STPBP_PTS);
  if (stopperCount > 0) {
    for (i = 0; i < stopperCount; i = i + 1) {
      Real ptrX;
      Real ptrY;

      transIn->PtX = input->StpbpX[i];
      transIn->PtY = input->StpbpY[i];
      transIn->PtTheta = 0.0f;
      transIn->Ori2X = input->VehPosX;
      transIn->Ori2Y = input->VehPosY;
      transIn->Ori2Theta = input->VehPosTheta;
      geoAxisTrans(transIn, transOut);

      ptrX = transOut->ResultX;
      ptrY = transOut->ResultY;

      if ((fabsf(ptrY) < 1.05f) && (ptrX < -1.0f) && (ptrX > -2.0f)) {
        prkOutDir[3] = false;
      }
      if ((fabsf(ptrY) < 1.05f) && (ptrX > 3.5f) && (ptrX < 7.0f)) {
        prkOutDir[2] = false;
      }
    }
  }

  if (stopperCount > 0) {
    for (i = 0; i < stopperCount; i = i + 1) {
      Real ptrX;
      Real ptrY;

      transIn->PtX = input->StpbpX[i];
      transIn->PtY = input->StpbpY[i];
      transIn->PtTheta = 0.0f;
      transIn->Ori2X = input->VehPosX;
      transIn->Ori2Y = input->VehPosY;
      transIn->Ori2Theta = input->VehPosTheta;
      geoAxisTrans(transIn, transOut);

      ptrX = transOut->ResultX;
      ptrY = transOut->ResultY;

      if ((ptrY > 0.8f) && (ptrY < 3.0f) && (ptrX > 0.0f) && (ptrX < 4.0f)) {
        *ltAllow = false;
      }
      if ((ptrY < -0.8f) && (ptrY > -3.0f) && (ptrX > 0.0f) && (ptrX < 4.0f)) {
        *rtAllow = false;
      }
    }
  }
}

static inline void geoPrkOutDirAllowCheckBoundaries(const GeoPrkOutDirAllowInput_t *input,
                                                    Boolean *ltAllow,
                                                    Boolean *rtAllow,
                                                    Real x1,
                                                    Real x2,
                                                    Real y1,
                                                    Real y2,
                                                    Real dx,
                                                    Real dy,
                                                    GeoAxisTransInput_t *transIn,
                                                    GeoAxisTransOutput_t *transOut) {
  Real cVT;
  Real sVT;
  Real frontHang;
  Real halfWidth;
  Real ptFlX;
  Real ptFlY;
  Real ptFrX;
  Real ptFrY;
  Integer i;
  Integer lineCount;
  Integer pointCount;

  lineCount = apaClampCount(input->BoundaryLineNum, VCC_MAX_BOUNDARY_LINES);
  pointCount = apaClampCount(input->BoundarySepPtsNum, VCC_MAX_SEPARATE_PTS);

  cVT = cosf(input->VehPosTheta);
  sVT = sinf(input->VehPosTheta);
  frontHang = input->Para[21];
  halfWidth = input->Para[1] * 0.5f;

  ptFlX = input->VehPosX + frontHang * cVT + halfWidth * cosf(input->VehPosTheta + APA_HALF_PI);
  ptFlY = input->VehPosY + frontHang * sVT + halfWidth * sinf(input->VehPosTheta + APA_HALF_PI);

  if ((lineCount > 0) && (*ltAllow)) {
    i = 0;
    while ((i < lineCount) && (*ltAllow)) {
      geoPrkOutDirAllowProcessLineInfo(
          input, i, ptFlX, ptFlY, x1, x2, y1, y2, dx, dy, true, ltAllow, transIn, transOut);
      if (*ltAllow) {
        i = i + 1;
      }
    }
  }

  if ((pointCount > 0) && (*ltAllow)) {
    i = 0;
    while ((i < pointCount) && (*ltAllow)) {
      geoPrkOutDirAllowProcessSepPts(
          input, i, ptFlX, ptFlY, x1, x2, y1, y2, dx, dy, ltAllow, transIn, transOut);
      if (*ltAllow) {
        i = i + 1;
      }
    }
  }

  ptFrX = input->VehPosX + frontHang * cVT + halfWidth * cosf(input->VehPosTheta - APA_HALF_PI);
  ptFrY = input->VehPosY + frontHang * sVT + halfWidth * sinf(input->VehPosTheta - APA_HALF_PI);

  if ((lineCount > 0) && (*rtAllow)) {
    i = 0;
    while ((i < lineCount) && (*rtAllow)) {
      geoPrkOutDirAllowProcessLineInfo(
          input, i, ptFrX, ptFrY, x1, x2, y1, y2, dx, dy, false, rtAllow, transIn, transOut);
      if (*rtAllow) {
        i = i + 1;
      }
    }
  }

  if ((pointCount > 0) && (*rtAllow)) {
    i = 0;
    while ((i < pointCount) && (*rtAllow)) {
      geoPrkOutDirAllowProcessSepPtsRight(
          input, i, ptFrX, ptFrY, x1, x2, y1, y2, dx, dy, rtAllow, transIn, transOut);
      if (*rtAllow) {
        i = i + 1;
      }
    }
  }
}

/**
 * @file geoPrkOutDirAllow.c
 * @brief 泊出方向容许度判断
 * @en_name geoPrkOutDirAllow
 * @cn_name 泊出方向容许度判断
 * @type module
 * @param[IN] GeoPrkOutDirAllowInput_t Input 泊出方向输入
 * @param[OUT] GeoPrkOutDirAllowOutput_t Output 泊出方向输出
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 几何计算
 * @source matlab_rewrite geo_PrkOut_DirAllow.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
void geoPrkOutDirAllow(const GeoPrkOutDirAllowInput_t *input, GeoPrkOutDirAllowOutput_t *output) {
  Boolean prkOutDir[4];
  GeoAxisTransInput_t transIn;
  GeoAxisTransOutput_t transOut;
  Boolean ltAllow = true;
  Boolean rtAllow = true;
  Real x1 = 1.0f;
  Real x2 = 3.8f;
  Real y1 = 1.0f;
  Real y2 = 1.5f;
  Real dx = 1.0f;
  Real dy = 0.6f;

  prkOutDir[0] = false;
  prkOutDir[1] = false;
  prkOutDir[2] = false;
  prkOutDir[3] = false;

  transIn.Ori1X = 0.0f;
  transIn.Ori1Y = 0.0f;
  transIn.Ori1Theta = 0.0f;
  transIn.Ori2X = input->VehPosX;
  transIn.Ori2Y = input->VehPosY;
  transIn.Ori2Theta = input->VehPosTheta;

  geoPrkOutDirAllowCheckLras(input, prkOutDir, &transIn, &transOut);
  prkOutDir[3] = false;

  geoPrkOutDirAllowCheckStpbp(input, &ltAllow, &rtAllow, prkOutDir, &transIn, &transOut);

  geoPrkOutDirAllowCheckBoundaries(
      input, &ltAllow, &rtAllow, x1, x2, y1, y2, dx, dy, &transIn, &transOut);

  if (ltAllow) {
    prkOutDir[0] = true;
  }
  if (rtAllow) {
    prkOutDir[1] = true;
  }

  output->PrkOutDir[0] = prkOutDir[0];
  output->PrkOutDir[1] = prkOutDir[1];
  output->PrkOutDir[2] = prkOutDir[2];
  output->PrkOutDir[3] = prkOutDir[3];
}
