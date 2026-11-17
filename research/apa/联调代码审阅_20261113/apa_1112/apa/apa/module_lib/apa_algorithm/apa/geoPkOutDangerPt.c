#include "apaCountUtils.h"
#include "apaVehicleDefaults.h"
#include "geoPkOutDangerPtTypes.h"
#include "geoAxisTransTypes.h"
#include "geoLineIntrvCheckTypes.h"
#include <math.h>

typedef struct {
  Real cptX;
  Real cptY;
  Real lnC1;
  Real lnC2;
  Real lnC3;
  Real lnT1;
  Real lnT2;
  Real lnT3;
  Real kXmin;
} GeoPkOutDangerPtCtx_t;

static inline void geoPkOutDangerPtProcessLine(const GeoPkOutDangerPtInput_t *input,
                                               GeoPkOutDangerPtCtx_t *ctx,
                                               Real lnPtX1,
                                               Real lnPtY1,
                                               Real lnPtX2,
                                               Real lnPtY2,
                                               Integer ptSource,
                                               Real *distTmp2,
                                               Real *dangerPtX,
                                               Real *dangerPtY,
                                               Integer *ptEx,
                                               Integer *chCnt,
                                               GeoAxisTransInput_t *transIn,
                                               GeoAxisTransOutput_t *transOut) {
  Real lnB1 = lnPtY2 - lnPtY1;
  Real lnB2 = lnPtX1 - lnPtX2;
  Real lnB3 = lnPtX2 * lnPtY1 - lnPtX1 * lnPtY2;
  Real lnNormSqr = lnB1 * lnB1 + lnB2 * lnB2;

  if (lnNormSqr > 0.001f) {
    Real ptH1 = (lnB2 * lnB2 * ctx->cptX - lnB1 * lnB2 * ctx->cptY - lnB1 * lnB3) / lnNormSqr;
    Real ptH2 = (lnB1 * lnB1 * ctx->cptY - lnB1 * lnB2 * ctx->cptX - lnB2 * lnB3) / lnNormSqr;
    Real ptr1X;

    *chCnt = *chCnt + 1;

    transIn->PtX = ptH1;
    transIn->PtY = ptH2;
    transIn->PtTheta = 0.0f;
    transIn->Ori2X = input->Pt1sX;
    transIn->Ori2Y = input->Pt1sY;
    transIn->Ori2Theta = input->Pt1sTheta;
    geoAxisTrans(transIn, transOut);
    ptr1X = transOut->ResultX;

    if (((lnPtX1 - ptH1) * (lnPtX2 - ptH1) <= 0.0f) &&
        ((lnPtY1 - ptH2) * (lnPtY2 - ptH2) <= 0.0f)) {
      if ((ptr1X > ctx->kXmin) && (ptr1X < 7.0f)) {
        Real condA = ctx->lnC1 * ptH1 + ctx->lnC2 * ptH2 + ctx->lnC3;
        Real condB = ctx->lnT1 * ptH1 + ctx->lnT2 * ptH2 + ctx->lnT3;
        if ((condA * condB) < 0.0f) {
          Real dSqr =
              (ctx->cptX - ptH1) * (ctx->cptX - ptH1) + (ctx->cptY - ptH2) * (ctx->cptY - ptH2);
          if (dSqr < *distTmp2) {
            *ptEx = ptSource;
            *distTmp2 = dSqr;
            *dangerPtX = ptH1;
            *dangerPtY = ptH2;
          }
        }
      }
    } else {
      Real ltmp1 =
          (ctx->cptX - lnPtX1) * (ctx->cptX - lnPtX1) + (ctx->cptY - lnPtY1) * (ctx->cptY - lnPtY1);
      Real ltmp2 =
          (ctx->cptX - lnPtX2) * (ctx->cptX - lnPtX2) + (ctx->cptY - lnPtY2) * (ctx->cptY - lnPtY2);
      Real ptrR1X;
      Real ptrR2X;
      Real condC1A;
      Real condC1B;
      Boolean c1Safe;
      Real condC2A;
      Real condC2B;
      Boolean c2Safe;

      transIn->PtX = lnPtX1;
      transIn->PtY = lnPtY1;
      geoAxisTrans(transIn, transOut);
      ptrR1X = transOut->ResultX;

      transIn->PtX = lnPtX2;
      transIn->PtY = lnPtY2;
      geoAxisTrans(transIn, transOut);
      ptrR2X = transOut->ResultX;

      condC1A = ctx->lnC1 * lnPtX1 + ctx->lnC2 * lnPtY1 + ctx->lnC3;
      condC1B = ctx->lnT1 * lnPtX1 + ctx->lnT2 * lnPtY1 + ctx->lnT3;
      c1Safe = ((ptrR1X > ctx->kXmin) && (ptrR1X < 7.0f) && ((condC1A * condC1B) < 0.0f));

      condC2A = ctx->lnC1 * lnPtX2 + ctx->lnC2 * lnPtY2 + ctx->lnC3;
      condC2B = ctx->lnT1 * lnPtX2 + ctx->lnT2 * lnPtY2 + ctx->lnT3;
      c2Safe = ((ptrR2X > ctx->kXmin) && (ptrR2X < 7.0f) && ((condC2A * condC2B) < 0.0f));

      if (c1Safe) {
        if (c2Safe) {
          if (ltmp1 < ltmp2) {
            if (ltmp1 < *distTmp2) {
              *distTmp2 = ltmp1;
              *dangerPtX = lnPtX1;
              *dangerPtY = lnPtY1;
              *ptEx = ptSource;
            }
          } else {
            if (ltmp2 < *distTmp2) {
              *distTmp2 = ltmp2;
              *dangerPtX = lnPtX2;
              *dangerPtY = lnPtY2;
              *ptEx = ptSource;
            }
          }
        } else {
          if (ltmp1 < *distTmp2) {
            *distTmp2 = ltmp1;
            *dangerPtX = lnPtX1;
            *dangerPtY = lnPtY1;
            *ptEx = ptSource;
          }
        }
      } else {
        if (c2Safe) {
          if (ltmp2 < *distTmp2) {
            *distTmp2 = ltmp2;
            *dangerPtX = lnPtX2;
            *dangerPtY = lnPtY2;
            *ptEx = ptSource;
          }
        }
      }
    }
  }
}

#ifndef APA_MATH_EPSILON
#define APA_MATH_EPSILON 1e-6f
#endif

#ifndef GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES
#define GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES 360
#endif

#ifndef GEO_PKOUT_DANGER_ANGLE_MAX_INTERVALS
#define GEO_PKOUT_DANGER_ANGLE_MAX_INTERVALS 10
#endif

#ifndef GEO_PKOUT_DANGER_LAT_STEP
#define GEO_PKOUT_DANGER_LAT_STEP 0.3f
#endif

#ifndef GEO_PKOUT_DANGER_ANGLE_STEP
#define GEO_PKOUT_DANGER_ANGLE_STEP 0.0523598776f
#endif

/**
 * @brief 搜索平行泊出规划扇区内距离最近的真实或虚拟边界危险点。
 * @en_name geoPkOutDangerPt
 * @cn_name 泊出危险点搜索
 * @type module
 * @param[IN] GeoPkOutDangerPtInput_t Input 车辆、目标、搜索边界和障碍物集合
 * @param[OUT] GeoPkOutDangerPtOutput_t Output 危险点坐标、来源和检查次数
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 泊出规划
 * @source matlab_rewrite geo_PkOut_dangerPt.m
 * @version 1.1
 * @date 2026-07-31
 * @author aide
 */
void geoPkOutDangerPt(const GeoPkOutDangerPtInput_t *input, GeoPkOutDangerPtOutput_t *output) {
  Integer chCnt = 0;
  Integer ptEx = 0;
  Real distTmp2 = 10000.0f;
  Real dangerPtX = 0.0f;
  Real dangerPtY = 0.0f;
  Real cFac = 2.5f;
  Real kWd = 1.9f;
  Real kXmin = 4.0f;
  GeoAxisTransInput_t transIn;
  GeoAxisTransOutput_t transOut;
  Real ptrY;
  Real cptX;
  Real cptY;
  Real cptTrnX;
  Real cptTrnY;
  Real tsPtX;
  Real tsPtY;
  Real sPt1;
  Real cPt1;
  Real sTgt;
  Real cTgt;
  Real lnT1;
  Real lnT2;
  Real lnT3;
  Real lnC1;
  Real lnC2;
  Real lnC3;
  Integer ii;
  Integer itemCount;

  transIn.Ori1X = 0.0f;
  transIn.Ori1Y = 0.0f;
  transIn.Ori1Theta = 0.0f;
  transIn.PtX = input->VehX;
  transIn.PtY = input->VehY;
  transIn.PtTheta = input->VehTheta;
  transIn.Ori2X = input->TgtX;
  transIn.Ori2Y = input->TgtY;
  transIn.Ori2Theta = input->TgtTheta;
  geoAxisTrans(&transIn, &transOut);

  ptrY = transOut.ResultY;

  sPt1 = sinf(input->Pt1sTheta);
  cPt1 = cosf(input->Pt1sTheta);

  if ((ptrY > 0.0f) || (input->Dir == 1)) {
    cptX = input->Pt1sX - cFac * ptrY * sPt1;
    cptY = input->Pt1sY + cFac * ptrY * cPt1;
    cptTrnX = input->Pt1sX - ptrY * sPt1;
    cptTrnY = input->Pt1sY + ptrY * cPt1;
    tsPtX = input->Pt1sX + kWd * 0.5f * sPt1;
    tsPtY = input->Pt1sY - kWd * 0.5f * cPt1;
  } else {
    cptX = input->Pt1sX - cFac * ptrY * sPt1;
    cptY = input->Pt1sY + cFac * ptrY * cPt1;
    cptTrnX = input->Pt1sX - ptrY * sPt1;
    cptTrnY = input->Pt1sY + ptrY * cPt1;
    tsPtX = input->Pt1sX - kWd * 0.5f * sPt1;
    tsPtY = input->Pt1sY + kWd * 0.5f * cPt1;
  }

  sTgt = sinf(input->TgtTheta);
  cTgt = cosf(input->TgtTheta);
  lnT1 = sTgt;
  lnT2 = -cTgt;
  lnT3 = tsPtY * cTgt - tsPtX * sTgt;

  lnC1 = lnT1;
  lnC2 = lnT2;
  lnC3 = cptTrnY * cTgt - cptTrnX * sTgt;

  GeoPkOutDangerPtCtx_t ctx;
  ctx.cptX = cptX;
  ctx.cptY = cptY;
  ctx.lnC1 = lnC1;
  ctx.lnC2 = lnC2;
  ctx.lnC3 = lnC3;
  ctx.lnT1 = lnT1;
  ctx.lnT2 = lnT2;
  ctx.lnT3 = lnT3;
  ctx.kXmin = kXmin;

  itemCount = apaClampCount(input->BoundaryLineNum, VCC_MAX_BOUNDARY_LINES);
  for (ii = 0; ii < itemCount; ii = ii + 1) {
    geoPkOutDangerPtProcessLine(input,
                                &ctx,
                                input->BoundaryLineX1[ii],
                                input->BoundaryLineY1[ii],
                                input->BoundaryLineX2[ii],
                                input->BoundaryLineY2[ii],
                                1,
                                &distTmp2,
                                &dangerPtX,
                                &dangerPtY,
                                &ptEx,
                                &chCnt,
                                &transIn,
                                &transOut);
  }

  itemCount = apaClampCount(input->VirtualBoundaryLineNum, VCC_MAX_VIRTUAL_LINES);
  for (ii = 0; ii < itemCount; ii = ii + 1) {
    geoPkOutDangerPtProcessLine(input,
                                &ctx,
                                input->VirtualBoundaryLineX1[ii],
                                input->VirtualBoundaryLineY1[ii],
                                input->VirtualBoundaryLineX2[ii],
                                input->VirtualBoundaryLineY2[ii],
                                2,
                                &distTmp2,
                                &dangerPtX,
                                &dangerPtY,
                                &ptEx,
                                &chCnt,
                                &transIn,
                                &transOut);
  }

  itemCount = apaClampCount(input->BoundarySepPtsNum, VCC_MAX_SEPARATE_PTS);
  if (itemCount > 0) {
    for (ii = 0; ii < itemCount; ii = ii + 1) {
      Real px = input->BoundarySepPtsX[ii];
      Real py = input->BoundarySepPtsY[ii];
      Real ptrR1X;
      Real condA;
      Real condB;
      Boolean c1Safe;
      Boolean nonZeroPoint;

      nonZeroPoint = ((fabsf(px) > 1.0e-3f) || (fabsf(py) > 1.0e-3f));
      if (nonZeroPoint) {
        chCnt = chCnt + 1;
        transIn.PtX = px;
        transIn.PtY = py;
        transIn.PtTheta = 0.0f;
        transIn.Ori2X = input->TgtX;
        transIn.Ori2Y = input->TgtY;
        transIn.Ori2Theta = input->TgtTheta;
        geoAxisTrans(&transIn, &transOut);
        ptrR1X = transOut.ResultX;

        condA = lnC1 * px + lnC2 * py + lnC3;
        condB = lnT1 * px + lnT2 * py + lnT3;
        c1Safe = ((ptrR1X > kXmin) && (ptrR1X < 7.0f) && ((condA * condB) < 0.0f));

        if (c1Safe) {
          Real dSqr = (cptX - px) * (cptX - px) + (cptY - py) * (cptY - py);
          if (dSqr < distTmp2) {
            distTmp2 = dSqr;
            dangerPtX = px;
            dangerPtY = py;
            ptEx = 1;
          }
        }
      }
    }
  }

  itemCount = apaClampCount(input->SeparateStopperNum, VCC_MAX_SEPARATE_PTS);
  if (itemCount > 0) {
    for (ii = 0; ii < itemCount; ii = ii + 1) {
      Real px = input->SeparateStopperX[ii];
      Real py = input->SeparateStopperY[ii];
      Real ptrR1X;
      Real condA;
      Real condB;
      Boolean c1Safe;
      Boolean nonZeroPoint;

      nonZeroPoint = ((fabsf(px) > 1.0e-3f) || (fabsf(py) > 1.0e-3f));
      if (nonZeroPoint) {
        chCnt = chCnt + 1;
        transIn.PtX = px;
        transIn.PtY = py;
        transIn.PtTheta = 0.0f;
        transIn.Ori2X = input->TgtX;
        transIn.Ori2Y = input->TgtY;
        transIn.Ori2Theta = input->TgtTheta;
        geoAxisTrans(&transIn, &transOut);
        ptrR1X = transOut.ResultX;

        condA = lnC1 * px + lnC2 * py + lnC3;
        condB = lnT1 * px + lnT2 * py + lnT3;
        c1Safe = ((ptrR1X > kXmin) && (ptrR1X < 7.0f) && ((condA * condB) < 0.0f));

        if (c1Safe) {
          Real dSqr = (cptX - px) * (cptX - px) + (cptY - py) * (cptY - py);
          if (dSqr < distTmp2) {
            distTmp2 = dSqr;
            dangerPtX = px;
            dangerPtY = py;
            ptEx = 1;
          }
        }
      }
    }
  }

  output->DangerPtX = dangerPtX;
  output->DangerPtY = dangerPtY;
  output->ChCnt = chCnt;
  output->PtEx = ptEx;
}

static inline Real geoPkOutDangerGetVehWidth(const ApaPlannerVehConfig_t *vehConfig) {
  Real value;

  value = vehConfig->W;
  if (value <= APA_MATH_EPSILON) {
    value = APA_VEHICLE_WIDTH_M;
  }
  return value;
}

static inline Real geoPkOutDangerGetVehFront(const ApaPlannerVehConfig_t *vehConfig) {
  Real value;

  value = vehConfig->LF;
  if (value <= APA_MATH_EPSILON) {
    value = APA_VEHICLE_REAR_TO_FRONT_M;
  }
  return value;
}

static inline Real geoPkOutDangerGetVehRear(const ApaPlannerVehConfig_t *vehConfig) {
  Real value;

  value = vehConfig->LB;
  if (value <= APA_MATH_EPSILON) {
    value = APA_VEHICLE_REAR_TO_BACK_M;
  }
  return value;
}

static inline Real geoPkOutDangerGetVehWheelBase(const ApaPlannerVehConfig_t *vehConfig) {
  Real value;

  value = vehConfig->WB;
  if (value <= APA_MATH_EPSILON) {
    value = APA_VEHICLE_WHEELBASE_M;
  }
  return value;
}

static inline Real geoPkOutDangerGetLatExtend(Real latExtend) {
  Real value;

  value = latExtend;
  if (value <= APA_MATH_EPSILON) {
    value = 2.0f;
  }
  return value;
}

static inline Integer geoPkOutDangerGetSideSign(Integer sideSign) {
  Integer value;

  value = sideSign;
  if (value == 0) {
    value = 1;
  }
  if (value > 0) {
    value = 1;
  } else {
    value = -1;
  }
  return value;
}

static inline void geoPkOutDangerGroundToLocal(
    Real pointX, Real pointY, Real posX, Real posY, Real posTheta, Real *localX, Real *localY) {
  Real dx;
  Real dy;
  Real cosTheta;
  Real sinTheta;

  dx = pointX - posX;
  dy = pointY - posY;
  cosTheta = cosf(posTheta);
  sinTheta = sinf(posTheta);
  *localX = cosTheta * dx + sinTheta * dy;
  *localY = -sinTheta * dx + cosTheta * dy;
}

static inline Boolean geoPkOutDangerAreaInRange(const GeoPkOutDangerAreaSearchInput_t *input,
                                                Real pointX,
                                                Real pointY,
                                                Real frontLimit,
                                                Real rearLimit,
                                                Real latLimit,
                                                Integer sideSign) {
  Real localX;
  Real localY;
  Real signedLatLimit;
  Real lateralProduct;
  Boolean inLongitudinal;
  Boolean inLateral;
  Boolean inRange;

  geoPkOutDangerGroundToLocal(
      pointX, pointY, input->PosX, input->PosY, input->PosTheta, &localX, &localY);
  signedLatLimit = latLimit * (Real)sideSign;
  lateralProduct = localY * (localY - signedLatLimit);
  inLongitudinal = ((localX > rearLimit) && (localX < frontLimit));
  inLateral = (lateralProduct < 0.0f);
  inRange = false;
  if (inLongitudinal) {
    if (inLateral) {
      inRange = true;
    }
  }
  return inRange;
}

static inline void geoPkOutDangerAreaUpdateCandidate(Real candidateX,
                                                     Real candidateY,
                                                     Integer ptSource,
                                                     Real cptX,
                                                     Real cptY,
                                                     Real *bestDist2,
                                                     GeoPkOutDangerAreaSearchOutput_t *output) {
  Real dx;
  Real dy;
  Real dist2;

  dx = cptX - candidateX;
  dy = cptY - candidateY;
  dist2 = dx * dx + dy * dy;
  if (dist2 < *bestDist2) {
    *bestDist2 = dist2;
    output->DangerPtX = candidateX;
    output->DangerPtY = candidateY;
    output->PtEx = ptSource;
  }
}

static inline void geoPkOutDangerAreaCheckPoint(const GeoPkOutDangerAreaSearchInput_t *input,
                                                Real pointX,
                                                Real pointY,
                                                Integer ptSource,
                                                Real cptX,
                                                Real cptY,
                                                Real frontLimit,
                                                Real rearLimit,
                                                Real latLimit,
                                                Integer sideSign,
                                                Real *bestDist2,
                                                GeoPkOutDangerAreaSearchOutput_t *output) {
  Boolean inRange;
  Boolean nonZeroPoint;

  nonZeroPoint = ((fabsf(pointX) > 1.0e-3f) || (fabsf(pointY) > 1.0e-3f));
  if (nonZeroPoint) {
    output->CheckCount = output->CheckCount + 1;
    inRange =
        geoPkOutDangerAreaInRange(input, pointX, pointY, frontLimit, rearLimit, latLimit, sideSign);
    if (inRange) {
      geoPkOutDangerAreaUpdateCandidate(pointX, pointY, ptSource, cptX, cptY, bestDist2, output);
    }
  }
}

static inline void geoPkOutDangerAreaCheckLine(const GeoPkOutDangerAreaSearchInput_t *input,
                                               Real lineX1,
                                               Real lineY1,
                                               Real lineX2,
                                               Real lineY2,
                                               Integer ptSource,
                                               Real cptX,
                                               Real cptY,
                                               Real frontLimit,
                                               Real rearLimit,
                                               Real latLimit,
                                               Integer sideSign,
                                               Real *bestDist2,
                                               GeoPkOutDangerAreaSearchOutput_t *output) {
  Real lineA;
  Real lineB;
  Real lineC;
  Real lineNorm2;
  Real projX;
  Real projY;
  Boolean projectionOnSegment;
  Boolean projectionInRange;
  Boolean point1InRange;
  Boolean point2InRange;
  Real dist1;
  Real dist2;

  lineA = lineY2 - lineY1;
  lineB = lineX1 - lineX2;
  lineC = lineX2 * lineY1 - lineX1 * lineY2;
  lineNorm2 = lineA * lineA + lineB * lineB;
  if (lineNorm2 > 0.001f) {
    projX = (lineB * lineB * cptX - lineA * lineB * cptY - lineA * lineC) / lineNorm2;
    projY = (lineA * lineA * cptY - lineA * lineB * cptX - lineB * lineC) / lineNorm2;
    output->CheckCount = output->CheckCount + 1;
    projectionOnSegment = (((lineX1 - projX) * (lineX2 - projX) <= 0.0f) &&
                           ((lineY1 - projY) * (lineY2 - projY) <= 0.0f));
    if (projectionOnSegment) {
      projectionInRange =
          geoPkOutDangerAreaInRange(input, projX, projY, frontLimit, rearLimit, latLimit, sideSign);
      if (projectionInRange) {
        geoPkOutDangerAreaUpdateCandidate(projX, projY, ptSource, cptX, cptY, bestDist2, output);
      }
    } else {
      point1InRange = geoPkOutDangerAreaInRange(
          input, lineX1, lineY1, frontLimit, rearLimit, latLimit, sideSign);
      point2InRange = geoPkOutDangerAreaInRange(
          input, lineX2, lineY2, frontLimit, rearLimit, latLimit, sideSign);
      dist1 = (cptX - lineX1) * (cptX - lineX1) + (cptY - lineY1) * (cptY - lineY1);
      dist2 = (cptX - lineX2) * (cptX - lineX2) + (cptY - lineY2) * (cptY - lineY2);
      if (point1InRange) {
        if (point2InRange) {
          if (dist1 <= dist2) {
            geoPkOutDangerAreaUpdateCandidate(
                lineX1, lineY1, ptSource, cptX, cptY, bestDist2, output);
          } else {
            geoPkOutDangerAreaUpdateCandidate(
                lineX2, lineY2, ptSource, cptX, cptY, bestDist2, output);
          }
        } else {
          geoPkOutDangerAreaUpdateCandidate(
              lineX1, lineY1, ptSource, cptX, cptY, bestDist2, output);
        }
      } else {
        if (point2InRange) {
          geoPkOutDangerAreaUpdateCandidate(
              lineX2, lineY2, ptSource, cptX, cptY, bestDist2, output);
        }
      }
    }
  }
}

/**
 * @brief 泊出局部区域危险点搜索
 * @en_name geoPkOutDangerAreaSearch
 * @cn_name 泊出区域危险点搜索
 * @type module
 * @param[IN] GeoPkOutDangerAreaSearchInput_t input 搜索输入
 * @param[OUT] GeoPkOutDangerAreaSearchOutput_t output 搜索输出
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 泊出规划
 * @source matlab_rewrite geo_PkOut_dngPt_areaSrch.m
 * @version 1.0
 * @date 2026-04-30
 * @author aide
 */
void geoPkOutDangerAreaSearch(const GeoPkOutDangerAreaSearchInput_t *input,
                              GeoPkOutDangerAreaSearchOutput_t *output) {
  Real vehWidth;
  Real vehFront;
  Real vehRear;
  Real vehWheelBase;
  Real frontLimit;
  Real rearLimit;
  Real latLimit;
  Real cptX;
  Real cptY;
  Real bestDist2;
  Integer sideSign;
  Integer itemCount;
  Integer ii;

  *output = (GeoPkOutDangerAreaSearchOutput_t){0};
  vehWidth = geoPkOutDangerGetVehWidth(&input->VehConfig);
  vehFront = geoPkOutDangerGetVehFront(&input->VehConfig);
  vehRear = geoPkOutDangerGetVehRear(&input->VehConfig);
  vehWheelBase = geoPkOutDangerGetVehWheelBase(&input->VehConfig);
  frontLimit = vehFront + input->FrontExtend;
  rearLimit = -vehRear - input->RearExtend;
  latLimit = geoPkOutDangerGetLatExtend(input->LatExtend) + 0.5f * vehWidth;
  sideSign = geoPkOutDangerGetSideSign(input->SideSign);
  cptX = input->PosX + vehWheelBase * cosf(input->PosTheta);
  cptY = input->PosY + vehWheelBase * sinf(input->PosTheta);
  bestDist2 = 10000.0f;

  itemCount = apaClampCount(input->Obstacles.BoundaryLineNum, VCC_MAX_BOUNDARY_LINES);
  for (ii = 0; ii < itemCount; ii = ii + 1) {
    geoPkOutDangerAreaCheckLine(input,
                                input->Obstacles.BoundaryLineX1[ii],
                                input->Obstacles.BoundaryLineY1[ii],
                                input->Obstacles.BoundaryLineX2[ii],
                                input->Obstacles.BoundaryLineY2[ii],
                                1,
                                cptX,
                                cptY,
                                frontLimit,
                                rearLimit,
                                latLimit,
                                sideSign,
                                &bestDist2,
                                output);
  }

  itemCount = apaClampCount(input->Obstacles.VirtualBoundaryLineNum, VCC_MAX_VIRTUAL_LINES);
  for (ii = 0; ii < itemCount; ii = ii + 1) {
    geoPkOutDangerAreaCheckLine(input,
                                input->Obstacles.VirtualBoundaryLineX1[ii],
                                input->Obstacles.VirtualBoundaryLineY1[ii],
                                input->Obstacles.VirtualBoundaryLineX2[ii],
                                input->Obstacles.VirtualBoundaryLineY2[ii],
                                2,
                                cptX,
                                cptY,
                                frontLimit,
                                rearLimit,
                                latLimit,
                                sideSign,
                                &bestDist2,
                                output);
  }

  itemCount = apaClampCount(input->Obstacles.BoundarySepPtsNum, VCC_MAX_SEPARATE_PTS);
  for (ii = 0; ii < itemCount; ii = ii + 1) {
    geoPkOutDangerAreaCheckPoint(input,
                                 input->Obstacles.BoundarySepPtsX[ii],
                                 input->Obstacles.BoundarySepPtsY[ii],
                                 1,
                                 cptX,
                                 cptY,
                                 frontLimit,
                                 rearLimit,
                                 latLimit,
                                 sideSign,
                                 &bestDist2,
                                 output);
  }

  itemCount = apaClampCount(input->Obstacles.SeparateStopperNum, VCC_MAX_SEPARATE_PTS);
  for (ii = 0; ii < itemCount; ii = ii + 1) {
    geoPkOutDangerAreaCheckPoint(input,
                                 input->Obstacles.SeparateStopperX[ii],
                                 input->Obstacles.SeparateStopperY[ii],
                                 1,
                                 cptX,
                                 cptY,
                                 frontLimit,
                                 rearLimit,
                                 latLimit,
                                 sideSign,
                                 &bestDist2,
                                 output);
  }
}

static inline void geoPkOutDangerFillLineInput(const GeoPkOutDangerAngleSearchInput_t *input,
                                               GeoLineIntrvCheckInput_t *lineInput) {
  Integer itemCount;
  Integer ii;
  Real checkLen;
  Real checkStep;

  *lineInput = (GeoLineIntrvCheckInput_t){0};
  checkLen = input->FrontCheckLen;
  if (checkLen <= APA_MATH_EPSILON) {
    checkLen = 4.0f;
  }
  checkStep = input->CheckStep;
  if (checkStep <= APA_MATH_EPSILON) {
    checkStep = 0.1f;
  }
  lineInput->Pt1X = input->VehX;
  lineInput->Pt1Y = input->VehY;
  lineInput->Pt1Theta = input->VehTheta;
  lineInput->Pt2X = input->VehX + checkLen * cosf(input->VehTheta);
  lineInput->Pt2Y = input->VehY + checkLen * sinf(input->VehTheta);
  lineInput->Pt2Theta = input->VehTheta;
  lineInput->CheckStep = checkStep;
  lineInput->VehWidth = geoPkOutDangerGetVehWidth(&input->VehConfig);
  lineInput->VehFrontOffset = geoPkOutDangerGetVehFront(&input->VehConfig);
  lineInput->VehRearOffset = geoPkOutDangerGetVehRear(&input->VehConfig);
  lineInput->UsePlanningCheck = true;
  lineInput->DirOut = input->Dir;

  itemCount = apaClampCount(input->Obstacles.BoundaryLineNum, GEO_BOUNDARY_LINE_MAX);
  lineInput->BoundaryLineNum = itemCount;
  for (ii = 0; ii < itemCount; ii = ii + 1) {
    lineInput->BoundaryLineX1[ii] = input->Obstacles.BoundaryLineX1[ii];
    lineInput->BoundaryLineY1[ii] = input->Obstacles.BoundaryLineY1[ii];
    lineInput->BoundaryLineX2[ii] = input->Obstacles.BoundaryLineX2[ii];
    lineInput->BoundaryLineY2[ii] = input->Obstacles.BoundaryLineY2[ii];
  }

  itemCount = apaClampCount(input->Obstacles.VirtualBoundaryLineNum, VCC_MAX_VIRTUAL_LINES);
  lineInput->VirtualBoundaryLineNum = itemCount;
  for (ii = 0; ii < itemCount; ii = ii + 1) {
    lineInput->VirtualBoundaryLineX1[ii] = input->Obstacles.VirtualBoundaryLineX1[ii];
    lineInput->VirtualBoundaryLineY1[ii] = input->Obstacles.VirtualBoundaryLineY1[ii];
    lineInput->VirtualBoundaryLineX2[ii] = input->Obstacles.VirtualBoundaryLineX2[ii];
    lineInput->VirtualBoundaryLineY2[ii] = input->Obstacles.VirtualBoundaryLineY2[ii];
  }

  itemCount = apaClampCount(input->Obstacles.BoundarySepPtsNum, VCC_MAX_SEPARATE_PTS);
  lineInput->BoundarySepPtNum = itemCount;
  for (ii = 0; ii < itemCount; ii = ii + 1) {
    lineInput->BoundarySepPtX[ii] = input->Obstacles.BoundarySepPtsX[ii];
    lineInput->BoundarySepPtY[ii] = input->Obstacles.BoundarySepPtsY[ii];
  }

  itemCount = apaClampCount(input->Obstacles.SeparateStopperNum, VCC_MAX_SEPARATE_PTS);
  lineInput->SeparateStopperNum = itemCount;
  for (ii = 0; ii < itemCount; ii = ii + 1) {
    lineInput->SeparateStopperX[ii] = input->Obstacles.SeparateStopperX[ii];
    lineInput->SeparateStopperY[ii] = input->Obstacles.SeparateStopperY[ii];
  }

  itemCount = apaClampCount(input->Obstacles.SlotTrackStopperNum, VCC_MAX_TRACK_STOPPER);
  lineInput->SlotTrackStopperNum = itemCount;
  for (ii = 0; ii < itemCount; ii = ii + 1) {
    lineInput->SlotTrackStopperX1[ii] = input->Obstacles.SlotTrackStopperX1[ii];
    lineInput->SlotTrackStopperY1[ii] = input->Obstacles.SlotTrackStopperY1[ii];
    lineInput->SlotTrackStopperX2[ii] = input->Obstacles.SlotTrackStopperX2[ii];
    lineInput->SlotTrackStopperY2[ii] = input->Obstacles.SlotTrackStopperY2[ii];
  }
}

static inline Boolean
geoPkOutDangerAngleCollisionSafe(const VehicleCollisionCheckOutput_t *vccOutput) {
  Integer frontAreaSum;
  Integer rearAreaSum;
  Integer idx;
  Boolean isSafe;

  frontAreaSum = 0;
  rearAreaSum = 0;
  isSafe = false;
  if (!vccOutput->IsCollision) {
    isSafe = true;
  } else {
    for (idx = 0; idx < 6; idx = idx + 1) {
      frontAreaSum = frontAreaSum + (Integer)vccOutput->CollisionArea[idx];
    }
    for (idx = 6; idx < VCC_COLLISION_AREA_COUNT; idx = idx + 1) {
      rearAreaSum = rearAreaSum + (Integer)vccOutput->CollisionArea[idx];
    }
    if (frontAreaSum == 0) {
      if (rearAreaSum == 0 || vccOutput->CollisionDepth < 0.3f) {
        isSafe = true;
      }
    }
  }
  return isSafe;
}

static inline void geoPkOutDangerAngleCheckPose(const GeoPkOutDangerAngleSearchInput_t *input,
                                                Real localX,
                                                Real localY,
                                                Real localTheta,
                                                Real poseX,
                                                Real poseY,
                                                Real poseTheta,
                                                Boolean *isSafe) {
  VehicleCollisionCheckInput_t vccInput;
  VehicleCollisionCheckOutput_t vccOutput;
  Real vehWidth;
  Real vehFront;
  Real vehRear;
  Real cosLocal;
  Real sinLocal;

  vccInput = input->Obstacles;
  vccInput.CheckType = VCC_CHECK_TYPE_PLANNING;
  vccInput.CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  vccInput.CollisionDemand = VCC_COLLISION_DEMAND_ALL;
  vccInput.ParkingOutFlag = false;
  vccInput.SlotClass = VCC_SLOT_CLASS_PARALLEL;
  vccInput.UseAutoMargin = true;
  vccInput.VehPosX = poseX;
  vccInput.VehPosY = poseY;
  vccInput.VehPosTheta = poseTheta;
  vehWidth = geoPkOutDangerGetVehWidth(&input->VehConfig);
  vehFront = geoPkOutDangerGetVehFront(&input->VehConfig);
  vehRear = geoPkOutDangerGetVehRear(&input->VehConfig);
  vccInput.VehW = vehWidth;
  vccInput.VehLF = vehFront;
  vccInput.VehLB = vehRear;
  vccInput.VehLL = vccInput.VehLF + vccInput.VehLB;
  vccInput.VehWB = geoPkOutDangerGetVehWheelBase(&input->VehConfig);
  cosLocal = cosf(localTheta);
  sinLocal = sinf(localTheta);
  vccInput.SlotPtsValid = true;
  vccInput.SlotPts[0] = localX + vehFront * cosLocal - 0.5f * vehWidth * sinLocal;
  vccInput.SlotPts[1] = localY + vehFront * sinLocal + 0.5f * vehWidth * cosLocal;
  vccInput.SlotPts[2] = localX + vehFront * cosLocal + 0.5f * vehWidth * sinLocal;
  vccInput.SlotPts[3] = localY + vehFront * sinLocal - 0.5f * vehWidth * cosLocal;
  vccInput.SlotPts[4] = localX - vehRear * cosLocal + 0.5f * vehWidth * sinLocal;
  vccInput.SlotPts[5] = localY - vehRear * sinLocal - 0.5f * vehWidth * cosLocal;
  vccInput.SlotPts[6] = localX - vehRear * cosLocal - 0.5f * vehWidth * sinLocal;
  vccInput.SlotPts[7] = localY - vehRear * sinLocal + 0.5f * vehWidth * cosLocal;
  vehicleCollisionCheck(&vccInput, &vccOutput);
  *isSafe = geoPkOutDangerAngleCollisionSafe(&vccOutput);
}

static inline void geoPkOutDangerAngleLocalToWorld(const GeoPkOutDangerAngleSearchInput_t *input,
                                                   Real localX,
                                                   Real localY,
                                                   Real localTheta,
                                                   Real *poseX,
                                                   Real *poseY,
                                                   Real *poseTheta) {
  Real cosVeh;
  Real sinVeh;

  cosVeh = cosf(input->VehTheta);
  sinVeh = sinf(input->VehTheta);
  *poseX = input->VehX + localX * cosVeh - localY * sinVeh;
  *poseY = input->VehY + localX * sinVeh + localY * cosVeh;
  *poseTheta = input->VehTheta + localTheta;
}

static inline void
geoPkOutDangerAngleAppendCandidate(const GeoPkOutDangerAngleSearchInput_t *input,
                                   Real localX,
                                   Real localY,
                                   Real localTheta,
                                   Real poseXList[GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES],
                                   Real poseYList[GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES],
                                   Real poseThetaList[GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES],
                                   UInteger8 safeFlagList[GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES],
                                   Integer *candidateCount,
                                   Integer *collisionCheckCount) {
  Real poseX;
  Real poseY;
  Real poseTheta;
  Boolean isSafe;

  if (*candidateCount < GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES) {
    geoPkOutDangerAngleLocalToWorld(input, localX, localY, localTheta, &poseX, &poseY, &poseTheta);
    geoPkOutDangerAngleCheckPose(
        input, localX, localY, localTheta, poseX, poseY, poseTheta, &isSafe);
    poseXList[*candidateCount] = poseX;
    poseYList[*candidateCount] = poseY;
    poseThetaList[*candidateCount] = poseTheta;
    if (isSafe) {
      safeFlagList[*candidateCount] = 1U;
    } else {
      safeFlagList[*candidateCount] = 0U;
    }
    *candidateCount = *candidateCount + 1;
    *collisionCheckCount = *collisionCheckCount + 1;
  }
}

static inline void
geoPkOutDangerAngleBuildCandidates(const GeoPkOutDangerAngleSearchInput_t *input,
                                   Real poseXList[GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES],
                                   Real poseYList[GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES],
                                   Real poseThetaList[GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES],
                                   UInteger8 safeFlagList[GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES],
                                   Integer *candidateCount,
                                   Integer *collisionCheckCount) {
  Real vehWidth;
  Real vehFront;
  Real wheelBase;
  Real localX;
  Real localTheta;

  vehWidth = geoPkOutDangerGetVehWidth(&input->VehConfig);
  vehFront = geoPkOutDangerGetVehFront(&input->VehConfig);
  wheelBase = geoPkOutDangerGetVehWheelBase(&input->VehConfig);
  *candidateCount = 0;
  *collisionCheckCount = 0;

  localX = 0.0f;
  while (localX <= vehFront + APA_MATH_EPSILON) {
    geoPkOutDangerAngleAppendCandidate(input,
                                       localX,
                                       -0.6f,
                                       -APA_PI / 2.0f,
                                       poseXList,
                                       poseYList,
                                       poseThetaList,
                                       safeFlagList,
                                       candidateCount,
                                       collisionCheckCount);
    localX = localX + GEO_PKOUT_DANGER_LAT_STEP;
  }

  localTheta = -APA_PI / 2.0f;
  while (localTheta <= APA_MATH_EPSILON) {
    Real angleX;
    Real angleY;

    angleX = wheelBase + cosf(localTheta) * vehWidth * 0.5f;
    angleY = sinf(localTheta) * vehWidth * 0.5f;
    geoPkOutDangerAngleAppendCandidate(input,
                                       angleX,
                                       angleY,
                                       localTheta,
                                       poseXList,
                                       poseYList,
                                       poseThetaList,
                                       safeFlagList,
                                       candidateCount,
                                       collisionCheckCount);
    localTheta = localTheta + GEO_PKOUT_DANGER_ANGLE_STEP;
  }

  localTheta = 0.0f;
  while (localTheta <= (APA_PI / 2.0f) + APA_MATH_EPSILON) {
    Real angleX;
    Real angleY;

    angleX = wheelBase + cosf(localTheta) * vehWidth * 0.5f;
    angleY = sinf(localTheta) * vehWidth * 0.5f;
    geoPkOutDangerAngleAppendCandidate(input,
                                       angleX,
                                       angleY,
                                       localTheta,
                                       poseXList,
                                       poseYList,
                                       poseThetaList,
                                       safeFlagList,
                                       candidateCount,
                                       collisionCheckCount);
    localTheta = localTheta + GEO_PKOUT_DANGER_ANGLE_STEP;
  }

  localX = vehFront;
  while (localX >= -APA_MATH_EPSILON) {
    geoPkOutDangerAngleAppendCandidate(input,
                                       localX,
                                       0.6f,
                                       APA_PI / 2.0f,
                                       poseXList,
                                       poseYList,
                                       poseThetaList,
                                       safeFlagList,
                                       candidateCount,
                                       collisionCheckCount);
    localX = localX - GEO_PKOUT_DANGER_LAT_STEP;
  }
}

static inline void geoPkOutDangerAngleFindSafeIntervals(
    const UInteger8 safeFlagList[GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES],
    Integer candidateCount,
    Integer startIndexList[GEO_PKOUT_DANGER_ANGLE_MAX_INTERVALS],
    Integer endIndexList[GEO_PKOUT_DANGER_ANGLE_MAX_INTERVALS],
    Integer widthList[GEO_PKOUT_DANGER_ANGLE_MAX_INTERVALS],
    Integer *intervalCount) {
  Integer idx;
  Boolean inSafeInterval;
  Integer currentStart;

  *intervalCount = 0;
  inSafeInterval = false;
  currentStart = 0;
  for (idx = 0; idx < candidateCount; idx = idx + 1) {
    if (safeFlagList[idx] > 0U) {
      if (!inSafeInterval) {
        if (*intervalCount < GEO_PKOUT_DANGER_ANGLE_MAX_INTERVALS) {
          currentStart = idx;
          startIndexList[*intervalCount] = idx;
          inSafeInterval = true;
        }
      }
    } else {
      if (inSafeInterval) {
        endIndexList[*intervalCount] = idx;
        widthList[*intervalCount] = idx - currentStart + 1;
        *intervalCount = *intervalCount + 1;
        inSafeInterval = false;
      }
    }
  }
  if (inSafeInterval) {
    endIndexList[*intervalCount] = candidateCount - 1;
    widthList[*intervalCount] = candidateCount - currentStart;
    *intervalCount = *intervalCount + 1;
  }
}

static inline Integer geoPkOutDangerAngleClampIndex(Integer indexValue, Integer candidateCount) {
  Integer result;

  result = indexValue;
  if (result < 0) {
    result = 0;
  }
  if (candidateCount > 0) {
    if (result >= candidateCount) {
      result = candidateCount - 1;
    }
  }
  return result;
}

static inline Boolean geoPkOutDangerAngleSelectIntervalPose(
    Integer originalDir,
    Integer sideDir,
    const Real poseThetaList[GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES],
    Integer candidateCount,
    const Integer startIndexList[GEO_PKOUT_DANGER_ANGLE_MAX_INTERVALS],
    const Integer endIndexList[GEO_PKOUT_DANGER_ANGLE_MAX_INTERVALS],
    const Integer widthList[GEO_PKOUT_DANGER_ANGLE_MAX_INTERVALS],
    Integer intervalCount,
    Integer *selectedIndex,
    Integer *selectedPoseType) {
  Integer intervalIdx;
  Integer safeIntervalIdx;
  Integer resultIndex;
  Integer resultPoseType;
  Boolean found;
  Real ang1;
  Real ang2;
  Real boundaryAngle;

  found = false;
  safeIntervalIdx = -1;
  resultIndex = 0;
  *selectedPoseType = 0;
  ang1 = 45.0f * APA_PI / 180.0f;
  ang2 = 30.0f * APA_PI / 180.0f;
  boundaryAngle = 35.0f * APA_PI / 180.0f;
  if (originalDir == 1) {
    ang2 = 35.0f * APA_PI / 180.0f;
  }
  if (originalDir == 3) {
    boundaryAngle = 20.0f * APA_PI / 180.0f;
  }
  if (sideDir == 1) {
    for (intervalIdx = 0; intervalIdx < intervalCount; intervalIdx = intervalIdx + 1) {
      if (!found) {
        if (widthList[intervalIdx] > 6) {
          if (poseThetaList[endIndexList[intervalIdx]] > boundaryAngle) {
            safeIntervalIdx = intervalIdx;
            found = true;
          }
        }
      }
    }
  } else if (sideDir == 2) {
    intervalIdx = intervalCount - 1;
    while (intervalIdx >= 0 && !found) {
      if (widthList[intervalIdx] > 6) {
        if (poseThetaList[startIndexList[intervalIdx]] < -boundaryAngle) {
          safeIntervalIdx = intervalIdx;
          found = true;
        }
      }
      intervalIdx = intervalIdx - 1;
    }
  }

  if (found) {
    Integer stIndex;
    Integer edIndex;
    Real stTheta;
    Real edTheta;

    stIndex = startIndexList[safeIntervalIdx];
    edIndex = endIndexList[safeIntervalIdx];
    stTheta = poseThetaList[stIndex];
    edTheta = poseThetaList[edIndex];
    if (widthList[safeIntervalIdx] < 8) {
      if (sideDir == 1) {
        resultIndex = (stIndex + edIndex + 1) / 2;
      } else {
        resultIndex = (stIndex + edIndex) / 2;
      }
      resultPoseType = 1;
    } else if (sideDir == 1) {
      if (edTheta < ang1) {
        resultIndex = edIndex - 3;
        resultPoseType = 2;
      } else if (edTheta > 60.0f * APA_PI / 180.0f) {
        Integer offset;
        Boolean offsetFound;

        offset = 2;
        offsetFound = false;
        while (offset < widthList[safeIntervalIdx] && !offsetFound) {
          resultIndex = stIndex + offset;
          if (poseThetaList[resultIndex] >= ang2) {
            offsetFound = true;
          }
          offset = offset + 1;
        }
        if (!offsetFound) {
          resultIndex = stIndex + 2;
        }
        resultPoseType = 3;
      } else if (stTheta > 10.0f * APA_PI / 180.0f) {
        resultIndex = edIndex - 3;
        resultPoseType = 2;
      } else {
        resultIndex = edIndex - 6;
        resultPoseType = 2;
      }
    } else {
      if (stTheta > -ang1) {
        resultIndex = stIndex + 3;
        resultPoseType = 3;
      } else if (stTheta < -60.0f * APA_PI / 180.0f) {
        Integer offset;
        Boolean offsetFound;

        offset = 2;
        offsetFound = false;
        while (offset < widthList[safeIntervalIdx] && !offsetFound) {
          resultIndex = edIndex - offset;
          if (poseThetaList[resultIndex] <= -ang2) {
            offsetFound = true;
          }
          offset = offset + 1;
        }
        if (!offsetFound) {
          resultIndex = edIndex - 2;
        }
        resultPoseType = 2;
      } else if (edTheta < -10.0f * APA_PI / 180.0f) {
        resultIndex = stIndex + 3;
        resultPoseType = 3;
      } else {
        resultIndex = stIndex + 6;
        resultPoseType = 3;
      }
    }
    *selectedIndex = geoPkOutDangerAngleClampIndex(resultIndex, candidateCount);
    *selectedPoseType = resultPoseType;
  }
  return found;
}

static inline Real geoPkOutDangerAngleGetVirtualOffset(Integer dir, Integer poseType) {
  Real lofst;

  lofst = 0.3f;
  if (dir == 1) {
    if (poseType == 2) {
      lofst = 0.4f;
    } else if (poseType == 3) {
      lofst = 0.1f;
    }
  } else if (dir == 2) {
    if (poseType == 2) {
      lofst = 0.1f;
    } else if (poseType == 3) {
      lofst = 0.4f;
    }
  }
  return lofst;
}

static inline void
geoPkOutDangerAngleSynthesizeVirtual(const GeoPkOutDangerAngleSearchInput_t *input,
                                     const GeoPkOutDangerAreaSearchInput_t *areaInput,
                                     Integer sideSign,
                                     Integer sideDir,
                                     Integer poseType,
                                     GeoPkOutDangerAngleSearchOutput_t *output) {
  Real vehWidth;
  Real vehFront;
  Real vehRear;
  Real lofst;
  Real lateralOffset;
  Real cosTheta;
  Real sinTheta;
  Real ptFrontX;
  Real ptFrontY;
  Real ptRearX;
  Real ptRearY;
  Real mixFront;
  Real mixRear;
  Real minCircle;
  Real outerFrontRadius;
  Real oneTurnLongitudinal;

  vehWidth = geoPkOutDangerGetVehWidth(&areaInput->VehConfig);
  vehFront = geoPkOutDangerGetVehFront(&areaInput->VehConfig);
  vehRear = geoPkOutDangerGetVehRear(&areaInput->VehConfig);
  lofst = geoPkOutDangerAngleGetVirtualOffset(sideDir, poseType);
  lateralOffset = ((Real)sideSign) * (0.5f * vehWidth + lofst);
  cosTheta = cosf(areaInput->PosTheta);
  sinTheta = sinf(areaInput->PosTheta);
  ptFrontX = areaInput->PosX + vehFront * cosTheta - lateralOffset * sinTheta;
  ptFrontY = areaInput->PosY + vehFront * sinTheta + lateralOffset * cosTheta;
  ptRearX = areaInput->PosX - vehRear * cosTheta - lateralOffset * sinTheta;
  ptRearY = areaInput->PosY - vehRear * sinTheta + lateralOffset * cosTheta;
  mixFront = 0.8f;
  mixRear = 0.2f;
  output->DangerPtX = mixFront * ptFrontX + mixRear * ptRearX;
  output->DangerPtY = mixFront * ptFrontY + mixRear * ptRearY;
  minCircle = input->VehConfig.MinCircle;
  if (minCircle <= APA_MATH_EPSILON) {
    minCircle = APA_VEHICLE_MIN_TURN_RADIUS_M;
  }
  outerFrontRadius =
      sqrtf(vehFront * vehFront + (minCircle + 0.5f * vehWidth) * (minCircle + 0.5f * vehWidth)) +
      0.2f;
  oneTurnLongitudinal = sqrtf(outerFrontRadius * outerFrontRadius -
                              (minCircle - 0.5f * vehWidth) * (minCircle - 0.5f * vehWidth));
  if ((sideDir == 1) && (output->ForwardSafeLen > oneTurnLongitudinal - vehFront)) {
    Real circleCenterX;
    Real circleCenterY;
    Real factor;
    Boolean searching;

    circleCenterX = input->VehX + minCircle * cosf(input->VehTheta + APA_PI / 2.0f);
    circleCenterY = input->VehY + minCircle * sinf(input->VehTheta + APA_PI / 2.0f);
    factor = 0.6f;
    searching = true;
    while ((factor <= 5.0f + APA_MATH_EPSILON) && searching) {
      Real dx;
      Real dy;

      output->DangerPtX = (1.0f - factor) * ptRearX + factor * ptFrontX;
      output->DangerPtY = (1.0f - factor) * ptRearY + factor * ptFrontY;
      dx = output->DangerPtX - circleCenterX;
      dy = output->DangerPtY - circleCenterY;
      if (dx * dx + dy * dy > outerFrontRadius * outerFrontRadius) {
        searching = false;
      } else {
        factor = factor + 0.1f;
      }
    }
  }
  output->PtEx = 2;
}

static inline Boolean
geoPkOutDangerAngleAreaPointNeedsVirtual(const GeoPkOutDangerAngleSearchInput_t *input,
                                         const GeoPkOutDangerAreaSearchOutput_t *areaOutput) {
  Real localX;
  Real localY;
  Real vehWidth;
  Real lateralLimit;
  Boolean needsVirtual;

  needsVirtual = false;
  if (areaOutput->PtEx > 0) {
    geoPkOutDangerGroundToLocal(areaOutput->DangerPtX,
                                areaOutput->DangerPtY,
                                input->VehX,
                                input->VehY,
                                input->VehTheta,
                                &localX,
                                &localY);
    vehWidth = geoPkOutDangerGetVehWidth(&input->VehConfig);
    lateralLimit = vehWidth / 6.0f;
    if (input->Dir == 1) {
      if (localY < -lateralLimit) {
        needsVirtual = true;
      }
    } else if (input->Dir == 2) {
      if (localY > lateralLimit) {
        needsVirtual = true;
      }
    }
  }
  return needsVirtual;
}

/**
 * @brief 泊出角度安全空间危险点搜索
 * @en_name geoPkOutDangerAngleSearch
 * @cn_name 泊出角度危险点搜索
 * @type module
 * @param[IN] GeoPkOutDangerAngleSearchInput_t input 搜索输入
 * @param[OUT] GeoPkOutDangerAngleSearchOutput_t output 搜索输出
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 泊出规划
 * @source matlab_rewrite geo_PkOut_dangerPt_AngSrch.m
 * @version 1.0
 * @date 2026-04-30
 * @author aide
 */
void geoPkOutDangerAngleSearch(const GeoPkOutDangerAngleSearchInput_t *input,
                               GeoPkOutDangerAngleSearchOutput_t *output) {
  GeoLineIntrvCheckInput_t lineInput;
  GeoLineIntrvCheckOutput_t lineOutput;
  GeoPkOutDangerAreaSearchInput_t areaInput;
  GeoPkOutDangerAreaSearchOutput_t areaOutput;
  Real poseXList[GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES];
  Real poseYList[GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES];
  Real poseThetaList[GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES];
  UInteger8 safeFlagList[GEO_PKOUT_DANGER_ANGLE_MAX_CANDIDATES];
  Integer startIndexList[GEO_PKOUT_DANGER_ANGLE_MAX_INTERVALS];
  Integer endIndexList[GEO_PKOUT_DANGER_ANGLE_MAX_INTERVALS];
  Integer widthList[GEO_PKOUT_DANGER_ANGLE_MAX_INTERVALS];
  Real poseX;
  Real poseY;
  Real poseTheta;
  Integer sideSign;
  Integer candidateCount;
  Integer collisionCheckCount;
  Integer intervalCount;
  Integer selectedIndex;
  Integer selectedPoseType;
  Integer sideDir;
  Boolean foundSafePose;
  Boolean useVirtualPoint;

  *output = (GeoPkOutDangerAngleSearchOutput_t){0};
  geoPkOutDangerFillLineInput(input, &lineInput);
  geoLineIntrvCheckPkOt(&lineInput, &lineOutput);
  output->ForwardSafeLen = lineOutput.SafeLen;
  output->CheckCount = lineOutput.CheckCount;

  geoPkOutDangerAngleBuildCandidates(input,
                                     poseXList,
                                     poseYList,
                                     poseThetaList,
                                     safeFlagList,
                                     &candidateCount,
                                     &collisionCheckCount);
  output->CheckCount = output->CheckCount + collisionCheckCount;
  geoPkOutDangerAngleFindSafeIntervals(
      safeFlagList, candidateCount, startIndexList, endIndexList, widthList, &intervalCount);
  if (intervalCount < 1) {
    output->SafeSpaceFlag = 0;
    output->PtEx = 1;
    output->DangerPtX = input->VehX;
    output->DangerPtY = input->VehY;
  } else {
    sideDir = 0;
    if ((input->Dir == 1) ||
        ((input->Dir == 3) && (input->FrontUs[3] > 0.3f) && (input->FrontUs[3] < 3.5f))) {
      sideDir = 1;
    } else if ((input->Dir == 2) ||
               ((input->Dir == 3) && (input->FrontUs[0] > 0.3f) && (input->FrontUs[0] < 3.5f))) {
      sideDir = 2;
    }
    if (sideDir > 0) {
      foundSafePose = geoPkOutDangerAngleSelectIntervalPose(input->Dir,
                                                            sideDir,
                                                            poseThetaList,
                                                            candidateCount,
                                                            startIndexList,
                                                            endIndexList,
                                                            widthList,
                                                            intervalCount,
                                                            &selectedIndex,
                                                            &selectedPoseType);
      if (foundSafePose) {
        poseX = poseXList[selectedIndex];
        poseY = poseYList[selectedIndex];
        poseTheta = poseThetaList[selectedIndex];
        sideSign = -1;
        if (sideDir == 2) {
          sideSign = 1;
        }
        output->SelectedPoseX = poseX;
        output->SelectedPoseY = poseY;
        output->SelectedPoseTheta = poseTheta;
        output->SafeSpaceFlag = 1;
        areaInput = (GeoPkOutDangerAreaSearchInput_t){0};
        areaInput.PosX = poseX;
        areaInput.PosY = poseY;
        areaInput.PosTheta = poseTheta;
        areaInput.LatExtend = 2.0f;
        areaInput.SideSign = sideSign;
        areaInput.VehConfig = input->VehConfig;
        areaInput.Obstacles = input->Obstacles;
        geoPkOutDangerAreaSearch(&areaInput, &areaOutput);
        useVirtualPoint = true;
        if (areaOutput.PtEx > 0) {
          useVirtualPoint = geoPkOutDangerAngleAreaPointNeedsVirtual(input, &areaOutput);
        }
        if (!useVirtualPoint) {
          output->PtEx = areaOutput.PtEx;
          output->DangerPtX = areaOutput.DangerPtX;
          output->DangerPtY = areaOutput.DangerPtY;
        } else {
          geoPkOutDangerAngleSynthesizeVirtual(
              input, &areaInput, sideSign, sideDir, selectedPoseType, output);
        }
        if (areaOutput.PtEx > 0) {
          Real areaLocalX;
          Real areaLocalY;

          geoPkOutDangerGroundToLocal(areaOutput.DangerPtX,
                                      areaOutput.DangerPtY,
                                      input->VehX,
                                      input->VehY,
                                      input->VehTheta,
                                      &areaLocalX,
                                      &areaLocalY);
          if (areaLocalX > 6.5f) {
            output->PtEx = 0;
          }
        }
      }
    } else if (input->Dir == 3) {
      Real checkLen;

      checkLen = input->FrontCheckLen;
      if (checkLen <= APA_MATH_EPSILON) {
        checkLen = 4.0f;
      }
      output->SafeSpaceFlag = 1;
      if (lineOutput.SafeLen <= checkLen - 0.1f) {
        Real localDangerX;

        localDangerX = lineOutput.SafeLen + geoPkOutDangerGetVehFront(&input->VehConfig);
        geoPkOutDangerAngleLocalToWorld(
            input, localDangerX, 0.0f, 0.0f, &output->DangerPtX, &output->DangerPtY, &poseTheta);
        output->PtEx = 1;
      }
    }
  }
}
