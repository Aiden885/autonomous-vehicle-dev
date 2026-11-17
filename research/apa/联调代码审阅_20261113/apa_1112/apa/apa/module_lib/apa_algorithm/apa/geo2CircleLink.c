#include "geo2CircleLinkTypes.h"
#include "geoAxisTransTypes.h"
#include <math.h>

#define GEO_EPSILON 1e-6f

static void geo2CircleLinkCalcGeomPos(Integer dir,
                                      Real r1,
                                      Real r2,
                                      Real posRelaX,
                                      Real posRelaY,
                                      Real posRelaTheta,
                                      Integer *trnD1,
                                      Integer *trnD2,
                                      Integer *plnFlg,
                                      Real *ct1rx,
                                      Real *ct1ry,
                                      Real *ct2rx,
                                      Real *ct2ry,
                                      Real *st2rx,
                                      Real *st2ry,
                                      Real *st2rTheta,
                                      Real *st3rx,
                                      Real *tht1,
                                      Real *tht2) {
  *trnD1 = -1;
  *trnD2 = 1;

  Real sinPr = sinf(posRelaTheta - APA_HALF_PI);
  Real cosPr = cosf(posRelaTheta - APA_HALF_PI);

  *ct1rx = posRelaX + r1 * cosPr;
  *ct1ry = posRelaY + r1 * sinPr;

  if (dir == 1) {
    *ct2ry = (Real)(*trnD2) * r2;

    Real rSum = r1 + r2;
    Real dCy = *ct2ry - *ct1ry;
    if (rSum < dCy) {
      *plnFlg = 0;
    } else {
      Real dCy2 = dCy * dCy;
      Real rSum2 = rSum * rSum;
      Real rad = rSum2 - dCy2;
      if (rad < 0.0f)
        rad = 0.0f;
      *ct2rx = *ct1rx + sqrtf(rad);

      Real cosArg;
      Real absDd = fabsf(rSum);
      if (absDd < GEO_EPSILON)
        cosArg = 0.0f;
      else
        cosArg = dCy / rSum;
      if (cosArg < -1.0f)
        cosArg = -1.0f;
      if (cosArg > 1.0f)
        cosArg = 1.0f;
      *tht2 = -acosf(cosArg);
      *tht1 = posRelaTheta - *tht2;
    }
  } else {
    *ct2ry = r2;
    Real rSum = r1 + r2;
    Real dCy = *ct2ry - *ct1ry;
    if (rSum < dCy) {
      *plnFlg = 0;
    } else {
      Real dCy2 = dCy * dCy;
      Real rSum2 = rSum * rSum;
      Real rad = rSum2 - dCy2;
      if (rad < 0.0f)
        rad = 0.0f;
      *ct2rx = *ct1rx - sqrtf(rad);

      Real cosArg;
      Real absDd = fabsf(rSum);
      if (absDd < GEO_EPSILON)
        cosArg = 0.0f;
      else
        cosArg = dCy / rSum;
      if (cosArg < -1.0f)
        cosArg = -1.0f;
      if (cosArg > 1.0f)
        cosArg = 1.0f;
      *tht2 = acosf(cosArg);
      *tht1 = posRelaTheta - *tht2;
    }
  }

  if (*plnFlg == 1) {
    *st3rx = *ct2rx;
    Real angT2 = *tht2 - APA_HALF_PI;
    *st2rx = *ct2rx + r2 * cosf(angT2);
    *st2ry = *ct2ry + r2 * sinf(angT2);
    *st2rTheta = *tht2;
  }
}

static void geo2CircleLinkCalcGeomNeg(Integer dir,
                                      Real r1,
                                      Real r2,
                                      Real posRelaX,
                                      Real posRelaY,
                                      Real posRelaTheta,
                                      Integer *trnD1,
                                      Integer *trnD2,
                                      Integer *plnFlg,
                                      Real *ct1rx,
                                      Real *ct1ry,
                                      Real *ct2rx,
                                      Real *ct2ry,
                                      Real *st2rx,
                                      Real *st2ry,
                                      Real *st2rTheta,
                                      Real *st3rx,
                                      Real *tht1,
                                      Real *tht2) {
  *trnD1 = 1;
  *trnD2 = -1;

  Real sinPr = sinf(posRelaTheta + APA_HALF_PI);
  Real cosPr = cosf(posRelaTheta + APA_HALF_PI);

  *ct1rx = posRelaX + r1 * cosPr;
  *ct1ry = posRelaY + r1 * sinPr;

  if (dir == -1) {
    *ct2ry = -r2;
    Real rSum = r1 + r2;
    Real dCy = *ct2ry - *ct1ry;
    if (rSum < dCy) {
      *plnFlg = 0;
    } else {
      Real dCy2 = dCy * dCy;
      Real rSum2 = rSum * rSum;
      Real rad = rSum2 - dCy2;
      if (rad < 0.0f)
        rad = 0.0f;
      *ct2rx = *ct1rx - sqrtf(rad);

      Real cosArg;
      Real ct1mct2 = *ct1ry - *ct2ry;
      Real absDd = fabsf(rSum);
      if (absDd < GEO_EPSILON)
        cosArg = 0.0f;
      else
        cosArg = ct1mct2 / rSum;
      if (cosArg < -1.0f)
        cosArg = -1.0f;
      if (cosArg > 1.0f)
        cosArg = 1.0f;
      *tht2 = -acosf(cosArg);
      *tht1 = posRelaTheta - *tht2;
    }
  } else {
    *ct2ry = -r2;
    Real rSum = r1 + r2;
    Real dCy = *ct2ry - *ct1ry;
    if (rSum < dCy) {
      *plnFlg = 0;
    } else {
      Real dCy2 = dCy * dCy;
      Real rSum2 = rSum * rSum;
      Real rad = rSum2 - dCy2;
      if (rad < 0.0f)
        rad = 0.0f;
      *ct2rx = *ct1rx + sqrtf(rad);

      Real cosArg;
      Real ct1mct2 = *ct1ry - *ct2ry;
      Real absDd = fabsf(rSum);
      if (absDd < GEO_EPSILON)
        cosArg = 0.0f;
      else
        cosArg = ct1mct2 / rSum;
      if (cosArg < -1.0f)
        cosArg = -1.0f;
      if (cosArg > 1.0f)
        cosArg = 1.0f;
      *tht2 = acosf(cosArg);
      *tht1 = posRelaTheta - *tht2;
    }
  }

  if (*plnFlg == 1) {
    *st3rx = *ct2rx;
    Real angT2 = *tht2 + APA_HALF_PI;
    *st2rx = *ct2rx + r2 * cosf(angT2);
    *st2ry = *ct2ry + r2 * sinf(angT2);
    *st2rTheta = *tht2;
  }
}

static void geo2CircleLinkCalcTransform(Real pt2X,
                                        Real pt2Y,
                                        Real pt2Theta,
                                        Real ct1rx,
                                        Real ct1ry,
                                        Real ct2rx,
                                        Real ct2ry,
                                        Real st2rx,
                                        Real st2ry,
                                        Real st2rTheta,
                                        Real st3rx,
                                        Real *ct1X,
                                        Real *ct1Y,
                                        Real *ct2X,
                                        Real *ct2Y,
                                        Real *st2X,
                                        Real *st2Y,
                                        Real *st2Theta,
                                        Real *st3X,
                                        Real *st3Y,
                                        Real *st3Theta) {
  Real cosP2 = cosf(pt2Theta);
  Real sinP2 = sinf(pt2Theta);
  *ct1X = cosP2 * ct1rx - sinP2 * ct1ry + pt2X;
  *ct1Y = sinP2 * ct1rx + cosP2 * ct1ry + pt2Y;
  *ct2X = cosP2 * ct2rx - sinP2 * ct2ry + pt2X;
  *ct2Y = sinP2 * ct2rx + cosP2 * ct2ry + pt2Y;
  geoAxisTransFromTarget(st2rx, st2ry, st2rTheta, pt2X, pt2Y, pt2Theta, st2X, st2Y, st2Theta);
  geoAxisTransFromTarget(st3rx, 0.0f, 0.0f, pt2X, pt2Y, pt2Theta, st3X, st3Y, st3Theta);
}

/**
 * @brief 通过等半径双圆弧S曲线连接起点P1到终点P2，计算两段圆弧的圆心衔接点弧长等参数
 * @en_name geo2CircleLink
 * @cn_name 双圆弧S曲线
 * @type module
 * @param[IN] Real Pt1X 起点X
 * @param[IN] Real Pt1Y 起点Y
 * @param[IN] Real Pt1Theta 起点航向角
 * @param[IN] Real Pt2X 终点X
 * @param[IN] Real Pt2Y 终点Y
 * @param[IN] Real Pt2Theta 终点航向角
 * @param[IN] Integer Dir 行驶方向1前进负1后退
 * @param[IN] Real R1 第一段圆弧半径
 * @param[IN] Real R2 第二段圆弧半径
 * @param[OUT] Real PosRelaX Pt1在Pt2坐标系X
 * @param[OUT] Real PosRelaY Pt1在Pt2坐标系Y
 * @param[OUT] Real PosRelaTheta Pt1在Pt2坐标系航向
 * @param[OUT] Real Ct1X 圆心1 X
 * @param[OUT] Real Ct1Y 圆心1 Y
 * @param[OUT] Real Ct2X 圆心2 X
 * @param[OUT] Real Ct2Y 圆心2 Y
 * @param[OUT] Real St2X 衔接点X
 * @param[OUT] Real St2Y 衔接点Y
 * @param[OUT] Real St2Theta 衔接点航向角
 * @param[OUT] Real St3X 终端点X
 * @param[OUT] Real St3Y 终端点Y
 * @param[OUT] Real St3Theta 终端点航向角
 * @param[OUT] Real St3rX 终端点在Pt2坐标系X
 * @param[OUT] Real St3rY 终端点在Pt2坐标系Y
 * @param[OUT] Real St3rTheta 终端点在Pt2坐标系航向
 * @param[OUT] Real Tht1 第一段圆弧角
 * @param[OUT] Real Tht2 第二段圆弧角
 * @param[OUT] Integer TrnD1 第一段转弯方向
 * @param[OUT] Integer TrnD2 第二段转弯方向
 * @param[OUT] Real Len1 第一段弧长
 * @param[OUT] Real Len2 第二段弧长
 * @param[OUT] Integer PlnFlg 规划成功标志1成功0失败
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 几何原语
 * @source matlab_rewrite geo_2Circle_Link.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
void geo2CircleLink(const Geo2CircleLinkInput_t *input, Geo2CircleLinkOutput_t *output) {

  Real pt1X;
  Real pt1Y;
  Real pt1Theta;
  Real pt2X;
  Real pt2Y;
  Real pt2Theta;
  Integer dir;
  Real r1;
  Real r2;

  pt1X = input->Pt1X;
  pt1Y = input->Pt1Y;
  pt1Theta = input->Pt1Theta;
  pt2X = input->Pt2X;
  pt2Y = input->Pt2Y;
  pt2Theta = input->Pt2Theta;
  dir = input->Dir;
  r1 = input->R1;
  r2 = input->R2;

  Real posRelaX;
  Real posRelaY;
  Real posRelaTheta;
  geoAxisTransToTarget(
      pt1X, pt1Y, pt1Theta, pt2X, pt2Y, pt2Theta, &posRelaX, &posRelaY, &posRelaTheta);

  Real ct1X;
  Real ct1Y;
  Real ct2X;
  Real ct2Y;
  Real st2X;
  Real st2Y;
  Real st2Theta;
  Real st3X;
  Real st3Y;
  Real st3Theta;
  Real tht1;
  Real tht2;
  Integer trnD1;
  Integer trnD2;
  Real len1;
  Real len2;
  Integer plnFlg;

  ct1X = 0.0f;
  ct1Y = 0.0f;
  ct2X = 0.0f;
  ct2Y = 0.0f;
  st2X = 0.0f;
  st2Y = 0.0f;
  st2Theta = 0.0f;
  st3X = 0.0f;
  st3Y = 0.0f;
  st3Theta = 0.0f;
  tht1 = 0.0f;
  tht2 = 0.0f;
  trnD1 = 0;
  trnD2 = 0;
  len1 = 0.0f;
  len2 = 0.0f;
  plnFlg = 1;

  Real ct1rx = 0.0f;
  Real ct1ry = 0.0f;
  Real ct2rx = 0.0f;
  Real ct2ry = 0.0f;
  Real st2rx = 0.0f;
  Real st2ry = 0.0f;
  Real st2rTheta = 0.0f;
  Real st3rx = 0.0f;

  if (posRelaY > 0.0f) {
    geo2CircleLinkCalcGeomPos(dir,
                              r1,
                              r2,
                              posRelaX,
                              posRelaY,
                              posRelaTheta,
                              &trnD1,
                              &trnD2,
                              &plnFlg,
                              &ct1rx,
                              &ct1ry,
                              &ct2rx,
                              &ct2ry,
                              &st2rx,
                              &st2ry,
                              &st2rTheta,
                              &st3rx,
                              &tht1,
                              &tht2);
  } else {
    geo2CircleLinkCalcGeomNeg(dir,
                              r1,
                              r2,
                              posRelaX,
                              posRelaY,
                              posRelaTheta,
                              &trnD1,
                              &trnD2,
                              &plnFlg,
                              &ct1rx,
                              &ct1ry,
                              &ct2rx,
                              &ct2ry,
                              &st2rx,
                              &st2ry,
                              &st2rTheta,
                              &st3rx,
                              &tht1,
                              &tht2);
  }

  if (plnFlg == 1) {

    Real absTht1;
    Real absTht2;
    absTht1 = fabsf(tht1);
    absTht2 = fabsf(tht2);
    len1 = absTht1 * r1;
    len2 = absTht2 * r2;

    geo2CircleLinkCalcTransform(pt2X,
                                pt2Y,
                                pt2Theta,
                                ct1rx,
                                ct1ry,
                                ct2rx,
                                ct2ry,
                                st2rx,
                                st2ry,
                                st2rTheta,
                                st3rx,
                                &ct1X,
                                &ct1Y,
                                &ct2X,
                                &ct2Y,
                                &st2X,
                                &st2Y,
                                &st2Theta,
                                &st3X,
                                &st3Y,
                                &st3Theta);
  }

  output->PosRelaX = posRelaX;
  output->PosRelaY = posRelaY;
  output->PosRelaTheta = posRelaTheta;
  output->Ct1X = ct1X;
  output->Ct1Y = ct1Y;
  output->Ct2X = ct2X;
  output->Ct2Y = ct2Y;
  output->St2X = st2X;
  output->St2Y = st2Y;
  output->St2Theta = st2Theta;
  output->St3X = st3X;
  output->St3Y = st3Y;
  output->St3Theta = st3Theta;

  if (plnFlg == 1) {
    output->St3rX = st3rx;
  } else {
    output->St3rX = 0.0f;
  }
  output->St3rY = 0.0f;
  output->St3rTheta = 0.0f;
  output->Tht1 = tht1;
  output->Tht2 = tht2;
  output->TrnD1 = trnD1;
  output->TrnD2 = trnD2;
  output->Len1 = len1;
  output->Len2 = len2;
  output->PlnFlg = plnFlg;
}
