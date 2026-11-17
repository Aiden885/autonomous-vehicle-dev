#include "geoRev2CircleLinkTypes.h"
#include "geoAxisTransTypes.h"
#include <math.h>

typedef struct {
  Real Ct1rx;
  Real Ct1ry;
  Real Ct2rx;
  Real Ct2ry;
  Real St2rx;
  Real St2ry;
  Real St2rTheta;
  Real St3rx;
  Real Tht1;
  Real Tht2;
  Integer TrnD1;
  Integer TrnD2;
  Integer GeoOk;
} GeoRev2CircleState_t;

static void geoRev2CircleLinkLeftRight(Real r1,
                                       Real r2,
                                       Real posRelaX,
                                       Real posRelaY,
                                       Real posRelaTheta,
                                       Boolean condDirPos,
                                       GeoRev2CircleState_t *state) {
  state->TrnD1 = 1;
  state->TrnD2 = -1;

  Real angPr;
  Real sinPr;
  Real cosPr;
  angPr = posRelaTheta + APA_HALF_PI;
  sinPr = sinf(angPr);
  cosPr = cosf(angPr);
  state->Ct1rx = posRelaX + r1 * cosPr;
  state->Ct1ry = posRelaY + r1 * sinPr;

  Real trnD2f;
  trnD2f = (Real)state->TrnD2;
  state->Ct2ry = trnD2f * r2;

  Real rSum;
  Real dCy;
  Real dCy2;
  Real rSum2;
  rSum = r1 + r2;
  dCy = state->Ct2ry - state->Ct1ry;
  dCy2 = dCy * dCy;
  rSum2 = rSum * rSum;
  Boolean condOk;
  condOk = (rSum2 > dCy2);

  if (condOk) {
    state->GeoOk = 1;
    Real inner;
    Real sqrtVal;
    inner = rSum2 - dCy2;
    sqrtVal = sqrtf(inner);

    Real ct1mct2;
    ct1mct2 = state->Ct1ry - state->Ct2ry;

    Real cosArg;
    Real absDd;
    absDd = fabsf(rSum);
    Boolean condTiny;
    condTiny = (absDd < GEO_EPSILON);
    if (condTiny) {
      cosArg = 0.0f;
    } else {
      cosArg = ct1mct2 / rSum;
    }
    Boolean condLow;
    condLow = (cosArg < -1.0f);
    if (condLow) {
      cosArg = -1.0f;
    }
    Boolean condHigh;
    condHigh = (cosArg > 1.0f);
    if (condHigh) {
      cosArg = 1.0f;
    }

    if (condDirPos) {
      Boolean condThetaNeg2;
      condThetaNeg2 = (posRelaTheta < 0.0f);
      if (condThetaNeg2) {
        state->Ct2rx = state->Ct1rx - sqrtVal;
      } else {
        state->Ct2rx = state->Ct1rx + sqrtVal;
      }
      state->Tht2 = -acosf(cosArg);
    } else {
      Boolean condThetaPos2;
      condThetaPos2 = (posRelaTheta >= 0.0f);
      if (condThetaPos2) {
        state->Ct2rx = state->Ct1rx + sqrtVal;
      } else {
        state->Ct2rx = state->Ct1rx - sqrtVal;
      }
      state->Tht2 = acosf(cosArg);
    }
    state->Tht1 = posRelaTheta - state->Tht2;

    state->St3rx = state->Ct2rx;
    Real sinT2;
    Real cosT2;
    sinT2 = sinf(state->Tht2);
    cosT2 = cosf(state->Tht2);
    state->St2rx = state->Ct2rx - r2 * sinT2;
    state->St2ry = state->Ct2ry + r2 * cosT2;
    state->St2rTheta = state->Tht2;
  }
}

static void geoRev2CircleLinkRightLeft(Real r1,
                                       Real r2,
                                       Real posRelaX,
                                       Real posRelaY,
                                       Real posRelaTheta,
                                       Boolean condDirPos,
                                       GeoRev2CircleState_t *state) {
  state->TrnD1 = -1;
  state->TrnD2 = 1;

  Real angPr;
  Real sinPr;
  Real cosPr;
  angPr = posRelaTheta - APA_HALF_PI;
  sinPr = sinf(angPr);
  cosPr = cosf(angPr);
  state->Ct1rx = posRelaX + r1 * cosPr;
  state->Ct1ry = posRelaY + r1 * sinPr;

  state->Ct2ry = r2;

  Real rSum;
  Real dCy;
  Real dCy2;
  Real rSum2;
  rSum = r1 + r2;
  dCy = state->Ct2ry - state->Ct1ry;
  dCy2 = dCy * dCy;
  rSum2 = rSum * rSum;
  Boolean condOk;
  condOk = (rSum2 > dCy2);

  if (condOk) {
    state->GeoOk = 1;
    Real inner;
    Real sqrtVal;
    inner = rSum2 - dCy2;
    sqrtVal = sqrtf(inner);

    Real ct2mct1;
    ct2mct1 = state->Ct2ry - state->Ct1ry;

    Real cosArg;
    Real absDd;
    absDd = fabsf(rSum);
    Boolean condTiny;
    condTiny = (absDd < GEO_EPSILON);
    if (condTiny) {
      cosArg = 0.0f;
    } else {
      cosArg = ct2mct1 / rSum;
    }
    Boolean condLow;
    condLow = (cosArg < -1.0f);
    if (condLow) {
      cosArg = -1.0f;
    }
    Boolean condHigh;
    condHigh = (cosArg > 1.0f);
    if (condHigh) {
      cosArg = 1.0f;
    }

    if (condDirPos) {
      Boolean condThetaPos2;
      condThetaPos2 = (posRelaTheta >= 0.0f);
      if (condThetaPos2) {
        state->Ct2rx = state->Ct1rx - sqrtVal;
      } else {
        state->Ct2rx = state->Ct1rx + sqrtVal;
      }
      state->Tht2 = acosf(cosArg);
    } else {
      Boolean condThetaNeg2;
      condThetaNeg2 = (posRelaTheta < 0.0f);
      if (condThetaNeg2) {
        state->Ct2rx = state->Ct1rx + sqrtVal;
      } else {
        state->Ct2rx = state->Ct1rx - sqrtVal;
      }
      state->Tht2 = -acosf(cosArg);
    }
    state->Tht1 = posRelaTheta - state->Tht2;

    state->St3rx = state->Ct2rx;
    Real sinT2;
    Real cosT2;
    sinT2 = sinf(state->Tht2);
    cosT2 = cosf(state->Tht2);
    state->St2rx = state->Ct2rx + r2 * sinT2;
    state->St2ry = state->Ct2ry - r2 * cosT2;
    state->St2rTheta = state->Tht2;
  }
}

/**
 * @brief 换向双圆弧连接，通过等半径反向双圆弧从起点P1换向行驶到终点P2，含几何可行性判断
 * @en_name geoRev2CircleLink
 * @cn_name 换向双圆弧连接
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
 * @param[OUT] Integer GeoOk 几何可行标志1可行0不可行
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 几何原语
 * @source matlab_rewrite geo_rev2Circle_Link.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
void geoRev2CircleLink(const GeoRev2CircleLinkInput_t *input, GeoRev2CircleLinkOutput_t *output) {

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
  Integer geoOk;

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
  len1 = 0.0f;
  len2 = 0.0f;

  Real ct1rx;
  Real ct1ry;
  Real ct2rx;
  Real ct2ry;
  Real st2rx;
  Real st2ry;
  Real st2rTheta;
  Real st3rx;

  Boolean condThetaPos;
  Boolean condThetaNeg;
  Boolean condDirPos;
  Boolean condDirNeg;
  condThetaPos = (posRelaTheta >= 0.0f);
  condThetaNeg = (posRelaTheta < 0.0f);
  condDirPos = (dir > 0);
  condDirNeg = (dir < 0);

  Boolean condBranch1a;
  Boolean condBranch1b;
  Boolean condBranch1;

  if (condThetaPos) {
    condBranch1a = condDirNeg;
  } else {
    condBranch1a = false;
  }
  if (condThetaNeg) {
    condBranch1b = condDirPos;
  } else {
    condBranch1b = false;
  }
  if (condBranch1a) {
    condBranch1 = true;
  } else {
    condBranch1 = condBranch1b;
  }

  GeoRev2CircleState_t hState;
  hState.Ct1rx = 0.0f;
  hState.Ct1ry = 0.0f;
  hState.Ct2rx = 0.0f;
  hState.Ct2ry = 0.0f;
  hState.St2rx = 0.0f;
  hState.St2ry = 0.0f;
  hState.St2rTheta = 0.0f;
  hState.St3rx = 0.0f;
  hState.Tht1 = 0.0f;
  hState.Tht2 = 0.0f;
  hState.TrnD1 = 0;
  hState.TrnD2 = 0;
  hState.GeoOk = 0;

  if (condBranch1) {
    geoRev2CircleLinkLeftRight(r1, r2, posRelaX, posRelaY, posRelaTheta, condDirPos, &hState);
  } else {
    geoRev2CircleLinkRightLeft(r1, r2, posRelaX, posRelaY, posRelaTheta, condDirPos, &hState);
  }

  ct1rx = hState.Ct1rx;
  ct1ry = hState.Ct1ry;
  ct2rx = hState.Ct2rx;
  ct2ry = hState.Ct2ry;
  st2rx = hState.St2rx;
  st2ry = hState.St2ry;
  st2rTheta = hState.St2rTheta;
  st3rx = hState.St3rx;
  tht1 = hState.Tht1;
  tht2 = hState.Tht2;
  trnD1 = hState.TrnD1;
  trnD2 = hState.TrnD2;
  geoOk = hState.GeoOk;

  if (geoOk == 1) {

    Real absTht1;
    Real absTht2;
    absTht1 = fabsf(tht1);
    absTht2 = fabsf(tht2);
    len1 = absTht1 * r1;
    len2 = absTht2 * r2;

    Real cosP2;
    Real sinP2;
    cosP2 = cosf(pt2Theta);
    sinP2 = sinf(pt2Theta);

    Real tmp1;
    Real tmp2;
    tmp1 = cosP2 * ct1rx;
    tmp2 = sinP2 * ct1ry;
    ct1X = tmp1 - tmp2 + pt2X;
    tmp1 = sinP2 * ct1rx;
    tmp2 = cosP2 * ct1ry;
    ct1Y = tmp1 + tmp2 + pt2Y;

    tmp1 = cosP2 * ct2rx;
    tmp2 = sinP2 * ct2ry;
    ct2X = tmp1 - tmp2 + pt2X;
    tmp1 = sinP2 * ct2rx;
    tmp2 = cosP2 * ct2ry;
    ct2Y = tmp1 + tmp2 + pt2Y;

    geoAxisTransFromTarget(st2rx, st2ry, st2rTheta, pt2X, pt2Y, pt2Theta, &st2X, &st2Y, &st2Theta);
    geoAxisTransFromTarget(st3rx, 0.0f, 0.0f, pt2X, pt2Y, pt2Theta, &st3X, &st3Y, &st3Theta);
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

  if (geoOk == 1) {
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
  output->GeoOk = geoOk;
}
