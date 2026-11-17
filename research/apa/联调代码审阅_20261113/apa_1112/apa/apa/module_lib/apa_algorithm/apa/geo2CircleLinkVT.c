#include "geo2CircleLinkVTTypes.h"
#include "geoAxisTransTypes.h"
#include <math.h>

#define GEO_EPSILON 1e-6f

/**
 * @brief VT双圆弧连接，按相对前后位置选择双圆弧几何并输出相对与绝对轨迹关键点
 * @en_name geo2CircleLinkVT
 * @cn_name VT双圆弧连接
 * @type module
 * @param[IN] Real Pt1X 起点X
 * @param[IN] Real Pt1Y 起点Y
 * @param[IN] Real Pt1Theta 起点航向角
 * @param[IN] Real Pt2X 终点X
 * @param[IN] Real Pt2Y 终点Y
 * @param[IN] Real Pt2Theta 终点航向角
 * @param[IN] Integer Dir 方向负1左侧模式正1右侧模式
 * @param[IN] Real R1 第一段圆弧半径
 * @param[IN] Real R2 第二段圆弧半径
 * @param[OUT] Real PosRelaX 起点在终点坐标系X
 * @param[OUT] Real PosRelaY 起点在终点坐标系Y
 * @param[OUT] Real PosRelaTheta 起点在终点坐标系航向
 * @param[OUT] Real Ct1X 圆心1 X
 * @param[OUT] Real Ct1Y 圆心1 Y
 * @param[OUT] Real Ct2X 圆心2 X
 * @param[OUT] Real Ct2Y 圆心2 Y
 * @param[OUT] Real St2X 衔接点X
 * @param[OUT] Real St2Y 衔接点Y
 * @param[OUT] Real St2Theta 衔接点航向
 * @param[OUT] Real St3X 终端点X
 * @param[OUT] Real St3Y 终端点Y
 * @param[OUT] Real St3Theta 终端点航向
 * @param[OUT] Real St3RelX 终端点相对坐标X
 * @param[OUT] Real St3RelY 终端点相对坐标Y
 * @param[OUT] Real St3RelTheta 终端点相对坐标航向
 * @param[OUT] Real Tht1 第一段圆弧角
 * @param[OUT] Real Tht2 第二段圆弧角
 * @param[OUT] Integer TrnD1 第一段转向
 * @param[OUT] Integer TrnD2 第二段转向
 * @param[OUT] Real Len1 第一段弧长
 * @param[OUT] Real Len2 第二段弧长
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 几何原语
 * @source matlab_rewrite geo_2Circle_LinkVT.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
void geo2CircleLinkVT(const Geo2CircleLinkVTInput_t *input, Geo2CircleLinkVTOutput_t *output) {
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

  Real posRx;
  Real posRy;
  Real posRtheta;
  geoAxisTransToTarget(pt1X, pt1Y, pt1Theta, pt2X, pt2Y, pt2Theta, &posRx, &posRy, &posRtheta);

  Real ct1rx;
  Real ct1ry;
  Real ct2rx;
  Real ct2ry;
  Real st2rx;
  Real st2ry;
  Real st2rTheta;
  Real st3rx;
  Real st3ry;
  Real st3rTheta;
  Real tht1;
  Real tht2;
  Integer trnD1;
  Integer trnD2;
  Real len1;
  Real len2;

  Real signPosX;
  if (posRx > 0.0f) {
    signPosX = 1.0f;
  } else {
    if (posRx < 0.0f) {
      signPosX = -1.0f;
    } else {
      signPosX = 0.0f;
    }
  }

  if (dir < 0) {
    trnD1 = -1;
    trnD2 = 1;
    Real angPr = posRtheta - 1.57079632679f;
    ct1rx = posRx + r1 * cosf(angPr);
    ct1ry = posRy + r1 * sinf(angPr);
    ct2ry = (Real)trnD2 * r2;
    Real rSum = r1 + r2;
    Real dCy = ct2ry - ct1ry;
    Real inner = rSum * rSum - dCy * dCy;
    Real innerAbs = fabsf(inner);
    Real innerSqrt = sqrtf(innerAbs);
    ct2rx = ct1rx - signPosX * innerSqrt;
    Real ratio;
    Real absDen = fabsf(rSum);
    if (absDen < GEO_EPSILON)
      ratio = 0.0f;
    else
      ratio = dCy / rSum;
    if (ratio > 1.0f)
      ratio = 1.0f;
    if (ratio < -1.0f)
      ratio = -1.0f;
    tht2 = -signPosX * acosf(ratio);
    tht1 = posRtheta - tht2;
    st3rx = ct2rx;
    st3ry = 0.0f;
    st3rTheta = 0.0f;
    st2rx = ct2rx - r2 * sinf(tht2);
    st2ry = ct2ry - r2 * cosf(tht2);
    st2rTheta = -tht2;
  } else {
    trnD1 = 1;
    trnD2 = -1;
    Real angPr = posRtheta + 1.57079632679f;
    ct1rx = posRx + r1 * cosf(angPr);
    ct1ry = posRy + r1 * sinf(angPr);
    ct2ry = (Real)trnD2 * r2;
    Real rSum = r1 + r2;
    Real dCy = ct2ry - ct1ry;
    Real inner = rSum * rSum - dCy * dCy;
    Real innerAbs = fabsf(inner);
    Real innerSqrt = sqrtf(innerAbs);
    ct2rx = ct1rx - signPosX * innerSqrt;
    Real ratio;
    Real absDen = fabsf(rSum);
    if (absDen < GEO_EPSILON)
      ratio = 0.0f;
    else
      ratio = (ct1ry - ct2ry) / rSum;
    if (ratio > 1.0f)
      ratio = 1.0f;
    if (ratio < -1.0f)
      ratio = -1.0f;
    tht2 = signPosX * acosf(ratio);
    tht1 = posRtheta - tht2;
    st3rx = ct2rx;
    st3ry = 0.0f;
    st3rTheta = 0.0f;
    st2rx = ct2rx + r2 * sinf(tht2);
    st2ry = ct2ry + r2 * cosf(tht2);
    st2rTheta = -tht2;
  }

  len1 = fabsf(tht1 * r1);
  len2 = fabsf(tht2 * r2);

  Real cosPt2;
  Real sinPt2;
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

  cosPt2 = cosf(pt2Theta);
  sinPt2 = sinf(pt2Theta);
  ct1X = cosPt2 * ct1rx - sinPt2 * ct1ry + pt2X;
  ct1Y = sinPt2 * ct1rx + cosPt2 * ct1ry + pt2Y;
  ct2X = cosPt2 * ct2rx - sinPt2 * ct2ry + pt2X;
  ct2Y = sinPt2 * ct2rx + cosPt2 * ct2ry + pt2Y;
  geoAxisTransFromTarget(st2rx, st2ry, st2rTheta, pt2X, pt2Y, pt2Theta, &st2X, &st2Y, &st2Theta);
  geoAxisTransFromTarget(st3rx, st3ry, st3rTheta, pt2X, pt2Y, pt2Theta, &st3X, &st3Y, &st3Theta);

  output->PosRelaX = posRx;
  output->PosRelaY = posRy;
  output->PosRelaTheta = posRtheta;
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
  output->St3RelX = st3rx;
  output->St3RelY = st3ry;
  output->St3RelTheta = st3rTheta;
  output->Tht1 = tht1;
  output->Tht2 = tht2;
  output->TrnD1 = trnD1;
  output->TrnD2 = trnD2;
  output->Len1 = len1;
  output->Len2 = len2;
}
