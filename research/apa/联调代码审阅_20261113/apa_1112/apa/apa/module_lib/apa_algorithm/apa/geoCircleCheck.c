#include "geoCircleCheckTypes.h"
#include <math.h>

#ifndef APA_PI
#define APA_PI 3.1415926535f
#endif

#ifndef APA_TWO_PI
#define APA_TWO_PI 6.2831853072f
#endif

/**
 * @file geoCircleCheck.c
 * @brief 几何圆弧连接许可计算
 * @en_name geoCircleCheck
 * @cn_name 几何圆弧连接许可计算
 * @type module
 * @param[IN] GeoCircleCheckInput_t Input 圆弧连接输入
 * @param[OUT] GeoCircleCheckOutput_t Output 圆弧连接输出
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 几何计算
 * @source matlab_rewrite geo_Circle_Check.m
 * @version 1.0
 * @date 2026-04-14
 * @author aide
 */
void geoCircleCheck(const GeoCircleCheckInput_t *input, GeoCircleCheckOutput_t *output) {
  Real posRelaX;
  Real posRelaY;
  Real posRelaTheta;
  Real cosP1;
  Real sinP1;
  Real dX;
  Real dY;
  Real cP2;

  cosP1 = cosf(input->Pt1Theta);
  sinP1 = sinf(input->Pt1Theta);
  dX = input->Pt2X - input->Pt1X;
  dY = input->Pt2Y - input->Pt1Y;

  posRelaX = cosP1 * dX + sinP1 * dY;
  posRelaY = cosP1 * dY - sinP1 * dX;
  posRelaTheta = input->Pt2Theta - input->Pt1Theta;

  if (posRelaTheta > APA_PI) {
    posRelaTheta = posRelaTheta - APA_TWO_PI;
  } else if (posRelaTheta < -APA_PI) {
    posRelaTheta = posRelaTheta + APA_TWO_PI;
  }

  cP2 = posRelaY * cosf(posRelaTheta) - posRelaX * sinf(posRelaTheta);

  if (((posRelaY > 0.0f) && (cP2 < 0.0f)) || ((posRelaY < 0.0f) && (cP2 > 0.0f))) {
    output->COk = true;
  } else {
    output->COk = false;
  }

  output->PosRelaX = posRelaX;
  output->PosRelaY = posRelaY;
  output->PosRelaTheta = posRelaTheta;
}
