#include "geoLineCrossPtTypes.h"
#include <math.h>

/**
 * @brief 计算两直线交点坐标，当两直线近似平行时取直线1中点作为退化结果
 * @en_name geoLineCrossPt
 * @cn_name 两直线交点
 * @type module
 * @param[IN] Real Ln1X0 直线1起点X
 * @param[IN] Real Ln1Y0 直线1起点Y
 * @param[IN] Real Ln1X1 直线1终点X
 * @param[IN] Real Ln1Y1 直线1终点Y
 * @param[IN] Real Ln2X0 直线2起点X
 * @param[IN] Real Ln2Y0 直线2起点Y
 * @param[IN] Real Ln2X1 直线2终点X
 * @param[IN] Real Ln2Y1 直线2终点Y
 * @param[OUT] Real CrossX 交点X坐标
 * @param[OUT] Real CrossY 交点Y坐标
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 几何原语
 * @formula \f[ P = \frac{1}{A_1 B_2 - A_2 B_1} \begin{pmatrix} B_1 C_2 - B_2
 * C_1 \\ A_2 C_1 - A_1 C_2 \end{pmatrix} \f]
 * @source matlab_rewrite geo_line_cross_pt.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
void geoLineCrossPt(const GeoLineCrossPtInput_t *input, GeoLineCrossPtOutput_t *output) {

  Real ln1x0;
  Real ln1y0;
  Real ln1x1;
  Real ln1y1;
  Real ln2x0;
  Real ln2y0;
  Real ln2x1;
  Real ln2y1;

  ln1x0 = input->Ln1X0;
  ln1y0 = input->Ln1Y0;
  ln1x1 = input->Ln1X1;
  ln1y1 = input->Ln1Y1;
  ln2x0 = input->Ln2X0;
  ln2y0 = input->Ln2Y0;
  ln2x1 = input->Ln2X1;
  ln2y1 = input->Ln2Y1;

  Real aa1;
  Real bb1;
  Real cc1;
  Real tmp1;
  Real tmp2;
  aa1 = ln1y1 - ln1y0;
  bb1 = ln1x0 - ln1x1;
  tmp1 = ln1x1 * ln1y0;
  tmp2 = ln1x0 * ln1y1;
  cc1 = tmp1 - tmp2;

  Real aa2;
  Real bb2;
  Real cc2;
  Real tmp3;
  Real tmp4;
  aa2 = ln2y1 - ln2y0;
  bb2 = ln2x0 - ln2x1;
  tmp3 = ln2x1 * ln2y0;
  tmp4 = ln2x0 * ln2y1;
  cc2 = tmp3 - tmp4;

  Real dd;
  Real prod1;
  Real prod2;
  prod1 = aa1 * bb2;
  prod2 = aa2 * bb1;
  dd = prod1 - prod2;

  Real absDd;
  Boolean condParallel;
  absDd = fabsf(dd);
  condParallel = (absDd < 1e-3f);

  if (condParallel) {

    Real sumX;
    Real sumY;
    sumX = ln1x0 + ln1x1;
    output->CrossX = sumX * 0.5f;
    sumY = ln1y0 + ln1y1;
    output->CrossY = sumY * 0.5f;
  } else {

    Real numX1;
    Real numX2;
    Real numX;
    Real numY1;
    Real numY2;
    Real numY;
    numX1 = bb1 * cc2;
    numX2 = bb2 * cc1;
    numX = numX1 - numX2;
    output->CrossX = numX / dd;
    numY1 = aa2 * cc1;
    numY2 = aa1 * cc2;
    numY = numY1 - numY2;
    output->CrossY = numY / dd;
  }
}
