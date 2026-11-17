#include "geoAxisTransTypes.h"
#include <math.h>

/**
 * @brief 坐标系变换，将点从源坐标系Ori1变换到目标坐标系Ori2中的表示，包含旋转平移和航向角归一化
 * @en_name geoAxisTrans
 * @cn_name 坐标系变换
 * @type module
 * @param[IN] Real PtX 待变换点的X坐标
 * @param[IN] Real PtY 待变换点的Y坐标
 * @param[IN] Real PtTheta 待变换点的航向角
 * @param[IN] Real Ori1X 源坐标系原点X
 * @param[IN] Real Ori1Y 源坐标系原点Y
 * @param[IN] Real Ori1Theta 源坐标系原点航向角
 * @param[IN] Real Ori2X 目标坐标系原点X
 * @param[IN] Real Ori2Y 目标坐标系原点Y
 * @param[IN] Real Ori2Theta 目标坐标系原点航向角
 * @param[OUT] Real ResultX 变换后的X坐标
 * @param[OUT] Real ResultY 变换后的Y坐标
 * @param[OUT] Real ResultTheta 变换后的航向角
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 坐标变换
 * @formula \f[
 * \Delta = R(\theta_1)^T \cdot (O_2 - O_1), \quad
 * P_r = R(\Delta\theta)^T \cdot (P - \Delta_{xy})
 * \f]
 * @source matlab_rewrite geo_axis_trans.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
void geoAxisTrans(const GeoAxisTransInput_t *input, GeoAxisTransOutput_t *output) {

  Real ptX0;
  Real ptY0;
  Real ptTheta0;
  Real ori1X0;
  Real ori1Y0;
  Real ori1Theta0;
  Real ori2X0;
  Real ori2Y0;
  Real ori2Theta0;

  ptX0 = input->PtX;
  ptY0 = input->PtY;
  ptTheta0 = input->PtTheta;
  ori1X0 = input->Ori1X;
  ori1Y0 = input->Ori1Y;
  ori1Theta0 = input->Ori1Theta;
  ori2X0 = input->Ori2X;
  ori2Y0 = input->Ori2Y;
  ori2Theta0 = input->Ori2Theta;

  Real cosOri1;
  Real sinOri1;
  cosOri1 = cosf(ori1Theta0);
  sinOri1 = sinf(ori1Theta0);

  Real diffX0;
  Real diffY0;
  diffX0 = ori2X0 - ori1X0;
  diffY0 = ori2Y0 - ori1Y0;

  Real term1a;
  Real term1b;
  Real term2a;
  Real term2b;
  Real dltX0;
  Real dltY0;
  Real dltTheta0;
  term1a = cosOri1 * diffX0;
  term1b = sinOri1 * diffY0;
  dltX0 = term1a + term1b;
  term2a = cosOri1 * diffY0;
  term2b = sinOri1 * diffX0;
  dltY0 = term2a - term2b;
  dltTheta0 = ori2Theta0 - ori1Theta0;

  Real cosDlt;
  Real sinDlt;
  cosDlt = cosf(dltTheta0);
  sinDlt = sinf(dltTheta0);

  Real localX0;
  Real localY0;
  localX0 = ptX0 - dltX0;
  localY0 = ptY0 - dltY0;

  Real term3a;
  Real term3b;
  Real term4a;
  Real term4b;
  Real resultX0;
  Real resultY0;
  Real resultTheta0;
  Real resultTheta1;
  Real resultTheta2;
  term3a = cosDlt * localX0;
  term3b = sinDlt * localY0;
  resultX0 = term3a + term3b;
  term4a = cosDlt * localY0;
  term4b = sinDlt * localX0;
  resultY0 = term4a - term4b;
  resultTheta0 = ptTheta0 - dltTheta0;

  Boolean condLessThanNegPi;
  Boolean condGreaterThanPi;
  condGreaterThanPi = (resultTheta0 > APA_PI);
  if (condGreaterThanPi) {
    resultTheta1 = resultTheta0 - APA_TWO_PI;
  } else {
    resultTheta1 = resultTheta0;
  }
  condLessThanNegPi = (resultTheta1 < -APA_PI);
  if (condLessThanNegPi) {
    resultTheta2 = resultTheta1 + APA_TWO_PI;
  } else {
    resultTheta2 = resultTheta1;
  }

  output->ResultX = resultX0;
  output->ResultY = resultY0;
  output->ResultTheta = resultTheta2;
}
