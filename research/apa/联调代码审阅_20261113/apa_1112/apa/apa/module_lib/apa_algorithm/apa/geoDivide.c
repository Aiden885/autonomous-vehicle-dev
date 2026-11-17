#include "geoDivideTypes.h"
#include <math.h>

/**
 * @brief 安全除法运算，当除数绝对值小于精度阈值时使用阈值替代除数以防止除零错误，根据除数符号保持结果方向性
 * @en_name geoDivide
 * @cn_name 安全除法
 * @type module
 * @param[IN] Real Numerator 被除数
 * @param[IN] Real Denominator 除数
 * @param[OUT] Real Result 除法结果
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 基础工具
 * @source matlab_rewrite geo_divide.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
void geoDivide(const GeoDivideInput_t *input, GeoDivideOutput_t *output) {
  Real numerator0;
  Real denominator0;
  Real absDenom0;
  Boolean condSmall;
  Boolean condNonNeg;
  Real safeResult1;
  Real normalResult0;
  Real result0;

  numerator0 = input->Numerator;
  denominator0 = input->Denominator;
  absDenom0 = fabsf(denominator0);

  condSmall = (absDenom0 < DIVIDE_ZERO_EPSILON);
  if (condSmall) {
    condNonNeg = (denominator0 >= 0.0f);
    if (condNonNeg) {
      safeResult1 = numerator0 / DIVIDE_ZERO_EPSILON;
    } else {
      safeResult1 = numerator0 / (-DIVIDE_ZERO_EPSILON);
    }
    result0 = safeResult1;
  } else {
    normalResult0 = numerator0 / denominator0;
    result0 = normalResult0;
  }

  output->Result = result0;
}
