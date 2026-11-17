#ifndef GEO_DIVIDE_TYPES_H
#define GEO_DIVIDE_TYPES_H

#include "baseTypes.h"

/** @brief 除零保护精度 */
#define DIVIDE_ZERO_EPSILON 1e-6f

/**
 * @brief 安全除法输入
 * @en_name GeoDivideInput
 * @cn_name 安全除法输入
 * @tag STRUCT
 * @field name=Numerator, type=Real, unit=1, desc=被除数
 * @field name=Denominator, type=Real, unit=1, desc=除数
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real Numerator;   // 被除数
  Real Denominator; // 除数
} GeoDivideInput_t;

/**
 * @brief 安全除法输出
 * @en_name GeoDivideOutput
 * @cn_name 安全除法输出
 * @tag STRUCT
 * @field name=Result, type=Real, unit=1, desc=除法结果
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real Result; // 除法结果
} GeoDivideOutput_t;

void geoDivide(const GeoDivideInput_t *input, GeoDivideOutput_t *output);

/** @brief Scalar adapter for the canonical MATLAB geo_divide module. */
static inline Real geoDivideScalar(Real numerator, Real denominator) {
  GeoDivideInput_t input;
  GeoDivideOutput_t output;

  input.Numerator = numerator;
  input.Denominator = denominator;
  output = (GeoDivideOutput_t){0};
  geoDivide(&input, &output);
  return output.Result;
}

#endif
