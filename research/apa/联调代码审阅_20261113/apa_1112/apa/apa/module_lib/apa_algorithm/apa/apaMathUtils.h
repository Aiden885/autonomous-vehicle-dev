/**
 * @file apaMathUtils.h
 * @brief APA 共享数学工具函数
 *
 * 对应 MATLAB 基础模块库中的数学运算模块：
 *   - uisee_float32_equal.m   → apaMathEqual
 *   - uisee_float32_less.m    → apaMathLess
 *   - uisee_float32_greater.m → apaMathGreater
 *   - float32_divide_zero_process.m → apaMathDivideZeroProcess
 *   - uisee_float32_cross_3D.m     → apaMathCross3D
 *   - circle_curvature_cal.m       → apaMathCircleCurvature
 *
 * @date 2026-03-15
 */
#ifndef APA_MATH_UTILS_H
#define APA_MATH_UTILS_H

#include "baseTypes.h"
#include <math.h>

#ifndef APA_MATH_EPSILON
#define APA_MATH_EPSILON 1e-6f
#endif

#ifndef APA_MATH_INF
#define APA_MATH_INF 1e6f
#endif

#ifndef APA_PI
#define APA_PI 3.14159265f
#endif

/** @brief Reject NaN and non-physical magnitudes at public path boundaries. */
static inline Boolean apaMathIsFinitePathValue(Real value) {
  Boolean condFinite0;

  condFinite0 = (value == value) && (fabsf(value) < 1.0e20f);
  return condFinite0;
}

/** @brief 浮点等于比较 — 对应 uisee_float32_equal.m */
static inline Boolean apaMathEqual(Real floatA, Real floatB) {
  Real diff;

  diff = floatA - floatB;
  if (diff < 0.0f) {
    diff = -diff;
  }
  return (diff < APA_MATH_EPSILON);
}

/** @brief 浮点小于比较 — 对应 uisee_float32_less.m */
static inline Boolean apaMathLess(Real floatA, Real floatB) {
  return (!apaMathEqual(floatA, floatB)) && (floatA < floatB);
}

/** @brief 浮点大于比较 — 对应 uisee_float32_greater.m */
static inline Boolean apaMathGreater(Real floatA, Real floatB) {
  return (!apaMathEqual(floatA, floatB)) && (floatA > floatB);
}

/** @brief 绝对值 */
static inline Real apaMathAbs(Real value) {
  if (value < 0.0f) {
    return -value;
  }
  return value;
}

/** @brief Return -1, 0, or 1 from the strict sign of a scalar. */
static inline Integer apaMathSign(Real value) {
  Integer result0;

  result0 = 0;
  if (value > 0.0f) {
    result0 = 1;
  } else if (value < 0.0f) {
    result0 = -1;
  }
  return result0;
}

/** @brief Return the greater of two scalar values. */
static inline Real apaMathMax(Real firstValue, Real secondValue) {
  Real result0;

  result0 = secondValue;
  if (firstValue > secondValue) {
    result0 = firstValue;
  }
  return result0;
}

/** @brief Return the lesser of two scalar values. */
static inline Real apaMathMin(Real firstValue, Real secondValue) {
  Real result0;

  result0 = secondValue;
  if (firstValue < secondValue) {
    result0 = firstValue;
  }
  return result0;
}

/** @brief Clamp a scalar value to [low, high]; requires low <= high. */
static inline Real apaMathClamp(Real value, Real low, Real high) {
  Real result0;

  result0 = value;
  if (result0 < low) {
    result0 = low;
  }
  if (result0 > high) {
    result0 = high;
  }
  return result0;
}

/** @brief Clamp a scalar value to the trigonometric domain [-1, 1]. */
static inline Real apaMathClampUnit(Real value) {
  return apaMathClamp(value, -1.0f, 1.0f);
}

/** @brief Use a fallback when a nominal positive value is at numerical zero. */
static inline Real apaMathResolvePositiveOrFallback(Real value, Real fallbackValue) {
  Real result0;

  result0 = value;
  if (result0 <= APA_MATH_EPSILON) {
    result0 = fallbackValue;
  }
  return result0;
}

/** @brief Linear interpolation without implicit ratio clamping. */
static inline Real apaMathLinearInterpolate(Real minValue, Real maxValue, Real ratio) {
  return minValue + ((maxValue - minValue) * ratio);
}

/** @brief 除零保护 — 对应 float32_divide_zero_process.m */
static inline Real apaMathDivideZeroProcess(Real inputValue) {
  Real absVal;

  absVal = inputValue;
  if (absVal < 0.0f) {
    absVal = -absVal;
  }
  if (absVal < APA_MATH_EPSILON) {
    if (inputValue >= 0.0f) {
      return APA_MATH_EPSILON;
    }
    return -APA_MATH_EPSILON;
  }
  return inputValue;
}

/** @brief 3D 叉积 — 对应 uisee_float32_cross_3D.m */
static inline void apaMathCross3D(
    Real x1, Real y1, Real z1, Real x2, Real y2, Real z2, Real *outX, Real *outY, Real *outZ) {
  *outX = y1 * z2 - z1 * y2;
  *outY = z1 * x2 - x1 * z2;
  *outZ = x1 * y2 - y1 * x2;
}

/**
 * @brief 三点定圆曲率计算 — 对应 circle_curvature_cal.m
 *
 * 通过三角形外接圆公式: kappa = 2*sin(A)/a
 * @param ax,ay 点A坐标 (当前点)
 * @param bx,by 点B坐标 (前一点)
 * @param cx,cy 点C坐标 (后一点)
 * @param withSign 是否带符号（true: 根据BC方向确定正负）
 * @return 曲率值
 */
static inline Real
apaMathCircleCurvature(Real ax, Real ay, Real bx, Real by, Real cx, Real cy, Boolean withSign) {
  Real sideC;
  Real sideA;
  Real sideB;
  Real cosAlpha;
  Real alphaA;
  Real kappa;

  sideC = sqrtf((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
  sideA = sqrtf((bx - cx) * (bx - cx) + (by - cy) * (by - cy));
  sideB = sqrtf((ax - cx) * (ax - cx) + (ay - cy) * (ay - cy));

  if ((sideA < APA_MATH_EPSILON) || (sideB < APA_MATH_EPSILON) || (sideC < APA_MATH_EPSILON)) {
    return APA_MATH_INF;
  }

  if (((sideA + sideB - sideC) < APA_MATH_EPSILON) ||
      ((sideA + sideC - sideB) < APA_MATH_EPSILON) ||
      ((sideB + sideC - sideA) < APA_MATH_EPSILON)) {
    Real tempCos;
    Real angleCheck;

    tempCos = (sideB * sideB + sideC * sideC - sideA * sideA) / (2.0f * sideB * sideC);
    tempCos = apaMathClampUnit(tempCos);
    angleCheck = acosf(tempCos);
    if (angleCheck < 0.01f) {
      return APA_MATH_INF;
    }
    if (angleCheck > (2.356194f)) {
      return 0.0f;
    }
    return APA_MATH_INF;
  }

  cosAlpha = (sideB * sideB + sideC * sideC - sideA * sideA) / (2.0f * sideB * sideC);
  cosAlpha = apaMathClampUnit(cosAlpha);
  alphaA = acosf(cosAlpha);
  kappa = 2.0f * sinf(alphaA) / apaMathDivideZeroProcess(sideA);

  if (!withSign) {
    return kappa;
  }

  {
    Real slopeK;
    Real interceptM;

    slopeK = (by - cy) / apaMathDivideZeroProcess(bx - cx);
    interceptM = by - slopeK * bx;
    if ((slopeK * ax + interceptM) < ay) {
      kappa = -kappa;
    }
  }

  return kappa;
}

/** @brief 2D 叉积 — 对应 uisee_float32_cross_2D.m */
static inline Real apaMathCross2D(Real ax, Real ay, Real bx, Real by) {
  return ax * by - ay * bx;
}

/** @brief 2D 点积 */
static inline Real apaMathDot2D(Real ax, Real ay, Real bx, Real by) {
  return ax * bx + ay * by;
}

/** @brief 2D 向量 2-范数 — 对应 uisee_float32_norm.m (p=2) */
static inline Real apaMathNorm2D(Real x, Real y) {
  return sqrtf(x * x + y * y);
}

/** @brief Normalize a 2D vector with the shared near-zero denominator rule. */
static inline void apaMathNormalize2D(Real inputX, Real inputY, Real *outputX, Real *outputY) {
  Real norm0;
  Real denominator0;

  norm0 = apaMathNorm2D(inputX, inputY);
  denominator0 = apaMathDivideZeroProcess(norm0);
  *outputX = inputX / denominator0;
  *outputY = inputY / denominator0;
}

/** @brief Squared Euclidean distance between two 2D points. */
static inline Real apaMathDistanceSquared2D(Real x0, Real y0, Real x1, Real y1) {
  Real deltaX;
  Real deltaY;

  deltaX = x0 - x1;
  deltaY = y0 - y1;
  return (deltaX * deltaX) + (deltaY * deltaY);
}

/** @brief Euclidean distance between two 2D points. */
static inline Real apaMathDistance2D(Real x0, Real y0, Real x1, Real y1) {
  return sqrtf(apaMathDistanceSquared2D(x0, y0, x1, y1));
}

/** @brief 2D 向量 1-范数 — 对应 uisee_float32_norm.m (p=1) */
static inline Real apaMathNorm2DL1(Real x, Real y) {
  Real absX;
  Real absY;

  absX = x;
  if (absX < 0.0f) {
    absX = -absX;
  }
  absY = y;
  if (absY < 0.0f) {
    absY = -absY;
  }
  return absX + absY;
}

/** @brief 2D 向量无穷范数 — 对应 uisee_float32_norm.m (p=inf) */
static inline Real apaMathNorm2DInf(Real x, Real y) {
  Real absX;
  Real absY;
  Real maxAbs;

  absX = x;
  if (absX < 0.0f) {
    absX = -absX;
  }
  absY = y;
  if (absY < 0.0f) {
    absY = -absY;
  }
  maxAbs = absY;
  if (absX > absY) {
    maxAbs = absX;
  }
  return maxAbs;
}

/** @brief 2D 向量按阶范数封装 */
static inline Real apaMathNorm2DWithOrder(Real x, Real y, UInteger8 order) {
  if (order == 1U) {
    return apaMathNorm2DL1(x, y);
  }
  if (order == 2U) {
    return apaMathNorm2D(x, y);
  }
  return apaMathNorm2DInf(x, y);
}

/** @brief 角度转弧度 */
static inline Real apaMathDegToRad(Real degreeValue) {
  return degreeValue * (APA_PI / 180.0f);
}

/** @brief 弧度转角度 */
static inline Real apaMathRadToDeg(Real radianValue) {
  return radianValue * (180.0f / APA_PI);
}

/** @brief 点到线段距离与最近点 */
static inline void apaMathPointToSegmentDistance(const Real point[2],
                                                 const Real pointA[2],
                                                 const Real pointB[2],
                                                 Real *distance,
                                                 Real closestPoint[2]) {
  Real abX;
  Real abY;
  Real apX;
  Real apY;
  Real ab2;
  Real t;

  abX = pointB[0] - pointA[0];
  abY = pointB[1] - pointA[1];
  apX = point[0] - pointA[0];
  apY = point[1] - pointA[1];
  ab2 = apaMathDot2D(abX, abY, abX, abY);

  if (ab2 <= APA_MATH_EPSILON) {
    closestPoint[0] = pointA[0];
    closestPoint[1] = pointA[1];
    *distance = apaMathNorm2D(point[0] - pointA[0], point[1] - pointA[1]);
    return;
  }

  t = apaMathDot2D(apX, apY, abX, abY) / ab2;
  if (t < 0.0f) {
    t = 0.0f;
  }
  if (t > 1.0f) {
    t = 1.0f;
  }

  closestPoint[0] = pointA[0] + (t * abX);
  closestPoint[1] = pointA[1] + (t * abY);
  *distance = apaMathNorm2D(point[0] - closestPoint[0], point[1] - closestPoint[1]);
}

/** @brief 已知离端点距离，求线段上的点 */
static inline void apaMathPointOnLineByDistance(const Real pointA[2],
                                                const Real pointB[2],
                                                Real distanceToA,
                                                Real pointOut[2]) {
  Real vecX;
  Real vecY;
  Real distance;
  Real scale;

  vecX = pointB[0] - pointA[0];
  vecY = pointB[1] - pointA[1];
  distance = apaMathNorm2D(vecX, vecY);
  scale = distanceToA / apaMathDivideZeroProcess(distance);
  pointOut[0] = pointA[0] + (vecX * scale);
  pointOut[1] = pointA[1] + (vecY * scale);
}

/**
 * @brief 判断点是否在凸四边形内 — 对应 ClassMath.isPtInRect
 *
 * 先做 AABB 快速排除，再做叉积同侧判定。
 * rect 存储格式：[Ax, Ay, Bx, By, Cx, Cy, Dx, Dy]（8元素）
 */
static inline Boolean apaMathIsPtInRect(Real px, Real py, const Real rect[8]) {
  Real xMin;
  Real xMax;
  Real yMin;
  Real yMax;

  xMin = rect[0];
  xMax = rect[0];
  yMin = rect[1];
  yMax = rect[1];
  {
    Integer ri;
    for (ri = 1; ri < 4; ri = ri + 1) {
      Real rx;
      Real ry;

      rx = rect[ri * 2];
      ry = rect[ri * 2 + 1];
      if (rx < xMin) {
        xMin = rx;
      }
      if (rx > xMax) {
        xMax = rx;
      }
      if (ry < yMin) {
        yMin = ry;
      }
      if (ry > yMax) {
        yMax = ry;
      }
    }
  }

  if (apaMathGreater(px, xMax)) {
    return false;
  }
  if (apaMathGreater(py, yMax)) {
    return false;
  }
  if (apaMathLess(px, xMin)) {
    return false;
  }
  if (apaMathLess(py, yMin)) {
    return false;
  }

  {
    Real c1;
    Real c2;

    c1 = apaMathCross2D(rect[2] - rect[0], rect[3] - rect[1], px - rect[0], py - rect[1]);
    c2 = apaMathCross2D(rect[4] - rect[6], rect[5] - rect[7], px - rect[6], py - rect[7]);
    if (apaMathGreater(c1 * c2, 0.0f)) {
      return false;
    }
  }
  {
    Real c3;
    Real c4;

    c3 = apaMathCross2D(rect[4] - rect[2], rect[5] - rect[3], px - rect[2], py - rect[3]);
    c4 = apaMathCross2D(rect[6] - rect[0], rect[7] - rect[1], px - rect[0], py - rect[1]);
    if (apaMathGreater(c3 * c4, 0.0f)) {
      return false;
    }
  }

  return true;
}

#endif
