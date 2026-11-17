#ifndef GEO_AXIS_TRANS_TYPES_H
#define GEO_AXIS_TRANS_TYPES_H

#include "baseTypes.h"

/** @brief 圆周率 */
#ifndef APA_PI
#define APA_PI 3.14159265f
#endif
/** @brief 两倍圆周率 */
#ifndef APA_TWO_PI
#define APA_TWO_PI 6.2831853f
#endif

/**
 * @brief 坐标系变换输入
 * @en_name GeoAxisTransInput
 * @cn_name 坐标系变换输入
 * @tag STRUCT
 * @field name=PtX, type=Real, unit=m, desc=待变换点的X坐标
 * @field name=PtY, type=Real, unit=m, desc=待变换点的Y坐标
 * @field name=PtTheta, type=Real, unit=rad, desc=待变换点的航向角
 * @field name=Ori1X, type=Real, unit=m, desc=源坐标系原点X
 * @field name=Ori1Y, type=Real, unit=m, desc=源坐标系原点Y
 * @field name=Ori1Theta, type=Real, unit=rad, desc=源坐标系原点航向角
 * @field name=Ori2X, type=Real, unit=m, desc=目标坐标系原点X
 * @field name=Ori2Y, type=Real, unit=m, desc=目标坐标系原点Y
 * @field name=Ori2Theta, type=Real, unit=rad, desc=目标坐标系原点航向角
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real PtX;       // 待变换点的X坐标（m）
  Real PtY;       // 待变换点的Y坐标（m）
  Real PtTheta;   // 待变换点的航向角（rad）
  Real Ori1X;     // 源坐标系原点X（m）
  Real Ori1Y;     // 源坐标系原点Y（m）
  Real Ori1Theta; // 源坐标系原点航向角（rad）
  Real Ori2X;     // 目标坐标系原点X（m）
  Real Ori2Y;     // 目标坐标系原点Y（m）
  Real Ori2Theta; // 目标坐标系原点航向角（rad）
} GeoAxisTransInput_t;

/**
 * @brief 坐标系变换输出
 * @en_name GeoAxisTransOutput
 * @cn_name 坐标系变换输出
 * @tag STRUCT
 * @field name=ResultX, type=Real, unit=m, desc=变换后的X坐标
 * @field name=ResultY, type=Real, unit=m, desc=变换后的Y坐标
 * @field name=ResultTheta, type=Real, unit=rad, desc=变换后的航向角
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real ResultX;     // 变换后的X坐标（m）
  Real ResultY;     // 变换后的Y坐标（m）
  Real ResultTheta; // 变换后的航向角（rad）
} GeoAxisTransOutput_t;

void geoAxisTrans(const GeoAxisTransInput_t *input, GeoAxisTransOutput_t *output);

/** @brief Scalar adapter that transforms a ground-frame pose to a target frame. */
static inline void geoAxisTransToTarget(Real sourceX,
                                        Real sourceY,
                                        Real sourceTheta,
                                        Real targetX,
                                        Real targetY,
                                        Real targetTheta,
                                        Real *relativeX,
                                        Real *relativeY,
                                        Real *relativeTheta) {
  GeoAxisTransInput_t input;
  GeoAxisTransOutput_t output;

  input.PtX = sourceX;
  input.PtY = sourceY;
  input.PtTheta = sourceTheta;
  input.Ori1X = 0.0f;
  input.Ori1Y = 0.0f;
  input.Ori1Theta = 0.0f;
  input.Ori2X = targetX;
  input.Ori2Y = targetY;
  input.Ori2Theta = targetTheta;
  output = (GeoAxisTransOutput_t){0};
  geoAxisTrans(&input, &output);
  *relativeX = output.ResultX;
  *relativeY = output.ResultY;
  *relativeTheta = output.ResultTheta;
}

/** @brief Scalar adapter that transforms a target-frame pose back to ground. */
static inline void geoAxisTransFromTarget(Real relativeX,
                                          Real relativeY,
                                          Real relativeTheta,
                                          Real targetX,
                                          Real targetY,
                                          Real targetTheta,
                                          Real *groundX,
                                          Real *groundY,
                                          Real *groundTheta) {
  GeoAxisTransInput_t input;
  GeoAxisTransOutput_t output;

  input.PtX = relativeX;
  input.PtY = relativeY;
  input.PtTheta = relativeTheta;
  input.Ori1X = targetX;
  input.Ori1Y = targetY;
  input.Ori1Theta = targetTheta;
  input.Ori2X = 0.0f;
  input.Ori2Y = 0.0f;
  input.Ori2Theta = 0.0f;
  output = (GeoAxisTransOutput_t){0};
  geoAxisTrans(&input, &output);
  *groundX = output.ResultX;
  *groundY = output.ResultY;
  *groundTheta = output.ResultTheta;
}

#endif
