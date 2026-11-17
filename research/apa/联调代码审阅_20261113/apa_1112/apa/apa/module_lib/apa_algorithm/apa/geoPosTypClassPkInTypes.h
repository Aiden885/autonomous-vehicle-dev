#ifndef GEO_POS_TYP_CLASS_PK_IN_TYPES_H
#define GEO_POS_TYP_CLASS_PK_IN_TYPES_H

#include "baseTypes.h"

/** @brief 圆周率 */
#ifndef APA_PI
#define APA_PI 3.14159265f
#endif
/** @brief 半圆周率 */
#ifndef APA_HALF_PI
#define APA_HALF_PI 1.57079632f
#endif

/**
 * @brief 泊入位置分类输入
 * @en_name GeoPosTypClassPkInInput
 * @cn_name 泊入位置分类输入
 * @tag STRUCT
 * @field name=StPtX, type=Real, unit=m, desc=起点X
 * @field name=StPtY, type=Real, unit=m, desc=起点Y
 * @field name=StPtTheta, type=Real, unit=rad, desc=起点航向角
 * @field name=EdPtX, type=Real, unit=m, desc=目标点X
 * @field name=EdPtY, type=Real, unit=m, desc=目标点Y
 * @field name=EdPtTheta, type=Real, unit=rad, desc=目标点航向角
 * @field name=Dir, type=Integer, unit=1, desc=行驶方向1前进负1后退
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real StPtX;
  Real StPtY;
  Real StPtTheta;
  Real EdPtX;
  Real EdPtY;
  Real EdPtTheta;
  Integer Dir;
} GeoPosTypClassPkInInput_t;

/**
 * @brief 泊入位置分类输出
 * @en_name GeoPosTypClassPkInOutput
 * @cn_name 泊入位置分类输出
 * @tag STRUCT
 * @field name=TarPTyp, type=Integer, unit=1, desc=位置类型编号1到47
 * @field name=PtRelaX, type=Real, unit=m, desc=相对坐标X
 * @field name=PtRelaY, type=Real, unit=m, desc=相对坐标Y
 * @field name=PtRelaTheta, type=Real, unit=rad, desc=相对航向角
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Integer TarPTyp;
  Real PtRelaX;
  Real PtRelaY;
  Real PtRelaTheta;
} GeoPosTypClassPkInOutput_t;

void geoPosTypClassPkIn(const GeoPosTypClassPkInInput_t *input, GeoPosTypClassPkInOutput_t *output);

typedef GeoPosTypClassPkInInput_t GeoPosTypClassInput_t;
typedef GeoPosTypClassPkInOutput_t GeoPosTypClassOutput_t;

void geoPosTypClass(const GeoPosTypClassInput_t *input, GeoPosTypClassOutput_t *output);

#endif
