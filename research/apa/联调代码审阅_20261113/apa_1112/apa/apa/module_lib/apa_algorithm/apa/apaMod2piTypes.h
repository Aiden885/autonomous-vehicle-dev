#ifndef APA_MOD2PI_TYPES_H
#define APA_MOD2PI_TYPES_H

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
 * @brief 角度归一化输入
 * @en_name ApaMod2piInput
 * @cn_name 角度归一化输入
 * @tag STRUCT
 * @field name=Angle, type=Real, unit=rad, desc=输入角度值
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real Angle; // 输入角度值（弧度）
} ApaMod2piInput_t;

/**
 * @brief 角度归一化输出
 * @en_name ApaMod2piOutput
 * @cn_name 角度归一化输出
 * @tag STRUCT
 * @field name=Result, type=Real, unit=rad, desc=归一化后的角度值
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
typedef struct {
  Real Result; // 归一化后的角度值（弧度，范围 [-π, π]）
} ApaMod2piOutput_t;

void apaMod2pi(const ApaMod2piInput_t *input, ApaMod2piOutput_t *output);

#endif
