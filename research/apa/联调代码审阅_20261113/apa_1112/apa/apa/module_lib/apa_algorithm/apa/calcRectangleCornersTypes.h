#ifndef CALC_RECTANGLE_CORNERS_TYPES_H
#define CALC_RECTANGLE_CORNERS_TYPES_H

#include "baseTypes.h"

/**
 * @brief 通用矩形角点计算输入
 * @en_name CalcRectangleCornersInput
 * @cn_name 通用矩形角点计算输入
 * @tag STRUCT
 * @field name=CenterX, type=Real, unit=m, desc=矩形中心X
 * @field name=CenterY, type=Real, unit=m, desc=矩形中心Y
 * @field name=Theta, type=Real, unit=rad, desc=矩形朝向
 * @field name=Length, type=Real, unit=m, desc=矩形长度(沿朝向)
 * @field name=Width, type=Real, unit=m, desc=矩形宽度(垂直朝向)
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real CenterX; // 矩形中心X
  Real CenterY; // 矩形中心Y
  Real Theta;   // 矩形朝向
  Real Length;  // 矩形长度
  Real Width;   // 矩形宽度
} CalcRectangleCornersInput_t;

/**
 * @brief 通用矩形角点计算输出
 * @en_name CalcRectangleCornersOutput
 * @cn_name 通用矩形角点计算输出
 * @tag STRUCT
 * @field name=LuX, type=Real, unit=m, desc=左上角X
 * @field name=LuY, type=Real, unit=m, desc=左上角Y
 * @field name=RuX, type=Real, unit=m, desc=右上角X
 * @field name=RuY, type=Real, unit=m, desc=右上角Y
 * @field name=RlX, type=Real, unit=m, desc=右下角X
 * @field name=RlY, type=Real, unit=m, desc=右下角Y
 * @field name=LlX, type=Real, unit=m, desc=左下角X
 * @field name=LlY, type=Real, unit=m, desc=左下角Y
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real LuX; // 左上角X
  Real LuY; // 左上角Y
  Real RuX; // 右上角X
  Real RuY; // 右上角Y
  Real RlX; // 右下角X
  Real RlY; // 右下角Y
  Real LlX; // 左下角X
  Real LlY; // 左下角Y
} CalcRectangleCornersOutput_t;

void calcRectangleCorners(const CalcRectangleCornersInput_t *input,
                          CalcRectangleCornersOutput_t *output);

#endif
