#ifndef CALC_VEH_CORNER_POINT_TYPES_H
#define CALC_VEH_CORNER_POINT_TYPES_H

#include "baseTypes.h"

/**
 * @brief 车辆角点计算输入
 * @en_name CalcVehCornerPointInput
 * @cn_name 车辆角点计算输入
 * @tag STRUCT
 * @field name=PosX, type=Real, unit=m, desc=车辆后轴中心X
 * @field name=PosY, type=Real, unit=m, desc=车辆后轴中心Y
 * @field name=PosTheta, type=Real, unit=rad, desc=车辆航向角
 * @field name=FrontOverhang, type=Real, unit=m, desc=前悬(前轴到车头)加轴距
 * @field name=RearOverhang, type=Real, unit=m, desc=后悬(后轴到车尾)
 * @field name=VehWidth, type=Real, unit=m, desc=车宽
 * @field name=FrontMargin, type=Real, unit=m, desc=前部安全裕量
 * @field name=RearMargin, type=Real, unit=m, desc=后部安全裕量
 * @field name=SideMargin, type=Real, unit=m, desc=侧向安全裕量
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real PosX;          // 车辆后轴中心X
  Real PosY;          // 车辆后轴中心Y
  Real PosTheta;      // 车辆航向角
  Real FrontOverhang; // 前悬+轴距 (LF)
  Real RearOverhang;  // 后悬 (LB)
  Real VehWidth;      // 车宽
  Real FrontMargin;   // 前部安全裕量
  Real RearMargin;    // 后部安全裕量
  Real SideMargin;    // 侧向安全裕量
} CalcVehCornerPointInput_t;

/**
 * @brief 车辆角点计算输出 (前左/前右/后右/后左 1x8 格式)
 * @en_name CalcVehCornerPointOutput
 * @cn_name 车辆角点计算输出
 * @tag STRUCT
 * @field name=FlX, type=Real, unit=m, desc=前左X
 * @field name=FlY, type=Real, unit=m, desc=前左Y
 * @field name=FrX, type=Real, unit=m, desc=前右X
 * @field name=FrY, type=Real, unit=m, desc=前右Y
 * @field name=RrX, type=Real, unit=m, desc=后右X
 * @field name=RrY, type=Real, unit=m, desc=后右Y
 * @field name=RlX, type=Real, unit=m, desc=后左X
 * @field name=RlY, type=Real, unit=m, desc=后左Y
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real FlX; // 前左X
  Real FlY; // 前左Y
  Real FrX; // 前右X
  Real FrY; // 前右Y
  Real RrX; // 后右X
  Real RrY; // 后右Y
  Real RlX; // 后左X
  Real RlY; // 后左Y
} CalcVehCornerPointOutput_t;

void calcVehCornerPoint(const CalcVehCornerPointInput_t *input, CalcVehCornerPointOutput_t *output);

#endif
