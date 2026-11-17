#ifndef APA_TARGET_SLOT_CONSTRAINT_TYPES_H
#define APA_TARGET_SLOT_CONSTRAINT_TYPES_H

#include "baseTypes.h"

/* Target geometry uses the actual body, without an extra tracking envelope. */
#define APA_TARGET_SLOT_TRACK_MARGIN 0.0f

/**
 * @brief Single-call target-slot policy, separate from physical obstacles.
 * @en_name ApaTargetSlotConstraint
 * @cn_name 目标车位约束
 * @tag STRUCT
 * @field name=Enabled, type=Boolean, unit=1, desc=是否检查非入口边
 * @field name=SlotPts, type=Array<Real, [8]>, unit=m, desc=当前四角坐标
 * @field name=EnterEdge, type=Integer, unit=1, desc=开放入口边1至4
 * @field name=VehFront, type=Real, unit=m, desc=后轴至前包络
 * @field name=VehRear, type=Real, unit=m, desc=后轴至后包络
 * @field name=VehWidth, type=Real, unit=m, desc=车宽
 * @field name=Margin, type=Real, unit=m, desc=显式车位框几何间距运行规划为0
 */
typedef struct {
  Boolean Enabled;
  Real SlotPts[8];
  Integer EnterEdge;
  Real VehFront;
  Real VehRear;
  Real VehWidth;
  Real Margin;
} ApaTargetSlotConstraint_t;

/**
 * @brief Rear-axle constant-curvature motion for slot-frame sampling.
 * @en_name ApaTargetSlotMotion
 * @cn_name 车位约束运动段
 * @tag STRUCT
 * @field name=StartX, type=Real, unit=m, desc=起点后轴X
 * @field name=StartY, type=Real, unit=m, desc=起点后轴Y
 * @field name=StartTheta, type=Real, unit=rad, desc=起点车头航向
 * @field name=Curvature, type=Real, unit=1/m, desc=有符号曲率
 * @field name=Direction, type=Integer, unit=1, desc=前进1倒车负1
 * @field name=Length, type=Real, unit=m, desc=非负弧长
 */
typedef struct {
  Real StartX;
  Real StartY;
  Real StartTheta;
  Real Curvature;
  Integer Direction;
  Real Length;
} ApaTargetSlotMotion_t;

/**
 * @brief First rejected sample and last clear sample for a slot-frame check.
 * @en_name ApaTargetSlotMotionCheck
 * @cn_name 车位约束运动检查结果
 * @tag STRUCT
 */
typedef struct {
  Boolean IsClear;
  Integer Edge;
  Integer CheckCount;
  Real SafeLength;
  Real SafeX;
  Real SafeY;
  Real SafeTheta;
  Real CollisionX;
  Real CollisionY;
  Real CollisionTheta;
} ApaTargetSlotMotionCheck_t;

#endif
