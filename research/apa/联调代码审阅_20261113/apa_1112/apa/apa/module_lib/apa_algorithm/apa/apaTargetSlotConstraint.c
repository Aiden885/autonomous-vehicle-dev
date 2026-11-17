#include "apaTargetSlotConstraint.h"
#include "apaCoordUtils.h"
#include "apaGeoUtils.h"
#include "apaSlotTypes.h"
#include <math.h>

/**
 * @brief Bound and validate the motion before sampling or float-to-int conversion.
 * @en_name apaTargetSlotConstraintMotionIsValid
 * @cn_name 车位约束运动输入检查
 * @type module
 * @param[IN] ApaTargetSlotMotion_t Motion 运动段
 * @param[OUT] Boolean Valid 返回有效标志
 * @retval Boolean 输入是否有限且有界
 * @granularity atomic
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
static Boolean apaTargetSlotConstraintMotionIsValid(const ApaTargetSlotMotion_t *motion) {
  Boolean valid0;
  valid0 = isfinite(motion->StartX) && isfinite(motion->StartY) && isfinite(motion->StartTheta) &&
           isfinite(motion->Curvature) && isfinite(motion->Length);
  if (valid0) {
    valid0 = (motion->Length >= 0.0f) && (motion->Length <= 1000.0f);
  }
  if (valid0) {
    valid0 = (motion->Direction == 1) || (motion->Direction == -1);
  }
  return valid0;
}

/**
 * @brief Check one slot edge against a vehicle rectangle.
 * @en_name apaTargetSlotConstraintEdgeTouchesRectangle
 * @cn_name 车位边与车身相交检查
 * @type module
 * @param[IN] Real[8] SlotPts 车位角点
 * @param[IN] Integer Edge 车位边编号
 * @param[IN] Real[4] RectX 车身角点X
 * @param[IN] Real[4] RectY 车身角点Y
 * @param[OUT] Boolean Touches 返回相交标志
 * @retval Boolean 是否相交
 * @granularity atomic
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
static Boolean apaTargetSlotConstraintEdgeTouchesOctagon(const Real slotPts[8],
                                                         Integer edge,
                                                         const Real polyX[8],
                                                         const Real polyY[8]) {
  Real edgeX1;
  Real edgeY1;
  Real edgeX2;
  Real edgeY2;
  apaSlotGetEdge(slotPts, edge, &edgeX1, &edgeY1, &edgeX2, &edgeY2);
  return apaGeoSegmentTouchesOctagon(edgeX1, edgeY1, edgeX2, edgeY2, polyX, polyY);
}

/**
 * @brief Apply the shared non-entry-slot-edge policy to one rear-axle pose.
 * @en_name apaTargetSlotConstraintPoseIsClear
 * @cn_name 目标车位约束位姿检查
 * @type module
 * @param[IN] ApaTargetSlotConstraint_t Constraint 本次调用的车位及车身规则
 * @param[IN] Real PoseX 后轴X
 * @param[IN] Real PoseY 后轴Y
 * @param[IN] Real PoseTheta 车头航向
 * @param[OUT] Integer CollisionEdge 相交边编号无相交为0
 * @retval Boolean 是否满足车位约束
 * @granularity atomic
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
Boolean apaTargetSlotConstraintPoseIsClear(const ApaTargetSlotConstraint_t *constraint,
                                           Real poseX,
                                           Real poseY,
                                           Real poseTheta,
                                           Integer *collisionEdge) {
  Real polyX[8];
  Real polyY[8];
  Integer edge;
  Boolean clear0;
  Boolean enabled0;
  Boolean loop0;
  *collisionEdge = 0;
  clear0 = true;
  enabled0 = (constraint != (const ApaTargetSlotConstraint_t *)0);
  if (enabled0) {
    enabled0 = constraint->Enabled;
  }
  if (enabled0) {
    apaGeoBuildVehicleOctagon(poseX,
                              poseY,
                              poseTheta,
                              constraint->VehFront,
                              constraint->VehRear,
                              constraint->VehWidth,
                              constraint->Margin,
                              polyX,
                              polyY);
    edge = 1;
    loop0 = true;
    while (loop0) {
      Boolean touches0;
      touches0 = false;
      if (edge != constraint->EnterEdge) {
        touches0 =
            apaTargetSlotConstraintEdgeTouchesOctagon(constraint->SlotPts, edge, polyX, polyY);
      }
      if (touches0) {
        *collisionEdge = edge;
        clear0 = false;
      }
      edge = edge + 1;
      loop0 = (edge <= 4);
      if (!clear0) {
        loop0 = false;
      }
    }
  }
  return clear0;
}

/**
 * @brief Evaluate the exact rear-axle pose at a motion distance.
 * @en_name apaTargetSlotConstraintMotionPose
 * @cn_name 车位约束运动位姿
 * @type module
 * @param[IN] ApaTargetSlotMotion_t Motion 运动段
 * @param[IN] Real Travel 非负采样距离
 * @param[OUT] Real X 后轴X
 * @param[OUT] Real Y 后轴Y
 * @param[OUT] Real Theta 车头航向
 * @granularity atomic
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
static void apaTargetSlotConstraintMotionPose(
    const ApaTargetSlotMotion_t *motion, Real travel, Real *x, Real *y, Real *theta) {
  Real signedTravel0;
  Real theta0;
  signedTravel0 = ((Real)motion->Direction) * travel;
  theta0 = motion->StartTheta + (motion->Curvature * signedTravel0);
  if (fabsf(motion->Curvature) > 1.0e-6f) {
    *x = motion->StartX + ((sinf(theta0) - sinf(motion->StartTheta)) / motion->Curvature);
    *y = motion->StartY - ((cosf(theta0) - cosf(motion->StartTheta)) / motion->Curvature);
  } else {
    *x = motion->StartX + (signedTravel0 * cosf(motion->StartTheta));
    *y = motion->StartY + (signedTravel0 * sinf(motion->StartTheta));
  }
  *theta = apaCoordWrapToPi(theta0);
}

/**
 * @brief Sample a motion independently of physical-map capacity.
 * @en_name apaTargetSlotConstraintCheckMotion
 * @cn_name 目标车位约束运动检查
 * @type module
 * @param[IN] ApaTargetSlotConstraint_t Constraint 车位规则
 * @param[IN] ApaTargetSlotMotion_t Motion 后轴运动段
 * @param[OUT] ApaTargetSlotMotionCheck_t Output 首次拒绝与最后安全位姿
 * @granularity composite
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
void apaTargetSlotConstraintCheckMotion(const ApaTargetSlotConstraint_t *constraint,
                                        const ApaTargetSlotMotion_t *motion,
                                        ApaTargetSlotMotionCheck_t *output) {
  Integer sampleCount;
  Integer sample;
  Boolean loop0;
  Boolean lengthValid0;
  *output = (ApaTargetSlotMotionCheck_t){0};
  output->IsClear = true;
  output->SafeX = motion->StartX;
  output->SafeY = motion->StartY;
  output->SafeTheta = motion->StartTheta;
  lengthValid0 = apaTargetSlotConstraintMotionIsValid(motion);
  output->IsClear = lengthValid0;
  sampleCount = 1;
  if (lengthValid0) {
    sampleCount = (Integer)ceilf(motion->Length / 0.1f);
    if (sampleCount < 1) {
      sampleCount = 1;
    }
  }
  sample = 0;
  loop0 = lengthValid0;
  while (loop0) {
    Real travel0;
    Real x;
    Real y;
    Real theta;
    travel0 = motion->Length * (((Real)sample) / ((Real)sampleCount));
    apaTargetSlotConstraintMotionPose(motion, travel0, &x, &y, &theta);
    output->IsClear = apaTargetSlotConstraintPoseIsClear(constraint, x, y, theta, &output->Edge);
    output->CheckCount = output->CheckCount + 1;
    if (output->IsClear) {
      output->SafeLength = travel0;
      output->SafeX = x;
      output->SafeY = y;
      output->SafeTheta = theta;
    } else {
      output->CollisionX = x;
      output->CollisionY = y;
      output->CollisionTheta = theta;
    }
    sample = sample + 1;
    loop0 = (sample <= sampleCount);
    if (!output->IsClear) {
      loop0 = false;
    }
  }
}
