/**
 * @file apaCoordUtils.h
 * @brief APA 共享坐标变换工具函数
 *
 * 对应 MATLAB 基础模块库中的坐标变换模块：
 *   - uisee_angle2dcm.m                → apaCoordAngle2Dcm
 *   - GroundCS_2_VehCS.m               → apaCoordGround2Veh
 *   - VehCS_2_GroundCS.m               → apaCoordVeh2Ground
 *   - translation_transform.m          → apaCoordTranslate
 *   - transform_ego_pt_to_global.m     → apaCoordEgo2Global
 *   - transform_global_pt_to_ego.m     → apaCoordGlobal2Ego
 *   - transform_ipm_pt_to_global.m     → apaCoordIpm2Global
 *   - ClassMath.rectPtsTranslationTransform.m → apaClassMathRectPtsTranslationTransform
 *   - ClassMath.linePtsTranslationTransform.m → apaClassMathLinePtsTranslationTransform
 *
 * @date 2026-03-15
 */
#ifndef APA_COORD_UTILS_H
#define APA_COORD_UTILS_H

#include "baseTypes.h"
#include <math.h>

#ifndef APA_PI
#define APA_PI 3.14159265f
#endif
#ifndef APA_TWO_PI
#define APA_TWO_PI 6.2831853f
#endif
#ifndef APA_PI_REAL64
#define APA_PI_REAL64 3.14159265358979323846
#endif
#ifndef APA_TWO_PI_REAL64
#define APA_TWO_PI_REAL64 6.28318530717958647692
#endif

/**
 * @brief 内部模归一化 [-π, π]
 *
 * 保留 MATLAB `ClassMath.mod2pi(single(x))` 的混合精度语义：`rem` 的
 * 2π 常量及边界比较为 double，每次写回 `v` 时再收窄为 single。
 */
static inline Real apaCoordWrapToPi(Real angle) {
  Real64 remainder64;
  Real remainder;
  Real wrapped;

  remainder64 = fmod((Real64)angle, APA_TWO_PI_REAL64);
  remainder = (Real)remainder64;
  if ((Real64)remainder < -APA_PI_REAL64) {
    wrapped = (Real)((Real64)remainder + APA_TWO_PI_REAL64);
  } else if ((Real64)remainder > APA_PI_REAL64) {
    wrapped = (Real)((Real64)remainder - APA_TWO_PI_REAL64);
  } else {
    wrapped = remainder;
  }
  return wrapped;
}

/** @brief Normalize an angle to the half-open interval [-π, π). */
static inline Real apaCoordWrapToHalfOpenPi(Real angle) {
  Real wrapped;

  wrapped = fmodf(angle + APA_PI, APA_TWO_PI);
  if (wrapped < 0.0f) {
    wrapped = wrapped + APA_TWO_PI;
  }
  wrapped = wrapped - APA_PI;
  return wrapped;
}

/**
 * @brief 欧拉角→方向余弦矩阵 — 对应 uisee_angle2dcm.m
 *
 * ZYX 顺序（yaw, pitch, roll），输出 3x3 按行优先存储。
 * APA 中主要只用 yaw (r1)，pitch/roll 常为 0。
 */
static inline void apaCoordAngle2Dcm(Real r1, Real r2, Real r3, Real dcm[9]) {
  Real c1;
  Real s1;
  Real c2;
  Real s2;
  Real c3;
  Real s3;

  c1 = cosf(r1);
  s1 = sinf(r1);
  c2 = cosf(r2);
  s2 = sinf(r2);
  c3 = cosf(r3);
  s3 = sinf(r3);

  dcm[0] = c2 * c1;
  dcm[1] = c2 * s1;
  dcm[2] = -s2;
  dcm[3] = s3 * s2 * c1 - c3 * s1;
  dcm[4] = s3 * s2 * s1 + c3 * c1;
  dcm[5] = s3 * c2;
  dcm[6] = c3 * s2 * c1 + s3 * s1;
  dcm[7] = c3 * s2 * s1 - s3 * c1;
  dcm[8] = c3 * c2;
}

/**
 * @brief 地面坐标系→车辆坐标系 — 对应 GroundCS_2_VehCS.m
 *
 * 输入 vehPos = (x, y, yaw)，将全局点 (gx, gy, gTheta) 变换到车辆坐标系。
 */
static inline void apaCoordGround2Veh(Real vehX,
                                      Real vehY,
                                      Real vehYaw,
                                      Real gx,
                                      Real gy,
                                      Real gTheta,
                                      Real *vx,
                                      Real *vy,
                                      Real *vTheta) {
  Real dx;
  Real dy;
  Real cosYaw;
  Real sinYaw;

  dx = gx - vehX;
  dy = gy - vehY;
  cosYaw = cosf(vehYaw);
  sinYaw = sinf(vehYaw);

  *vx = dx * cosYaw + dy * sinYaw;
  *vy = -dx * sinYaw + dy * cosYaw;
  *vTheta = apaCoordWrapToPi(gTheta - vehYaw);
}

/**
 * @brief 车辆坐标系→地面坐标系 — 对应 VehCS_2_GroundCS.m
 */
static inline void apaCoordVeh2Ground(Real vehX,
                                      Real vehY,
                                      Real vehYaw,
                                      Real lx,
                                      Real ly,
                                      Real lTheta,
                                      Real *gx,
                                      Real *gy,
                                      Real *gTheta) {
  Real cosYaw;
  Real sinYaw;

  cosYaw = cosf(vehYaw);
  sinYaw = sinf(vehYaw);

  *gx = lx * cosYaw - ly * sinYaw + vehX;
  *gy = lx * sinYaw + ly * cosYaw + vehY;
  *gTheta = apaCoordWrapToPi(lTheta + vehYaw);
}

/**
 * @brief 地面位姿→车辆位姿封装
 *
 * 输入输出均为 [x, y, yaw]。
 */
static inline void
apaCoordGroundToVehPose(const Real vehPose[3], const Real groundPose[3], Real localPose[3]) {
  apaCoordGround2Veh(vehPose[0],
                     vehPose[1],
                     vehPose[2],
                     groundPose[0],
                     groundPose[1],
                     groundPose[2],
                     &localPose[0],
                     &localPose[1],
                     &localPose[2]);
}

/**
 * @brief 车辆位姿→地面位姿封装
 *
 * 输入输出均为 [x, y, yaw]。
 */
static inline void
apaCoordVehToGroundPose(const Real vehPose[3], const Real localPose[3], Real groundPose[3]) {
  apaCoordVeh2Ground(vehPose[0],
                     vehPose[1],
                     vehPose[2],
                     localPose[0],
                     localPose[1],
                     localPose[2],
                     &groundPose[0],
                     &groundPose[1],
                     &groundPose[2]);
}

/**
 * @brief 平移变换 — 对应 translation_transform.m
 */
static inline void
apaCoordTranslate(Real px, Real py, Real vecX, Real vecY, Real *outX, Real *outY) {
  *outX = px + vecX;
  *outY = py + vecY;
}

/**
 * @brief 矩形点列批量平移 - 对应 ClassMath.rectPtsTranslationTransform
 *
 * 输入和输出均为 [x1,y1,x2,y2,x3,y3,x4,y4]。允许输入输出指向同一数组。
 */
static inline void apaClassMathRectPtsTranslationTransform(const Real rectPts[8],
                                                           const Real vecBias[2],
                                                           Real rectPtsOut[8]) {
  Integer pointIdx0;
  Integer baseIdx0;
  Real outX0;
  Real outY0;

  pointIdx0 = 0;
  while (pointIdx0 < 4) {
    baseIdx0 = pointIdx0 * 2;
    apaCoordTranslate(
        rectPts[baseIdx0], rectPts[baseIdx0 + 1], vecBias[0], vecBias[1], &outX0, &outY0);
    rectPtsOut[baseIdx0] = outX0;
    rectPtsOut[baseIdx0 + 1] = outY0;
    pointIdx0 = pointIdx0 + 1;
  }
}

/**
 * @brief 线段点列批量平移 - 对应 ClassMath.linePtsTranslationTransform
 *
 * 输入和输出均为 [x1,y1,x2,y2]。允许输入输出指向同一数组。
 */
static inline void apaClassMathLinePtsTranslationTransform(const Real linePts[4],
                                                           const Real vecBias[2],
                                                           Real linePtsOut[4]) {
  Integer pointIdx0;
  Integer baseIdx0;
  Real outX0;
  Real outY0;

  pointIdx0 = 0;
  while (pointIdx0 < 2) {
    baseIdx0 = pointIdx0 * 2;
    apaCoordTranslate(
        linePts[baseIdx0], linePts[baseIdx0 + 1], vecBias[0], vecBias[1], &outX0, &outY0);
    linePtsOut[baseIdx0] = outX0;
    linePtsOut[baseIdx0 + 1] = outY0;
    pointIdx0 = pointIdx0 + 1;
  }
}

/**
 * @brief 车体坐标→全局坐标（2D 点） — 对应 transform_ego_pt_to_global.m
 *
 * 仅变换 xy 坐标，不含航向。
 */
static inline void apaCoordEgo2Global(
    Real egoPtX, Real egoPtY, Real vehX, Real vehY, Real vehYaw, Real *globalX, Real *globalY) {
  Real cosYaw;
  Real sinYaw;

  cosYaw = cosf(vehYaw);
  sinYaw = sinf(vehYaw);

  *globalX = cosYaw * egoPtX - sinYaw * egoPtY + vehX;
  *globalY = sinYaw * egoPtX + cosYaw * egoPtY + vehY;
}

/**
 * @brief 全局坐标→车体坐标（2D 点） — 对应 transform_global_pt_to_ego.m
 */
static inline void apaCoordGlobal2Ego(
    Real globalX, Real globalY, Real vehX, Real vehY, Real vehYaw, Real *egoX, Real *egoY) {
  Real dx;
  Real dy;
  Real cosYaw;
  Real sinYaw;

  dx = globalX - vehX;
  dy = globalY - vehY;
  cosYaw = cosf(vehYaw);
  sinYaw = sinf(vehYaw);

  *egoX = cosYaw * dx + sinYaw * dy;
  *egoY = -sinYaw * dx + cosYaw * dy;
}

/**
 * @brief 沿当前朝向直线前进后的位姿
 */
static inline void
apaCoordAdvanceStraightPose(const Real currentPose[3], Real travelDistance, Real nextPose[3]) {
  apaCoordEgo2Global(travelDistance,
                     0.0f,
                     currentPose[0],
                     currentPose[1],
                     currentPose[2],
                     &nextPose[0],
                     &nextPose[1]);
  nextPose[2] = currentPose[2];
}

/**
 * @brief 根据车辆中心、朝向和前后长宽构造矩形四角
 */
static inline void apaCoordBuildRect(
    Real posX, Real posY, Real yaw, Real frontLen, Real rearLen, Real halfWidth, Real rect[8]) {
  apaCoordEgo2Global(frontLen, halfWidth, posX, posY, yaw, &rect[0], &rect[1]);
  apaCoordEgo2Global(frontLen, -halfWidth, posX, posY, yaw, &rect[2], &rect[3]);
  apaCoordEgo2Global(-rearLen, -halfWidth, posX, posY, yaw, &rect[4], &rect[5]);
  apaCoordEgo2Global(-rearLen, halfWidth, posX, posY, yaw, &rect[6], &rect[7]);
}

/**
 * @brief 根据后轴中心姿态计算前轴中心点
 */
static inline void
apaCoordFrontAxlePoint(Real posX, Real posY, Real yaw, Real wheelBase, Real *frontX, Real *frontY) {
  apaCoordEgo2Global(wheelBase, 0.0f, posX, posY, yaw, frontX, frontY);
}

/**
 * @brief IPM坐标→全局坐标 — 对应 transform_ipm_pt_to_global.m
 *
 * IPM 坐标原点在后轴中心，需要先偏移到车辆中心再变换。
 */
static inline void apaCoordIpm2Global(Real ipmX,
                                      Real ipmY,
                                      Real vehX,
                                      Real vehY,
                                      Real vehYaw,
                                      Real rearAxleToCenterDist,
                                      Real *globalX,
                                      Real *globalY) {
  Real egoX;

  egoX = ipmX + rearAxleToCenterDist;
  apaCoordEgo2Global(egoX, ipmY, vehX, vehY, vehYaw, globalX, globalY);
}

#endif
