/**
 * @file apaVehicleUtils.h
 * @brief APA 共享车辆几何工具函数
 *
 * 对应 MATLAB 基础模块库中的车辆几何模块：
 *   - get_ego_corners.m         → apaVehicleEgoCorners
 *   - get_veh_corner_point.m    → apaVehicleCornerPoint
 *   - get_veh_rect.m            → apaVehicleRect
 *   - get_front_axle_ct_pt.m    → apaVehicleFrontAxlePt
 *
 * 注：MATLAB 版本使用 get_planner_global('veh') 获取车辆参数，
 * C 版本将参数显式传入，避免全局依赖。
 *
 * @date 2026-03-15
 */
#ifndef APA_VEHICLE_UTILS_H
#define APA_VEHICLE_UTILS_H

#include "baseTypes.h"
#include <math.h>

/**
 * @brief 计算自车四角点 — 对应 get_ego_corners.m
 *
 * 给定车辆姿态和尺寸参数，计算四角点全局坐标。
 * 角点顺序：左前, 右前, 右后, 左后。
 *
 * @param posX,posY,posYaw 车辆中心位置和航向
 * @param lf 前悬（中心→前端距离）
 * @param lb 后悬（中心→后端距离）
 * @param halfWidth 半车宽
 * @param cornersX,cornersY 输出4个角点坐标 [FL, FR, RR, RL]
 */
static inline void apaVehicleEgoCorners(Real posX,
                                        Real posY,
                                        Real posYaw,
                                        Real lf,
                                        Real lb,
                                        Real halfWidth,
                                        Real cornersX[4],
                                        Real cornersY[4]) {
  Real cosYaw;
  Real sinYaw;
  Real localX[4];
  Real localY[4];
  Integer idx;

  cosYaw = cosf(posYaw);
  sinYaw = sinf(posYaw);

  localX[0] = lf;
  localY[0] = halfWidth;

  localX[1] = lf;
  localY[1] = -halfWidth;

  localX[2] = -lb;
  localY[2] = -halfWidth;

  localX[3] = -lb;
  localY[3] = halfWidth;

  idx = 0;
  while (idx < 4) {
    cornersX[idx] = posX + localX[idx] * cosYaw - localY[idx] * sinYaw;
    cornersY[idx] = posY + localX[idx] * sinYaw + localY[idx] * cosYaw;
    idx = idx + 1;
  }
}

/**
 * @brief 计算车辆角点（含安全余量） — 对应 get_veh_corner_point.m
 *
 * 在 apaVehicleEgoCorners 基础上增加前后左右的安全余量。
 */
static inline void apaVehicleCornerPoint(Real posX,
                                         Real posY,
                                         Real posYaw,
                                         Real lf,
                                         Real lb,
                                         Real halfWidth,
                                         Real frontMargin,
                                         Real rearMargin,
                                         Real sideMargin,
                                         Real cornersX[4],
                                         Real cornersY[4]) {
  apaVehicleEgoCorners(posX,
                       posY,
                       posYaw,
                       lf + frontMargin,
                       lb + rearMargin,
                       halfWidth + sideMargin,
                       cornersX,
                       cornersY);
}

/**
 * @brief 构建车辆矩形边线段 — 对应 get_veh_rect.m
 *
 * 输出4条边线段。每条边存为 (x1, y1, x2, y2)。
 * rectEdges[i] = { x1, y1, x2, y2 } 共4条边。
 */
static inline void apaVehicleRect(Real posX,
                                  Real posY,
                                  Real posYaw,
                                  Real lf,
                                  Real lb,
                                  Real halfWidth,
                                  Real edgesX1[4],
                                  Real edgesY1[4],
                                  Real edgesX2[4],
                                  Real edgesY2[4]) {
  Real cx[4];
  Real cy[4];
  Integer idx;
  Integer nextIdx;

  apaVehicleEgoCorners(posX, posY, posYaw, lf, lb, halfWidth, cx, cy);

  idx = 0;
  while (idx < 4) {
    nextIdx = idx + 1;
    if (nextIdx >= 4) {
      nextIdx = 0;
    }
    edgesX1[idx] = cx[idx];
    edgesY1[idx] = cy[idx];
    edgesX2[idx] = cx[nextIdx];
    edgesY2[idx] = cy[nextIdx];
    idx = idx + 1;
  }
}

/**
 * @brief 计算前轴中心点全局坐标 — 对应 get_front_axle_ct_pt.m
 *
 * 给定后轴中心姿态和轴距，输出前轴中心全局坐标。
 */
static inline void apaVehicleFrontAxlePt(
    Real rearX, Real rearY, Real rearYaw, Real wheelBase, Real *frontX, Real *frontY) {
  Real cosYaw;
  Real sinYaw;

  cosYaw = cosf(rearYaw);
  sinYaw = sinf(rearYaw);

  *frontX = rearX + wheelBase * cosYaw;
  *frontY = rearY + wheelBase * sinYaw;
}

#endif
