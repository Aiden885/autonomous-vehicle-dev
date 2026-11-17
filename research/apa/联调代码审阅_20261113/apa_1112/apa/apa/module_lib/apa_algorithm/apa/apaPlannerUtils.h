/**
 * @file apaPlannerUtils.h
 * @brief APA 搜索规划辅助工具函数
 *
 * 对应 MATLAB PlanningSubFcn 中的独立小工具函数：
 *   - cal_seg_num.m       → apaPlannerCalcSegNum
 *   - get_dir_from_veh_pos.m → apaPlannerGetDirFromVehPos
 *   - veh_is_stopped.m    → apaPlannerVehIsStopped
 *   - veh_is_moving.m     → apaPlannerVehIsMoving
 *
 * @date 2026-04-13
 */
#ifndef APA_PLANNER_UTILS_H
#define APA_PLANNER_UTILS_H

#include "baseTypes.h"
#include "apaEnums.h"
#include "apaMathUtils.h"
#include "apaCoordUtils.h"
#include "vehicleCollisionCheckTypes.h"

/**
 * @brief Map collision categories to the MATLAB planning failure codes.
 *
 * Boundary, stopper, and virtual-boundary collisions map to 4, 5, and 6.
 * Unmatched categories return the supplied default code.
 */
static inline Integer apaPlannerCollisionFailReason(Integer collisionType,
                                                    Integer defaultFailReason) {
  Integer failReason0;

  failReason0 = defaultFailReason;
  if (collisionType == VCC_COLLISION_BOUNDARY) {
    failReason0 = 4;
  } else if (collisionType == VCC_COLLISION_STOPPER) {
    failReason0 = 5;
  } else if (collisionType == VCC_COLLISION_VIRTUAL) {
    failReason0 = 6;
  }
  return failReason0;
}

/**
 * @brief 计算路径段数 — 对应 cal_seg_num.m
 *
 * 通过检测里程表符号翻转（正→负或负→正）来统计路径中的换挡段数。
 *
 * @param odoList 里程表数组
 * @param ptsN 有效点数
 * @return 段数（首个里程为0时返回30表示异常）
 */
static inline Integer apaPlannerCalcSegNum(const Real *odoList, Integer ptsN) {
  Integer segNum;
  Integer i;

  if (apaMathEqual(odoList[0], 0.0f)) {
    return 30;
  }

  segNum = 1;
  for (i = 0; i < ptsN - 2; i = i + 1) {
    Real product;

    product = odoList[i] * odoList[i + 1];
    if (apaMathLess(product, 0.0f)) {
      segNum = segNum + 1;
    }
  }

  return segNum;
}

/**
 * @brief 判断目标位置在车辆前方还是后方 — 对应 get_dir_from_veh_pos.m
 *
 * @param posX,posY,posYaw 车辆位姿
 * @param obsX,obsY 目标位置
 * @return 0=无效, 1=前方, 2=后方
 */
static inline Integer
apaPlannerGetDirFromVehPos(Real posX, Real posY, Real posYaw, Real obsX, Real obsY) {
  Real vx;
  Real vy;
  Real vTheta;

  apaCoordGround2Veh(posX, posY, posYaw, obsX, obsY, 0.0f, &vx, &vy, &vTheta);

  if (apaMathLess(vx, 0.0f)) {
    return 2;
  }
  if (apaMathGreater(vx, 0.0f)) {
    return 1;
  }
  return 0;
}

/**
 * @brief 判断车辆是否静止 — 对应 veh_is_stopped.m
 *
 * @param vel 车速 [kph]
 * @return true 表示静止（速度 < 0.2 kph）
 */
static inline Boolean apaPlannerVehIsStopped(Real vel) {
  return apaMathLess(vel, 0.2f);
}

/**
 * @brief 判断车辆是否运动 — 对应 veh_is_moving.m
 *
 * @param vel 车速 [kph]
 * @return true 表示运动（速度 > 0.31 kph）
 */
static inline Boolean apaPlannerVehIsMoving(Real vel) {
  return apaMathGreater(vel, 0.31f);
}

/** @brief APA 状态映射到泊车/搜索主状态 — 对应 getParkingSysMainState.m */
static inline Integer apaPlannerGetParkingSysMainState(Integer apaState) {
  if (apaState == 3 || apaState == 4 || apaState == 5 || apaState == 10 || apaState == 11 ||
      apaState == 17 || apaState == 18 || apaState == 20) {
    return APA_SYS_STATE_PARKING;
  }
  if (apaState == 1 || apaState == 15 || apaState == 16) {
    return APA_SYS_STATE_SEARCHING;
  }
  return APA_SYS_STATE_INVALID;
}

/** @brief 获取 uint8 语义的泊车状态 — 对应 get_parking_state.m */
static inline Integer apaPlannerGetParkingState(Integer apaState) {
  return (Integer)apaPlannerGetParkingSysMainState(apaState);
}

#endif
