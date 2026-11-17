/**
 * @file apaParallelParkoutPlan.h
 * @brief APA 平行泊出搜索的主规划器入口API
 *
 * @date 2026-04-13
 */
#ifndef APA_PARALLEL_PARKOUT_PLAN_H
#define APA_PARALLEL_PARKOUT_PLAN_H

#include "baseTypes.h"
#include "apaParallelParkoutPlanTypes.h"
#include "vehicleCollisionCheckTypes.h"
#include "apaPathTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 并行泊出核心编排网 (Batch 3 后完成暴漏)
 *
 * @param ctx           泊出所需的全局与目标车位配置
 * @param obsInput      全局碰撞边界集输入
 * @param vehConfig     车辆基础配置 (用于确定包围框)
 * @param outPath       输出复合路径装配体
 * @return Boolean      泊出规划是否安全建链成功
 */
Boolean apaParallelParkoutPlan(const ApaParallelParkoutContext_t *ctx,
                               const VehicleCollisionCheckInput_t *obsInput,
                               const ApaPlannerVehConfig_t *vehConfig,
                               ApaComplexPath_t *outPath);

/**
 * @brief 并行泊出核心编排网，带诊断输出
 *
 * @param ctx           泊出所需的全局与目标车位配置
 * @param obsInput      全局碰撞边界集输入
 * @param vehConfig     车辆基础配置 (用于确定包围框)
 * @param diag          泊出失败原因与危险点诊断输出
 * @param outPath       输出复合路径装配体
 * @return Boolean      泊出规划是否安全建链成功
 */
Boolean apaParallelParkoutPlanWithDiag(const ApaParallelParkoutContext_t *ctx,
                                       const VehicleCollisionCheckInput_t *obsInput,
                                       const ApaPlannerVehConfig_t *vehConfig,
                                       ApaParallelParkoutDiag_t *diag,
                                       ApaComplexPath_t *outPath);

/**
 * @brief MATLAB get_out_of_trouble_point.m parity helper.
 *
 * @param vehPos        车辆位姿 [x, y, theta]
 * @param mebDir        MEB 方向，输出点沿 -mebDir 前后 0.5m
 * @param point         输出脱困目标点 [x, y, theta]
 */
void apaParkoutGetOutOfTroublePoint(const Real vehPos[3], Integer mebDir, Real point[3]);

#ifdef __cplusplus
}
#endif

#endif
