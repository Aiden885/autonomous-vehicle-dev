#include "purePursuit.h"
#include <math.h>

/**
 * @brief 基于 Pure Pursuit 几何模型计算前馈转向角
 * @en_name purePursuit
 * @cn_name 纯追踪模型前馈转向角计算
 * @type module
 * @param[IN] State* state 当前车辆运动车辆状态指针
 * @param[IN] TrajPoint* target 轨迹点数组指针
 * @param[IN] double wheelBase 车辆轴距
 * @retval double pureSteer0 前馈转向角
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 车辆控制
 * @tag_level2 横向控制
 * @formula
 * @version 1.0
 * @date 2026-01-19
 * @author lupeng
 */
double purePursuit(const State* state, const TrajPoint* target, double wheelBase)
{
    /* ---------- 相对位置 ---------- */
    double dx0 = target->x - state->x;
    double dy0 = target->y - state->y;

    /* ---------- 航向角 ---------- */
    double yaw0 = state->yaw;

    /* ---------- 距离平方 ---------- */
    double distSq0 = dx0 * dx0 + dy0 * dy0;

    /* ---------- 实际距离 ---------- */
    double dist0 = sqrt(distSq0);

    /* ---------- 横向误差（车体坐标系） ---------- */
    double sinYaw0 = sin(yaw0);
    double cosYaw0 = cos(yaw0);

    double deltaLateral0 = -dx0 * sinYaw0 + dy0 * cosYaw0;

    /* ---------- Pure Pursuit 转角 ---------- */
    double numerator0   = 2.0 * deltaLateral0 * wheelBase;
    double curvature0   = numerator0 / distSq0;
    double pureSteer0   = atan2(curvature0, 1.0);

    return pureSteer0;
}
