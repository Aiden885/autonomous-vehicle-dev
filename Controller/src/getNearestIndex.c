#include "getNearestIndex.h"
#include <stdio.h>
#include <math.h>
#include <float.h>

/**
 * @brief 获取车辆当前位置最近的轨迹点索引
 * @en_name getNearestIndex
 * @cn_name 获取最近轨迹点索引
 * @type module
 * @param[IN] State* state 当前车辆状态指针
 * @param[IN] Traj* traj 轨迹点数组指针
 * @retval int nearestIndex 返回最近轨迹点索引,若输入无效或数组为空返回-1
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 车辆定位
 * @tag_level2 位置匹配
 * @formula 
 * @version 1.0
 * @date 2026-01-19
 * @author lupeng
 */
int getNearestIndex(const State* state, const Traj* traj)
{
    // if (state == NULL || traj == NULL || traj->points == NULL || traj->size == 0)
    // {
    //     return -1;
    // }

    int nearestIndex = -1;
    double minDistanceSquared = DBL_MAX;

    for (size_t i = 0; i < traj->size; ++i)
    {
        const TrajPoint* p = &traj->points[i];

        double dx = p->x - state->x;
        double dy = p->y - state->y;

        double distanceSquared = dx * dx + dy * dy;

        if (distanceSquared < minDistanceSquared)
        {
            minDistanceSquared = distanceSquared;
            nearestIndex = (int)i;
        }
    }

    return nearestIndex;
}
