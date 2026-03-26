#include "getPreviewIndex.h"
#include "trajPoint.h"
#include <math.h>

/**
 * @brief 基于累计距离获取轨迹预瞄点索引
 * @en_name getPreviewIndex
 * @cn_name 获取轨迹预瞄点索引
 * @type module
 * @param[IN] traj 轨迹结构体指针
 * @param[IN] previewDist 预瞄距离（m）
 * @retval int 返回满足累计距离>=previewDist的轨迹点索引，若距离不足返回最后一个点索引
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 车辆定位
 * @tag_level2 位置匹配
 * @version 1.0
 * @date 2026-03-11
 * @author lupeng
 */
int getPreviewIndex(const Traj* traj, double previewDist)
{
    // if (!traj || !traj->points || traj->size == 0)
    //     return -1;

    // if (traj->size == 1)
    //     return 0;

    double accumulatedDist0 = 0.0;
    double accumulatedDist1 = accumulatedDist0;

    for (size_t i = 1; i < traj->size; ++i)
    {
        double dx0 = traj->points[i].x - traj->points[i - 1].x;
        double dy0 = traj->points[i].y - traj->points[i - 1].y;

        double segmentDist0 = sqrt(dx0 * dx0 + dy0 * dy0);

        double accumulatedNext = accumulatedDist1 + segmentDist0;

        if (accumulatedNext >= previewDist)
            return (int)i;

        accumulatedDist1 = accumulatedNext;
    }

    int finalIndex0 = (int)(traj->size - 1);
    return finalIndex0;
}