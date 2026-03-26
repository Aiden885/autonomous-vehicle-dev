#include "trajDataStructTrans.h"
#include <cstdlib>



void trajPointTrans(
    const Control::TrajPoint& src,
    TrajPoint* dst
)
{
    if (dst == nullptr)
    {
        return;
    }

    dst->x   = src.x;
    dst->y   = src.y;
    dst->yaw = src.yaw;
    dst->v   = src.v;
    dst->k   = src.k;
    dst->s   = src.s;
    dst->acc = src.acc;
}



/**
 * @brief Control::Traj 转 C 结构体 Traj
 */
void trajDataStructTrans(
    const Control::Traj& src,
    Traj* dst
)
{
    if (dst == nullptr)
    {
        return;
    }

    size_t size = src.size();

    dst->size = size;

    if (size == 0)
    {
        dst->points = nullptr;
        return;
    }

    /* 申请轨迹点内存 */
    dst->points = (TrajPoint*)malloc(size * sizeof(TrajPoint));

    if (dst->points == nullptr)
    {
        dst->size = 0;
        return;
    }

    /* 数据拷贝 */
    for (size_t i = 0; i < size; i++)
    {
        dst->points[i].x   = src[i].x;
        dst->points[i].y   = src[i].y;
        dst->points[i].yaw = src[i].yaw;
        dst->points[i].v   = src[i].v;
        dst->points[i].k   = src[i].k;
        dst->points[i].s   = src[i].s;
        dst->points[i].acc = src[i].acc;
    }
}

/**
 * @brief 释放 Traj 内存
 */
void freeTraj(
    Traj* traj
)
{
    if (traj == nullptr)
    {
        return;
    }

    if (traj->points != nullptr)
    {
        free(traj->points);
        traj->points = nullptr;
    }

    traj->size = 0;
}