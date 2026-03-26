#ifndef TRAJ_DATA_STRUCT_TRANS_H
#define TRAJ_DATA_STRUCT_TRANS_H

#include "trajPoint.h"
#include "controlData.hpp"


void trajPointTrans(
    const Control::TrajPoint& src,
    TrajPoint* dst
);


/**
 * @brief Control::Traj 转换为 C 结构体 Traj
 * @en_name trajDataStructTrans
 * @cn_name 轨迹数据结构转换
 * @type module
 *
 * @param[in]  src  Control模块轨迹
 * @param[out] dst  C算法模块轨迹
 *
 * @note
 * 会根据 src.size() 动态申请 TrajPoint 内存
 */
void trajDataStructTrans(
    const Control::Traj& src,
    Traj* dst
);

/**
 * @brief 释放 Traj 内存
 * @en_name freeTraj
 * @cn_name 释放轨迹内存
 */
void freeTraj(
    Traj* traj
);

#endif