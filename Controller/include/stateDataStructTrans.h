#ifndef STATE_DATA_STRUCT_TRANS_H
#define STATE_DATA_STRUCT_TRANS_H

#include "state.h"
#include "controlData.hpp"

/**
 * @brief Control::State 转换为 C 结构体 State
 * @en_name stateDataStructTrans
 * @cn_name 状态数据结构转换
 * @type module
 *
 * @param[in]  src  Control模块中的车辆状态
 * @param[out] dst  C算法模块使用的车辆状态
 *
 * @note
 * 用于 C++ 控制模块 与 C 算法模块之间的数据结构转换
 *
 * @version 1.0
 * @date 2026-03-11
 */
void stateDataStructTrans(
    const Control::State& src,
    State* dst
);

#endif