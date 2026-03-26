#include "stateDataStructTrans.h"


// “结构体拷贝转换层” 在 C/C++ 混合工程里是一个非常常见且合理的方案，尤其适用于：

// C++ 控制模块：Control::State

// C 算法模块：State

// 需要 解耦 C 与 C++ 数据结构

// 这种方式在很多自动驾驶代码中也常见（例如 Apollo 的 proto → struct 转换层）。


/**
 * 
 * @brief Control::State 转换为 C 结构体 State
 */
void stateDataStructTrans(
    const Control::State& src,
    State* dst
)
{
    if (dst == nullptr)
    {
        return;
    }

    dst->x = src.x;
    dst->y = src.y;
    dst->yaw = src.yaw;
    dst->v = src.v;
    dst->accX = src.accX;
    dst->rtkMode = src.rtkMode;
}