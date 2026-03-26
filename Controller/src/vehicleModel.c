#include "vehicleModel.h"

/**
 * @brief 更新车辆状态（基于简单纵向运动学模型）
 * @en_name vehicleModelUpdate
 * @cn_name 更新车辆状态函数
 * @type widget&module
 * @param type=double,enName=accel,cnName=当前时间步的纵向加速度,range=[-8,10],default=0.0,desc="当前时间步的纵向加速度"
 * @param type=double,enName=dt,cnName=控制周期,range=[0,10],default=0.0,desc="控制周期"
 * @param[IN] VehicleState* state 指向车辆状态结构体的指针
 * @retval void 返回值为空
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 车辆定位
 * @tag_level2 位置匹配
 * 
 * @formula \( x_{\text{new}} = x + v \, dt + \frac{1}{2} a \, dt^2, \quad v_{\text{new}} = v + a \, dt \)
 * @version 1.0
 * @date 2026-03-10
 * @author lupeng
 */
void vehicleModelUpdate(
    double accel,
    double dt,
    VehicleState* state
)
{
    // 读取旧状态
    double x0 = state->x;
    double v0 = state->v;
    double a0 = state->a;

    // ---------- 更新位置 ----------
    double x1 = x0 + v0 * dt + 0.5 * accel * dt * dt;

    // ---------- 更新速度 ----------
    double v1 = v0 + accel * dt;

    // ---------- 更新加速度 ----------
    double a1 = accel;

    // ---------- 写回状态 ----------
    state->x = x1;
    state->v = v1;
    state->a = a1;
}
