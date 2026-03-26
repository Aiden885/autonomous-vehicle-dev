#ifndef MPC_CONTROLLER_LINEAR_H
#define MPC_CONTROLLER_LINEAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "state.h"
#include "trajPoint.h"
#include "controlCmd.h"

#define MPC_HORIZON_MAX 20
#define STATE_DIM 3
#define CONTROL_DIM 2

/**
 * @brief Linear MPC 控制器主函数
 * @en_name mpc_controller_linear
 * @cn_name Linear MPC控制器
 * @type widget&module
 *
 * @param[IN] const State* state 当前车辆状态
 * @param[IN] const Traj* traj 参考轨迹
 * @param[OUT] ControlCmd* cmd 输出控制
 *
 * @param type=int,enName=horizon,cnName=预测步长
 * @param type=double,enName=dt,cnName=控制周期
 * @param type=double,enName=wheelbase,cnName=车辆轴距
 *
 * @param type=double,enName=w_cte,cnName=横向误差权重
 * @param type=double,enName=w_epsi,cnName=航向误差权重
 * @param type=double,enName=w_v,cnName=速度误差权重
 * @param type=double,enName=w_steer,cnName=转角权重
 * @param type=double,enName=w_acc,cnName=加速度权重
 */
void mpc_controller_linear(
    const State* state,
    const Traj* traj,
    ControlCmd* cmd,
    int horizon,
    double dt,
    double wheelbase,
    double w_cte,
    double w_epsi,
    double w_v,
    double w_steer,
    double w_acc
);

#ifdef __cplusplus
}
#endif

#endif