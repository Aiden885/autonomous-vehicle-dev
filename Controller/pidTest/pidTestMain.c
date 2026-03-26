#include "vehicleModel.h"
#include "pidController.h"

#include <stdio.h>

/**
 * @brief 基于简单纵向运动学模型完成PID模块测试
 * @en_name pidTestMain
 * @cn_name 测试主函数
 * @type module
 * @retval void 返回值为空
 * @granularity composite
 * @tag_level0 功能模块库
 * @tag_level1 车辆控制
 * @tag_level2 PID控制测试
 * @formula /
 * @version 1.0
 * @date 2026-03-10
 * @author lupeng
 */
void main()
{
    double dt = 0.05;

    PIDState pidState;
    pidState.integral = 0.0;
    pidState.lastError = 0.0;

    double kp = 1.2;
    double ki = 0.8;
    double kd = 0.05;

    double v_ref = 18;

    VehicleState vehicle;
    vehicle.x = 0.0;
    vehicle.v = 0.0;
    vehicle.a = 0.0;

    VehicleState *vehiclePoint = &vehicle;


    for(int i=0;i<500;i++)
    {
        double error = v_ref - vehiclePoint->v;

        double accel = pid(
            kp,ki,kd,
            dt,
            10,
            10,
            error,
            &pidState
        );

        vehicleModelUpdate(accel,dt,vehiclePoint);
        printf("t=%.2f v=%.2f x=%.2f a=%.2f\n",
            i * dt,
            vehicle.v,
            vehicle.x,
            vehicle.a);
    }
}