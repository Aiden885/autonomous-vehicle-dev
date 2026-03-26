#include <iostream>
#include "plotSpeed.h"
#include <opencv2/opencv.hpp>
#include "vehicleModel.h"
#include "mathC.h"

extern "C"
{
#include "pidController.h"

}


/**
 * @brief 基于简单纵向运动学模型完成PID模块测试
 * @en_name main
 * @cn_name 测试主函数
 * @type widget&module
 * @retval int 返回值为0
 * @granularity composite
 * @tag_level1 controller
 * @tag_level2 Method&Model
 * 
 * @formula /
 * @version 1.0
 * @date 2026-03-10
 * @author lupeng
 */
int main()
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

    speedPlotInit();

    for(int i=0;i<500;i++)
    {
        double time = i * dt;

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
        speedPlotUpdate(
            time,
            vehicle.v,
            v_ref
        );

        // 加短延迟模拟实时刷新
        cv::waitKey(5);
    }

    // 循环结束后阻塞，让窗口不关闭
    std::cout << "Simulation finished. Press any key to close window..." << std::endl;
    cv::waitKey(0);

    speedPlotClose();

    return 0;
}