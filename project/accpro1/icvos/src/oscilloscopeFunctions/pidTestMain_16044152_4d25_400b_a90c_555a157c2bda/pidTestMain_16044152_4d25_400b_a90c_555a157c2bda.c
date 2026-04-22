
#include "accpro1.h"

/* 基于简单纵向运动学模型完成PID模块测试 */
void pidTestMain_16044152_4d25_400b_a90c_555a157c2bda(
)
{
    
    
    PIDState pidState;
    VehicleState vehicle;
    int start = 0;
    int step = 1;
    int target = 200;
    VehicleState * vehiclePoint;
    double kp = 1.2;
    double ki = 0.1;
    double kd = 0.05;
    double dt = 0.05;
    
    
    
         pidState.integral = 0.0;

         vehicle.x = 0.0;  // 车辆在全局坐标系下的纵向 x 位置

         pidState.lastError = 0.0;

         vehicle.v = 0;  // 车辆对应的车辆速度

         vehicle.a = 0.0;  // 车辆对应的纵向加速度
    vehiclePoint = &vehicle;
    for(start; start < target; start += step) {
        /* for循环体 */
        execute_dd594018(vehiclePoint, kp, ki, kd, dt, pidState);

    }

    
}