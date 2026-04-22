
#include "accpro1.h"

/* for循环体内部逻辑 */
void execute_dd594018(
    VehicleState * vehiclePoint,  // [in] 外部变量 vehiclePoint
    double kp,  // [in] 外部变量 kp
    double ki,  // [in] 外部变量 ki
    double kd,  // [in] 外部变量 kd
    double dt,  // [in] 外部变量 dt
    PIDState pidState  // [in] 外部变量 pidState
)
{
    
    const double leadX = 30;  // 常量
    const double leadV = 3;  // 常量
    
    double temp146678;
    PIDState * temp146818;
    double temp147211;
    double temp148259;
    double temp148428;
    
    
    temp146818 = &pidState;
    /* 纵向车间距计算 */
    temp148259 = computeDistance1D(leadX, vehiclePoint->x);

    /* 计算ACC目标跟车速度 */
    temp148428 = accComputeTargetSpeed(vehiclePoint->v, leadV, temp148259);

    /* 速度误差计算 */
    temp146678 = computeSpeedError(temp148428, vehiclePoint->v);

    /* 更新PID控制器的输出值 */
    temp147211 = pid_db9453ce_cfb7_4712_9c92_507dced3adf5(kp, ki, kd, dt, 10, 10, temp146678, temp146818);

    /* 更新车辆状态函数 */
    vehicleModelUpdate(temp147211, dt, vehiclePoint);


    
}