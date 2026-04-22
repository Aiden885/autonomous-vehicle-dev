
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
    
    double temp794019;
    PIDState * temp794179;
    double temp794464;
    double temp795199;
    double temp795241;
    
    
    temp794179 = &pidState;
    /* 纵向车间距计算 */
    temp795199 = computeDistance1D(leadX, vehiclePoint->x);

    /* 仿真示波器 */
    scope_push_send("7a3e41bd-f37b-457f-a8c6-56e5de3ba478", "Component_merge_input_1", "VehicleState", vehiclePoint->v);

    /* 计算ACC目标跟车速度 */
    temp795241 = accComputeTargetSpeed(vehiclePoint->v, leadV, temp795199);

    /* 速度误差计算 */
    temp794019 = computeSpeedError(temp795241, vehiclePoint->v);

    /* 更新PID控制器的输出值 */
    temp794464 = pid_db9453ce_cfb7_4712_9c92_507dced3adf5(kp, ki, kd, dt, 10, 10, temp794019, temp794179);

    /* 更新车辆状态函数 */
    vehicleModelUpdate(temp794464, dt, vehiclePoint);


    
}