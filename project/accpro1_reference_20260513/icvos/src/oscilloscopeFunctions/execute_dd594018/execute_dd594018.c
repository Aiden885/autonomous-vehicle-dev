
#include "accpro1.h"

/* for循环体内部逻辑 */
void execute_dd594018(
    VehicleState * vehiclePoint,  // [in] 外部变量 vehiclePoint
    double kp,  // [in] 外部变量 kp
    double ki,  // [in] 外部变量 ki
    double kd,  // [in] 外部变量 kd
    double dt,  // [in] 外部变量 dt
    PIDState pidState,  // [in] 外部变量 pidState
    VehicleState leadPoint  // [in] 函数输入端口
)
{
    
    const double accel = 0;  // 常量
    
    double temp567813;
    PIDState * temp568099;
    double temp568448;
    double temp569683;
    double temp569884;
    
    
    temp568099 = &pidState;
    /* 更新车辆状态函数 */
    vehicleModelUpdate(accel, dt, leadPoint);

    /* 纵向车间距计算 */
    temp569683 = computeDistance1D(leadPoint->x, vehiclePoint->x);

    /* 计算ACC目标跟车速度 */
    temp569884 = accComputeTargetSpeed(vehiclePoint->v, leadPoint->v, temp569683);

    /* 仿真示波器 */
    scope_push_send("7a3e41bd-f37b-457f-a8c6-56e5de3ba478", "Component_merge_input_1", "VehicleState", vehiclePoint->v, "Component_merge_input_2", "double", temp569683);

    /* 速度误差计算 */
    temp567813 = computeSpeedError(temp569884, vehiclePoint->v);

    /* 更新PID控制器的输出值 */
    temp568448 = pid_db9453ce_cfb7_4712_9c92_507dced3adf5(kp, ki, kd, dt, 10, 10, temp567813, temp568099);

    /* 更新车辆状态函数 */
    vehicleModelUpdate(temp568448, dt, vehiclePoint);


    
}