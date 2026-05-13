
#include "accpro1.h"

/* for循环体内部逻辑 */
void execute_dd594018(
    VehicleState * vehiclePoint,  // [in] 外部变量 vehiclePoint
    double kp,  // [in] 外部变量 kp
    double ki,  // [in] 外部变量 ki
    double kd,  // [in] 外部变量 kd
    double dt,  // [in] 外部变量 dt
    PIDState pidState,  // [in] 外部变量 pidState
    VehicleState * leadPoint  // [in] 函数输入端口（指针，指向外部leadVehicle）
)
{
    
    const double accel = 0;  // 常量
    
    double temp865543;
    PIDState * temp865653;
    double temp865886;
    double temp866621;
    double temp866723;
    
    
    temp865653 = &pidState;
    /* 更新车辆状态函数 */
    vehicleModelUpdate(accel, dt, leadPoint);

    /* 纵向车间距计算 */
    temp866621 = computeDistance1D(leadPoint->x, vehiclePoint->x);

    /* 计算ACC目标跟车速度 */
    temp866723 = accComputeTargetSpeed(vehiclePoint->v, leadPoint->v, temp866621);

    /* 速度误差计算 */
    temp865543 = computeSpeedError(temp866723, vehiclePoint->v);

    /* 更新PID控制器的输出值 */
    temp865886 = pid_db9453ce_cfb7_4712_9c92_507dced3adf5(kp, ki, kd, dt, 10, 10, temp865543, temp865653);

    /* 更新车辆状态函数 */
    vehicleModelUpdate(temp865886, dt, vehiclePoint);


    
}