
#include "accpro1.h"

/* 更新PID控制器的输出值 */
double pid_db9453ce_cfb7_4712_9c92_507dced3adf5(
    double kp,  // [in]
    double ki,  // [in]
    double kd,  // [in]
    double dt,  // [in]
    double integral_max,  // [in]
    double derivative_max,  // [in]
    double error,  // [in] 误差值
    PIDState * state  // [inout] PID状态参数结构体
)
{
    
    
    double output;
    double temp134711;
    double temp135025;
    double temp135589;
    double temp135741;
    double temp137079;
    double temp137663;
    double temp138244;
    double temp138594;
    double temp139066;
    double temp139597;
    
    
    temp137079 = error * dt;
    temp138244 = kp * error;
    temp137663 = error - state->lastError;
    temp134711 = state->integral + temp137079;
    temp135589 = temp137663 / dt;
    /* 对称限幅函数 */
    temp135025 = limitSymmetrical(temp134711, integral_max);

    /* 对称限幅函数 */
    temp135741 = limitSymmetrical(temp135589, derivative_max);

    temp138594 = ki * temp135025;

         state->integral = temp135025;
    temp139066 = kd * temp135741;
    temp139597 = temp138244 + temp138594;

         state->lastError = error;
    output = temp139597 + temp139066;

    
    return output;
}