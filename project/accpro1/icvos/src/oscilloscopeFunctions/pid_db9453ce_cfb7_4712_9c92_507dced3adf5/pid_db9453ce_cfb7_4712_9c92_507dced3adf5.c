
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
    double temp785779;
    double temp785918;
    double temp786158;
    double temp786380;
    double temp787366;
    double temp787613;
    double temp788016;
    double temp788286;
    double temp788489;
    double temp788731;
    
    
    temp787366 = error * dt;
    temp788016 = kp * error;
    temp787613 = error - state->lastError;
    temp785779 = state->integral + temp787366;
    temp786158 = temp787613 / dt;
    /* 对称限幅函数 */
    temp785918 = limitSymmetrical(temp785779, integral_max);

    /* 对称限幅函数 */
    temp786380 = limitSymmetrical(temp786158, derivative_max);

    temp788286 = ki * temp785918;

         state->integral = temp785918;
    temp788489 = kd * temp786380;
    temp788731 = temp788016 + temp788286;

         state->lastError = error;
    output = temp788731 + temp788489;

    
    return output;
}