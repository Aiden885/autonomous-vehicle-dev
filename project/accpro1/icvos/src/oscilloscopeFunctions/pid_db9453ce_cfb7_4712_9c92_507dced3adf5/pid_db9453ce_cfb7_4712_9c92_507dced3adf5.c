
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
    double temp106825;
    double temp107427;
    double temp107927;
    double temp108494;
    double temp111157;
    double temp112540;
    double temp113529;
    double temp113941;
    double temp114356;
    double temp114863;
    
    
    temp111157 = error * dt;
    temp113529 = kp * error;
    temp112540 = error - state->lastError;
    temp106825 = state->integral + temp111157;
    temp107927 = temp112540 / dt;
    /* 对称限幅函数 */
    temp107427 = limitSymmetrical(temp106825, integral_max);

    /* 对称限幅函数 */
    temp108494 = limitSymmetrical(temp107927, derivative_max);

    temp113941 = ki * temp107427;

         state->integral = temp107427;
    temp114356 = kd * temp108494;
    temp114863 = temp113529 + temp113941;

         state->lastError = error;
    output = temp114863 + temp114356;

    
    return output;
}