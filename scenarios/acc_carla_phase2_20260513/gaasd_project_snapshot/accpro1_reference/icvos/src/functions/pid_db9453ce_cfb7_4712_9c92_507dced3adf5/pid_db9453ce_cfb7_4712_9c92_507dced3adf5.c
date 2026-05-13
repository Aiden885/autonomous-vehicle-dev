
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
    double temp854773;
    double temp854987;
    double temp855154;
    double temp855363;
    double temp856240;
    double temp856594;
    double temp856995;
    double temp857184;
    double temp857337;
    double temp857656;
    
    
    temp856240 = error * dt;
    temp856995 = kp * error;
    temp856594 = error - state->lastError;
    temp854773 = state->integral + temp856240;
    temp855154 = temp856594 / dt;
    /* 对称限幅函数 */
    temp854987 = limitSymmetrical(temp854773, integral_max);

    /* 对称限幅函数 */
    temp855363 = limitSymmetrical(temp855154, derivative_max);

    temp857184 = ki * temp854987;

         state->integral = temp854987;
    temp857337 = kd * temp855363;
    temp857656 = temp856995 + temp857184;

         state->lastError = error;
    output = temp857656 + temp857337;

    
    return output;
}