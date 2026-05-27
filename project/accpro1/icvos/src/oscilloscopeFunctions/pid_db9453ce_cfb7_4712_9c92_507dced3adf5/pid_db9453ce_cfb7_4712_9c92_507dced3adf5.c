
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
    double temp554894;
    double temp555187;
    double temp555574;
    double temp555720;
    double temp557198;
    double temp557748;
    double temp558492;
    double temp558814;
    double temp559285;
    double temp559749;
    
    
    temp557198 = error * dt;
    temp558492 = kp * error;
    temp557748 = error - state->lastError;
    temp554894 = state->integral + temp557198;
    temp555574 = temp557748 / dt;
    /* 对称限幅函数 */
    temp555187 = limitSymmetrical(temp554894, integral_max);

    /* 对称限幅函数 */
    temp555720 = limitSymmetrical(temp555574, derivative_max);

    temp558814 = ki * temp555187;

         state->integral = temp555187;
    temp559285 = kd * temp555720;
    temp559749 = temp558492 + temp558814;

         state->lastError = error;
    output = temp559749 + temp559285;

    
    return output;
}