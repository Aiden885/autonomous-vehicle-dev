#include "pidController.h"
#include "mathC.h"

/**
 * @brief 更新PID控制器的输出值
 * @en_name pid
 * @cn_name 更新PID控制器的输出值
 * @type widget&module
 * @param[IN] PIDState* state PID状态参数结构体
 * @param type=double,enName=kp,cnName=比例增益,range=[0,100],default=0.0,desc="比例增益，放大误差"
 * @param type=double,enName=ki,cnName=积分增益,range=[0,10],default=0.0,desc="积分增益，消除静态误差"
 * @param type=double,enName=kd,cnName=微分增益,range=[0,10],default=0.0,desc="微分增益，预测误差变化趋势"
 * @param type=double,enName=dt,cnName=控制周期,range=[0,10],default=0.0,desc="离散PID控制周期"
 * @param type=double,enName=integral_max,cnName=积分限幅,range=[0,10],default=0.0,desc="积分限幅"
 * @param type=double,enName=derivative_max,cnName=微分限幅,range=[0,10],default=0.0,desc="微分限幅"
 * @param[IN] double error 误差值
 * @retval double output0 PID控制器输出
 * @granularity composite
 * @tag_level0 功能模块库
 * @tag_level1 车辆控制
 * @tag_level2 PID控制
 * @formula /
 * @version 1.0
 * @date 2026-01-27
 * @author lupeng
 */
double pid(
    double kp,double ki,double kd,double dt,
    double integral_max,double derivative_max,
    double error,
    PIDState* state
)
{
    /* ---------- 旧状态读取 ---------- */
    double integral0  = state->integral;
    double lastError0 = state->lastError;

    /* ---------- 积分计算 ---------- */
    double integral1 = integral0 + error * dt;
    double integral2 = limitSymmetrical(integral1, integral_max);

    /* ---------- 微分计算 ---------- */
    double derivative0 = (error - lastError0) / dt;
    double derivative1 = limitSymmetrical(derivative0, derivative_max);

    /* ---------- 输出计算 ---------- */
    double pTerm0 = kp * error;
    double iTerm0 = ki * integral2;
    double dTerm0 = kd * derivative1;

    double output0 = pTerm0 + iTerm0 + dTerm0;

    /* ---------- 状态回写 ---------- */
    state->integral  = integral2;
    state->lastError = error;

    /* ---------- 返回 ---------- */
    return output0;
}
