#include "run_a15a0aa8_22ce_4f0e_8b20_5deb0a4478e5.hpp"
namespace control {
/**
 * @brief 根据驾驶指令更新目标速度上限。
 */
void run_a15a0aa8_22ce_4f0e_8b20_5deb0a4478e5::run(const Input &input, Output &output)
{
    
    const double DECISION_SPEED_DECREASE = 1;  // 常量
    const double DECISION_SPEED_INCREASE = 2;  // 常量
    
    Real temp115351;
    Real temp116827;
    Real temp117043;
    Real temp117221;
    Real temp117425;
    Real temp117687;
    
    
    temp116827 = input.decisionCode == DECISION_SPEED_DECREASE;
    temp117043 = input.decisionCode == DECISION_SPEED_INCREASE;
    temp117221 = temp116827 * input.speedStepMps;
    temp117425 = temp117043 * input.speedStepMps;
    temp117687 = input.maxSpeedMps - temp117221;
    temp115351 = temp117687 + temp117425;
    /* 浮点最大值 */
    output.nextMaxSpeedMps = fmax(temp115351, input.minSpeedMps);


    
}
} // namespace control