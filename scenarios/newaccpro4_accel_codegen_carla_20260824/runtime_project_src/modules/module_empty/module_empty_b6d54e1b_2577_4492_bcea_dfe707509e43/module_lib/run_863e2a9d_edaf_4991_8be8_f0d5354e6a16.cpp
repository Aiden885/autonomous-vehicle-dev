#include "run_863e2a9d_edaf_4991_8be8_f0d5354e6a16.hpp"
namespace control {
/**
 * @brief 根据驾驶指令更新目标速度上限。
 */
void run_863e2a9d_edaf_4991_8be8_f0d5354e6a16::run(const Input &input, Output &output)
{
    
    const double DECISION_SPEED_DECREASE = 1;  // 常量
    const double DECISION_SPEED_INCREASE = 2;  // 常量
    
    Real temp086027;
    Real temp087775;
    Real temp087966;
    Real temp088199;
    Real temp088391;
    Real temp088532;
    
    
    temp087775 = input.decisionCode == DECISION_SPEED_DECREASE;
    temp087966 = input.decisionCode == DECISION_SPEED_INCREASE;
    temp088199 = temp087775 * input.speedStepMps;
    temp088391 = temp087966 * input.speedStepMps;
    temp088532 = input.maxSpeedMps - temp088199;
    temp086027 = temp088532 + temp088391;
    /* 浮点最大值 */
    output.nextMaxSpeedMps = fmax(temp086027, input.minSpeedMps);


    
}
} // namespace control