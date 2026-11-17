#include "run_a08264b6_d877_4a94_8756_9c8b9c5fde57.hpp"
namespace control {
/**
 * @brief 根据驾驶指令更新目标时距。
 */
void run_a08264b6_d877_4a94_8756_9c8b9c5fde57::run(const Input &input, Output &output)
{
    
    const double DECISION_GAP_DECREASE = 3;  // 常量
    const double DECISION_GAP_INCREASE = 4;  // 常量
    
    Real temp134157;
    Real temp134217;
    Real temp138797;
    Real temp138939;
    Real temp139144;
    Real temp139360;
    Real temp139522;
    
    
    temp138797 = input.decisionCode == DECISION_GAP_DECREASE;
    temp138939 = input.decisionCode == DECISION_GAP_INCREASE;
    temp139360 = input.timeGapStepS * temp138797;
    temp139144 = temp138939 * input.timeGapStepS;
    temp139522 = input.timeGapS + temp139144;
    temp134157 = temp139522 - temp139360;
    /* 浮点最大值 */
    temp134217 = fmax(temp134157, input.minTimeGapS);

    /* 浮点最小值 */
    output.nextTimeGapS = fmin(temp134217, input.maxTimeGapS);


    
}
} // namespace control