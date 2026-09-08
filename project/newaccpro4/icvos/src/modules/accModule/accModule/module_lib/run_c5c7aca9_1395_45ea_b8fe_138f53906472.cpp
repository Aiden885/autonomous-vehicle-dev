#include "run_c5c7aca9_1395_45ea_b8fe_138f53906472.hpp"
namespace control {
/**
 * @brief 根据驾驶指令更新目标时距。
 */
void run_c5c7aca9_1395_45ea_b8fe_138f53906472::run(const Input &input, Output &output)
{
    
    const double DECISION_GAP_DECREASE = 3;  // 常量
    const double DECISION_GAP_INCREASE = 4;  // 常量
    
    Real temp103683;
    Real temp103861;
    Real temp106269;
    Real temp106479;
    Real temp106668;
    Real temp106820;
    Real temp107043;
    
    
    temp106479 = input.decisionCode == DECISION_GAP_INCREASE;
    temp106269 = input.decisionCode == DECISION_GAP_DECREASE;
    temp106668 = temp106479 * input.timeGapStepS;
    temp106820 = input.timeGapStepS * temp106269;
    temp107043 = input.timeGapS + temp106668;
    temp103683 = temp107043 - temp106820;
    /* 浮点最大值 */
    temp103861 = fmax(temp103683, input.minTimeGapS);

    /* 浮点最小值 */
    output.nextTimeGapS = fmin(temp103861, input.maxTimeGapS);


    
}
} // namespace control