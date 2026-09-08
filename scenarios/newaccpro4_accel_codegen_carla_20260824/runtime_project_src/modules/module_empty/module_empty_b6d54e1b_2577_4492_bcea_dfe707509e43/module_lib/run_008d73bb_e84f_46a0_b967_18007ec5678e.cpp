#include "run_008d73bb_e84f_46a0_b967_18007ec5678e.hpp"
namespace control {
/**
 * @brief 更新ACC控制使能、在控记忆和历史状态。
 */
void run_008d73bb_e84f_46a0_b967_18007ec5678e::run(const Input &input, Output &output)
{
    
    const double DECISION_ENGAGE = 5;  // 常量
    const double DECISION_RESUME = 6;  // 常量
    const double COMMAND_BRAKE = 6;  // 常量
    const double COMMAND_CANCEL = 7;  // 常量
    
    Real temp142394;
    Real temp142759;
    Real temp142947;
    Real temp143123;
    Real temp143392;
    Real temp143590;
    Real temp143787;
    Real temp143999;
    Real temp144127;
    
    
    temp143123 = input.commandType == COMMAND_BRAKE;
    temp143392 = input.commandType == COMMAND_CANCEL;
    temp142947 = input.decisionCode == DECISION_RESUME;
    temp142759 = DECISION_ENGAGE == input.decisionCode;
    temp143787 = temp143123 || temp143392;
    temp143590 = temp142759 || temp142947;
    temp143999 = !temp143787;
    temp144127 = input.isControlActive || temp143590;
    temp142394 = temp144127 && temp143999;
    output.hasControlHistoryNext = input.hasControlHistory || temp142394;
    output.isControlActiveNext = temp142394;

    
}
} // namespace control