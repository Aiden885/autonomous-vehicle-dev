#include "run_3d336890_b7fc_408b_ac40_17cd5dce2b59.hpp"
namespace control {
/**
 * @brief 更新ACC控制使能、在控记忆和历史状态。
 */
void run_3d336890_b7fc_408b_ac40_17cd5dce2b59::run(const Input &input, Output &output)
{
    
    const double DECISION_ENGAGE = 5;  // 常量
    const double DECISION_RESUME = 6;  // 常量
    const double COMMAND_BRAKE = 6;  // 常量
    const double COMMAND_CANCEL = 7;  // 常量
    
    Real temp135257;
    Real temp135696;
    Real temp135894;
    Real temp136028;
    Real temp136210;
    Real temp136452;
    Real temp136697;
    Real temp136966;
    Real temp137085;
    
    
    temp135894 = input.decisionCode == DECISION_RESUME;
    temp135696 = DECISION_ENGAGE == input.decisionCode;
    temp136028 = input.commandType == COMMAND_BRAKE;
    temp136210 = input.commandType == COMMAND_CANCEL;
    temp136452 = temp135696 || temp135894;
    temp136697 = temp136028 || temp136210;
    temp137085 = input.isControlActive || temp136452;
    temp136966 = !temp136697;
    temp135257 = temp137085 && temp136966;
    output.hasControlHistoryNext = input.hasControlHistory || temp135257;
    output.isControlActiveNext = temp135257;

    
}
} // namespace control