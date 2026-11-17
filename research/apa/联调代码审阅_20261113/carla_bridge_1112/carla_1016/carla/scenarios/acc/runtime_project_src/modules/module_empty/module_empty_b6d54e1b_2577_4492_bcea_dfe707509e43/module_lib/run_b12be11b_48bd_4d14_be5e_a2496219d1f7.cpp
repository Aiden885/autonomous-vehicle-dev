#include "run_b12be11b_48bd_4d14_be5e_a2496219d1f7.hpp"
namespace control {
/**
 * @brief 根据车速、在控记忆与历史状态识别ACC系统状态。
 */
void run_b12be11b_48bd_4d14_be5e_a2496219d1f7::run(const Input &input, Output &output)
{
    
    const double STATE_STANDBY_NO_HISTORY = 2;  // 常量
    const double STATE_LOW_SPEED = 3;  // 常量
    
    Real temp130635;
    Real temp130869;
    Real temp130958;
    Real temp131192;
    Real temp131218;
    Real temp131548;
    Real temp132098;
    Real temp132235;
    
    
    temp130958 = !input.isControlActive;
    temp131192 = !input.hasControlHistory;
    temp130635 = input.egoSpeedMps < input.minimumOperatingSpeedMps;
    temp130869 = !temp130635;
    temp132235 = temp130635 * STATE_LOW_SPEED;
    output.isControlEnabled = temp130869 && input.isControlActive;
    temp131218 = temp130869 && temp130958 && input.hasControlHistory;
    temp131548 = temp130869 && temp130958 && temp131192;
    temp132098 = STATE_STANDBY_NO_HISTORY * temp131548;
    output.systemState = temp131218 + temp132098 + temp132235;

    
}
} // namespace control