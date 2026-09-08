#include "run_b71d71c8_ff00_4f2b_b015_08ac387673a6.hpp"
namespace control {
/**
 * @brief 根据车速、在控记忆与历史状态识别ACC系统状态。
 */
void run_b71d71c8_ff00_4f2b_b015_08ac387673a6::run(const Input &input, Output &output)
{
    
    const double STATE_STANDBY_NO_HISTORY = 2;  // 常量
    const double STATE_LOW_SPEED = 3;  // 常量
    
    Real temp131646;
    Real temp131891;
    Real temp131981;
    Real temp132178;
    Real temp132241;
    Real temp132539;
    Real temp133096;
    Real temp133280;
    
    
    temp132178 = !input.hasControlHistory;
    temp131646 = input.egoSpeedMps < input.minimumOperatingSpeedMps;
    temp131981 = !input.isControlActive;
    temp131891 = !temp131646;
    temp133280 = temp131646 * STATE_LOW_SPEED;
    output.isControlEnabled = temp131891 && input.isControlActive;
    temp132241 = temp131891 && temp131981 && input.hasControlHistory;
    temp132539 = temp131891 && temp131981 && temp132178;
    temp133096 = STATE_STANDBY_NO_HISTORY * temp132539;
    output.systemState = temp132241 + temp133096 + temp133280;

    
}
} // namespace control