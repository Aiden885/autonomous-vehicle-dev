#include "composite_block_empty_fde75d0d_6428_4789_a7ea_30b7a3291b00.hpp"
namespace control {
/**
 * @brief 汇总制动、低速和驾驶员主动转向条件，输出LKS控制使能。
 */
void composite_block_empty_fde75d0d_6428_4789_a7ea_30b7a3291b00::run(const Input &input, Output &output)
{
    
    
    Real temp718986;
    
    
    temp718986 = input.isBrakePressed || input.isDriverOverride || input.isSpeedUnavailable;
    output.isControlEnabled = !temp718986;

    
}
} // namespace control