#include "composite_block_empty_8a3df3a4_11ef_4cf0_9f3a_b380382dd3a9.hpp"
namespace control {
/**
 * @brief 根据车速、制动和驾驶员转向输入判断LKS是否允许控制。
 */
void composite_block_empty_8a3df3a4_11ef_4cf0_9f3a_b380382dd3a9::run(const Input &input, Output &output)
{
    
    
    
    
    composite_block_empty_179b05d8_8bbd_4e95_b179_c4e4788576c4Traits::Input checkOperatingSpeed_in;
    checkOperatingSpeed_in.egoSpeedMps = input.egoSpeedMps;
    composite_block_empty_179b05d8_8bbd_4e95_b179_c4e4788576c4Traits::Output checkOperatingSpeed_out;

    sub_.checkOperatingSpeed.run(checkOperatingSpeed_in, checkOperatingSpeed_out);
    composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637dTraits::Input detectDriverOverride_in;
    detectDriverOverride_in.driverSteerNorm = input.driverSteerNorm;
    composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637dTraits::Output detectDriverOverride_out;

    sub_.detectDriverOverride.run(detectDriverOverride_in, detectDriverOverride_out);
    composite_block_empty_fde75d0d_6428_4789_a7ea_30b7a3291b00Traits::Input determineControlEnable_in;
    determineControlEnable_in.isSpeedUnavailable = checkOperatingSpeed_out.isSpeedUnavailable;
    determineControlEnable_in.isDriverOverride = detectDriverOverride_out.isDriverOverride;
    determineControlEnable_in.isBrakePressed = input.isBrakePressed;
    composite_block_empty_fde75d0d_6428_4789_a7ea_30b7a3291b00Traits::Output determineControlEnable_out;

    sub_.determineControlEnable.run(determineControlEnable_in, determineControlEnable_out);
    output.isControlEnabled = determineControlEnable_out.isControlEnabled;

    
}
} // namespace control