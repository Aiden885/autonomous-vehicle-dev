#include "composite_block_empty_d57b8b07_7b62_49d0_a4d9_f37434b892f4.hpp"
namespace control {
/**
 * @brief 计算受横向加速度约束并经过使能门控的方向盘转角命令。
 */
void composite_block_empty_d57b8b07_7b62_49d0_a4d9_f37434b892f4::run(const Input &input, Output &output)
{
    
    
    
    
    composite_block_empty_5c77e1f3_497c_42c2_92d4_32dce5cd71b9Traits::Input calculateRawSteer_in;
    calculateRawSteer_in.weightedLateralErrorM = input.weightedLateralErrorM;
    composite_block_empty_5c77e1f3_497c_42c2_92d4_32dce5cd71b9Traits::Output calculateRawSteer_out;

    sub_.calculateRawSteer.run(calculateRawSteer_in, calculateRawSteer_out);
    composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560Traits::Input limitSteerByLateralAcceleration_in;
    limitSteerByLateralAcceleration_in.rawSteerNorm = calculateRawSteer_out.rawSteerNorm;
    limitSteerByLateralAcceleration_in.egoSpeedMps = input.egoSpeedMps;
    composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560Traits::Output limitSteerByLateralAcceleration_out;

    sub_.limitSteerByLateralAcceleration.run(limitSteerByLateralAcceleration_in, limitSteerByLateralAcceleration_out);
    composite_block_empty_64c982f5_0edb_4c03_b06c_9a22f84347c5Traits::Input applySteerEnable_in;
    applySteerEnable_in.limitedSteerNorm = limitSteerByLateralAcceleration_out.limitedSteerNorm;
    applySteerEnable_in.isControlEnabled = input.isControlEnabled;
    composite_block_empty_64c982f5_0edb_4c03_b06c_9a22f84347c5Traits::Output applySteerEnable_out;

    sub_.applySteerEnable.run(applySteerEnable_in, applySteerEnable_out);
    output.steerRad = applySteerEnable_out.steerRad;

    
}
} // namespace control