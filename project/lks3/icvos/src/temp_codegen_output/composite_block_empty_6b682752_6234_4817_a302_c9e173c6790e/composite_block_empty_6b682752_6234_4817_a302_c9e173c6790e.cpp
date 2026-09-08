#include "composite_block_empty_6b682752_6234_4817_a302_c9e173c6790e.hpp"
namespace control {
/**
 * @brief 完成预瞄距离、三点车道误差、误差加权和方向盘转角命令计算。
 */
void composite_block_empty_6b682752_6234_4817_a302_c9e173c6790e::run(const Input &input, Output &output)
{
    
    
    
    
    composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50Traits::Input calculatePreviewDistance_in;
    calculatePreviewDistance_in.egoSpeedMps = input.egoSpeedMps;
    calculatePreviewDistance_in.roadCurvaturePerM = input.roadCurvaturePerM;
    composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50Traits::Output calculatePreviewDistance_out;

    sub_.calculatePreviewDistance.run(calculatePreviewDistance_in, calculatePreviewDistance_out);
    composite_block_empty_155716e6_f219_4239_bb0c_e7c06396c0aeTraits::Input calculatePreviewErrors_in;
    calculatePreviewErrors_in.c0 = input.c0;
    calculatePreviewErrors_in.c1 = input.c1;
    calculatePreviewErrors_in.c2 = input.c2;
    calculatePreviewErrors_in.c3 = input.c3;
    calculatePreviewErrors_in.previewDistanceM = calculatePreviewDistance_out.previewDistanceM;
    composite_block_empty_155716e6_f219_4239_bb0c_e7c06396c0aeTraits::Output calculatePreviewErrors_out;

    sub_.calculatePreviewErrors.run(calculatePreviewErrors_in, calculatePreviewErrors_out);
    composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1Traits::Input calculateWeightedError_in;
    calculateWeightedError_in.nearLateralErrorM = calculatePreviewErrors_out.nearLateralErrorM;
    calculateWeightedError_in.middleLateralErrorM = calculatePreviewErrors_out.middleLateralErrorM;
    calculateWeightedError_in.farLateralErrorM = calculatePreviewErrors_out.farLateralErrorM;
    composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1Traits::Output calculateWeightedError_out;

    sub_.calculateWeightedError.run(calculateWeightedError_in, calculateWeightedError_out);
    composite_block_empty_d57b8b07_7b62_49d0_a4d9_f37434b892f4Traits::Input generateSteerCommand_in;
    generateSteerCommand_in.weightedLateralErrorM = calculateWeightedError_out.weightedLateralErrorM;
    generateSteerCommand_in.egoSpeedMps = input.egoSpeedMps;
    generateSteerCommand_in.isControlEnabled = input.isControlEnabled;
    composite_block_empty_d57b8b07_7b62_49d0_a4d9_f37434b892f4Traits::Output generateSteerCommand_out;

    sub_.generateSteerCommand.run(generateSteerCommand_in, generateSteerCommand_out);
    output.steerRad = generateSteerCommand_out.steerRad;

    
}
} // namespace control