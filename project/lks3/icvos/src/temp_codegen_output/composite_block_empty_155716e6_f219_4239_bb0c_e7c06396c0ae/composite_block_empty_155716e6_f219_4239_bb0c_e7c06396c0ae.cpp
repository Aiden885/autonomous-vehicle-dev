#include "composite_block_empty_155716e6_f219_4239_bb0c_e7c06396c0ae.hpp"
namespace control {
/**
 * @brief 计算近、中、远三个预瞄点处的三次车道多项式横向误差。
 */
void composite_block_empty_155716e6_f219_4239_bb0c_e7c06396c0ae::run(const Input &input, Output &output)
{
    
    const double PREVIEW_DISTANCE_DIVISOR = 2;  // 常量
    
    Real temp813880;
    
    
    composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667Traits::Input calculateNearPreviewError_in;
    calculateNearPreviewError_in.c0 = input.c0;
    calculateNearPreviewError_in.c1 = input.c1;
    calculateNearPreviewError_in.c2 = input.c2;
    calculateNearPreviewError_in.c3 = input.c3;
    calculateNearPreviewError_in.previewPointDistanceM = global::params.lks_nearPreviewDistance;
    composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667Traits::Output calculateNearPreviewError_out;

    sub_.calculateNearPreviewError.run(calculateNearPreviewError_in, calculateNearPreviewError_out);
    temp813880 = input.previewDistanceM / PREVIEW_DISTANCE_DIVISOR;
    composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364dTraits::Input calculateFarPreviewError_in;
    calculateFarPreviewError_in.c0 = input.c0;
    calculateFarPreviewError_in.c1 = input.c1;
    calculateFarPreviewError_in.c2 = input.c2;
    calculateFarPreviewError_in.c3 = input.c3;
    calculateFarPreviewError_in.previewPointDistanceM = input.previewDistanceM;
    composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364dTraits::Output calculateFarPreviewError_out;

    sub_.calculateFarPreviewError.run(calculateFarPreviewError_in, calculateFarPreviewError_out);
    composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83Traits::Input calculateMiddlePreviewError_in;
    calculateMiddlePreviewError_in.c0 = input.c0;
    calculateMiddlePreviewError_in.c1 = input.c1;
    calculateMiddlePreviewError_in.c2 = input.c2;
    calculateMiddlePreviewError_in.c3 = input.c3;
    calculateMiddlePreviewError_in.previewPointDistanceM = temp813880;
    composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83Traits::Output calculateMiddlePreviewError_out;

    sub_.calculateMiddlePreviewError.run(calculateMiddlePreviewError_in, calculateMiddlePreviewError_out);
    output.nearLateralErrorM = calculateNearPreviewError_out.lateralErrorM;
    output.farLateralErrorM = calculateFarPreviewError_out.lateralErrorM;
    output.middleLateralErrorM = calculateMiddlePreviewError_out.lateralErrorM;

    
}
} // namespace control