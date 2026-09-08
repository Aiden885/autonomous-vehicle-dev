#include "composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364d.hpp"
namespace control {
/**
 * @brief 计算远预瞄点处的三次车道多项式横向误差。
 */
void composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364d::run(const Input &input, Output &output)
{
    
    
    Real temp831757;
    Real temp831955;
    Real temp832171;
    Real temp832349;
    Real temp832569;
    
    
    temp832569 = input.c3 * input.previewPointDistanceM;
    temp832349 = temp832569 + input.c2;
    temp832171 = temp832349 * input.previewPointDistanceM;
    temp831955 = input.c1 + temp832171;
    temp831757 = input.previewPointDistanceM * temp831955;
    output.lateralErrorM = temp831757 + input.c0;

    
}
} // namespace control