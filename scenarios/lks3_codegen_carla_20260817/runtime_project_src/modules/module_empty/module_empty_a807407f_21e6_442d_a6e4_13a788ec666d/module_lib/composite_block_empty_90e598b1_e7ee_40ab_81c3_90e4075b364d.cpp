#include "composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364d.hpp"
namespace control {
/**
 * @brief 计算远预瞄点处的三次车道多项式横向误差。
 */
void composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364d::run(const Input &input, Output &output)
{
    
    
    Real temp720763;
    Real temp721075;
    Real temp721130;
    Real temp721194;
    Real temp721363;
    
    
    temp721363 = input.c3 * input.previewPointDistanceM;
    temp721194 = temp721363 + input.c2;
    temp721130 = temp721194 * input.previewPointDistanceM;
    temp721075 = input.c1 + temp721130;
    temp720763 = input.previewPointDistanceM * temp721075;
    output.lateralErrorM = temp720763 + input.c0;

    
}
} // namespace control