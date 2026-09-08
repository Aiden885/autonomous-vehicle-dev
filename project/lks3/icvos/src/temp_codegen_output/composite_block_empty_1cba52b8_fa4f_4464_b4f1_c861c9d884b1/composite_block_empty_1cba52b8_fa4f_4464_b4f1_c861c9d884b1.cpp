#include "composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1.hpp"
namespace control {
/**
 * @brief 按三组权重合成近、中、远预瞄点横向误差。
 */
void composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1::run(const Input &input, Output &output)
{
    
    
    Real temp844828;
    Real temp845214;
    Real temp845651;
    
    
    temp845214 = global::params.lks_w2 * input.middleLateralErrorM;
    temp845651 = input.farLateralErrorM * global::params.lks_w3;
    temp844828 = global::params.lks_w1 * input.nearLateralErrorM;
    output.weightedLateralErrorM = temp844828 + temp845214 + temp845651;

    
}
} // namespace control