#include "composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1.hpp"
namespace control {
/**
 * @brief 按三组权重合成近、中、远预瞄点横向误差。
 */
void composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1::run(const Input &input, Output &output)
{
    
    
    Real temp731889;
    Real temp732056;
    Real temp732263;
    
    
    temp732056 = global::params.lks_w2 * input.middleLateralErrorM;
    temp732263 = input.farLateralErrorM * global::params.lks_w3;
    temp731889 = global::params.lks_w1 * input.nearLateralErrorM;
    output.weightedLateralErrorM = temp731889 + temp732056 + temp732263;

    
}
} // namespace control