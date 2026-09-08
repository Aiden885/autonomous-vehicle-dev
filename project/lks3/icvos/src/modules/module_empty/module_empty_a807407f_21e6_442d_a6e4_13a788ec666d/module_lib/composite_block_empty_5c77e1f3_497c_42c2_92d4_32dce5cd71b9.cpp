#include "composite_block_empty_5c77e1f3_497c_42c2_92d4_32dce5cd71b9.hpp"
namespace control {
/**
 * @brief 根据加权横向误差和比例增益计算未限幅归一化转向指令。
 */
void composite_block_empty_5c77e1f3_497c_42c2_92d4_32dce5cd71b9::run(const Input &input, Output &output)
{
    
    
    
    
    output.rawSteerNorm = global::params.lks_Kp * input.weightedLateralErrorM;

    
}
} // namespace control