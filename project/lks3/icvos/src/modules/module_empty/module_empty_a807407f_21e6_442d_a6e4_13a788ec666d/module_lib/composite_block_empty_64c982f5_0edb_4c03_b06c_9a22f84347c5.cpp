#include "composite_block_empty_64c982f5_0edb_4c03_b06c_9a22f84347c5.hpp"
namespace control {
/**
 * @brief 将限幅后的归一化转向量换算为弧度，并按LKS使能门控输出。
 */
void composite_block_empty_64c982f5_0edb_4c03_b06c_9a22f84347c5::run(const Input &input, Output &output)
{
    
    
    Real temp841961;
    
    
    temp841961 = input.limitedSteerNorm * global::params.lks_steerScale;
    output.steerRad = temp841961 * input.isControlEnabled;

    
}
} // namespace control