#include "composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637d.hpp"
namespace control {
/**
 * @brief 根据驾驶员方向盘输入判断是否发生主动转向接管。
 */
void composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637d::run(const Input &input, Output &output)
{
    
    
    Real temp787463;
    
    
    /* 浮点绝对值 */
    temp787463 = fabs(input.driverSteerNorm);

    output.isDriverOverride = temp787463 >= global::params.lks_driverSteerThreshold;

    
}
} // namespace control