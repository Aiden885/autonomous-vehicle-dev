#include "composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637d.hpp"
namespace control {
/**
 * @brief 根据驾驶员方向盘输入判断是否发生主动转向接管。
 */
void composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637d::run(const Input &input, Output &output)
{
    
    
    Real temp687163;
    
    
    /* calculateDriverSteerMagnitude */
    temp687163 = fabs(input.driverSteerNorm);

    output.isDriverOverride = temp687163 >= global::params.lks_driverSteerThreshold;

    
}
} // namespace control