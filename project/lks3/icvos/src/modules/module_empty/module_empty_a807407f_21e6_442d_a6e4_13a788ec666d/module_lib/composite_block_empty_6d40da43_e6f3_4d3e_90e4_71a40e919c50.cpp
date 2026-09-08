#include "composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50.hpp"
namespace control {
/**
 * @brief 根据车速和道路曲率计算LKS远预瞄距离。
 */
void composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50::run(const Input &input, Output &output)
{
    
    const double ONE = 1;  // 常量
    
    Real temp836242;
    Real temp837140;
    Real temp837485;
    Real temp837828;
    Real temp838225;
    Real temp838695;
    Real temp839057;
    
    
    temp837140 = global::params.lks_rt * input.egoSpeedMps;
    temp838225 = ONE - global::params.lks_rAlpha;
    /* 浮点绝对值 */
    temp836242 = fabs(input.roadCurvaturePerM);

    temp837485 = global::params.lks_l0 + temp837140;
    temp837828 = temp836242 >= global::params.lks_curvatureThreshold;
    temp838695 = temp837828 * temp838225;
    temp839057 = ONE - temp838695;
    output.previewDistanceM = temp837485 * temp839057;

    
}
} // namespace control