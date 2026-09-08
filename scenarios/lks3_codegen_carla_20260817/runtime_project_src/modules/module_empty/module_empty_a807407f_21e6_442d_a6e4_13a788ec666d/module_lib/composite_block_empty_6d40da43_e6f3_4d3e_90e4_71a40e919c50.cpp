#include "composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50.hpp"
namespace control {
/**
 * @brief 根据车速和道路曲率计算LKS远预瞄距离。
 */
void composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50::run(const Input &input, Output &output)
{
    
    const double ONE = 1;  // 常量
    
    Real temp725283;
    Real temp726671;
    Real temp727024;
    Real temp727350;
    Real temp727670;
    Real temp728072;
    Real temp728374;
    
    
    temp726671 = global::params.lks_rt * input.egoSpeedMps;
    temp727670 = ONE - global::params.lks_rAlpha;
    /* calculateCurvatureMagnitude */
    temp725283 = fabs(input.roadCurvaturePerM);

    temp727024 = global::params.lks_l0 + temp726671;
    temp727350 = temp725283 >= global::params.lks_curvatureThreshold;
    temp728072 = temp727350 * temp727670;
    temp728374 = ONE - temp728072;
    output.previewDistanceM = temp727024 * temp728374;

    
}
} // namespace control