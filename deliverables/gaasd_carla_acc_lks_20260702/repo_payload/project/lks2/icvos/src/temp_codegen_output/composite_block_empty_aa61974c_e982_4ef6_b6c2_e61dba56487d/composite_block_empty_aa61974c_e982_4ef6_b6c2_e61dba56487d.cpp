#include "composite_block_empty_aa61974c_e982_4ef6_b6c2_e61dba56487d.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_aa61974c_e982_4ef6_b6c2_e61dba56487d::run(const Input &input, Output &output)
{
    
    const double One = 1;  // 常量
    
    Real temp354919;
    Real temp355525;
    Real temp355876;
    Real temp356235;
    Real temp356515;
    Real temp356941;
    Real temp357297;
    
    
    temp355525 = global::params.lks_rt * input.egoV;
    /* AbsCurv */
    temp354919 = fabs(input.curvature);

    temp356515 = One - global::params.lks_rAlpha;
    temp355876 = global::params.lks_l0 + temp355525;
    temp356235 = temp354919 >= global::params.lks_curvatureThreshold;
    temp356941 = temp356235 * temp356515;
    temp357297 = One - temp356941;
    output.previewDistance = temp355876 * temp357297;
    output.previewDistance = output.previewDistance;

    
}
} // namespace control
