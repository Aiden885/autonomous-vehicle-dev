#include "composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667.hpp"
namespace control {
/**
 * @brief 计算近预瞄点处的三次车道多项式横向误差。
 */
void composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667::run(const Input &input, Output &output)
{
    
    
    Real temp818010;
    Real temp818276;
    Real temp818329;
    Real temp818495;
    Real temp818728;
    
    
    temp818728 = input.c3 * input.previewPointDistanceM;
    temp818495 = temp818728 + input.c2;
    temp818329 = temp818495 * input.previewPointDistanceM;
    temp818276 = input.c1 + temp818329;
    temp818010 = input.previewPointDistanceM * temp818276;
    output.lateralErrorM = temp818010 + input.c0;

    
}
} // namespace control