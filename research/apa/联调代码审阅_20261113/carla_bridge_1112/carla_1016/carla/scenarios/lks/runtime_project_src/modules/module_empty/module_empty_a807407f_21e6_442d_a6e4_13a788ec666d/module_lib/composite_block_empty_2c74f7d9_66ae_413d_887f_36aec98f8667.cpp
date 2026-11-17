#include "composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667.hpp"
namespace control {
/**
 * @brief 计算近预瞄点处的三次车道多项式横向误差。
 */
void composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667::run(const Input &input, Output &output)
{
    
    
    Real temp712369;
    Real temp712466;
    Real temp712551;
    Real temp712569;
    Real temp712741;
    
    
    temp712741 = input.c3 * input.previewPointDistanceM;
    temp712569 = temp712741 + input.c2;
    temp712551 = temp712569 * input.previewPointDistanceM;
    temp712466 = input.c1 + temp712551;
    temp712369 = input.previewPointDistanceM * temp712466;
    output.lateralErrorM = temp712369 + input.c0;

    
}
} // namespace control