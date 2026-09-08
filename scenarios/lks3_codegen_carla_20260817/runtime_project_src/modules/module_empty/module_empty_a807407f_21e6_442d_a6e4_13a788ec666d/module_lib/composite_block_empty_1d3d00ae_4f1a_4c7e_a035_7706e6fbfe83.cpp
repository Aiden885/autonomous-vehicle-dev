#include "composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83.hpp"
namespace control {
/**
 * @brief 计算中预瞄点处的三次车道多项式横向误差。
 */
void composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83::run(const Input &input, Output &output)
{
    
    
    Real temp684563;
    Real temp684678;
    Real temp684715;
    Real temp684868;
    Real temp684977;
    
    
    temp684977 = input.c3 * input.previewPointDistanceM;
    temp684868 = temp684977 + input.c2;
    temp684715 = temp684868 * input.previewPointDistanceM;
    temp684678 = input.c1 + temp684715;
    temp684563 = input.previewPointDistanceM * temp684678;
    output.lateralErrorM = temp684563 + input.c0;

    
}
} // namespace control