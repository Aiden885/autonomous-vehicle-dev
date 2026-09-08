#include "composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83.hpp"
namespace control {
/**
 * @brief 计算中预瞄点处的三次车道多项式横向误差。
 */
void composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83::run(const Input &input, Output &output)
{
    
    
    Real temp784484;
    Real temp784584;
    Real temp784696;
    Real temp784716;
    Real temp784865;
    
    
    temp784865 = input.c3 * input.previewPointDistanceM;
    temp784716 = temp784865 + input.c2;
    temp784696 = temp784716 * input.previewPointDistanceM;
    temp784584 = input.c1 + temp784696;
    temp784484 = input.previewPointDistanceM * temp784584;
    output.lateralErrorM = temp784484 + input.c0;

    
}
} // namespace control