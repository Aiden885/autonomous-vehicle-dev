#include "composite_block_empty_e6613cb4_7f79_4b07_9207_efebebea2cbb.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_e6613cb4_7f79_4b07_9207_efebebea2cbb::composite_block(const Input &input, Output &output)
{
    
    
    Real e;
    Real temp758664;
    Real temp758792;
    Real temp758849;
    Real temp758934;
    Real temp759165;
    
    
    temp759165 = c3 * x;
    temp758934 = temp759165 + c2;
    temp758849 = temp758934 * x;
    temp758792 = c1 + temp758849;
    temp758664 = x * temp758792;
    e = temp758664 + c0;
    output.e = e;

    
}
} // namespace control