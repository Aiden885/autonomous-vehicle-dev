#include "composite_block_empty_b25a47b1_cf11_4375_a368_3b21bdca7e68.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_b25a47b1_cf11_4375_a368_3b21bdca7e68::composite_block(const Input &input, Output &output)
{
    
    
    Real temp332578;
    Real temp332743;
    Real temp332968;
    Real temp333075;
    Real temp333211;
    
    
    temp333211 = input.c3 * input.x;
    temp333075 = temp333211 + input.c2;
    temp332968 = temp333075 * input.x;
    temp332743 = input.c1 + temp332968;
    temp332578 = input.x * temp332743;
    output.e = temp332578 + input.c0;
    output.e = output.e;

    
}
} // namespace control