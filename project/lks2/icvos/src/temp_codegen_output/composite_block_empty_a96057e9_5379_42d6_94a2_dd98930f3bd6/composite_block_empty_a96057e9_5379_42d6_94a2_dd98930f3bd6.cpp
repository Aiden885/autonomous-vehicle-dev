#include "composite_block_empty_a96057e9_5379_42d6_94a2_dd98930f3bd6.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_a96057e9_5379_42d6_94a2_dd98930f3bd6::composite_block(const Input &input, Output &output)
{
    
    
    Real temp328158;
    Real temp328390;
    Real temp328534;
    Real temp328648;
    Real temp328834;
    
    
    temp328834 = input.c3 * input.x;
    temp328648 = temp328834 + input.c2;
    temp328534 = temp328648 * input.x;
    temp328390 = input.c1 + temp328534;
    temp328158 = input.x * temp328390;
    output.e = temp328158 + input.c0;
    output.e = output.e;

    
}
} // namespace control