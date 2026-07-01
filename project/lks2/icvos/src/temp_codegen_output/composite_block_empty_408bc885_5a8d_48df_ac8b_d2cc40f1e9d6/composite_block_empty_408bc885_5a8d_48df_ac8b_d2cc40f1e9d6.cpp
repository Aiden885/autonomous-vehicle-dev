#include "composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6::composite_block(const Input &input, Output &output)
{
    
    
    Real temp351417;
    Real temp351813;
    Real temp352177;
    
    
    temp351417 = temp351699 * input.e1;
    temp351813 = temp351929 * input.e2;
    temp352177 = input.e3 * temp352326;
    output.weightedError = temp351417 + temp351813 + temp352177;
    output.weightedError = output.weightedError;

    
}
} // namespace control