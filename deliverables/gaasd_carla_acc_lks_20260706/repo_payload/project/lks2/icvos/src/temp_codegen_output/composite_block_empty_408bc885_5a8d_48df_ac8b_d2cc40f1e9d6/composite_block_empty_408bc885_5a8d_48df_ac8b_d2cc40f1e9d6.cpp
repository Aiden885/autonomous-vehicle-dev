#include "composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6::run(const Input &input, Output &output)
{
    
    
    Real temp351417;
    Real temp351813;
    Real temp352177;
    
    
    temp351417 = global::params.lks_w1 * input.e1;
    temp351813 = global::params.lks_w2 * input.e2;
    temp352177 = input.e3 * global::params.lks_w3;
    output.weightedError = temp351417 + temp351813 + temp352177;
    output.weightedError = output.weightedError;

    
}
} // namespace control
