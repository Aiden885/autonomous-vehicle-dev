#include "composite_block_empty_e940d230_a6d8_49ce_aa01_87b34eb8cce5.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_e940d230_a6d8_49ce_aa01_87b34eb8cce5::composite_block(const Input &input, Output &output)
{
    
    
    Real weightedError;
    Real temp756347;
    Real temp756545;
    Real temp756766;
    
    
    temp756347 = temp756413 * e1;
    temp756545 = temp756646 * e2;
    temp756766 = e3 * temp756924;
    weightedError = temp756347 + temp756545 + temp756766;
    output.weightedError = weightedError;

    
}
} // namespace control