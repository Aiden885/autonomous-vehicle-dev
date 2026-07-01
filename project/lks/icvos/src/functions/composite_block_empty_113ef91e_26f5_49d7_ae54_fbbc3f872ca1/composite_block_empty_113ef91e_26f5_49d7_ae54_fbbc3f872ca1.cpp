#include "composite_block_empty_113ef91e_26f5_49d7_ae54_fbbc3f872ca1.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_113ef91e_26f5_49d7_ae54_fbbc3f872ca1::composite_block(const Input &input, Output &output)
{
    
    
    Real e;
    Real temp744493;
    Real temp744613;
    Real temp744779;
    Real temp744747;
    Real temp744941;
    
    
    temp744941 = c3 * x;
    temp744747 = temp744941 + c2;
    temp744779 = temp744747 * x;
    temp744613 = c1 + temp744779;
    temp744493 = x * temp744613;
    e = temp744493 + c0;
    output.e = e;

    
}
} // namespace control