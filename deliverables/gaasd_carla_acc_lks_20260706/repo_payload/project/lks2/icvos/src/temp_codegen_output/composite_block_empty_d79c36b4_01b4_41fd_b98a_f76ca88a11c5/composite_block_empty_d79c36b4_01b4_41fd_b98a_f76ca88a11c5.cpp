#include "composite_block_empty_d79c36b4_01b4_41fd_b98a_f76ca88a11c5.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_d79c36b4_01b4_41fd_b98a_f76ca88a11c5::run(const Input &input, Output &output)
{
    
    
    Real temp347319;
    Real temp347448;
    Real temp347625;
    Real temp347745;
    Real temp347943;
    
    
    temp347943 = input.c3 * input.x;
    temp347745 = temp347943 + input.c2;
    temp347625 = temp347745 * input.x;
    temp347448 = input.c1 + temp347625;
    temp347319 = input.x * temp347448;
    output.e = temp347319 + input.c0;
    output.e = output.e;

    
}
} // namespace control
