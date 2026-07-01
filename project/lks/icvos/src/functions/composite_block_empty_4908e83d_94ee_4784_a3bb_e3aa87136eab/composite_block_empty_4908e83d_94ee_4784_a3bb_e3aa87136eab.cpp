#include "composite_block_empty_4908e83d_94ee_4784_a3bb_e3aa87136eab.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_4908e83d_94ee_4784_a3bb_e3aa87136eab::composite_block(const Input &input, Output &output)
{
    
    
    Real e;
    Real temp761325;
    Real temp761417;
    Real temp761637;
    Real temp761657;
    Real temp761840;
    
    
    temp761840 = c3 * x;
    temp761657 = temp761840 + c2;
    temp761637 = temp761657 * x;
    temp761417 = c1 + temp761637;
    temp761325 = x * temp761417;
    e = temp761325 + c0;
    output.e = e;

    
}
} // namespace control