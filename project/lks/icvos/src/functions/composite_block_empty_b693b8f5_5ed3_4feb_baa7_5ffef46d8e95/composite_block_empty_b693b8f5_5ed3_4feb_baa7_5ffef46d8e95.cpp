#include "composite_block_empty_b693b8f5_5ed3_4feb_baa7_5ffef46d8e95.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_b693b8f5_5ed3_4feb_baa7_5ffef46d8e95::composite_block(const Input &input, Output &output)
{
    
    
    Real temp737084;
    Real controlEnabled;
    Real temp737845;
    Real temp738163;
    Real temp738333;
    Real temp738593;
    
    
    /* 浮点绝对值 */
    temp737084 = fabs(driverSteerNorm);

    temp737845 = egoV >= temp738045;
    temp738333 = temp737084 >= temp738485;
    temp738163 = !temp737845;
    temp738593 = brakePressed || temp738333 || temp738163;
    controlEnabled = !temp738593;
    output.controlEnabled = controlEnabled;

    
}
} // namespace control