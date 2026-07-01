#include "composite_block_empty_376e82ac_81e2_4f82_91cc_5c118b024c83.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_376e82ac_81e2_4f82_91cc_5c118b024c83::composite_block(const Input &input, Output &output)
{
    
    const double One = 1;  // 常量
    
    Real temp740864;
    Real previewDistance;
    Real temp741227;
    Real temp741550;
    Real temp741725;
    Real temp741978;
    Real temp742138;
    Real temp742398;
    
    
    temp741978 = One - temp742035;
    /* 浮点绝对值 */
    temp740864 = fabs(curvature);

    temp741227 = temp741316 * egoV;
    temp741725 = temp740864 >= temp741818;
    temp741550 = temp741567 + temp741227;
    temp742138 = temp741725 * temp741978;
    temp742398 = One - temp742138;
    previewDistance = temp741550 * temp742398;
    output.previewDistance = previewDistance;

    
}
} // namespace control