#include "composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7::composite_block(const Input &input, Output &output)
{
    
    const double 0.25 = 0.25;  // 常量
    
    Real temp364326;
    Real temp364556;
    Real temp364877;
    Real temp365049;
    Real temp365365;
    Real temp365497;
    Real temp365647;
    Real temp365971;
    Real temp366046;
    Real temp367859;
    
    
    temp364326 = input.egoV * input.egoV;
    temp365971 = temp366950 * input.weightedError;
    temp367859 = temp367966 * temp367952;
    /* VSqSafe */
    temp364556 = fmax(temp364326, 0.25);

    temp364877 = temp367859 / temp364556;
    /* AtanPhi */
    temp365049 = atan(temp364877);

    temp366046 = temp365049 / temp368697;
    temp365497 = -temp366046;
    /* FminUp */
    temp365365 = fmin(temp365971, temp366046);

    /* FmaxLo */
    temp365647 = fmax(temp365365, temp365497);

    output.steerRad = temp365647 * temp367363;
    output.steerRad = output.steerRad;

    
}
} // namespace control