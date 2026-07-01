#include "composite_block_empty_e92430b6_35aa_4184_bc4f_623e36299e44.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_e92430b6_35aa_4184_bc4f_623e36299e44::composite_block(const Input &input, Output &output)
{
    
    const double 0.25 = 0.25;  // 常量
    
    Real temp751887;
    Real temp751923;
    Real temp752160;
    Real temp752263;
    Real temp752481;
    Real temp752535;
    Real temp752693;
    Real temp752821;
    Real temp752926;
    Real steerRad;
    Real temp754178;
    
    
    temp751887 = egoV * egoV;
    temp752821 = temp753562 * weightedError;
    temp754178 = temp754256 * temp754366;
    /* 浮点最大值 */
    temp751923 = fmax(temp751887, 0.25);

    temp752160 = temp754178 / temp751923;
    /* 反正切 */
    temp752263 = atan(temp752160);

    temp752926 = temp752263 / temp754776;
    temp752535 = -temp752926;
    /* 浮点最小值 */
    temp752481 = fmin(temp752821, temp752926);

    /* 浮点最大值 */
    temp752693 = fmax(temp752481, temp752535);

    steerRad = temp752693 * temp753994;
    output.steerRad = steerRad;

    
}
} // namespace control