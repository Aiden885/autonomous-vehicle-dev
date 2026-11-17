#include "composite_block_empty_179b05d8_8bbd_4e95_b179_c4e4788576c4.hpp"
namespace control {
/**
 * @brief 根据自车速度和最低工作速度判断LKS速度条件是否满足。
 */
void composite_block_empty_179b05d8_8bbd_4e95_b179_c4e4788576c4::run(const Input &input, Output &output)
{
    
    
    Real temp714831;
    
    
    temp714831 = input.egoSpeedMps >= global::params.lks_vMin;
    output.isSpeedUnavailable = !temp714831;

    
}
} // namespace control