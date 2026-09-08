#include "run_f4207eef_c51d_5cc8_b12e_c2930682ae01.hpp"
namespace control {
/**
 * @brief 根据目标速度与自车速度误差计算纵向加速度命令，执行加减速度限幅和控制使能门控。
 */
void run_f4207eef_c51d_5cc8_b12e_c2930682ae01::run(const Input &input, Output &output)
{
    
    const double accelerationGainPerS = 0.6;  // 常量
    const double minAccelerationMps2 = -6;  // 常量
    const double maxAccelerationMps2 = 1.5;  // 常量
    
    Real temp100647;
    Real temp100934;
    Real temp102261;
    Real temp103466;
    
    
    temp103466 = input.targetSpeedMps - input.egoSpeedMps;
    temp100647 = temp103466 * accelerationGainPerS;
    /* 浮点最大值 */
    temp100934 = fmax(temp100647, minAccelerationMps2);

    /* 浮点最小值 */
    temp102261 = fmin(temp100934, maxAccelerationMps2);

    output.accelerationCommandMps2 = temp102261 * input.isControlEnabled;

    
}
} // namespace control