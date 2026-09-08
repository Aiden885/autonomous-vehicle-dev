#include "run_db765728_3454_4e75_86e9_1f8380521b21.hpp"
namespace control {
/**
 * @brief 根据目标速度与自车速度误差计算纵向加速度命令，执行加减速度限幅和控制使能门控。
 */
void run_db765728_3454_4e75_86e9_1f8380521b21::run(const Input &input, Output &output)
{
    
    const double accelerationGainPerS = 0.6;  // 常量
    const double minAccelerationMps2 = -6;  // 常量
    const double maxAccelerationMps2 = 1.5;  // 常量
    
    Real temp139938;
    Real temp140067;
    Real temp141270;
    Real temp142526;
    
    
    temp142526 = input.targetSpeedMps - input.egoSpeedMps;
    temp139938 = temp142526 * accelerationGainPerS;
    /* 浮点最大值 */
    temp140067 = fmax(temp139938, minAccelerationMps2);

    /* 浮点最小值 */
    temp141270 = fmin(temp140067, maxAccelerationMps2);

    output.accelerationCommandMps2 = temp141270 * input.isControlEnabled;

    
}
} // namespace control