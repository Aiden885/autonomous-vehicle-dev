#include "run_53728aa4_1a0c_4dbf_b2ed_63c9c2a4016a.hpp"
namespace control {
/**
 * @brief 合成前车速度和两项反馈修正量，执行速度限幅与控制使能门控。
 */
void run_53728aa4_1a0c_4dbf_b2ed_63c9c2a4016a::run(const Input &input, Output &output)
{
    
    const double zeroSpeed = 0;  // 常量
    
    Real temp126653;
    Real temp126843;
    Real temp127919;
    
    
    temp126653 = input.leadSpeedMps + input.distanceFeedbackMps + input.relativeSpeedFeedbackMps;
    /* 浮点最大值 */
    temp126843 = fmax(temp126653, zeroSpeed);

    /* 浮点最小值 */
    temp127919 = fmin(temp126843, input.maxSpeedMps);

    output.targetSpeedMps = temp127919 * input.isControlEnabled;

    
}
} // namespace control