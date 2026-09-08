#include "run_51f83135_f79a_42dc_8c89_9463090201b8.hpp"
namespace control {
/**
 * @brief 合成前车速度和两项反馈修正量，执行速度限幅与控制使能门控。
 */
void run_51f83135_f79a_42dc_8c89_9463090201b8::run(const Input &input, Output &output)
{
    
    const double zeroSpeed = 0;  // 常量
    
    Real temp109669;
    Real temp109872;
    Real temp111091;
    
    
    temp109669 = input.leadSpeedMps + input.distanceFeedbackMps + input.relativeSpeedFeedbackMps;
    /* 浮点最大值 */
    temp109872 = fmax(temp109669, zeroSpeed);

    /* 浮点最小值 */
    temp111091 = fmin(temp109872, input.maxSpeedMps);

    output.targetSpeedMps = temp111091 * input.isControlEnabled;

    
}
} // namespace control