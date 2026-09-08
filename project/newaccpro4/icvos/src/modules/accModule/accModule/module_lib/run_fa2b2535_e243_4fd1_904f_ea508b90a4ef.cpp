#include "run_fa2b2535_e243_4fd1_904f_ea508b90a4ef.hpp"
namespace control {
/**
 * @brief 根据目标时距计算期望跟车距离，并形成距离误差反馈修正量。
 */
void run_fa2b2535_e243_4fd1_904f_ea508b90a4ef::run(const Input &input, Output &output)
{
    
    
    Real temp121695;
    Real temp121777;
    Real temp123236;
    
    
    temp121695 = input.egoSpeedMps * input.timeGapS;
    /* 浮点最大值 */
    temp121777 = fmax(temp121695, input.minDistanceM);

    temp123236 = input.leadDistanceM - temp121777;
    output.distanceFeedbackMps = temp123236 * input.distanceErrorGainPerS;

    
}
} // namespace control