#include "run_a5e68afa_fef4_4c76_9139_cb7fef8f0aa2.hpp"
namespace control {
/**
 * @brief 根据目标时距计算期望跟车距离，并形成距离误差反馈修正量。
 */
void run_a5e68afa_fef4_4c76_9139_cb7fef8f0aa2::run(const Input &input, Output &output)
{
    
    
    Real temp146167;
    Real temp146239;
    Real temp147651;
    
    
    temp146167 = input.egoSpeedMps * input.timeGapS;
    /* 浮点最大值 */
    temp146239 = fmax(temp146167, input.minDistanceM);

    temp147651 = input.leadDistanceM - temp146239;
    output.distanceFeedbackMps = temp147651 * input.distanceErrorGainPerS;

    
}
} // namespace control