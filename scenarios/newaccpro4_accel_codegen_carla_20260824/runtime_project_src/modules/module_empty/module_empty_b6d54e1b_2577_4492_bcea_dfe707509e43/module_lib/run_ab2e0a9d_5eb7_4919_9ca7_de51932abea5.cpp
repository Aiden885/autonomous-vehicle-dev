#include "run_ab2e0a9d_5eb7_4919_9ca7_de51932abea5.hpp"
namespace control {
/**
 * @brief 根据前车与自车的相对速度形成速度反馈修正量。
 */
void run_ab2e0a9d_5eb7_4919_9ca7_de51932abea5::run(const Input &input, Output &output)
{
    
    
    Real temp107957;
    
    
    temp107957 = input.leadSpeedMps - input.egoSpeedMps;
    output.relativeSpeedFeedbackMps = temp107957 * input.relativeSpeedGain;

    
}
} // namespace control