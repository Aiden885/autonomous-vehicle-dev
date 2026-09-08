#include "run_8f7b5338_baaf_44cb_ab98_c448a82d8b2b.hpp"
namespace control {
/**
 * @brief 根据前车与自车的相对速度形成速度反馈修正量。
 */
void run_8f7b5338_baaf_44cb_ab98_c448a82d8b2b::run(const Input &input, Output &output)
{
    
    
    Real temp125012;
    
    
    temp125012 = input.leadSpeedMps - input.egoSpeedMps;
    output.relativeSpeedFeedbackMps = temp125012 * input.relativeSpeedGain;

    
}
} // namespace control