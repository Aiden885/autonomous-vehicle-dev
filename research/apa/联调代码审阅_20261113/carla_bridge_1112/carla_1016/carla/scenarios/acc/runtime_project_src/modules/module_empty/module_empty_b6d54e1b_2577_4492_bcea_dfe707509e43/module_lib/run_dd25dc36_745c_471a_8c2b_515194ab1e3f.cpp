#include "run_dd25dc36_745c_471a_8c2b_515194ab1e3f.hpp"
namespace control {
/**
 * @brief 根据前车状态、跟车误差和决策参数计算ACC目标速度。
 */
void run_dd25dc36_745c_471a_8c2b_515194ab1e3f::run(const Input &input, Output &output)
{
    
    
    
    
    run_ab2e0a9d_5eb7_4919_9ca7_de51932abea5Traits::Input calculateRelativeSpeedFeedback_1_in;
    calculateRelativeSpeedFeedback_1_in.egoSpeedMps = input.egoSpeedMps;
    calculateRelativeSpeedFeedback_1_in.leadSpeedMps = input.leadSpeedMps;
    calculateRelativeSpeedFeedback_1_in.relativeSpeedGain = input.relativeSpeedGain;
    run_ab2e0a9d_5eb7_4919_9ca7_de51932abea5Traits::Output calculateRelativeSpeedFeedback_1_out;

    sub_.calculateRelativeSpeedFeedback_1.run(calculateRelativeSpeedFeedback_1_in, calculateRelativeSpeedFeedback_1_out);
    run_a5e68afa_fef4_4c76_9139_cb7fef8f0aa2Traits::Input calculateSpacingFeedback_1_in;
    calculateSpacingFeedback_1_in.egoSpeedMps = input.egoSpeedMps;
    calculateSpacingFeedback_1_in.timeGapS = input.timeGapS;
    calculateSpacingFeedback_1_in.minDistanceM = input.minDistanceM;
    calculateSpacingFeedback_1_in.leadDistanceM = input.leadDistanceM;
    calculateSpacingFeedback_1_in.distanceErrorGainPerS = input.distanceErrorGainPerS;
    run_a5e68afa_fef4_4c76_9139_cb7fef8f0aa2Traits::Output calculateSpacingFeedback_1_out;

    sub_.calculateSpacingFeedback_1.run(calculateSpacingFeedback_1_in, calculateSpacingFeedback_1_out);
    run_51f83135_f79a_42dc_8c89_9463090201b8Traits::Input generateTargetSpeed_1_in;
    generateTargetSpeed_1_in.leadSpeedMps = input.leadSpeedMps;
    generateTargetSpeed_1_in.distanceFeedbackMps = calculateSpacingFeedback_1_out.distanceFeedbackMps;
    generateTargetSpeed_1_in.relativeSpeedFeedbackMps = calculateRelativeSpeedFeedback_1_out.relativeSpeedFeedbackMps;
    generateTargetSpeed_1_in.maxSpeedMps = input.maxSpeedMps;
    generateTargetSpeed_1_in.isControlEnabled = input.isControlEnabled;
    run_51f83135_f79a_42dc_8c89_9463090201b8Traits::Output generateTargetSpeed_1_out;

    sub_.generateTargetSpeed_1.run(generateTargetSpeed_1_in, generateTargetSpeed_1_out);
    run_f4207eef_c51d_5cc8_b12e_c2930682ae01Traits::Input calculateAccelerationCommand_1_in;
    calculateAccelerationCommand_1_in.targetSpeedMps = generateTargetSpeed_1_out.targetSpeedMps;
    calculateAccelerationCommand_1_in.egoSpeedMps = input.egoSpeedMps;
    calculateAccelerationCommand_1_in.isControlEnabled = input.isControlEnabled;
    run_f4207eef_c51d_5cc8_b12e_c2930682ae01Traits::Output calculateAccelerationCommand_1_out;

    sub_.calculateAccelerationCommand_1.run(calculateAccelerationCommand_1_in, calculateAccelerationCommand_1_out);
    output.targetSpeedMps = calculateAccelerationCommand_1_out.accelerationCommandMps2;

    
}
} // namespace control