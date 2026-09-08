#include "run_96195785_aad3_4e07_96eb_347ef47d3318.hpp"
namespace control {
/**
 * @brief 根据前车状态、跟车误差和决策参数计算ACC目标速度。
 */
void run_96195785_aad3_4e07_96eb_347ef47d3318::run(const Input &input, Output &output)
{
    
    
    
    
    run_fa2b2535_e243_4fd1_904f_ea508b90a4efTraits::Input calculateSpacingFeedback_2_in;
    calculateSpacingFeedback_2_in.egoSpeedMps = input.egoSpeedMps;
    calculateSpacingFeedback_2_in.timeGapS = input.timeGapS;
    calculateSpacingFeedback_2_in.minDistanceM = input.minDistanceM;
    calculateSpacingFeedback_2_in.leadDistanceM = input.leadDistanceM;
    calculateSpacingFeedback_2_in.distanceErrorGainPerS = input.distanceErrorGainPerS;
    run_fa2b2535_e243_4fd1_904f_ea508b90a4efTraits::Output calculateSpacingFeedback_2_out;

    sub_.calculateSpacingFeedback_2.run(calculateSpacingFeedback_2_in, calculateSpacingFeedback_2_out);
    run_8f7b5338_baaf_44cb_ab98_c448a82d8b2bTraits::Input calculateRelativeSpeedFeedback_2_in;
    calculateRelativeSpeedFeedback_2_in.egoSpeedMps = input.egoSpeedMps;
    calculateRelativeSpeedFeedback_2_in.leadSpeedMps = input.leadSpeedMps;
    calculateRelativeSpeedFeedback_2_in.relativeSpeedGain = input.relativeSpeedGain;
    run_8f7b5338_baaf_44cb_ab98_c448a82d8b2bTraits::Output calculateRelativeSpeedFeedback_2_out;

    sub_.calculateRelativeSpeedFeedback_2.run(calculateRelativeSpeedFeedback_2_in, calculateRelativeSpeedFeedback_2_out);
    run_53728aa4_1a0c_4dbf_b2ed_63c9c2a4016aTraits::Input generateTargetSpeed_2_in;
    generateTargetSpeed_2_in.leadSpeedMps = input.leadSpeedMps;
    generateTargetSpeed_2_in.distanceFeedbackMps = calculateSpacingFeedback_2_out.distanceFeedbackMps;
    generateTargetSpeed_2_in.relativeSpeedFeedbackMps = calculateRelativeSpeedFeedback_2_out.relativeSpeedFeedbackMps;
    generateTargetSpeed_2_in.maxSpeedMps = input.maxSpeedMps;
    generateTargetSpeed_2_in.isControlEnabled = input.isControlEnabled;
    run_53728aa4_1a0c_4dbf_b2ed_63c9c2a4016aTraits::Output generateTargetSpeed_2_out;

    sub_.generateTargetSpeed_2.run(generateTargetSpeed_2_in, generateTargetSpeed_2_out);
    run_db765728_3454_4e75_86e9_1f8380521b21Traits::Input calculateAccelerationCommand_2_in;
    calculateAccelerationCommand_2_in.targetSpeedMps = generateTargetSpeed_2_out.targetSpeedMps;
    calculateAccelerationCommand_2_in.egoSpeedMps = input.egoSpeedMps;
    calculateAccelerationCommand_2_in.isControlEnabled = input.isControlEnabled;
    run_db765728_3454_4e75_86e9_1f8380521b21Traits::Output calculateAccelerationCommand_2_out;

    sub_.calculateAccelerationCommand_2.run(calculateAccelerationCommand_2_in, calculateAccelerationCommand_2_out);
    output.targetSpeedMps = calculateAccelerationCommand_2_out.accelerationCommandMps2;

    
}
} // namespace control