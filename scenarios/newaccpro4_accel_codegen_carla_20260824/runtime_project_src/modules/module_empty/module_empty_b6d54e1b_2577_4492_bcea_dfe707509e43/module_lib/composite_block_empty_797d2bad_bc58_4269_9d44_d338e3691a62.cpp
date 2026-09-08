#include "composite_block_empty_797d2bad_bc58_4269_9d44_d338e3691a62.hpp"
namespace control {
/**
 * @brief 根据车辆状态和驾驶指令更新ACC状态、目标时距与速度上限。
 */
void composite_block_empty_797d2bad_bc58_4269_9d44_d338e3691a62::run(const Input &input, Output &output)
{
    
    
    int temp120058;
    
    
    run_b12be11b_48bd_4d14_be5e_a2496219d1f7Traits::Input classifySystemState_1_in;
    classifySystemState_1_in.egoSpeedMps = input.egoSpeedMps;
    classifySystemState_1_in.minimumOperatingSpeedMps = param_.minimumOperatingSpeedMps;
    classifySystemState_1_in.isControlActive = state_.isControlActive;
    classifySystemState_1_in.hasControlHistory = state_.hasControlHistory;
    run_b12be11b_48bd_4d14_be5e_a2496219d1f7Traits::Output classifySystemState_1_out;

    sub_.classifySystemState_1.run(classifySystemState_1_in, classifySystemState_1_out);
    /* 真值表 */
    temp120058 = truth_table_8d98fa7f_4daf_49ae_bb29_69662b8ce656(classifySystemState_1_out.systemState, input.commandType);

    run_008d73bb_e84f_46a0_b967_18007ec5678eTraits::Input updateControlState_1_in;
    updateControlState_1_in.decisionCode = temp120058;
    updateControlState_1_in.commandType = input.commandType;
    updateControlState_1_in.isControlActive = state_.isControlActive;
    updateControlState_1_in.hasControlHistory = state_.hasControlHistory;
    run_008d73bb_e84f_46a0_b967_18007ec5678eTraits::Output updateControlState_1_out;

    sub_.updateControlState_1.run(updateControlState_1_in, updateControlState_1_out);
    run_a08264b6_d877_4a94_8756_9c8b9c5fde57Traits::Input updateTimeGap_1_in;
    updateTimeGap_1_in.decisionCode = temp120058;
    updateTimeGap_1_in.timeGapS = state_.timeGapS;
    updateTimeGap_1_in.timeGapStepS = param_.timeGapStepS;
    updateTimeGap_1_in.minTimeGapS = param_.minTimeGapS;
    updateTimeGap_1_in.maxTimeGapS = param_.maxTimeGapS;
    run_a08264b6_d877_4a94_8756_9c8b9c5fde57Traits::Output updateTimeGap_1_out;

    sub_.updateTimeGap_1.run(updateTimeGap_1_in, updateTimeGap_1_out);
    run_863e2a9d_edaf_4991_8be8_f0d5354e6a16Traits::Input updateMaxSpeed_1_in;
    updateMaxSpeed_1_in.decisionCode = temp120058;
    updateMaxSpeed_1_in.maxSpeedMps = state_.maxSpeedMps;
    updateMaxSpeed_1_in.speedStepMps = param_.speedStepMps;
    updateMaxSpeed_1_in.minSpeedMps = param_.minSpeedMps;
    run_863e2a9d_edaf_4991_8be8_f0d5354e6a16Traits::Output updateMaxSpeed_1_out;

    sub_.updateMaxSpeed_1.run(updateMaxSpeed_1_in, updateMaxSpeed_1_out);
    state_.isControlActive = updateControlState_1_out.isControlActiveNext;
    state_.hasControlHistory = updateControlState_1_out.hasControlHistoryNext;
    state_.timeGapS = updateTimeGap_1_out.nextTimeGapS;
    state_.maxSpeedMps = updateMaxSpeed_1_out.nextMaxSpeedMps;
    output.isControlEnabled = classifySystemState_1_out.isControlEnabled;
    output.timeGapS = updateTimeGap_1_out.nextTimeGapS;
    output.maxSpeedMps = updateMaxSpeed_1_out.nextMaxSpeedMps;

    
}
} // namespace control