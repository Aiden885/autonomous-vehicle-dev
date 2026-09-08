#include "composite_block_empty_0583d8c2_dfeb_41ff_8153_3eb79777b834.hpp"
namespace control {
/**
 * @brief 根据车辆状态和驾驶指令更新ACC状态、目标时距与速度上限。
 */
void composite_block_empty_0583d8c2_dfeb_41ff_8153_3eb79777b834::run(const Input &input, Output &output)
{
    
    
    int temp069386;
    
    
    run_b71d71c8_ff00_4f2b_b015_08ac387673a6Traits::Input classifySystemState_2_in;
    classifySystemState_2_in.egoSpeedMps = input.egoSpeedMps;
    classifySystemState_2_in.minimumOperatingSpeedMps = param_.minimumOperatingSpeedMps;
    classifySystemState_2_in.isControlActive = state_.isControlActive;
    classifySystemState_2_in.hasControlHistory = state_.hasControlHistory;
    run_b71d71c8_ff00_4f2b_b015_08ac387673a6Traits::Output classifySystemState_2_out;

    sub_.classifySystemState_2.run(classifySystemState_2_in, classifySystemState_2_out);
    /* 真值表 */
    temp069386 = truth_table_a5de6758_3b22_4570_950f_388d963d3433(classifySystemState_2_out.systemState, input.commandType);

    run_3d336890_b7fc_408b_ac40_17cd5dce2b59Traits::Input updateControlState_2_in;
    updateControlState_2_in.decisionCode = temp069386;
    updateControlState_2_in.commandType = input.commandType;
    updateControlState_2_in.isControlActive = state_.isControlActive;
    updateControlState_2_in.hasControlHistory = state_.hasControlHistory;
    run_3d336890_b7fc_408b_ac40_17cd5dce2b59Traits::Output updateControlState_2_out;

    sub_.updateControlState_2.run(updateControlState_2_in, updateControlState_2_out);
    run_c5c7aca9_1395_45ea_b8fe_138f53906472Traits::Input updateTimeGap_2_in;
    updateTimeGap_2_in.decisionCode = temp069386;
    updateTimeGap_2_in.timeGapS = state_.timeGapS;
    updateTimeGap_2_in.timeGapStepS = param_.timeGapStepS;
    updateTimeGap_2_in.minTimeGapS = param_.minTimeGapS;
    updateTimeGap_2_in.maxTimeGapS = param_.maxTimeGapS;
    run_c5c7aca9_1395_45ea_b8fe_138f53906472Traits::Output updateTimeGap_2_out;

    sub_.updateTimeGap_2.run(updateTimeGap_2_in, updateTimeGap_2_out);
    run_a15a0aa8_22ce_4f0e_8b20_5deb0a4478e5Traits::Input updateMaxSpeed_2_in;
    updateMaxSpeed_2_in.decisionCode = temp069386;
    updateMaxSpeed_2_in.maxSpeedMps = state_.maxSpeedMps;
    updateMaxSpeed_2_in.speedStepMps = param_.speedStepMps;
    updateMaxSpeed_2_in.minSpeedMps = param_.minSpeedMps;
    run_a15a0aa8_22ce_4f0e_8b20_5deb0a4478e5Traits::Output updateMaxSpeed_2_out;

    sub_.updateMaxSpeed_2.run(updateMaxSpeed_2_in, updateMaxSpeed_2_out);
    state_.isControlActive = updateControlState_2_out.isControlActiveNext;
    state_.hasControlHistory = updateControlState_2_out.hasControlHistoryNext;
    state_.timeGapS = updateTimeGap_2_out.nextTimeGapS;
    state_.maxSpeedMps = updateMaxSpeed_2_out.nextMaxSpeedMps;
    output.isControlEnabled = classifySystemState_2_out.isControlEnabled;
    output.timeGapS = updateTimeGap_2_out.nextTimeGapS;
    output.maxSpeedMps = updateMaxSpeed_2_out.nextMaxSpeedMps;

    
}
} // namespace control