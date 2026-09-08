#include "run_23d2a150_83e4_470e_a9b7_189357a9dfd1.hpp"
#include "scope_push.h"

namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void run_23d2a150_83e4_470e_a9b7_189357a9dfd1::run(const Input &input, Output &output)
{
    
    
    
    
    composite_block_empty_0583d8c2_dfeb_41ff_8153_3eb79777b834Traits::Input decision_2_in;
    decision_2_in.commandType = input.commandType;
    decision_2_in.egoSpeedMps = input.egoSpeedMps;
    composite_block_empty_0583d8c2_dfeb_41ff_8153_3eb79777b834Traits::Output decision_2_out;

    sub_.decision_2.run(decision_2_in, decision_2_out);
    run_96195785_aad3_4e07_96eb_347ef47d3318Traits::Input control_2_in;
    control_2_in.egoSpeedMps = input.egoSpeedMps;
    control_2_in.leadSpeedMps = input.leadSpeedMps;
    control_2_in.leadDistanceM = input.leadDistanceM;
    control_2_in.timeGapS = decision_2_out.timeGapS;
    control_2_in.maxSpeedMps = decision_2_out.maxSpeedMps;
    control_2_in.isControlEnabled = decision_2_out.isControlEnabled;
    control_2_in.minDistanceM = param_.minDistanceM;
    control_2_in.distanceErrorGainPerS = param_.distanceErrorGainPerS;
    control_2_in.relativeSpeedGain = param_.relativeSpeedGain;
    run_96195785_aad3_4e07_96eb_347ef47d3318Traits::Output control_2_out;

    sub_.control_2.run(control_2_in, control_2_out);
    /* 仿真示波器 */
    scope_push_send("e1f8763b-630b-47d0-b3a4-b906b1530df8", "Component_merge_input_1", "Real", control_2_out.targetSpeedMps, "Component_merge_input_2", "double", input.egoSpeedMps, "Component_merge_input_3", "double", input.leadSpeedMps, "Component_merge_input_4", "double", input.leadDistanceM);

    output.isControlEnabled = decision_2_out.isControlEnabled;
    output.targetSpeedMps = control_2_out.targetSpeedMps;

    
}
} // namespace control