#include "run_f1807d24_61ef_407b_932e_4df61bda48e7.hpp"
#include "scope_push.h"

namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void run_f1807d24_61ef_407b_932e_4df61bda48e7::run(const Input &input, Output &output)
{
    
    
    
    
    composite_block_empty_797d2bad_bc58_4269_9d44_d338e3691a62Traits::Input decision_1_in;
    decision_1_in.commandType = input.commandType;
    decision_1_in.egoSpeedMps = input.egoSpeedMps;
    composite_block_empty_797d2bad_bc58_4269_9d44_d338e3691a62Traits::Output decision_1_out;

    sub_.decision_1.run(decision_1_in, decision_1_out);
    run_dd25dc36_745c_471a_8c2b_515194ab1e3fTraits::Input control_1_in;
    control_1_in.egoSpeedMps = input.egoSpeedMps;
    control_1_in.leadSpeedMps = input.leadSpeedMps;
    control_1_in.leadDistanceM = input.leadDistanceM;
    control_1_in.timeGapS = decision_1_out.timeGapS;
    control_1_in.maxSpeedMps = decision_1_out.maxSpeedMps;
    control_1_in.isControlEnabled = decision_1_out.isControlEnabled;
    control_1_in.minDistanceM = param_.minDistanceM;
    control_1_in.distanceErrorGainPerS = param_.distanceErrorGainPerS;
    control_1_in.relativeSpeedGain = param_.relativeSpeedGain;
    run_dd25dc36_745c_471a_8c2b_515194ab1e3fTraits::Output control_1_out;

    sub_.control_1.run(control_1_in, control_1_out);
    /* 仿真示波器 */
    scope_push_send("e3bedff2-91c3-4ccb-96f6-a2301ebf2edd", "Component_merge_input_1", "Real", control_1_out.targetSpeedMps, "Component_merge_input_2", "double", input.egoSpeedMps, "Component_merge_input_3", "double", input.leadSpeedMps, "Component_merge_input_4", "double", input.leadDistanceM);

    output.isControlEnabled = decision_1_out.isControlEnabled;
    output.targetSpeedMps = control_1_out.targetSpeedMps;

    
}
} // namespace control