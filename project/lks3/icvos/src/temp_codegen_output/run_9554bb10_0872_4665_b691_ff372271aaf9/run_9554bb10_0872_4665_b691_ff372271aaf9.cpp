#include "run_9554bb10_0872_4665_b691_ff372271aaf9.hpp"
#include "scope_push.h"

namespace control {
/**
 * @brief LKS决策与横向控制主流程。
 */
void run_9554bb10_0872_4665_b691_ff372271aaf9::run(const Input &input, Output &output)
{
    
    
    
    
    composite_block_empty_8a3df3a4_11ef_4cf0_9f3a_b380382dd3a9Traits::Input decision_in;
    decision_in.egoSpeedMps = input.egoSpeedMps;
    decision_in.isBrakePressed = input.isBrakePressed;
    decision_in.driverSteerNorm = input.driverSteerNorm;
    composite_block_empty_8a3df3a4_11ef_4cf0_9f3a_b380382dd3a9Traits::Output decision_out;

    sub_.decision.run(decision_in, decision_out);
    composite_block_empty_6b682752_6234_4817_a302_c9e173c6790eTraits::Input control_in;
    control_in.egoSpeedMps = input.egoSpeedMps;
    control_in.c0 = input.c0;
    control_in.c1 = input.c1;
    control_in.c2 = input.c2;
    control_in.c3 = input.c3;
    control_in.roadCurvaturePerM = input.roadCurvaturePerM;
    control_in.isControlEnabled = decision_out.isControlEnabled;
    composite_block_empty_6b682752_6234_4817_a302_c9e173c6790eTraits::Output control_out;

    sub_.control.run(control_in, control_out);
    /* 仿真示波器 */
    scope_push_send("8a62f3b7-f4d1-4a9f-96e7-f93831da6336", "input_port_1", "Real", control_out.steerRad, "input_port_2", "bool", decision_out.isControlEnabled);

    output.isControlEnabled = decision_out.isControlEnabled;
    output.steerRad = control_out.steerRad;

    
}
} // namespace control