#include "c_process_4f6d0719_82ad_45c7_85bc_52f4ac9d4271.hpp"
#include "scope_push.h"


namespace control {

void c_process_4f6d0719_82ad_45c7_85bc_52f4ac9d4271::run(const Input &input, Output &output)
{
    (void)input;
    (void)output;

    const double C_ZeroSpeed = 0;  // 常量

    Real temp753020;
    Real temp753317;
    Real temp753634;
    Real temp754110;
    Real temp755140;
    Real temp755454;
    Real temp755797;
    Real temp755994;
    Real C++_None;
    Real temp757638;
    Real temp757988;
    Real temp758384;
    Real temp758644;

    /* CARLA前车速度 */
    temp753317 = CARLAACCLeadSpeed();

    /* CARLA前车距离 */
    temp753020 = CARLAACCLeadDistance();


    /* CARLA驾驶指令 */
    C++_None = CARLAACCDriverCommand();

    /* CARLA自车速度 */
    temp754110 = CARLAACCEgoSpeed();



    composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0Traits::Input compositeBlockInstance_1_in;
    composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0Traits::Output compositeBlockInstance_1_out;

    sub_.compositeBlockInstance_1.composite_block(compositeBlockInstance_1_in, compositeBlockInstance_1_out);
    temp758384 = temp753317 - temp754110;
    temp755140 = temp754110 * compositeBlockInstance_1_out.timeGap;
    temp758644 = temp758384 * temp758829;
    /* 浮点最大值 */
    temp755454 = fmax(temp755140, temp755383);

    temp757638 = temp753020 - temp755454;
    temp757988 = temp757638 * temp758133;
    temp755797 = temp757988 + temp758644 + temp753317;
    /* 浮点最大值 */
    temp755994 = fmax(temp755797, C_ZeroSpeed);

    /* 浮点最小值 */
    temp753634 = fmin(temp755994, compositeBlockInstance_1_out.maxSpeed);

    /* CARLA纵向控制命令 */
    CARLAACCLongitudinalCmd(temp753634, compositeBlockInstance_1_out.enable);

    /* 仿真示波器 */
    scope_push_send("f4e6f770-f387-459c-8fbf-c37b150ec6b9", "Component_merge_input_1", "Real", temp753634, "Component_merge_input_2", "Real", temp754110, "Component_merge_input_3", "double", temp753317, "Component_merge_input_4", "double", temp753020);
}

} // namespace control
