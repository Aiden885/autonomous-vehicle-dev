#include "c_process_4f6d0719_82ad_45c7_85bc_52f4ac9d4271.hpp"

namespace control {

void c_process_4f6d0719_82ad_45c7_85bc_52f4ac9d4271::run(const Input &input, Output &output)
{
    (void)input;
    (void)output;

    const double C_ZeroSpeed = 0;  // 常量

    Real temp328480;
    Real temp328749;
    Real temp329185;
    Real temp329674;
    Real temp330286;
    Real temp330419;
    Real temp330796;
    Real temp330925;
    Real C++_None;
    Real temp332923;
    Real temp333642;
    Real temp334435;
    Real temp334825;

    /* CARLA前车速度 */
    temp328749 = CARLAACCLeadSpeed();

    /* CARLA前车距离 */
    temp328480 = CARLAACCLeadDistance();


    /* CARLA驾驶指令 */
    C++_None = CARLAACCDriverCommand();

    /* CARLA自车速度 */
    temp329674 = CARLAACCEgoSpeed();



    composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0Traits::Input compositeBlockInstance_1_in;
    composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0Traits::Output compositeBlockInstance_1_out;

    sub_.compositeBlockInstance_1.composite_block(compositeBlockInstance_1_in, compositeBlockInstance_1_out);
    temp334435 = temp328749 - temp329674;
    temp330286 = temp329674 * compositeBlockInstance_1_out.timeGap;
    temp334825 = temp334435 * temp335060;
    /* 浮点最大值 */
    temp330419 = fmax(temp330286, temp330330);

    temp332923 = temp328480 - temp330419;
    temp333642 = temp332923 * temp333810;
    temp330796 = temp333642 + temp334825 + temp328749;
    /* 浮点最大值 */
    temp330925 = fmax(temp330796, C_ZeroSpeed);

    /* 浮点最小值 */
    temp329185 = fmin(temp330925, compositeBlockInstance_1_out.maxSpeed);

    /* CARLA纵向控制命令 */
    CARLAACCLongitudinalCmd(temp329185, compositeBlockInstance_1_out.enable);
}

} // namespace control
