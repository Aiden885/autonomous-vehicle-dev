#include "run_2f3edfa5_8227_43eb_859d_1a6db0afd957.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void run_2f3edfa5_8227_43eb_859d_1a6db0afd957::run(const Input &input, Output &output)
{
    (void)input;
    (void)output;
    
    const double C_ZeroSpeed = 0;  // 常量
    
    Real temp195871;
    Real temp196110;
    Real temp196413;
    Real temp196675;
    Real temp196815;
    Real temp197010;
    Real temp197474;
    Real temp197625;
    int tempDriverCommand;
    Real temp198623;
    Real temp199092;
    Real temp199243;
    Real temp199657;
    const Real temp196788 = global_.MinDistance;
    const Real temp199342 = global_.Kdist;
    const Real temp199951 = global_.Kspeed;
    
    
    /* CARLA驾驶指令 */
    tempDriverCommand = CARLAACCDriverCommand();

    /* CARLA前车速度 */
    temp196110 = CARLAACCLeadSpeed();

    /* CARLA自车速度 */
    temp197010 = CARLAACCEgoSpeed();

    /* CARLA前车距离 */
    temp195871 = CARLAACCLeadDistance();

    composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988adTraits::Input compositeBlockInstance_2_in;
    composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988adTraits::Output compositeBlockInstance_2_out;
    compositeBlockInstance_2_in.commandType = tempDriverCommand;
    compositeBlockInstance_2_in.egoV = temp197010;

    sub_.compositeBlockInstance_2.run(compositeBlockInstance_2_in, compositeBlockInstance_2_out);
    temp199092 = temp196110 - temp197010;
    temp196675 = temp197010 * compositeBlockInstance_2_out.timeGap;
    temp199657 = temp199092 * temp199951;
    /* DesiredDistance */
    temp196815 = fmax(temp196675, temp196788);

    temp198623 = temp195871 - temp196815;
    temp199243 = temp198623 * temp199342;
    temp197474 = temp199243 + temp199657 + temp196110;
    /* 浮点最大值 */
    temp197625 = fmax(temp197474, C_ZeroSpeed);

    /* 浮点最小值 */
    temp196413 = fmin(temp197625, compositeBlockInstance_2_out.maxSpeed);

    /* CARLA纵向控制命令 */
    CARLAACCLongitudinalCmd(temp196413, compositeBlockInstance_2_out.enable);


    
}
} // namespace control
