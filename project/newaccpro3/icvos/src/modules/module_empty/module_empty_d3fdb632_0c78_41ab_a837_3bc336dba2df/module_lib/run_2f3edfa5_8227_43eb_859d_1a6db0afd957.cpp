#include "run_2f3edfa5_8227_43eb_859d_1a6db0afd957.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void run_2f3edfa5_8227_43eb_859d_1a6db0afd957::run(const Input &input, Output &output)
{
    
    const double C_ZeroSpeed = 0;  // 常量
    
    Real temp345745;
    Real temp346238;
    Real temp346694;
    Real temp347070;
    Real temp347181;
    Real temp347464;
    Real temp348118;
    Real temp348379;
    Real C++_None;
    Real temp349624;
    Real temp350011;
    Real temp350210;
    Real temp350611;
    
    
    /* CARLA驾驶指令 */
    C++_None = CARLAACCDriverCommand();

    /* CARLA前车速度 */
    temp346238 = CARLAACCLeadSpeed();

    /* CARLA自车速度 */
    temp347464 = CARLAACCEgoSpeed();

    /* CARLA前车距离 */
    temp345745 = CARLAACCLeadDistance();

    composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988adTraits::Input compositeBlockInstance_2_in;
    composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988adTraits::Output compositeBlockInstance_2_out;

    sub_.compositeBlockInstance_2.composite_block(compositeBlockInstance_2_in, compositeBlockInstance_2_out);
    temp350011 = temp346238 - temp347464;
    temp347070 = temp347464 * compositeBlockInstance_2_out.timeGap;
    temp350611 = temp350011 * temp351172;
    /* DesiredDistance */
    temp347181 = fmax(temp347070, temp347055);

    temp349624 = temp345745 - temp347181;
    temp350210 = temp349624 * temp350453;
    temp348118 = temp350210 + temp350611 + temp346238;
    /* 浮点最大值 */
    temp348379 = fmax(temp348118, C_ZeroSpeed);

    /* 浮点最小值 */
    temp346694 = fmin(temp348379, compositeBlockInstance_2_out.maxSpeed);

    /* CARLA纵向控制命令 */
    CARLAACCLongitudinalCmd(temp346694, compositeBlockInstance_2_out.enable);


    
}
} // namespace control