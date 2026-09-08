#include "composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560.hpp"
namespace control {
/**
 * @brief 根据车速、轴距和横向加速度上限限制归一化转向指令。
 */
void composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560::run(const Input &input, Output &output)
{
    
    const double MINIMUM_SPEED_SQUARED_M2PS2 = 0.25;  // 常量
    
    Real temp801893;
    Real temp802032;
    Real temp802272;
    Real temp802327;
    Real temp802645;
    Real temp802715;
    Real temp803125;
    Real temp804091;
    
    
    temp804091 = global::params.lks_wheelBase * global::params.lks_ayMax;
    temp801893 = input.egoSpeedMps * input.egoSpeedMps;
    /* 浮点最大值 */
    temp802032 = fmax(temp801893, MINIMUM_SPEED_SQUARED_M2PS2);

    temp802272 = temp804091 / temp802032;
    /* 反正切 */
    temp802327 = atan(temp802272);

    temp803125 = temp802327 / global::params.lks_frontWheelMaxRad;
    temp802715 = -temp803125;
    /* 浮点最小值 */
    temp802645 = fmin(input.rawSteerNorm, temp803125);

    /* 浮点最大值 */
    output.limitedSteerNorm = fmax(temp802645, temp802715);


    
}
} // namespace control