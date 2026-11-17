#include "composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560.hpp"
namespace control {
/**
 * @brief 根据车速、轴距和横向加速度上限限制归一化转向指令。
 */
void composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560::run(const Input &input, Output &output)
{
    
    const double MINIMUM_SPEED_SQUARED_M2PS2 = 0.25;  // 常量
    
    Real temp701671;
    Real temp701871;
    Real temp702067;
    Real temp702164;
    Real temp702464;
    Real temp702432;
    Real temp702837;
    Real temp703669;
    
    
    temp703669 = global::params.lks_wheelBase * global::params.lks_ayMax;
    temp701671 = input.egoSpeedMps * input.egoSpeedMps;
    /* applyMinimumSpeedSquare */
    temp701871 = fmax(temp701671, MINIMUM_SPEED_SQUARED_M2PS2);

    temp702067 = temp703669 / temp701871;
    /* calculateSteerLimitRad */
    temp702164 = atan(temp702067);

    temp702837 = temp702164 / global::params.lks_frontWheelMaxRad;
    temp702432 = -temp702837;
    /* applyUpperSteerLimit */
    temp702464 = fmin(input.rawSteerNorm, temp702837);

    /* applyLowerSteerLimit */
    output.limitedSteerNorm = fmax(temp702464, temp702432);


    
}
} // namespace control