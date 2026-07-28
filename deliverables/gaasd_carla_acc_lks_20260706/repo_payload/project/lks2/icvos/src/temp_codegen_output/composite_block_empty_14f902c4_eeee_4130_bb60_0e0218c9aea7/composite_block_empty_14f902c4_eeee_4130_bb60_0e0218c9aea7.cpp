#include "composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7::run(const Input &input, Output &output)
{
    const double speedSquareFloor = 0.25;
    Real speedSquare;
    Real speedSquareSafe;
    Real lateralAccelNumerator;
    Real lateralAccelRatio;
    Real lateralAccelLimitRad;
    Real normalizedSteerLimit;
    Real negativeSteerLimit;
    Real rawSteerNorm;
    Real upperLimitedSteer;
    Real limitedSteer;

    speedSquare = input.egoV * input.egoV;
    rawSteerNorm = global::params.lks_Kp * input.weightedError;
    lateralAccelNumerator = global::params.lks_ayMax * global::params.lks_wheelBase;
    speedSquareSafe = fmax(speedSquare, speedSquareFloor);
    lateralAccelRatio = lateralAccelNumerator / speedSquareSafe;
    lateralAccelLimitRad = atan(lateralAccelRatio);
    normalizedSteerLimit = lateralAccelLimitRad / global::params.lks_frontWheelMaxRad;
    negativeSteerLimit = -normalizedSteerLimit;
    upperLimitedSteer = fmin(rawSteerNorm, normalizedSteerLimit);
    limitedSteer = fmax(upperLimitedSteer, negativeSteerLimit);
    output.steerRad = limitedSteer * global::params.lks_steerScale;
}
} // namespace control
