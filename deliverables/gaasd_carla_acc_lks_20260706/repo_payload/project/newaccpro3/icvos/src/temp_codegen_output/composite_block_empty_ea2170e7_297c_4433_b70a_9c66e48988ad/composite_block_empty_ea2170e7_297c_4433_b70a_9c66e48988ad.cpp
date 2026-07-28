#include "composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad::run(const Input &input, Output &output)
{
    
    const double isS2 = 2;  // 常量
    const double isS3 = 3;  // 常量
    const double R5 = 5;  // 常量
    const double R6 = 6;  // 常量
    const double C_6 = 6;  // 常量
    const double C_7 = 7;  // 常量
    const double R_1 = 1;  // 常量
    const double R_2 = 2;  // 常量
    const double R_3 = 3;  // 常量
    const double R_4 = 4;  // 常量
    
    Real temp203065;
    Real temp203244;
    Real temp203449;
    Real temp203651;
    Real temp204061;
    Real temp204246;
    Real temp204582;
    Real temp204639;
    Real temp205447;
    Real temp205738;
    Real temp205891;
    Real temp205964;
    Real temp206172;
    Real temp206430;
    Real temp207055;
    Real temp207292;
    Real temp207781;
    Real temp208021;
    Real temp208463;
    Real temp208676;
    Real temp208862;
    Real temp209053;
    Real temp209346;
    Real temp209566;
    Real temp210225;
    Real temp210425;
    Real temp210649;
    Real temp210923;
    Real temp211134;
    Real temp211443;
    Real temp211639;
    Real temp212116;
    Real temp212363;
    Real temp212512;
    
    
    temp205964 = !state_.hasHistory;
    temp205891 = !state_.controlEnabled;
    temp205447 = input.egoV < param_.vMin;
    temp208676 = input.commandType == C_7;
    temp208463 = input.commandType == C_6;
    temp205738 = !temp205447;
    temp207292 = temp205447 * isS3;
    temp209053 = temp208463 || temp208676;
    output.enable = temp205738 && state_.controlEnabled;
    temp206172 = temp205738 && temp205891 && state_.hasHistory;
    temp206430 = temp205738 && temp205891 && temp205964;
    temp209346 = !temp209053;
    temp207055 = isS2 * temp206430;
    temp203065 = temp206172 + temp207055 + temp207292;
    /* 真值表 */
    temp203244 = truth_table_6957bd9f_a281_4c91_a9b5_5b2c83757f58(temp203065, input.commandType);

    temp207781 = R5 == temp203244;
    temp208021 = temp203244 == R6;
    temp210225 = temp203244 == R_1;
    temp210425 = temp203244 == R_2;
    temp210649 = temp203244 == R_3;
    temp210923 = temp203244 == R_4;
    temp208862 = temp207781 || temp208021;
    temp212116 = temp210225 * param_.SpdStep;
    temp212363 = temp210425 * param_.SpdStep;
    temp211443 = param_.GapStep * temp210649;
    temp211134 = temp210923 * param_.GapStep;
    temp209566 = state_.controlEnabled || temp208862;
    temp212512 = state_.maxSpeed - temp212116;
    temp211639 = state_.timeGap + temp211134;
    temp204582 = temp209566 && temp209346;
    temp204246 = temp212512 + temp212363;
    temp203449 = temp211639 - temp211443;
    state_.controlEnabled = temp204582;
    temp204639 = state_.hasHistory || temp204582;
    /* FMAX_Spd */
    output.maxSpeed = fmax(temp204246, param_.MinSpd);

    /* FMAX_Gap */
    temp203651 = fmax(temp203449, param_.MinGap);

    state_.hasHistory = temp204639;
    state_.maxSpeed = output.maxSpeed;
    /* FMIN_Gap */
    temp204061 = fmin(temp203651, param_.MaxGap);

    state_.timeGap = temp204061;
    output.timeGap = temp204061;

    
}
} // namespace control
