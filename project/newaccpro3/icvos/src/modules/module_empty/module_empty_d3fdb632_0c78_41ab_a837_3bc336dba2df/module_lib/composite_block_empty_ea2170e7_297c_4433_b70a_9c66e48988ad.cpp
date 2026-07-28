#include "composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad::composite_block(const Input &input, Output &output)
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
    
    Real temp355358;
    Real temp355463;
    Real temp355828;
    Real temp356014;
    Real temp358178;
    Real temp358782;
    Real temp358811;
    Real temp359642;
    Real temp359874;
    Real temp359994;
    Real temp360081;
    Real temp360211;
    Real temp360486;
    Real temp360965;
    Real temp361137;
    Real temp361729;
    Real temp361934;
    Real temp362184;
    Real temp362390;
    Real temp362599;
    Real temp362799;
    Real temp363052;
    Real temp363164;
    Real temp363760;
    Real temp364050;
    Real temp364290;
    Real temp364417;
    Real temp364622;
    Real temp364947;
    Real temp365152;
    Real temp365558;
    Real temp365753;
    Real temp365997;
    
    
    temp360081 = !state_.hasHistory;
    temp359994 = !state_.controlEnabled;
    temp359642 = input.egoV < param_.vMin;
    temp362390 = input.commandType == C_7;
    temp362184 = input.commandType == C_6;
    temp359874 = !temp359642;
    temp361137 = temp359642 * isS3;
    temp362799 = temp362184 || temp362390;
    output.enable = temp359874 && state_.controlEnabled;
    temp360211 = temp359874 && temp359994 && state_.hasHistory;
    temp360486 = temp359874 && temp359994 && temp360081;
    temp363052 = !temp362799;
    temp360965 = isS2 * temp360486;
    temp355358 = temp360211 + temp360965 + temp361137;
    /* 真值表 */
    temp355463 = truth_table_6957bd9f_a281_4c91_a9b5_5b2c83757f58(temp355358, input.commandType);

    temp361729 = R5 == temp355463;
    temp361934 = temp355463 == R6;
    temp363760 = temp355463 == R_1;
    temp364050 = temp355463 == R_2;
    temp364290 = temp355463 == R_3;
    temp364417 = temp355463 == R_4;
    temp362599 = temp361729 || temp361934;
    temp365558 = temp363760 * param_.SpdStep;
    temp365753 = temp364050 * param_.SpdStep;
    temp364947 = param_.GapStep * temp364290;
    temp364622 = temp364417 * param_.GapStep;
    temp363164 = state_.controlEnabled || temp362599;
    temp365997 = state_.maxSpeed - temp365558;
    temp365152 = state_.timeGap + temp364622;
    temp358782 = temp363164 && temp363052;
    temp358178 = temp365997 + temp365753;
    temp355828 = temp365152 - temp364947;
    state_.controlEnabled = temp358782;
    temp358811 = state_.hasHistory || temp358782;
    /* FMAX_Spd */
    output.maxSpeed = fmax(temp358178, param_.MinSpd);

    /* FMAX_Gap */
    temp356014 = fmax(temp355828, param_.MinGap);

    state_.hasHistory = temp358811;
    state_.maxSpeed = output.maxSpeed;
    /* FMIN_Gap */
    temp357483 = fmin(temp356014, param_.MaxGap);

    state_.timeGap = temp357483;

    
}
} // namespace control