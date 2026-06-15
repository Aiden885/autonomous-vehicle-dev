#include "composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0::composite_block(const Input &input, Output &output)
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
    
    Real temp765957;
    Real temp766277;
    Real temp766598;
    Real temp766771;
    Real temp767640;
    Real temp768121;
    Real temp768342;
    int enable;
    Real temp769448;
    Real temp769832;
    Real temp770028;
    Real temp770247;
    Real temp770519;
    Real temp771059;
    Real temp771895;
    Real temp772117;
    Real temp773091;
    Real temp773458;
    Real temp773858;
    Real temp774140;
    Real temp774467;
    Real temp774826;
    Real temp775255;
    Real temp775595;
    Real temp776693;
    Real temp777078;
    Real temp777350;
    Real temp777765;
    Real temp778155;
    Real temp778462;
    Real temp778841;
    Real temp779618;
    Real temp779940;
    Real temp780332;
    
    
    temp770028 = !state_.controlEnabled;
    temp770247 = !state_.hasHistory;
    temp773858 = commandType == C_6;
    temp769448 = egoV < param_.vMin;
    temp774140 = commandType == C_7;
    temp769832 = !temp769448;
    temp772117 = temp769448 * isS3;
    temp774826 = temp773858 || temp774140;
    enable = temp769832 && state_.controlEnabled;
    temp770519 = temp769832 && temp770028 && state_.hasHistory;
    temp771059 = temp769832 && temp770028 && temp770247;
    temp775255 = !temp774826;
    temp771895 = isS2 * temp771059;
    temp765957 = temp770519 + temp771895 + temp772117;
    /* 真值表 */
    temp766277 = truth_table_e6b051ea_907b_4ca4_9ef8_0950eb161a41(temp765957, commandType);

    temp773091 = R5 == temp766277;
    temp773458 = temp766277 == R6;
    temp776693 = temp766277 == R_1;
    temp777078 = temp766277 == R_2;
    temp777350 = temp766277 == R_3;
    temp777765 = temp766277 == R_4;
    temp774467 = temp773091 || temp773458;
    temp779618 = temp776693 * param_.SpdStep;
    temp779940 = temp777078 * param_.SpdStep;
    temp778462 = param_.GapStep * temp777350;
    temp778155 = temp777765 * param_.GapStep;
    temp775595 = state_.controlEnabled || temp774467;
    temp780332 = state_.maxSpeed - temp779618;
    temp778841 = state_.timeGap + temp778155;
    temp768121 = temp775595 && temp775255;
    temp767640 = temp780332 + temp779940;
    temp766598 = temp778841 - temp778462;
    state_.controlEnabled = temp768121;
    temp768342 = state_.hasHistory || temp768121;
    /* 浮点最大值 */
    maxSpeed = fmax(temp767640, param_.MinSpd);

    /* 浮点最大值 */
    temp766771 = fmax(temp766598, param_.MinGap);

    state_.hasHistory = temp768342;
    state_.maxSpeed = maxSpeed;
    /* 浮点最小值 */
    temp767335 = fmin(temp766771, param_.MaxGap);

    state_.timeGap = temp767335;
    output.enable = enable;
    output.maxSpeed = maxSpeed;
    output.timeGap = timeGap;

    
}
} // namespace control