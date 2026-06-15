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
    
    Real temp342341;
    Real temp342633;
    Real temp342913;
    Real temp343228;
    Real temp344068;
    Real temp344511;
    Real temp344792;
    int enable;
    Real temp345848;
    Real temp346149;
    Real temp346321;
    Real temp346670;
    Real temp346875;
    Real temp347229;
    Real temp348188;
    Real temp348416;
    Real temp349393;
    Real temp349624;
    Real temp350050;
    Real temp350390;
    Real temp350629;
    Real temp351022;
    Real temp351416;
    Real temp351732;
    Real temp352971;
    Real temp353473;
    Real temp353988;
    Real temp354323;
    Real temp354790;
    Real temp355151;
    Real temp355490;
    Real temp356235;
    Real temp356670;
    Real temp356952;
    
    
    temp346321 = !state_.controlEnabled;
    temp346670 = !state_.hasHistory;
    temp350050 = commandType == C_6;
    temp345848 = egoV < param_.vMin;
    temp350390 = commandType == C_7;
    temp346149 = !temp345848;
    temp348416 = temp345848 * isS3;
    temp351022 = temp350050 || temp350390;
    enable = temp346149 && state_.controlEnabled;
    temp346875 = temp346149 && temp346321 && state_.hasHistory;
    temp347229 = temp346149 && temp346321 && temp346670;
    temp351416 = !temp351022;
    temp348188 = isS2 * temp347229;
    temp342341 = temp346875 + temp348188 + temp348416;
    /* 真值表 */
    temp342633 = truth_table_e6b051ea_907b_4ca4_9ef8_0950eb161a41(temp342341, commandType);

    temp349393 = R5 == temp342633;
    temp349624 = temp342633 == R6;
    temp352971 = temp342633 == R_1;
    temp353473 = temp342633 == R_2;
    temp353988 = temp342633 == R_3;
    temp354323 = temp342633 == R_4;
    temp350629 = temp349393 || temp349624;
    temp356235 = temp352971 * param_.SpdStep;
    temp356670 = temp353473 * param_.SpdStep;
    temp355151 = param_.GapStep * temp353988;
    temp354790 = temp354323 * param_.GapStep;
    temp351732 = state_.controlEnabled || temp350629;
    temp356952 = state_.maxSpeed - temp356235;
    temp355490 = state_.timeGap + temp354790;
    temp344511 = temp351732 && temp351416;
    temp344068 = temp356952 + temp356670;
    temp342913 = temp355490 - temp355151;
    state_.controlEnabled = temp344511;
    temp344792 = state_.hasHistory || temp344511;
    /* 浮点最大值 */
    maxSpeed = fmax(temp344068, param_.MinSpd);

    /* 浮点最大值 */
    temp343228 = fmax(temp342913, param_.MinGap);

    state_.hasHistory = temp344792;
    state_.maxSpeed = maxSpeed;
    /* 浮点最小值 */
    temp343741 = fmin(temp343228, param_.MaxGap);

    state_.timeGap = temp343741;
    output.enable = enable;
    output.maxSpeed = maxSpeed;
    output.timeGap = timeGap;

    
}
} // namespace control