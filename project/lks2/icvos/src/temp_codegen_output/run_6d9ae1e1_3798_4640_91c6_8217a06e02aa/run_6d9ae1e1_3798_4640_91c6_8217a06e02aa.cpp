#include "run_6d9ae1e1_3798_4640_91c6_8217a06e02aa.hpp"
namespace control {
/**
 * @brief 空组件,用于自定义构造
 */
void run_6d9ae1e1_3798_4640_91c6_8217a06e02aa::run(const Input &input, Output &output)
{
    
    const double egoV = 5;  // 常量
    const double curvature = 0;  // 常量
    const double c1 = 0;  // 常量
    const double c2 = 0;  // 常量
    const double c3 = 0;  // 常量
    const double c0 = 0.8;  // 常量
    const double two = 2;  // 常量
    const double brakePresseddriverSteerNorm = 0;  // 常量
    
    Real C++_None;
    Real temp345036;
    
    
    composite_block_empty_35bfe571_0409_485f_b070_0999518ce7bcTraits::Input compositeBlockInstance_14_in;
    compositeBlockInstance_14_in.egoV = egoV;
    compositeBlockInstance_14_in.brakePressed = brakePresseddriverSteerNorm;
    compositeBlockInstance_14_in.driverSteerNorm = brakePresseddriverSteerNorm;
    composite_block_empty_35bfe571_0409_485f_b070_0999518ce7bcTraits::Output compositeBlockInstance_14_out;

    sub_.compositeBlockInstance_14.composite_block(compositeBlockInstance_14_in, compositeBlockInstance_14_out);
    composite_block_empty_aa61974c_e982_4ef6_b6c2_e61dba56487dTraits::Input compositeBlockInstance_8_in;
    compositeBlockInstance_8_in.egoV = egoV;
    compositeBlockInstance_8_in.curvature = curvature;
    composite_block_empty_aa61974c_e982_4ef6_b6c2_e61dba56487dTraits::Output compositeBlockInstance_8_out;

    sub_.compositeBlockInstance_8.composite_block(compositeBlockInstance_8_in, compositeBlockInstance_8_out);
    composite_block_empty_a96057e9_5379_42d6_94a2_dd98930f3bd6Traits::Input compositeBlockInstance_9_in;
    compositeBlockInstance_9_in.c0 = c0;
    compositeBlockInstance_9_in.c1 = c1;
    compositeBlockInstance_9_in.c2 = c2;
    compositeBlockInstance_9_in.c3 = c3;
    composite_block_empty_a96057e9_5379_42d6_94a2_dd98930f3bd6Traits::Output compositeBlockInstance_9_out;

    sub_.compositeBlockInstance_9.composite_block(compositeBlockInstance_9_in, compositeBlockInstance_9_out);
    C++_None = compositeBlockInstance_8_out.previewDistance / two;
    composite_block_empty_b25a47b1_cf11_4375_a368_3b21bdca7e68Traits::Input compositeBlockInstance_11_in;
    compositeBlockInstance_11_in.c0 = c0;
    compositeBlockInstance_11_in.c1 = c1;
    compositeBlockInstance_11_in.c2 = c2;
    compositeBlockInstance_11_in.c3 = c3;
    composite_block_empty_b25a47b1_cf11_4375_a368_3b21bdca7e68Traits::Output compositeBlockInstance_11_out;

    sub_.compositeBlockInstance_11.composite_block(compositeBlockInstance_11_in, compositeBlockInstance_11_out);
    composite_block_empty_d79c36b4_01b4_41fd_b98a_f76ca88a11c5Traits::Input compositeBlockInstance_10_in;
    compositeBlockInstance_10_in.c0 = c0;
    compositeBlockInstance_10_in.c1 = c1;
    compositeBlockInstance_10_in.c2 = c2;
    compositeBlockInstance_10_in.c3 = c3;
    composite_block_empty_d79c36b4_01b4_41fd_b98a_f76ca88a11c5Traits::Output compositeBlockInstance_10_out;

    sub_.compositeBlockInstance_10.composite_block(compositeBlockInstance_10_in, compositeBlockInstance_10_out);
    composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6Traits::Input compositeBlockInstance_12_in;
    compositeBlockInstance_12_in.e1 = compositeBlockInstance_9_out.e;
    compositeBlockInstance_12_in.e2 = compositeBlockInstance_10_out.e;
    compositeBlockInstance_12_in.e3 = compositeBlockInstance_11_out.e;
    composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6Traits::Output compositeBlockInstance_12_out;

    sub_.compositeBlockInstance_12.composite_block(compositeBlockInstance_12_in, compositeBlockInstance_12_out);
    composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7Traits::Input compositeBlockInstance_13_in;
    compositeBlockInstance_13_in.weightedError = compositeBlockInstance_12_out.weightedError;
    compositeBlockInstance_13_in.egoV = egoV;
    composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7Traits::Output compositeBlockInstance_13_out;

    sub_.compositeBlockInstance_13.composite_block(compositeBlockInstance_13_in, compositeBlockInstance_13_out);
    temp345036 = compositeBlockInstance_13_out.steerRad * compositeBlockInstance_14_out.controlEnabled;

    
}
} // namespace control