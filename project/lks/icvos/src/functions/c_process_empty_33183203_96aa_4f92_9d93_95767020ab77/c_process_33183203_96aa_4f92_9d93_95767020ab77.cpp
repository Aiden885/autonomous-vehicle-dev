#include "c_process_33183203_96aa_4f92_9d93_95767020ab77.hpp"

namespace control {

void c_process_33183203_96aa_4f92_9d93_95767020ab77::run(const Input &input, Output &output)
{
    (void)input;
    (void)output;

    const double egoV = 5;  // 常量
    const double curvature = 0;  // 常量
    const double c0 = 0.8;  // 常量
    const double c1 = 0;  // 常量
    const double c2 = 0;  // 常量
    const double c3 = 0;  // 常量
    const double two = 2;  // 常量
    const double brakePresseddriverSteerNorm = 0;  // 常量

    Real C++_None;
    Real temp750099;

    composite_block_empty_4908e83d_94ee_4784_a3bb_e3aa87136eabTraits::Input compositeBlockInstance_1_in;
    compositeBlockInstance_1_in.c0 = c0;
    compositeBlockInstance_1_in.c1 = c1;
    compositeBlockInstance_1_in.c2 = c2;
    compositeBlockInstance_1_in.c3 = c3;
    composite_block_empty_4908e83d_94ee_4784_a3bb_e3aa87136eabTraits::Output compositeBlockInstance_1_out;

    sub_.compositeBlockInstance_1.composite_block(compositeBlockInstance_1_in, compositeBlockInstance_1_out);
    composite_block_empty_376e82ac_81e2_4f82_91cc_5c118b024c83Traits::Input compositeBlockInstance_in;
    compositeBlockInstance_in.egoV = egoV;
    compositeBlockInstance_in.curvature = curvature;
    composite_block_empty_376e82ac_81e2_4f82_91cc_5c118b024c83Traits::Output compositeBlockInstance_out;

    sub_.compositeBlockInstance.composite_block(compositeBlockInstance_in, compositeBlockInstance_out);
    composite_block_empty_b693b8f5_5ed3_4feb_baa7_5ffef46d8e95Traits::Input compositeBlockInstance_6_in;
    compositeBlockInstance_6_in.egoV = egoV;
    compositeBlockInstance_6_in.brakePressed = brakePresseddriverSteerNorm;
    compositeBlockInstance_6_in.driverSteerNorm = brakePresseddriverSteerNorm;
    composite_block_empty_b693b8f5_5ed3_4feb_baa7_5ffef46d8e95Traits::Output compositeBlockInstance_6_out;

    sub_.compositeBlockInstance_6.composite_block(compositeBlockInstance_6_in, compositeBlockInstance_6_out);
    C++_None = compositeBlockInstance_out.previewDistance / two;
    composite_block_empty_113ef91e_26f5_49d7_ae54_fbbc3f872ca1Traits::Input compositeBlockInstance_3_in;
    compositeBlockInstance_3_in.c0 = c0;
    compositeBlockInstance_3_in.c1 = c1;
    compositeBlockInstance_3_in.c2 = c2;
    compositeBlockInstance_3_in.c3 = c3;
    compositeBlockInstance_3_in.x = compositeBlockInstance_out.previewDistance;
    composite_block_empty_113ef91e_26f5_49d7_ae54_fbbc3f872ca1Traits::Output compositeBlockInstance_3_out;

    sub_.compositeBlockInstance_3.composite_block(compositeBlockInstance_3_in, compositeBlockInstance_3_out);
    composite_block_empty_e6613cb4_7f79_4b07_9207_efebebea2cbbTraits::Input compositeBlockInstance_2_in;
    compositeBlockInstance_2_in.c0 = c0;
    compositeBlockInstance_2_in.c1 = c1;
    compositeBlockInstance_2_in.c2 = c2;
    compositeBlockInstance_2_in.c3 = c3;
    composite_block_empty_e6613cb4_7f79_4b07_9207_efebebea2cbbTraits::Output compositeBlockInstance_2_out;

    sub_.compositeBlockInstance_2.composite_block(compositeBlockInstance_2_in, compositeBlockInstance_2_out);
    composite_block_empty_e940d230_a6d8_49ce_aa01_87b34eb8cce5Traits::Input compositeBlockInstance_4_in;
    compositeBlockInstance_4_in.e1 = compositeBlockInstance_1_out.e;
    compositeBlockInstance_4_in.e2 = compositeBlockInstance_2_out.e;
    compositeBlockInstance_4_in.e3 = compositeBlockInstance_3_out.e;
    composite_block_empty_e940d230_a6d8_49ce_aa01_87b34eb8cce5Traits::Output compositeBlockInstance_4_out;

    sub_.compositeBlockInstance_4.composite_block(compositeBlockInstance_4_in, compositeBlockInstance_4_out);
    composite_block_empty_e92430b6_35aa_4184_bc4f_623e36299e44Traits::Input compositeBlockInstance_5_in;
    compositeBlockInstance_5_in.weightedError = compositeBlockInstance_4_out.weightedError;
    composite_block_empty_e92430b6_35aa_4184_bc4f_623e36299e44Traits::Output compositeBlockInstance_5_out;

    sub_.compositeBlockInstance_5.composite_block(compositeBlockInstance_5_in, compositeBlockInstance_5_out);
    temp750099 = compositeBlockInstance_5_out.steerRad * compositeBlockInstance_6_out.controlEnabled;
}

} // namespace control
