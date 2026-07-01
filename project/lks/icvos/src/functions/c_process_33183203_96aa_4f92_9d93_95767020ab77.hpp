#pragma once

#include "composite_block_empty_376e82ac_81e2_4f82_91cc_5c118b024c83.hpp"
#include "composite_block_empty_113ef91e_26f5_49d7_ae54_fbbc3f872ca1.hpp"
#include "composite_block_empty_e92430b6_35aa_4184_bc4f_623e36299e44.hpp"
#include "composite_block_empty_b693b8f5_5ed3_4feb_baa7_5ffef46d8e95.hpp"
#include "composite_block_empty_4908e83d_94ee_4784_a3bb_e3aa87136eab.hpp"
#include "composite_block_empty_e940d230_a6d8_49ce_aa01_87b34eb8cce5.hpp"
#include "composite_block_empty_e6613cb4_7f79_4b07_9207_efebebea2cbb.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "lks.hpp"

namespace control {

struct c_process_33183203_96aa_4f92_9d93_95767020ab77Traits {
  struct Input {
    
  };
  struct Output {
    
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    composite_block_empty_376e82ac_81e2_4f82_91cc_5c118b024c83 compositeBlockInstance;
    composite_block_empty_4908e83d_94ee_4784_a3bb_e3aa87136eab compositeBlockInstance_1;
    composite_block_empty_e6613cb4_7f79_4b07_9207_efebebea2cbb compositeBlockInstance_2;
    composite_block_empty_113ef91e_26f5_49d7_ae54_fbbc3f872ca1 compositeBlockInstance_3;
    composite_block_empty_e940d230_a6d8_49ce_aa01_87b34eb8cce5 compositeBlockInstance_4;
    composite_block_empty_e92430b6_35aa_4184_bc4f_623e36299e44 compositeBlockInstance_5;
    composite_block_empty_b693b8f5_5ed3_4feb_baa7_5ffef46d8e95 compositeBlockInstance_6;
  };

  using Global = GlobalParams;
};

class c_process_33183203_96aa_4f92_9d93_95767020ab77 : public FuncModule<c_process_33183203_96aa_4f92_9d93_95767020ab77Traits> {
public:

  c_process_33183203_96aa_4f92_9d93_95767020ab77(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                composite_block_empty_376e82ac_81e2_4f82_91cc_5c118b024c83(composite_block_empty_376e82ac_81e2_4f82_91cc_5c118b024c83Traits::Param{}, composite_block_empty_376e82ac_81e2_4f82_91cc_5c118b024c83Traits::State{}),
                composite_block_empty_4908e83d_94ee_4784_a3bb_e3aa87136eab(composite_block_empty_4908e83d_94ee_4784_a3bb_e3aa87136eabTraits::Param{}, composite_block_empty_4908e83d_94ee_4784_a3bb_e3aa87136eabTraits::State{}),
                composite_block_empty_e6613cb4_7f79_4b07_9207_efebebea2cbb(composite_block_empty_e6613cb4_7f79_4b07_9207_efebebea2cbbTraits::Param{}, composite_block_empty_e6613cb4_7f79_4b07_9207_efebebea2cbbTraits::State{}),
                composite_block_empty_113ef91e_26f5_49d7_ae54_fbbc3f872ca1(composite_block_empty_113ef91e_26f5_49d7_ae54_fbbc3f872ca1Traits::Param{}, composite_block_empty_113ef91e_26f5_49d7_ae54_fbbc3f872ca1Traits::State{}),
                composite_block_empty_e940d230_a6d8_49ce_aa01_87b34eb8cce5(composite_block_empty_e940d230_a6d8_49ce_aa01_87b34eb8cce5Traits::Param{}, composite_block_empty_e940d230_a6d8_49ce_aa01_87b34eb8cce5Traits::State{}),
                composite_block_empty_e92430b6_35aa_4184_bc4f_623e36299e44(composite_block_empty_e92430b6_35aa_4184_bc4f_623e36299e44Traits::Param{}, composite_block_empty_e92430b6_35aa_4184_bc4f_623e36299e44Traits::State{}),
                composite_block_empty_b693b8f5_5ed3_4feb_baa7_5ffef46d8e95(composite_block_empty_b693b8f5_5ed3_4feb_baa7_5ffef46d8e95Traits::Param{}, composite_block_empty_b693b8f5_5ed3_4feb_baa7_5ffef46d8e95Traits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control