#pragma once

#include <cmath>

#include "composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1.hpp"
#include "composite_block_empty_d57b8b07_7b62_49d0_a4d9_f37434b892f4.hpp"
#include "composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50.hpp"
#include "composite_block_empty_155716e6_f219_4239_bb0c_e7c06396c0ae.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_6b682752_6234_4817_a302_c9e173c6790eTraits {
  struct Input {
    Real egoSpeedMps;
    Real c0;
    Real c1;
    Real c2;
    Real c3;
    Real roadCurvaturePerM;
    bool isControlEnabled;
  };
  struct Output {
    Real steerRad;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50 calculatePreviewDistance;
    composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1 calculateWeightedError;
    composite_block_empty_d57b8b07_7b62_49d0_a4d9_f37434b892f4 generateSteerCommand;
    composite_block_empty_155716e6_f219_4239_bb0c_e7c06396c0ae calculatePreviewErrors;
  };

  using Global = GlobalParams;
};

class composite_block_empty_6b682752_6234_4817_a302_c9e173c6790e : public FuncModule<composite_block_empty_6b682752_6234_4817_a302_c9e173c6790eTraits> {
public:

  composite_block_empty_6b682752_6234_4817_a302_c9e173c6790e(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50(
                    composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50Traits::Param{},
                    composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50Traits::State{}),
                composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1(
                    composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1Traits::Param{},
                    composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1Traits::State{}),
                composite_block_empty_d57b8b07_7b62_49d0_a4d9_f37434b892f4(
                    composite_block_empty_d57b8b07_7b62_49d0_a4d9_f37434b892f4Traits::Param{},
                    composite_block_empty_d57b8b07_7b62_49d0_a4d9_f37434b892f4Traits::State{}),
                composite_block_empty_155716e6_f219_4239_bb0c_e7c06396c0ae(
                    composite_block_empty_155716e6_f219_4239_bb0c_e7c06396c0aeTraits::Param{},
                    composite_block_empty_155716e6_f219_4239_bb0c_e7c06396c0aeTraits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control