#pragma once

#include <cmath>

#include "composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364d.hpp"
#include "composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83.hpp"
#include "composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_155716e6_f219_4239_bb0c_e7c06396c0aeTraits {
  struct Input {
    Real c0;
    Real c1;
    Real c2;
    Real c3;
    Real previewDistanceM;
  };
  struct Output {
    Real nearLateralErrorM;
    Real middleLateralErrorM;
    Real farLateralErrorM;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667 calculateNearPreviewError;
    composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83 calculateMiddlePreviewError;
    composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364d calculateFarPreviewError;
  };

  using Global = GlobalParams;
};

class composite_block_empty_155716e6_f219_4239_bb0c_e7c06396c0ae : public FuncModule<composite_block_empty_155716e6_f219_4239_bb0c_e7c06396c0aeTraits> {
public:

  composite_block_empty_155716e6_f219_4239_bb0c_e7c06396c0ae(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667(
                    composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667Traits::Param{},
                    composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667Traits::State{}),
                composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83(
                    composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83Traits::Param{},
                    composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83Traits::State{}),
                composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364d(
                    composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364dTraits::Param{},
                    composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364dTraits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control