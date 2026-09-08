#pragma once

#include <cmath>

#include "run_8f7b5338_baaf_44cb_ab98_c448a82d8b2b.hpp"
#include "run_fa2b2535_e243_4fd1_904f_ea508b90a4ef.hpp"
#include "run_db765728_3454_4e75_86e9_1f8380521b21.hpp"
#include "run_53728aa4_1a0c_4dbf_b2ed_63c9c2a4016a.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_96195785_aad3_4e07_96eb_347ef47d3318Traits {
  struct Input {
    Real targetSpeedMps[1];
    Real egoSpeedMps[1];
    int isControlEnabled[1];
    Real leadSpeedMps;
    Real leadDistanceM;
    Real timeGapS;
    Real maxSpeedMps;
    Real minDistanceM;
    Real distanceErrorGainPerS;
    Real relativeSpeedGain;
  };
  struct Output {
    Real accelerationCommandMps2[1];
    Real targetSpeedMps;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    run_db765728_3454_4e75_86e9_1f8380521b21 fmax_530a5ed0_1c43_45b1_bf6c_0b7d3c2c39b7;
    run_db765728_3454_4e75_86e9_1f8380521b21 fmin_ef070586_af36_46b3_aceb_22a22054be98;
    run_fa2b2535_e243_4fd1_904f_ea508b90a4ef calculateSpacingFeedback_2;
    run_8f7b5338_baaf_44cb_ab98_c448a82d8b2b calculateRelativeSpeedFeedback_2;
    run_53728aa4_1a0c_4dbf_b2ed_63c9c2a4016a generateTargetSpeed_2;
    run_db765728_3454_4e75_86e9_1f8380521b21 calculateAccelerationCommand_2;
  };

  using Global = GlobalParams;
};

class run_96195785_aad3_4e07_96eb_347ef47d3318 : public FuncModule<run_96195785_aad3_4e07_96eb_347ef47d3318Traits> {
public:

  run_96195785_aad3_4e07_96eb_347ef47d3318(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                run_db765728_3454_4e75_86e9_1f8380521b21(run_db765728_3454_4e75_86e9_1f8380521b21Traits::Param{}, run_db765728_3454_4e75_86e9_1f8380521b21Traits::State{}),
                run_db765728_3454_4e75_86e9_1f8380521b21(run_db765728_3454_4e75_86e9_1f8380521b21Traits::Param{}, run_db765728_3454_4e75_86e9_1f8380521b21Traits::State{}),
                run_fa2b2535_e243_4fd1_904f_ea508b90a4ef(
                    run_fa2b2535_e243_4fd1_904f_ea508b90a4efTraits::Param{},
                    run_fa2b2535_e243_4fd1_904f_ea508b90a4efTraits::State{}),
                run_8f7b5338_baaf_44cb_ab98_c448a82d8b2b(
                    run_8f7b5338_baaf_44cb_ab98_c448a82d8b2bTraits::Param{},
                    run_8f7b5338_baaf_44cb_ab98_c448a82d8b2bTraits::State{}),
                run_53728aa4_1a0c_4dbf_b2ed_63c9c2a4016a(
                    run_53728aa4_1a0c_4dbf_b2ed_63c9c2a4016aTraits::Param{},
                    run_53728aa4_1a0c_4dbf_b2ed_63c9c2a4016aTraits::State{}),
                run_db765728_3454_4e75_86e9_1f8380521b21(
                    run_db765728_3454_4e75_86e9_1f8380521b21Traits::Param{},
                    run_db765728_3454_4e75_86e9_1f8380521b21Traits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control