#pragma once

#include <cmath>

#include "run_96195785_aad3_4e07_96eb_347ef47d3318.hpp"
#include "composite_block_empty_0583d8c2_dfeb_41ff_8153_3eb79777b834.hpp"
#include "run_db765728_3454_4e75_86e9_1f8380521b21.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_23d2a150_83e4_470e_a9b7_189357a9dfd1Traits {
  struct Input {
    Real targetSpeedMps[1];
    Real egoSpeedMps[1];
    int isControlEnabled[1];
    Real leadSpeedMps;
    int commandType;
    Real leadDistanceM;
  };
  struct Output {
    Real accelerationCommandMps2[1];
    bool isControlEnabled;
    Real targetSpeedMps;
  };
  struct Param {
    Real minDistanceM;
    Real distanceErrorGainPerS;
    Real relativeSpeedGain;
    Real minimumOperatingSpeedMps;
    Real timeGapStepS;
    Real minTimeGapS;
    Real maxTimeGapS;
    Real speedStepMps;
    Real minSpeedMps;
  };

  struct State {
    int isControlActive;
    int hasControlHistory;
    Real timeGapS;
    Real maxSpeedMps;
  };

  struct Sub {
    run_db765728_3454_4e75_86e9_1f8380521b21 fmax_530a5ed0_1c43_45b1_bf6c_0b7d3c2c39b7;
    run_db765728_3454_4e75_86e9_1f8380521b21 fmin_ef070586_af36_46b3_aceb_22a22054be98;
    composite_block_empty_0583d8c2_dfeb_41ff_8153_3eb79777b834 decision_2;
    run_96195785_aad3_4e07_96eb_347ef47d3318 control_2;
  };

  using Global = GlobalParams;
};

class run_23d2a150_83e4_470e_a9b7_189357a9dfd1 : public FuncModule<run_23d2a150_83e4_470e_a9b7_189357a9dfd1Traits> {
public:

  run_23d2a150_83e4_470e_a9b7_189357a9dfd1(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                run_db765728_3454_4e75_86e9_1f8380521b21(run_db765728_3454_4e75_86e9_1f8380521b21Traits::Param{}, run_db765728_3454_4e75_86e9_1f8380521b21Traits::State{}),
                run_db765728_3454_4e75_86e9_1f8380521b21(run_db765728_3454_4e75_86e9_1f8380521b21Traits::Param{}, run_db765728_3454_4e75_86e9_1f8380521b21Traits::State{}),
                composite_block_empty_0583d8c2_dfeb_41ff_8153_3eb79777b834(
                    composite_block_empty_0583d8c2_dfeb_41ff_8153_3eb79777b834Traits::Param{.minimumOperatingSpeedMps = 1, .timeGapStepS = 0.2, .minTimeGapS = 1, .maxTimeGapS = 5, .speedStepMps = 4, .minSpeedMps = 0},
                    composite_block_empty_0583d8c2_dfeb_41ff_8153_3eb79777b834Traits::State{.isControlActive = 0, .hasControlHistory = 0, .timeGapS = 1.8, .maxSpeedMps = 20}),
                run_96195785_aad3_4e07_96eb_347ef47d3318(
                    run_96195785_aad3_4e07_96eb_347ef47d3318Traits::Param{},
                    run_96195785_aad3_4e07_96eb_347ef47d3318Traits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control