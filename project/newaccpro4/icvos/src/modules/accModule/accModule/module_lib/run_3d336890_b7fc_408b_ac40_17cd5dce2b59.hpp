#pragma once

#include <cmath>

#include "run_db765728_3454_4e75_86e9_1f8380521b21.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_3d336890_b7fc_408b_ac40_17cd5dce2b59Traits {
  struct Input {
    Real targetSpeedMps[1];
    Real egoSpeedMps[1];
    int isControlEnabled[1];
    int decisionCode;
    int commandType;
    int isControlActive;
    int hasControlHistory;
  };
  struct Output {
    Real accelerationCommandMps2[1];
    int isControlActiveNext;
    int hasControlHistoryNext;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    run_db765728_3454_4e75_86e9_1f8380521b21 fmax_530a5ed0_1c43_45b1_bf6c_0b7d3c2c39b7;
    run_db765728_3454_4e75_86e9_1f8380521b21 fmin_ef070586_af36_46b3_aceb_22a22054be98;
  };

  using Global = GlobalParams;
};

class run_3d336890_b7fc_408b_ac40_17cd5dce2b59 : public FuncModule<run_3d336890_b7fc_408b_ac40_17cd5dce2b59Traits> {
public:

  run_3d336890_b7fc_408b_ac40_17cd5dce2b59(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                run_db765728_3454_4e75_86e9_1f8380521b21(run_db765728_3454_4e75_86e9_1f8380521b21Traits::Param{}, run_db765728_3454_4e75_86e9_1f8380521b21Traits::State{}),
                run_db765728_3454_4e75_86e9_1f8380521b21(run_db765728_3454_4e75_86e9_1f8380521b21Traits::Param{}, run_db765728_3454_4e75_86e9_1f8380521b21Traits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control