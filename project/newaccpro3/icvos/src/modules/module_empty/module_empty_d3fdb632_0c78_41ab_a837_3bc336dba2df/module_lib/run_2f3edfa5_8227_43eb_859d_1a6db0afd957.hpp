#pragma once

#include <cmath>

#include "composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_2f3edfa5_8227_43eb_859d_1a6db0afd957Traits {
  struct Input {
    
  };
  struct Output {
    
  };
  struct Param {
    Real vMin;
    Real GapStep;
    Real MinGap;
    Real MaxGap;
    Real SpdStep;
    Real MinSpd;
  };

  struct State {
    int controlEnabled;
    int hasHistory;
    Real timeGap;
    Real maxSpeed;
  };

  struct Sub {
    composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad compositeBlockInstance_2;
  };

  using Global = GlobalParams;
};

class run_2f3edfa5_8227_43eb_859d_1a6db0afd957 : public FuncModule<run_2f3edfa5_8227_43eb_859d_1a6db0afd957Traits> {
public:

  run_2f3edfa5_8227_43eb_859d_1a6db0afd957(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad(
                    composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988adTraits::Param{.vMin = 1, .GapStep = 0.2, .MinGap = 1, .MaxGap = 5, .SpdStep = 4, .MinSpd = 0},
                    composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988adTraits::State{.controlEnabled = 0, .hasHistory = 0, .timeGap = 1.8, .maxSpeed = 20})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control