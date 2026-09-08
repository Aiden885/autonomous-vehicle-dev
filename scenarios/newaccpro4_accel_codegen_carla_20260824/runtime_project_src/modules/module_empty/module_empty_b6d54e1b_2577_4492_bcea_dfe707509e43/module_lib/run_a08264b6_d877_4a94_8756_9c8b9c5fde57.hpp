#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_a08264b6_d877_4a94_8756_9c8b9c5fde57Traits {
  struct Input {
    int decisionCode;
    Real timeGapS;
    Real timeGapStepS;
    Real minTimeGapS;
    Real maxTimeGapS;
  };
  struct Output {
    Real nextTimeGapS;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class run_a08264b6_d877_4a94_8756_9c8b9c5fde57 : public FuncModule<run_a08264b6_d877_4a94_8756_9c8b9c5fde57Traits> {
public:

  run_a08264b6_d877_4a94_8756_9c8b9c5fde57(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                },
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control