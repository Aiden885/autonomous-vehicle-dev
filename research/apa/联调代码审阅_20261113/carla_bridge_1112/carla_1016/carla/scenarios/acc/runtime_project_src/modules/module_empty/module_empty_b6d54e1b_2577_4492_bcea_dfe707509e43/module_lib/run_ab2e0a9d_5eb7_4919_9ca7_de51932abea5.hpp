#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_ab2e0a9d_5eb7_4919_9ca7_de51932abea5Traits {
  struct Input {
    Real egoSpeedMps;
    Real leadSpeedMps;
    Real relativeSpeedGain;
  };
  struct Output {
    Real relativeSpeedFeedbackMps;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class run_ab2e0a9d_5eb7_4919_9ca7_de51932abea5 : public FuncModule<run_ab2e0a9d_5eb7_4919_9ca7_de51932abea5Traits> {
public:

  run_ab2e0a9d_5eb7_4919_9ca7_de51932abea5(Param param = Param{}, State state = State{})
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