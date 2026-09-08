#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_a5e68afa_fef4_4c76_9139_cb7fef8f0aa2Traits {
  struct Input {
    Real egoSpeedMps;
    Real timeGapS;
    Real minDistanceM;
    Real leadDistanceM;
    Real distanceErrorGainPerS;
  };
  struct Output {
    Real distanceFeedbackMps;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class run_a5e68afa_fef4_4c76_9139_cb7fef8f0aa2 : public FuncModule<run_a5e68afa_fef4_4c76_9139_cb7fef8f0aa2Traits> {
public:

  run_a5e68afa_fef4_4c76_9139_cb7fef8f0aa2(Param param = Param{}, State state = State{})
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