#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_51f83135_f79a_42dc_8c89_9463090201b8Traits {
  struct Input {
    Real leadSpeedMps;
    Real distanceFeedbackMps;
    Real relativeSpeedFeedbackMps;
    Real maxSpeedMps;
    int isControlEnabled;
  };
  struct Output {
    Real targetSpeedMps;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class run_51f83135_f79a_42dc_8c89_9463090201b8 : public FuncModule<run_51f83135_f79a_42dc_8c89_9463090201b8Traits> {
public:

  run_51f83135_f79a_42dc_8c89_9463090201b8(Param param = Param{}, State state = State{})
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