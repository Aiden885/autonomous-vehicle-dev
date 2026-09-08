#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_db765728_3454_4e75_86e9_1f8380521b21Traits {
  struct Input {
    Real targetSpeedMps[1];
    Real egoSpeedMps[1];
    int isControlEnabled[1];
  };
  struct Output {
    Real accelerationCommandMps2[1];
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class run_db765728_3454_4e75_86e9_1f8380521b21 : public FuncModule<run_db765728_3454_4e75_86e9_1f8380521b21Traits> {
public:

  run_db765728_3454_4e75_86e9_1f8380521b21(Param param = Param{}, State state = State{})
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