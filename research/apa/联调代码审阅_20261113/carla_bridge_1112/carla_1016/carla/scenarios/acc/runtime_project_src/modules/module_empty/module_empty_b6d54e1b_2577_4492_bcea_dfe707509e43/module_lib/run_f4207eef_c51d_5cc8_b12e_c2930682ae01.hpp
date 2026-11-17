#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_f4207eef_c51d_5cc8_b12e_c2930682ae01Traits {
  struct Input {
    Real targetSpeedMps;
    Real egoSpeedMps;
    int isControlEnabled;
  };
  struct Output {
    Real accelerationCommandMps2;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class run_f4207eef_c51d_5cc8_b12e_c2930682ae01 : public FuncModule<run_f4207eef_c51d_5cc8_b12e_c2930682ae01Traits> {
public:

  run_f4207eef_c51d_5cc8_b12e_c2930682ae01(Param param = Param{}, State state = State{})
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