#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_b12be11b_48bd_4d14_be5e_a2496219d1f7Traits {
  struct Input {
    Real egoSpeedMps;
    Real minimumOperatingSpeedMps;
    int isControlActive;
    int hasControlHistory;
  };
  struct Output {
    int systemState;
    int isControlEnabled;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class run_b12be11b_48bd_4d14_be5e_a2496219d1f7 : public FuncModule<run_b12be11b_48bd_4d14_be5e_a2496219d1f7Traits> {
public:

  run_b12be11b_48bd_4d14_be5e_a2496219d1f7(Param param = Param{}, State state = State{})
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