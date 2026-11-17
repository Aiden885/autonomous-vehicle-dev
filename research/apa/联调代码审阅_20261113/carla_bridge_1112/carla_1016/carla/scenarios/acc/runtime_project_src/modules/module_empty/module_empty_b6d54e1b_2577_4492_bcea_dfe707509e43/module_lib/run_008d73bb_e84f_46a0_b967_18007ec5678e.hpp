#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_008d73bb_e84f_46a0_b967_18007ec5678eTraits {
  struct Input {
    int decisionCode;
    int commandType;
    int isControlActive;
    int hasControlHistory;
  };
  struct Output {
    int isControlActiveNext;
    int hasControlHistoryNext;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class run_008d73bb_e84f_46a0_b967_18007ec5678e : public FuncModule<run_008d73bb_e84f_46a0_b967_18007ec5678eTraits> {
public:

  run_008d73bb_e84f_46a0_b967_18007ec5678e(Param param = Param{}, State state = State{})
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