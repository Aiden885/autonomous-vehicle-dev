#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct c_process_e4194137_2737_4c39_8400_09f835154e34Traits {
  struct Input {
    
  };
  struct Output {
    
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class c_process_e4194137_2737_4c39_8400_09f835154e34 : public FuncModule<c_process_e4194137_2737_4c39_8400_09f835154e34Traits> {
public:

  c_process_e4194137_2737_4c39_8400_09f835154e34(Param param = Param{}, State state = State{})
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