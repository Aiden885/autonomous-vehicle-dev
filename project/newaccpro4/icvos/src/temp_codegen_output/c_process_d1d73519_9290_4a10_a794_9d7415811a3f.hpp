#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct c_process_d1d73519_9290_4a10_a794_9d7415811a3fTraits {
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

class c_process_d1d73519_9290_4a10_a794_9d7415811a3f : public FuncModule<c_process_d1d73519_9290_4a10_a794_9d7415811a3fTraits> {
public:

  c_process_d1d73519_9290_4a10_a794_9d7415811a3f(Param param = Param{}, State state = State{})
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