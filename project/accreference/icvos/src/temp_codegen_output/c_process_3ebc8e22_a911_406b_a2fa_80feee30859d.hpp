#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct c_process_3ebc8e22_a911_406b_a2fa_80feee30859dTraits {
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

class c_process_3ebc8e22_a911_406b_a2fa_80feee30859d : public FuncModule<c_process_3ebc8e22_a911_406b_a2fa_80feee30859dTraits> {
public:

  c_process_3ebc8e22_a911_406b_a2fa_80feee30859d(Param param = Param{}, State state = State{})
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