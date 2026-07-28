#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6Traits {
  struct Input {
    Real e1;
    Real e2;
    Real e3;
  };
  struct Output {
    Real weightedError;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6 : public FuncModule<composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6Traits> {
public:

  composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6(Param param = Param{}, State state = State{})
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