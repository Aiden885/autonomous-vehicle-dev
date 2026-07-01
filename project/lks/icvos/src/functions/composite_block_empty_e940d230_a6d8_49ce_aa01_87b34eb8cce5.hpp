#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "lks.hpp"

namespace control {

struct composite_block_empty_e940d230_a6d8_49ce_aa01_87b34eb8cce5Traits {
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

class composite_block_empty_e940d230_a6d8_49ce_aa01_87b34eb8cce5 : public FuncModule<composite_block_empty_e940d230_a6d8_49ce_aa01_87b34eb8cce5Traits> {
public:

  composite_block_empty_e940d230_a6d8_49ce_aa01_87b34eb8cce5(Param param = Param{}, State state = State{})
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