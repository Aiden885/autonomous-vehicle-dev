#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_a96057e9_5379_42d6_94a2_dd98930f3bd6Traits {
  struct Input {
    Real c0;
    Real c1;
    Real c2;
    Real c3;
    Real x;
  };
  struct Output {
    Real e;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_a96057e9_5379_42d6_94a2_dd98930f3bd6 : public FuncModule<composite_block_empty_a96057e9_5379_42d6_94a2_dd98930f3bd6Traits> {
public:

  composite_block_empty_a96057e9_5379_42d6_94a2_dd98930f3bd6(Param param = Param{}, State state = State{})
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