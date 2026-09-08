#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_5c77e1f3_497c_42c2_92d4_32dce5cd71b9Traits {
  struct Input {
    Real weightedLateralErrorM;
  };
  struct Output {
    Real rawSteerNorm;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_5c77e1f3_497c_42c2_92d4_32dce5cd71b9 : public FuncModule<composite_block_empty_5c77e1f3_497c_42c2_92d4_32dce5cd71b9Traits> {
public:

  composite_block_empty_5c77e1f3_497c_42c2_92d4_32dce5cd71b9(Param param = Param{}, State state = State{})
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