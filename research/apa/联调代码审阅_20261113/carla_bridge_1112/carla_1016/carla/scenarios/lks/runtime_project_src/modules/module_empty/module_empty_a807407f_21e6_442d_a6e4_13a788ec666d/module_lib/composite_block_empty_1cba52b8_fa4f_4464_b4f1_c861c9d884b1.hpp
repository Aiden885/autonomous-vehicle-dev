#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1Traits {
  struct Input {
    Real nearLateralErrorM;
    Real middleLateralErrorM;
    Real farLateralErrorM;
  };
  struct Output {
    Real weightedLateralErrorM;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1 : public FuncModule<composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1Traits> {
public:

  composite_block_empty_1cba52b8_fa4f_4464_b4f1_c861c9d884b1(Param param = Param{}, State state = State{})
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