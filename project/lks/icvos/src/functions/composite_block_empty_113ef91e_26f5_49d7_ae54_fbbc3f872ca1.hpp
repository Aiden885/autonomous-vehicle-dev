#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "lks.hpp"

namespace control {

struct composite_block_empty_113ef91e_26f5_49d7_ae54_fbbc3f872ca1Traits {
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

class composite_block_empty_113ef91e_26f5_49d7_ae54_fbbc3f872ca1 : public FuncModule<composite_block_empty_113ef91e_26f5_49d7_ae54_fbbc3f872ca1Traits> {
public:

  composite_block_empty_113ef91e_26f5_49d7_ae54_fbbc3f872ca1(Param param = Param{}, State state = State{})
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