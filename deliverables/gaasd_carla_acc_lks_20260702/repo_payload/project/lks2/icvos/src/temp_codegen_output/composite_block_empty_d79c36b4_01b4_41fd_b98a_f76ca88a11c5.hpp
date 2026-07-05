#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_d79c36b4_01b4_41fd_b98a_f76ca88a11c5Traits {
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

class composite_block_empty_d79c36b4_01b4_41fd_b98a_f76ca88a11c5 : public FuncModule<composite_block_empty_d79c36b4_01b4_41fd_b98a_f76ca88a11c5Traits> {
public:

  composite_block_empty_d79c36b4_01b4_41fd_b98a_f76ca88a11c5(Param param = Param{}, State state = State{})
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