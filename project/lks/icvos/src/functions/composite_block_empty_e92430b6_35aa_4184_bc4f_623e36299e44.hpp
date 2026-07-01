#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "lks.hpp"

namespace control {

struct composite_block_empty_e92430b6_35aa_4184_bc4f_623e36299e44Traits {
  struct Input {
    Real weightedError;
    Real egoV;
  };
  struct Output {
    Real steerRad;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_e92430b6_35aa_4184_bc4f_623e36299e44 : public FuncModule<composite_block_empty_e92430b6_35aa_4184_bc4f_623e36299e44Traits> {
public:

  composite_block_empty_e92430b6_35aa_4184_bc4f_623e36299e44(Param param = Param{}, State state = State{})
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