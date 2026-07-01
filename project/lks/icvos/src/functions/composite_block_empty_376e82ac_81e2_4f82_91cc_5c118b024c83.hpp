#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "lks.hpp"

namespace control {

struct composite_block_empty_376e82ac_81e2_4f82_91cc_5c118b024c83Traits {
  struct Input {
    Real egoV;
    Real curvature;
  };
  struct Output {
    Real previewDistance;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_376e82ac_81e2_4f82_91cc_5c118b024c83 : public FuncModule<composite_block_empty_376e82ac_81e2_4f82_91cc_5c118b024c83Traits> {
public:

  composite_block_empty_376e82ac_81e2_4f82_91cc_5c118b024c83(Param param = Param{}, State state = State{})
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