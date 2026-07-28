#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7Traits {
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

class composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7 : public FuncModule<composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7Traits> {
public:

  composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7(Param param = Param{}, State state = State{})
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
