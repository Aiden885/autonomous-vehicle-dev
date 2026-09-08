#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_fde75d0d_6428_4789_a7ea_30b7a3291b00Traits {
  struct Input {
    bool isSpeedUnavailable;
    bool isDriverOverride;
    bool isBrakePressed;
  };
  struct Output {
    bool isControlEnabled;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_fde75d0d_6428_4789_a7ea_30b7a3291b00 : public FuncModule<composite_block_empty_fde75d0d_6428_4789_a7ea_30b7a3291b00Traits> {
public:

  composite_block_empty_fde75d0d_6428_4789_a7ea_30b7a3291b00(Param param = Param{}, State state = State{})
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