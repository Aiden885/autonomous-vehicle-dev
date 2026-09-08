#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560Traits {
  struct Input {
    Real rawSteerNorm;
    Real egoSpeedMps;
  };
  struct Output {
    Real limitedSteerNorm;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560 : public FuncModule<composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560Traits> {
public:

  composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560(Param param = Param{}, State state = State{})
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