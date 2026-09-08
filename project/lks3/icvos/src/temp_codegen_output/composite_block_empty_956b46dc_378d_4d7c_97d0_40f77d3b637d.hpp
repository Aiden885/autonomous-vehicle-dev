#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637dTraits {
  struct Input {
    Real driverSteerNorm;
  };
  struct Output {
    bool isDriverOverride;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637d : public FuncModule<composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637dTraits> {
public:

  composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637d(Param param = Param{}, State state = State{})
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