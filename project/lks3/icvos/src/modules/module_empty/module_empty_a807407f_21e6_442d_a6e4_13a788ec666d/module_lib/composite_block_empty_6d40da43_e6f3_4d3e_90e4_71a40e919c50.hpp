#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50Traits {
  struct Input {
    Real egoSpeedMps;
    Real roadCurvaturePerM;
  };
  struct Output {
    Real previewDistanceM;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50 : public FuncModule<composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50Traits> {
public:

  composite_block_empty_6d40da43_e6f3_4d3e_90e4_71a40e919c50(Param param = Param{}, State state = State{})
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