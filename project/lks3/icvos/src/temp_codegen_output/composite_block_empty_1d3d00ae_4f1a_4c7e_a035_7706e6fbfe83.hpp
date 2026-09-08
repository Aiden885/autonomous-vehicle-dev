#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83Traits {
  struct Input {
    Real c0;
    Real c1;
    Real c2;
    Real c3;
    Real previewPointDistanceM;
  };
  struct Output {
    Real lateralErrorM;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83 : public FuncModule<composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83Traits> {
public:

  composite_block_empty_1d3d00ae_4f1a_4c7e_a035_7706e6fbfe83(Param param = Param{}, State state = State{})
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