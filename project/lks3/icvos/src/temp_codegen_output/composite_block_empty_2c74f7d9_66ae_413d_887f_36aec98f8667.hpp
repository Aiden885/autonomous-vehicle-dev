#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667Traits {
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

class composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667 : public FuncModule<composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667Traits> {
public:

  composite_block_empty_2c74f7d9_66ae_413d_887f_36aec98f8667(Param param = Param{}, State state = State{})
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