#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364dTraits {
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

class composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364d : public FuncModule<composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364dTraits> {
public:

  composite_block_empty_90e598b1_e7ee_40ab_81c3_90e4075b364d(Param param = Param{}, State state = State{})
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