#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_863e2a9d_edaf_4991_8be8_f0d5354e6a16Traits {
  struct Input {
    int decisionCode;
    Real maxSpeedMps;
    Real speedStepMps;
    Real minSpeedMps;
  };
  struct Output {
    Real nextMaxSpeedMps;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class run_863e2a9d_edaf_4991_8be8_f0d5354e6a16 : public FuncModule<run_863e2a9d_edaf_4991_8be8_f0d5354e6a16Traits> {
public:

  run_863e2a9d_edaf_4991_8be8_f0d5354e6a16(Param param = Param{}, State state = State{})
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