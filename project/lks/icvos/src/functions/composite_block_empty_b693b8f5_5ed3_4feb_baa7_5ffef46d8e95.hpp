#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "lks.hpp"

namespace control {

struct composite_block_empty_b693b8f5_5ed3_4feb_baa7_5ffef46d8e95Traits {
  struct Input {
    Real egoV;
    Real brakePressed;
    Real driverSteerNorm;
  };
  struct Output {
    Real controlEnabled;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_b693b8f5_5ed3_4feb_baa7_5ffef46d8e95 : public FuncModule<composite_block_empty_b693b8f5_5ed3_4feb_baa7_5ffef46d8e95Traits> {
public:

  composite_block_empty_b693b8f5_5ed3_4feb_baa7_5ffef46d8e95(Param param = Param{}, State state = State{})
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