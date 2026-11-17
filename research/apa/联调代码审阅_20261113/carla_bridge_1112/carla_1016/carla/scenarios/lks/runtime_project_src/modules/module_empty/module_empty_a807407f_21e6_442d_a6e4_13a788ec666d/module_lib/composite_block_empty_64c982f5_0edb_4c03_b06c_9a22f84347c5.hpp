#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_64c982f5_0edb_4c03_b06c_9a22f84347c5Traits {
  struct Input {
    Real limitedSteerNorm;
    bool isControlEnabled;
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

class composite_block_empty_64c982f5_0edb_4c03_b06c_9a22f84347c5 : public FuncModule<composite_block_empty_64c982f5_0edb_4c03_b06c_9a22f84347c5Traits> {
public:

  composite_block_empty_64c982f5_0edb_4c03_b06c_9a22f84347c5(Param param = Param{}, State state = State{})
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