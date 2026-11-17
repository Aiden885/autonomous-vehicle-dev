#pragma once

#include <cmath>

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_179b05d8_8bbd_4e95_b179_c4e4788576c4Traits {
  struct Input {
    Real egoSpeedMps;
  };
  struct Output {
    bool isSpeedUnavailable;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_179b05d8_8bbd_4e95_b179_c4e4788576c4 : public FuncModule<composite_block_empty_179b05d8_8bbd_4e95_b179_c4e4788576c4Traits> {
public:

  composite_block_empty_179b05d8_8bbd_4e95_b179_c4e4788576c4(Param param = Param{}, State state = State{})
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