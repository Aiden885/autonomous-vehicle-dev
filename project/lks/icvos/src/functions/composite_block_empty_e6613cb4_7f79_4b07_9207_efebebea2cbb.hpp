#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "lks.hpp"

namespace control {

struct composite_block_empty_e6613cb4_7f79_4b07_9207_efebebea2cbbTraits {
  struct Input {
    Real c0;
    Real c1;
    Real c2;
    Real c3;
    Real x;
  };
  struct Output {
    Real e;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_e6613cb4_7f79_4b07_9207_efebebea2cbb : public FuncModule<composite_block_empty_e6613cb4_7f79_4b07_9207_efebebea2cbbTraits> {
public:

  composite_block_empty_e6613cb4_7f79_4b07_9207_efebebea2cbb(Param param = Param{}, State state = State{})
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