#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_35bfe571_0409_485f_b070_0999518ce7bcTraits {
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

class composite_block_empty_35bfe571_0409_485f_b070_0999518ce7bc : public FuncModule<composite_block_empty_35bfe571_0409_485f_b070_0999518ce7bcTraits> {
public:

  composite_block_empty_35bfe571_0409_485f_b070_0999518ce7bc(Param param = Param{}, State state = State{})
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