#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988adTraits {
  struct Input {
    int commandType;
    Real egoV;
  };
  struct Output {
    int enable;
    Real timeGap;
    Real maxSpeed;
  };
  struct Param {
    Real vMin;
    Real GapStep;
    Real MinGap;
    Real MaxGap;
    Real SpdStep;
    Real MinSpd;
  };

  struct State {
    int controlEnabled;
    int hasHistory;
    Real timeGap;
    Real maxSpeed;
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad : public FuncModule<composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988adTraits> {
public:

  composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad(Param param = Param{}, State state = State{})
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