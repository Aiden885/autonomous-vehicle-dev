#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "newaccpro2.hpp"

namespace control {

struct composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0Traits {
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

class composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0 : public FuncModule<composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0Traits> {
public:

  composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0(Param param = Param{}, State state = State{})
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