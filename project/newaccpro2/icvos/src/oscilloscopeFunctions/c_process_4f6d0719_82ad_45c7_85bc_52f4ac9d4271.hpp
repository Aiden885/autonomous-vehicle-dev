#pragma once

#include "composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "newaccpro2.hpp"

namespace control {

struct c_process_4f6d0719_82ad_45c7_85bc_52f4ac9d4271Traits {
  struct Input {
    
  };
  struct Output {
    
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
    composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0 compositeBlockInstance_1;
  };

  using Global = GlobalParams;
};

class c_process_4f6d0719_82ad_45c7_85bc_52f4ac9d4271 : public FuncModule<c_process_4f6d0719_82ad_45c7_85bc_52f4ac9d4271Traits> {
public:

  c_process_4f6d0719_82ad_45c7_85bc_52f4ac9d4271(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0(
                    composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0Traits::Param{.vMin = 1},
                    composite_block_empty_1caa04a5_c82a_4b55_b303_308a908a9ad0Traits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control