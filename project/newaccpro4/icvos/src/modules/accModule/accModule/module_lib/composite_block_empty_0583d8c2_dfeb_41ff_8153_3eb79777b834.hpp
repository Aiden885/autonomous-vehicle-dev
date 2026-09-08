#pragma once

#include <cmath>

#include "run_b71d71c8_ff00_4f2b_b015_08ac387673a6.hpp"
#include "run_c5c7aca9_1395_45ea_b8fe_138f53906472.hpp"
#include "run_3d336890_b7fc_408b_ac40_17cd5dce2b59.hpp"
#include "run_a15a0aa8_22ce_4f0e_8b20_5deb0a4478e5.hpp"
#include "truth_table_a5de6758_3b22_4570_950f_388d963d3433.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_0583d8c2_dfeb_41ff_8153_3eb79777b834Traits {
  struct Input {
    int commandType;
    Real egoSpeedMps;
  };
  struct Output {
    int isControlEnabled;
    Real timeGapS;
    Real maxSpeedMps;
  };
  struct Param {
    Real minimumOperatingSpeedMps;
    Real timeGapStepS;
    Real minTimeGapS;
    Real maxTimeGapS;
    Real speedStepMps;
    Real minSpeedMps;
  };

  struct State {
    int isControlActive;
    int hasControlHistory;
    Real timeGapS;
    Real maxSpeedMps;
  };

  struct Sub {
    run_b71d71c8_ff00_4f2b_b015_08ac387673a6 classifySystemState_2;
    run_3d336890_b7fc_408b_ac40_17cd5dce2b59 updateControlState_2;
    run_c5c7aca9_1395_45ea_b8fe_138f53906472 updateTimeGap_2;
    run_a15a0aa8_22ce_4f0e_8b20_5deb0a4478e5 updateMaxSpeed_2;
  };

  using Global = GlobalParams;
};

class composite_block_empty_0583d8c2_dfeb_41ff_8153_3eb79777b834 : public FuncModule<composite_block_empty_0583d8c2_dfeb_41ff_8153_3eb79777b834Traits> {
public:

  composite_block_empty_0583d8c2_dfeb_41ff_8153_3eb79777b834(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                run_b71d71c8_ff00_4f2b_b015_08ac387673a6(
                    run_b71d71c8_ff00_4f2b_b015_08ac387673a6Traits::Param{},
                    run_b71d71c8_ff00_4f2b_b015_08ac387673a6Traits::State{}),
                run_3d336890_b7fc_408b_ac40_17cd5dce2b59(
                    run_3d336890_b7fc_408b_ac40_17cd5dce2b59Traits::Param{},
                    run_3d336890_b7fc_408b_ac40_17cd5dce2b59Traits::State{}),
                run_c5c7aca9_1395_45ea_b8fe_138f53906472(
                    run_c5c7aca9_1395_45ea_b8fe_138f53906472Traits::Param{},
                    run_c5c7aca9_1395_45ea_b8fe_138f53906472Traits::State{}),
                run_a15a0aa8_22ce_4f0e_8b20_5deb0a4478e5(
                    run_a15a0aa8_22ce_4f0e_8b20_5deb0a4478e5Traits::Param{},
                    run_a15a0aa8_22ce_4f0e_8b20_5deb0a4478e5Traits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control