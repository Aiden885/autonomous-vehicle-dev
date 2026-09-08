#pragma once

#include <cmath>

#include "run_b12be11b_48bd_4d14_be5e_a2496219d1f7.hpp"
#include "run_863e2a9d_edaf_4991_8be8_f0d5354e6a16.hpp"
#include "run_a08264b6_d877_4a94_8756_9c8b9c5fde57.hpp"
#include "run_008d73bb_e84f_46a0_b967_18007ec5678e.hpp"
#include "truth_table_8d98fa7f_4daf_49ae_bb29_69662b8ce656.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_797d2bad_bc58_4269_9d44_d338e3691a62Traits {
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
    run_b12be11b_48bd_4d14_be5e_a2496219d1f7 classifySystemState_1;
    run_008d73bb_e84f_46a0_b967_18007ec5678e updateControlState_1;
    run_a08264b6_d877_4a94_8756_9c8b9c5fde57 updateTimeGap_1;
    run_863e2a9d_edaf_4991_8be8_f0d5354e6a16 updateMaxSpeed_1;
  };

  using Global = GlobalParams;
};

class composite_block_empty_797d2bad_bc58_4269_9d44_d338e3691a62 : public FuncModule<composite_block_empty_797d2bad_bc58_4269_9d44_d338e3691a62Traits> {
public:

  composite_block_empty_797d2bad_bc58_4269_9d44_d338e3691a62(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                run_b12be11b_48bd_4d14_be5e_a2496219d1f7(
                    run_b12be11b_48bd_4d14_be5e_a2496219d1f7Traits::Param{},
                    run_b12be11b_48bd_4d14_be5e_a2496219d1f7Traits::State{}),
                run_008d73bb_e84f_46a0_b967_18007ec5678e(
                    run_008d73bb_e84f_46a0_b967_18007ec5678eTraits::Param{},
                    run_008d73bb_e84f_46a0_b967_18007ec5678eTraits::State{}),
                run_a08264b6_d877_4a94_8756_9c8b9c5fde57(
                    run_a08264b6_d877_4a94_8756_9c8b9c5fde57Traits::Param{},
                    run_a08264b6_d877_4a94_8756_9c8b9c5fde57Traits::State{}),
                run_863e2a9d_edaf_4991_8be8_f0d5354e6a16(
                    run_863e2a9d_edaf_4991_8be8_f0d5354e6a16Traits::Param{},
                    run_863e2a9d_edaf_4991_8be8_f0d5354e6a16Traits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control