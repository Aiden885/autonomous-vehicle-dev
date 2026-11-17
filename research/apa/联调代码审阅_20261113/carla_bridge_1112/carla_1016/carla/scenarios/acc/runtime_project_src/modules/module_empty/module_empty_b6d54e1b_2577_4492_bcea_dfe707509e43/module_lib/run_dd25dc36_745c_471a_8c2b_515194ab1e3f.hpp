#pragma once

#include <cmath>

#include "run_a5e68afa_fef4_4c76_9139_cb7fef8f0aa2.hpp"
#include "run_ab2e0a9d_5eb7_4919_9ca7_de51932abea5.hpp"
#include "run_51f83135_f79a_42dc_8c89_9463090201b8.hpp"
#include "run_f4207eef_c51d_5cc8_b12e_c2930682ae01.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_dd25dc36_745c_471a_8c2b_515194ab1e3fTraits {
  struct Input {
    Real egoSpeedMps;
    Real leadSpeedMps;
    Real leadDistanceM;
    Real timeGapS;
    Real maxSpeedMps;
    int isControlEnabled;
    Real minDistanceM;
    Real distanceErrorGainPerS;
    Real relativeSpeedGain;
  };
  struct Output {
    Real targetSpeedMps;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    run_a5e68afa_fef4_4c76_9139_cb7fef8f0aa2 calculateSpacingFeedback_1;
    run_ab2e0a9d_5eb7_4919_9ca7_de51932abea5 calculateRelativeSpeedFeedback_1;
    run_51f83135_f79a_42dc_8c89_9463090201b8 generateTargetSpeed_1;
    run_f4207eef_c51d_5cc8_b12e_c2930682ae01 calculateAccelerationCommand_1;
  };

  using Global = GlobalParams;
};

class run_dd25dc36_745c_471a_8c2b_515194ab1e3f : public FuncModule<run_dd25dc36_745c_471a_8c2b_515194ab1e3fTraits> {
public:

  run_dd25dc36_745c_471a_8c2b_515194ab1e3f(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                run_a5e68afa_fef4_4c76_9139_cb7fef8f0aa2(
                    run_a5e68afa_fef4_4c76_9139_cb7fef8f0aa2Traits::Param{},
                    run_a5e68afa_fef4_4c76_9139_cb7fef8f0aa2Traits::State{}),
                run_ab2e0a9d_5eb7_4919_9ca7_de51932abea5(
                    run_ab2e0a9d_5eb7_4919_9ca7_de51932abea5Traits::Param{},
                    run_ab2e0a9d_5eb7_4919_9ca7_de51932abea5Traits::State{}),
                run_51f83135_f79a_42dc_8c89_9463090201b8(
                    run_51f83135_f79a_42dc_8c89_9463090201b8Traits::Param{},
                    run_51f83135_f79a_42dc_8c89_9463090201b8Traits::State{}),
                run_f4207eef_c51d_5cc8_b12e_c2930682ae01(
                    run_f4207eef_c51d_5cc8_b12e_c2930682ae01Traits::Param{},
                    run_f4207eef_c51d_5cc8_b12e_c2930682ae01Traits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control