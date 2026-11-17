#pragma once

#include <cmath>

#include "run_f1807d24_61ef_407b_932e_4df61bda48e7.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct c_process_cf2d7a2f_2987_4a9b_aaa4_a4caa419f982Traits {
  struct Input {
    
  };
  struct Output {
    
  };
  struct Param {
    Real minDistanceM;
    Real distanceErrorGainPerS;
    Real relativeSpeedGain;
  };

  struct State {
    
  };

  struct Sub {
    run_f1807d24_61ef_407b_932e_4df61bda48e7 compositeBlockInstance;
  };

  using Global = GlobalParams;
};

class c_process_cf2d7a2f_2987_4a9b_aaa4_a4caa419f982 : public FuncModule<c_process_cf2d7a2f_2987_4a9b_aaa4_a4caa419f982Traits> {
public:

  c_process_cf2d7a2f_2987_4a9b_aaa4_a4caa419f982(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                run_f1807d24_61ef_407b_932e_4df61bda48e7(
                    run_f1807d24_61ef_407b_932e_4df61bda48e7Traits::Param{.minDistanceM = 5, .distanceErrorGainPerS = 0.25, .relativeSpeedGain = 0.4},
                    run_f1807d24_61ef_407b_932e_4df61bda48e7Traits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control