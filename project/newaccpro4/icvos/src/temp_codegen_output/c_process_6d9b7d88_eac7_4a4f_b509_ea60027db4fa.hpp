#pragma once

#include <cmath>

#include "run_23d2a150_83e4_470e_a9b7_189357a9dfd1.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct c_process_6d9b7d88_eac7_4a4f_b509_ea60027db4faTraits {
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
    run_23d2a150_83e4_470e_a9b7_189357a9dfd1 compositeBlockInstance_1;
  };

  using Global = GlobalParams;
};

class c_process_6d9b7d88_eac7_4a4f_b509_ea60027db4fa : public FuncModule<c_process_6d9b7d88_eac7_4a4f_b509_ea60027db4faTraits> {
public:

  c_process_6d9b7d88_eac7_4a4f_b509_ea60027db4fa(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                run_23d2a150_83e4_470e_a9b7_189357a9dfd1(
                    run_23d2a150_83e4_470e_a9b7_189357a9dfd1Traits::Param{.minDistanceM = 5, .distanceErrorGainPerS = 0.25, .relativeSpeedGain = 0.4},
                    run_23d2a150_83e4_470e_a9b7_189357a9dfd1Traits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control