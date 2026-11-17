#pragma once

#include <cmath>

#include "run_9554bb10_0872_4665_b691_ff372271aaf9.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct c_process_3142a4bd_5591_4f61_8cfa_2e1a0f5fcf6eTraits {
  struct Input {
    
  };
  struct Output {
    
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    run_9554bb10_0872_4665_b691_ff372271aaf9 executeLks;
  };

  using Global = GlobalParams;
};

class c_process_3142a4bd_5591_4f61_8cfa_2e1a0f5fcf6e : public FuncModule<c_process_3142a4bd_5591_4f61_8cfa_2e1a0f5fcf6eTraits> {
public:

  c_process_3142a4bd_5591_4f61_8cfa_2e1a0f5fcf6e(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                run_9554bb10_0872_4665_b691_ff372271aaf9(
                    run_9554bb10_0872_4665_b691_ff372271aaf9Traits::Param{},
                    run_9554bb10_0872_4665_b691_ff372271aaf9Traits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control