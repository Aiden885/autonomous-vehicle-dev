#pragma once

#include <cmath>

#include "run_9554bb10_0872_4665_b691_ff372271aaf9.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct c_process_1c4be1a0_1274_416e_bd0d_f9836cd00acdTraits {
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

class c_process_1c4be1a0_1274_416e_bd0d_f9836cd00acd : public FuncModule<c_process_1c4be1a0_1274_416e_bd0d_f9836cd00acdTraits> {
public:

  c_process_1c4be1a0_1274_416e_bd0d_f9836cd00acd(Param param = Param{}, State state = State{})
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