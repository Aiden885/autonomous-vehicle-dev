#pragma once

#include <cmath>

#include "composite_block_empty_6b682752_6234_4817_a302_c9e173c6790e.hpp"
#include "composite_block_empty_8a3df3a4_11ef_4cf0_9f3a_b380382dd3a9.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_9554bb10_0872_4665_b691_ff372271aaf9Traits {
  struct Input {
    Real egoSpeedMps;
    Real c0;
    Real c1;
    Real c2;
    Real c3;
    Real roadCurvaturePerM;
    bool isBrakePressed;
    Real driverSteerNorm;
  };
  struct Output {
    Real steerRad;
    bool isControlEnabled;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    composite_block_empty_8a3df3a4_11ef_4cf0_9f3a_b380382dd3a9 decision;
    composite_block_empty_6b682752_6234_4817_a302_c9e173c6790e control;
  };

  using Global = GlobalParams;
};

class run_9554bb10_0872_4665_b691_ff372271aaf9 : public FuncModule<run_9554bb10_0872_4665_b691_ff372271aaf9Traits> {
public:

  run_9554bb10_0872_4665_b691_ff372271aaf9(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                composite_block_empty_8a3df3a4_11ef_4cf0_9f3a_b380382dd3a9(
                    composite_block_empty_8a3df3a4_11ef_4cf0_9f3a_b380382dd3a9Traits::Param{},
                    composite_block_empty_8a3df3a4_11ef_4cf0_9f3a_b380382dd3a9Traits::State{}),
                composite_block_empty_6b682752_6234_4817_a302_c9e173c6790e(
                    composite_block_empty_6b682752_6234_4817_a302_c9e173c6790eTraits::Param{},
                    composite_block_empty_6b682752_6234_4817_a302_c9e173c6790eTraits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control