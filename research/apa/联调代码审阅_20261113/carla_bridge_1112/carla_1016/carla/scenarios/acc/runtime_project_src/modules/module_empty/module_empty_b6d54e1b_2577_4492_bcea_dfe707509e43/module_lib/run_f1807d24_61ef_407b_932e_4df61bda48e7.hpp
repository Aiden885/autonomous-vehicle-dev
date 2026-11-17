#pragma once

#include <cmath>

#include "composite_block_empty_797d2bad_bc58_4269_9d44_d338e3691a62.hpp"
#include "run_dd25dc36_745c_471a_8c2b_515194ab1e3f.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_f1807d24_61ef_407b_932e_4df61bda48e7Traits {
  struct Input {
    Real egoSpeedMps;
    Real leadSpeedMps;
    int commandType;
    Real leadDistanceM;
  };
  struct Output {
    bool isControlEnabled;
    Real targetSpeedMps;
  };
  struct Param {
    Real minDistanceM;
    Real distanceErrorGainPerS;
    Real relativeSpeedGain;
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
    composite_block_empty_797d2bad_bc58_4269_9d44_d338e3691a62 decision_1;
    run_dd25dc36_745c_471a_8c2b_515194ab1e3f control_1;
  };

  using Global = GlobalParams;
};

class run_f1807d24_61ef_407b_932e_4df61bda48e7 : public FuncModule<run_f1807d24_61ef_407b_932e_4df61bda48e7Traits> {
public:

  run_f1807d24_61ef_407b_932e_4df61bda48e7(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                composite_block_empty_797d2bad_bc58_4269_9d44_d338e3691a62(
                    composite_block_empty_797d2bad_bc58_4269_9d44_d338e3691a62Traits::Param{.minimumOperatingSpeedMps = 1, .timeGapStepS = 0.2, .minTimeGapS = 1, .maxTimeGapS = 5, .speedStepMps = 4, .minSpeedMps = 0},
                    composite_block_empty_797d2bad_bc58_4269_9d44_d338e3691a62Traits::State{.isControlActive = 0, .hasControlHistory = 0, .timeGapS = 1.8, .maxSpeedMps = 20}),
                run_dd25dc36_745c_471a_8c2b_515194ab1e3f(
                    run_dd25dc36_745c_471a_8c2b_515194ab1e3fTraits::Param{},
                    run_dd25dc36_745c_471a_8c2b_515194ab1e3fTraits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control