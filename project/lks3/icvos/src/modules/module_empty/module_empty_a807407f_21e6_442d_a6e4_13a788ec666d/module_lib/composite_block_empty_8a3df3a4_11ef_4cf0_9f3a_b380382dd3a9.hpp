#pragma once

#include <cmath>

#include "composite_block_empty_fde75d0d_6428_4789_a7ea_30b7a3291b00.hpp"
#include "composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637d.hpp"
#include "composite_block_empty_179b05d8_8bbd_4e95_b179_c4e4788576c4.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_8a3df3a4_11ef_4cf0_9f3a_b380382dd3a9Traits {
  struct Input {
    Real egoSpeedMps;
    bool isBrakePressed;
    Real driverSteerNorm;
  };
  struct Output {
    bool isControlEnabled;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    composite_block_empty_179b05d8_8bbd_4e95_b179_c4e4788576c4 checkOperatingSpeed;
    composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637d detectDriverOverride;
    composite_block_empty_fde75d0d_6428_4789_a7ea_30b7a3291b00 determineControlEnable;
  };

  using Global = GlobalParams;
};

class composite_block_empty_8a3df3a4_11ef_4cf0_9f3a_b380382dd3a9 : public FuncModule<composite_block_empty_8a3df3a4_11ef_4cf0_9f3a_b380382dd3a9Traits> {
public:

  composite_block_empty_8a3df3a4_11ef_4cf0_9f3a_b380382dd3a9(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                composite_block_empty_179b05d8_8bbd_4e95_b179_c4e4788576c4(
                    composite_block_empty_179b05d8_8bbd_4e95_b179_c4e4788576c4Traits::Param{},
                    composite_block_empty_179b05d8_8bbd_4e95_b179_c4e4788576c4Traits::State{}),
                composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637d(
                    composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637dTraits::Param{},
                    composite_block_empty_956b46dc_378d_4d7c_97d0_40f77d3b637dTraits::State{}),
                composite_block_empty_fde75d0d_6428_4789_a7ea_30b7a3291b00(
                    composite_block_empty_fde75d0d_6428_4789_a7ea_30b7a3291b00Traits::Param{},
                    composite_block_empty_fde75d0d_6428_4789_a7ea_30b7a3291b00Traits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control