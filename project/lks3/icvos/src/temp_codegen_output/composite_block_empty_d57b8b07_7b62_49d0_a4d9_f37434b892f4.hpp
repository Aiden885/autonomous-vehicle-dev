#pragma once

#include <cmath>

#include "composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560.hpp"
#include "composite_block_empty_64c982f5_0edb_4c03_b06c_9a22f84347c5.hpp"
#include "composite_block_empty_5c77e1f3_497c_42c2_92d4_32dce5cd71b9.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_d57b8b07_7b62_49d0_a4d9_f37434b892f4Traits {
  struct Input {
    Real weightedLateralErrorM;
    Real egoSpeedMps;
    bool isControlEnabled;
  };
  struct Output {
    Real steerRad;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    composite_block_empty_5c77e1f3_497c_42c2_92d4_32dce5cd71b9 calculateRawSteer;
    composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560 limitSteerByLateralAcceleration;
    composite_block_empty_64c982f5_0edb_4c03_b06c_9a22f84347c5 applySteerEnable;
  };

  using Global = GlobalParams;
};

class composite_block_empty_d57b8b07_7b62_49d0_a4d9_f37434b892f4 : public FuncModule<composite_block_empty_d57b8b07_7b62_49d0_a4d9_f37434b892f4Traits> {
public:

  composite_block_empty_d57b8b07_7b62_49d0_a4d9_f37434b892f4(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                composite_block_empty_5c77e1f3_497c_42c2_92d4_32dce5cd71b9(
                    composite_block_empty_5c77e1f3_497c_42c2_92d4_32dce5cd71b9Traits::Param{},
                    composite_block_empty_5c77e1f3_497c_42c2_92d4_32dce5cd71b9Traits::State{}),
                composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560(
                    composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560Traits::Param{},
                    composite_block_empty_6974723c_cd33_4337_b0ed_1edecaacd560Traits::State{}),
                composite_block_empty_64c982f5_0edb_4c03_b06c_9a22f84347c5(
                    composite_block_empty_64c982f5_0edb_4c03_b06c_9a22f84347c5Traits::Param{},
                    composite_block_empty_64c982f5_0edb_4c03_b06c_9a22f84347c5Traits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control