#pragma once

#include "composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6.hpp"
#include "composite_block_empty_aa61974c_e982_4ef6_b6c2_e61dba56487d.hpp"
#include "composite_block_empty_b25a47b1_cf11_4375_a368_3b21bdca7e68.hpp"
#include "composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7.hpp"
#include "composite_block_empty_d79c36b4_01b4_41fd_b98a_f76ca88a11c5.hpp"
#include "composite_block_empty_a96057e9_5379_42d6_94a2_dd98930f3bd6.hpp"
#include "composite_block_empty_35bfe571_0409_485f_b070_0999518ce7bc.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct run_6d9ae1e1_3798_4640_91c6_8217a06e02aaTraits {
  struct Input {
    
  };
  struct Output {
    
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    composite_block_empty_aa61974c_e982_4ef6_b6c2_e61dba56487d compositeBlockInstance_8;
    composite_block_empty_a96057e9_5379_42d6_94a2_dd98930f3bd6 compositeBlockInstance_9;
    composite_block_empty_d79c36b4_01b4_41fd_b98a_f76ca88a11c5 compositeBlockInstance_10;
    composite_block_empty_b25a47b1_cf11_4375_a368_3b21bdca7e68 compositeBlockInstance_11;
    composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6 compositeBlockInstance_12;
    composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7 compositeBlockInstance_13;
    composite_block_empty_35bfe571_0409_485f_b070_0999518ce7bc compositeBlockInstance_14;
  };

  using Global = GlobalParams;
};

class run_6d9ae1e1_3798_4640_91c6_8217a06e02aa : public FuncModule<run_6d9ae1e1_3798_4640_91c6_8217a06e02aaTraits> {
public:

  run_6d9ae1e1_3798_4640_91c6_8217a06e02aa(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                composite_block_empty_aa61974c_e982_4ef6_b6c2_e61dba56487d(
                    composite_block_empty_aa61974c_e982_4ef6_b6c2_e61dba56487dTraits::Param{},
                    composite_block_empty_aa61974c_e982_4ef6_b6c2_e61dba56487dTraits::State{}),
                composite_block_empty_a96057e9_5379_42d6_94a2_dd98930f3bd6(
                    composite_block_empty_a96057e9_5379_42d6_94a2_dd98930f3bd6Traits::Param{},
                    composite_block_empty_a96057e9_5379_42d6_94a2_dd98930f3bd6Traits::State{}),
                composite_block_empty_d79c36b4_01b4_41fd_b98a_f76ca88a11c5(
                    composite_block_empty_d79c36b4_01b4_41fd_b98a_f76ca88a11c5Traits::Param{},
                    composite_block_empty_d79c36b4_01b4_41fd_b98a_f76ca88a11c5Traits::State{}),
                composite_block_empty_b25a47b1_cf11_4375_a368_3b21bdca7e68(
                    composite_block_empty_b25a47b1_cf11_4375_a368_3b21bdca7e68Traits::Param{},
                    composite_block_empty_b25a47b1_cf11_4375_a368_3b21bdca7e68Traits::State{}),
                composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6(
                    composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6Traits::Param{},
                    composite_block_empty_408bc885_5a8d_48df_ac8b_d2cc40f1e9d6Traits::State{}),
                composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7(
                    composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7Traits::Param{},
                    composite_block_empty_14f902c4_eeee_4130_bb60_0e0218c9aea7Traits::State{}),
                composite_block_empty_35bfe571_0409_485f_b070_0999518ce7bc(
                    composite_block_empty_35bfe571_0409_485f_b070_0999518ce7bcTraits::Param{},
                    composite_block_empty_35bfe571_0409_485f_b070_0999518ce7bcTraits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control