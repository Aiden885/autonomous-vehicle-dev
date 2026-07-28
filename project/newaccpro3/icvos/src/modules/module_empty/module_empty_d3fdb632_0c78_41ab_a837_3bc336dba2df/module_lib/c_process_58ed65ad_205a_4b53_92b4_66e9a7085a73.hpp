#pragma once

#include <cmath>

#include "run_2f3edfa5_8227_43eb_859d_1a6db0afd957.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct c_process_58ed65ad_205a_4b53_92b4_66e9a7085a73Traits {
  struct Input {
    
  };
  struct Output {
    
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    run_2f3edfa5_8227_43eb_859d_1a6db0afd957 compositeBlockInstance;
  };

  using Global = GlobalParams;
};

class c_process_58ed65ad_205a_4b53_92b4_66e9a7085a73 : public FuncModule<c_process_58ed65ad_205a_4b53_92b4_66e9a7085a73Traits> {
public:

  c_process_58ed65ad_205a_4b53_92b4_66e9a7085a73(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                run_2f3edfa5_8227_43eb_859d_1a6db0afd957(
                    run_2f3edfa5_8227_43eb_859d_1a6db0afd957Traits::Param{},
                    run_2f3edfa5_8227_43eb_859d_1a6db0afd957Traits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control