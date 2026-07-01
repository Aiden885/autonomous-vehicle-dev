#pragma once

#include "run_6d9ae1e1_3798_4640_91c6_8217a06e02aa.hpp"
#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct c_process_2127f508_badc_4885_90ef_379fa5649b3dTraits {
  struct Input {
    
  };
  struct Output {
    
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    run_6d9ae1e1_3798_4640_91c6_8217a06e02aa compositeBlockInstance_7;
  };

  using Global = GlobalParams;
};

class c_process_2127f508_badc_4885_90ef_379fa5649b3d : public FuncModule<c_process_2127f508_badc_4885_90ef_379fa5649b3dTraits> {
public:

  c_process_2127f508_badc_4885_90ef_379fa5649b3d(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                run_6d9ae1e1_3798_4640_91c6_8217a06e02aa(
                    run_6d9ae1e1_3798_4640_91c6_8217a06e02aaTraits::Param{},
                    run_6d9ae1e1_3798_4640_91c6_8217a06e02aaTraits::State{})},
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control