#pragma once

#include "FuncModule.hpp"
#include "GlobalContext.hpp"
#include "MainInclude.hpp"

namespace control {

struct composite_block_empty_aa61974c_e982_4ef6_b6c2_e61dba56487dTraits {
  struct Input {
    Real egoV;
    Real curvature;
  };
  struct Output {
    Real previewDistance;
  };
  struct Param {
    
  };

  struct State {
    
  };

  struct Sub {
    
  };

  using Global = GlobalParams;
};

class composite_block_empty_aa61974c_e982_4ef6_b6c2_e61dba56487d : public FuncModule<composite_block_empty_aa61974c_e982_4ef6_b6c2_e61dba56487dTraits> {
public:

  composite_block_empty_aa61974c_e982_4ef6_b6c2_e61dba56487d(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                },
            std::move(param),
            std::move(state),
            global::params}
  {}

  void run(const Input &input, Output &output) override;
};

} // namespace control