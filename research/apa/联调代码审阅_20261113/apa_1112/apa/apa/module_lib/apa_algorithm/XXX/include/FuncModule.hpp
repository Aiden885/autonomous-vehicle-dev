#pragma once

#include <cstdint>
#include <utility>

namespace apa {

using Real = float;
using Real64 = double;
using Integer = std::int32_t;
using UInteger = std::uint32_t;
using Integer16 = std::int16_t;
using UInteger16 = std::uint16_t;
using Integer8 = std::int8_t;
using UInteger8 = std::uint8_t;
using Boolean = bool;

template <typename Traits>
class FuncModule {
 public:
  using Input = typename Traits::Input;
  using Output = typename Traits::Output;
  using Param = typename Traits::Param;
  using State = typename Traits::State;
  using Sub = typename Traits::Sub;

  FuncModule(Param param = Param{}, State state = State{})
      : sub_{}, param_(std::move(param)), state_(std::move(state)) {}

  FuncModule(Sub sub, Param param, State state)
      : sub_(std::move(sub)),
        param_(std::move(param)),
        state_(std::move(state)) {}

  virtual ~FuncModule() = default;
  virtual void run(const Input& input, Output& output) = 0;

 protected:
  Sub sub_;
  Param param_;
  State state_;
};

}
