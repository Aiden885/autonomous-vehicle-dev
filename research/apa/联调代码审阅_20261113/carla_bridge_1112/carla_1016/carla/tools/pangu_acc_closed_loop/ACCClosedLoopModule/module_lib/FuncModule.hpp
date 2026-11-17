#pragma once

#include <type_traits>
#include <utility>

namespace control {

using Real = double;

template <typename Traits>
class FuncModule {
public:
  using Input  = typename Traits::Input;
  using Output = typename Traits::Output;
  using Param  = typename Traits::Param;
  using State  = typename Traits::State;
  using Sub    = typename Traits::Sub;

  FuncModule() = default;

  template <class ParamT, class StateT,
            class = std::enable_if_t<
                std::is_constructible_v<Param, ParamT&&> &&
                std::is_constructible_v<State, StateT&&>>>
  explicit FuncModule(ParamT&& param, StateT&& state)
      : param_(std::forward<ParamT>(param)),
        state_(std::forward<StateT>(state)),
        sub_() {}

  template <class SubT, class ParamT, class StateT,
            class = std::enable_if_t<
                std::is_constructible_v<Sub, SubT&&> &&
                std::is_constructible_v<Param, ParamT&&> &&
                std::is_constructible_v<State, StateT&&>>>
  explicit FuncModule(SubT&& sub, ParamT&& param, StateT&& state)
      : param_(std::forward<ParamT>(param)),
        state_(std::forward<StateT>(state)),
        sub_(std::forward<SubT>(sub)) {}

  virtual ~FuncModule() = default;

  virtual void run(const Input& input, Output& output) = 0;

  virtual void setParam(const Param& param) { param_ = param; }
  virtual void resetParam() { param_ = Param{}; }

  virtual void setState(const State& state) { state_ = state; }
  virtual void resetState() { state_ = State{}; }

protected:
  Param param_;
  State state_;
  Sub sub_;
};

} // namespace control
