#pragma once

#include <utility>

namespace control {

// 统一定义数值类型，所有模块共享
using Real = double;

// 功能模块基类模板（移除 C++20 Concept，纯传统模板）
template <typename Traits>
class FuncModule {
public:
    // 类型别名（从 Traits 中提取）
    using Input  = typename Traits::Input;
    using Output = typename Traits::Output;
    using Param  = typename Traits::Param;
    using State  = typename Traits::State;
    using Sub    = typename Traits::Sub;
    using Global = typename Traits::Global;

    // 构造函数注入 Sub、Param、State、Global
    explicit FuncModule(Sub sub = Sub{},
                        Param param = Param{},
                        State state = State{},
                        Global global = Global{})
        : param_(std::move(param)),
          sub_(std::move(sub)),
          state_(std::move(state)),
          global_(std::move(global)) {}

    virtual ~FuncModule() = default;

    // === 抽象接口（子类必须实现）===
    virtual void run(const Input& input, Output& output) = 0;

    // === 共性实现 ===
    virtual void setParam(const Param& param) { param_ = param; }
    virtual void resetParam() { param_ = Param{}; }

    virtual void setState(const State& state) { state_ = state; }
    virtual void resetState() { state_ = State{}; }

protected:
    // 四大存储槽
    Param   param_;    // 配置参数
    Sub     sub_;      // 子模块容器
    State   state_;    // 内部状态
    Global  global_;   // 全局注入
};

} // namespace control