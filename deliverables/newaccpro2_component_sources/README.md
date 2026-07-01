# newaccpro2 自定义组件源码

## 目录说明

- `c_components/`
  - 当前 `newaccpro2` 画布实际使用的 C 组件源码。
  - 当前项目 JSON 中嵌入的函数体由该源码扫描生成。
- `cpp_funcmodule_components/`
  - 按 `docs/C++代码规范.docx v1.0.33` 改写的 C++ `FuncModule` 实现。
  - 六个组件均采用结构化 `Input/Output/Param/State/Sub`。
  - 按 GAASD 团队确认的 `@type atomic` 规则，CARLA 边界组件的
    `run()` 内部允许调用 `carla_adapter_*` C ABI。
  - ZMQ/JSON 依赖不进入组件源码，仍隔离在 adapter 动态库中。
  - 已在 GCC 9.4 C++20 下使用
    `-Wall -Wextra -Wpedantic -Werror` 编译通过。
  - `校验报告.md` 给出了规则对应关系和边界说明。
- `adapter_cpp/`
  - `carla_adapter_*` C ABI 接口的 C++ 实现。
  - 通过 ZMQ/JSON 与 CARLA Bridge 交换状态和控制命令。
  - 该目录属于运行时基础设施，不是 GAASD 扫描组件。
  - 生成工程运行时需要链接 `libcarla_gaasd_adapter.so`，否则
    `cpp_funcmodule_components` 中的边界组件无法解析 `carla_adapter_*`
    符号。

## 当前画布使用的自定义组件

1. `CARLAACCEgoSpeed`
2. `CARLAACCLeadSpeed`
3. `CARLAACCLeadDistance`
4. `CARLAACCDriverCommand`
5. `CARLAACCLongitudinalCmd`

`CARLAACCComputeTargetSpeed` 保留在源码中作为历史参考，当前目标速度由
GAASD 画布基础模块计算，没有使用该组件。

## 复现说明

排查当前 `newaccpro2` 代码生成问题时，应使用 `c_components` 中的源码及项目
原始 JSON。验证新版 C++ 扫描链路时，应只扫描
`cpp_funcmodule_components`，不要把 `adapter_cpp` 放入扫描目录，也不要与
当前 C 组件 JSON 混合作为同一次复现输入。

当前 C++ 组件采用“边界原子组件”路线：

1. `CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、`CARLAACCLeadDistance`、
   `CARLAACCDriverCommand` 在 `run()` 中调用 adapter 读取 Bridge 数据。
2. `CARLAACCLongitudinalCmd` 在 `run()` 中调用 adapter 发布控制命令。
3. `CARLAACCComputeTargetSpeed` 是纯计算组件，当前画布可继续用基础模块替代。
