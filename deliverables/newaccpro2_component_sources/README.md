# newaccpro2 自定义组件源码

## 目录说明

- `c_components/`
  - 当前 `newaccpro2` 画布实际使用的 C 组件源码。
  - 当前项目 JSON 中嵌入的函数体由该源码扫描生成。
- `cpp_funcmodule_components/`
  - 按 `docs/C++代码规范.docx v1.0.33` 改写的 C++ `FuncModule` 实现。
  - 六个组件均采用结构化 `Input/Output/Param/State/Sub`，不含指针、
    外部函数调用、ZMQ/JSON 依赖或运行时副作用。
  - 已在 GCC 9.4 C++20 下使用
    `-Wall -Wextra -Wpedantic -Werror` 编译通过。
  - `校验报告.md` 给出了规则对应关系和边界说明。
- `adapter_cpp/`
  - `carla_adapter_*` C ABI 接口的 C++ 实现。
  - 通过 ZMQ/JSON 与 CARLA Bridge 交换状态和控制命令。
  - 该目录属于运行时基础设施，不是 GAASD 扫描组件，不能放入组件
    `run()` 内，否则会违反组件规范的依赖和函数体白名单。

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

规范版 C++ 组件不直接读写 Bridge。实际联调时，GAASD 运行时或节点框架需要：

1. 将 Bridge 的 `egoV/leadV/distance/commandType/valid` 注入相应组件输入。
2. 读取 `CARLAACCLongitudinalCmd` 的 `speed/enable/valid` 输出并发布到 Bridge。
