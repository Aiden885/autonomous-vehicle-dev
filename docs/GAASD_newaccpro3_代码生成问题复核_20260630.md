# GAASD newaccpro3 代码生成问题复核记录

日期：2026-06-30

## 1. 复核对象

本次复核对象为 `newaccpro3` 在新版 GAASD / Pangu 架构下生成的代码：

```text
project/newaccpro3/icvos/src/temp_codegen_output
project/newaccpro3/icvos/src/modules/module_empty_d3fdb632_0c78_41ab_a837_3bc336dba2df/module_empty
project/newaccpro3/log/2026-06-30.log
```

本次没有修改画布和生成源码，只做日志、JSON 连线和生成代码检查。

## 2. 结论

当前 `newaccpro3` 的画布关键连线在 JSON 中存在，但生成后的 C++ 代码仍不能作为 GAASD 生成代码验证轨道使用。

主要原因不是 ACC 算法公式本身，而是代码生成器在以下环节仍有问题：

1. 全局支撑文件缺失。
2. 复合组件 `run()` / `composite_block()` 接口不一致。
3. 已存在的画布连线没有正确生成到子组件输入结构体。
4. 部分临时变量没有声明。
5. 自定义原子组件返回值生成出非法变量名。
6. 真值表函数只被调用，没有生成函数声明/定义。
7. Pangu 模块节点没有把生成的 `FuncStep` / `c_process` 接入 `Proc()`。

因此当前不能直接进行“GAASD 生成代码 vs 手写 oracle”的数值 diff。手写 Pangu ACC oracle 和真实 Bridge 闭环仍可继续作为联调基准。

## 3. 编译探针结果

执行：

```bash
cmake -S project/newaccpro3/icvos/src/temp_codegen_output -B /tmp/newaccpro3_codegen_probe_build
cmake --build /tmp/newaccpro3_codegen_probe_build -j2
```

第一层错误：

```text
fatal error: GlobalContext.hpp: 没有那个文件或目录
```

对应文件：

```text
project/newaccpro3/icvos/src/temp_codegen_output/composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad.hpp
project/newaccpro3/icvos/src/temp_codegen_output/run_2f3edfa5_8227_43eb_859d_1a6db0afd957.hpp
project/newaccpro3/icvos/src/temp_codegen_output/c_process_f6f0a377_a753_46a0_a55c_69db6ec01886.hpp
```

生成代码引用了 `GlobalContext.hpp` 和 `MainInclude.hpp`，但 `temp_codegen_output` 目录下没有生成这两个文件。

## 4. 关键问题证据

### 4.1 画布连线存在，但生成代码没有赋值给子组件输入

JSON 中确认存在以下连线：

```text
CARLA自车速度 -> ACCDecision.egoV
CARLA驾驶指令 -> ACCDecision.commandType
ACCDecision.enable -> CARLAACCLongitudinalCmd.enable
ACCDecision.timeGap -> DesiredDistance / 跟车控制链路
ACCDecision.maxSpeed -> 目标速度限幅
```

但生成代码中：

```cpp
composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988adTraits::Input compositeBlockInstance_2_in;
composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988adTraits::Output compositeBlockInstance_2_out;

sub_.compositeBlockInstance_2.composite_block(compositeBlockInstance_2_in, compositeBlockInstance_2_out);
```

没有生成：

```cpp
compositeBlockInstance_2_in.egoV = ...;
compositeBlockInstance_2_in.commandType = ...;
```

这会导致 ACCDecision 收不到真实输入。

### 4.2 `run()` / `composite_block()` 接口不一致

头文件声明：

```cpp
void run(const Input &input, Output &output) override;
```

cpp 实现：

```cpp
void composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad::composite_block(const Input &input, Output &output)
```

外层调用：

```cpp
sub_.compositeBlockInstance_2.composite_block(...);
```

这会导致接口不一致，当前类没有实现基类要求的 `run()`。

### 4.3 非法变量名仍存在

生成代码中存在：

```cpp
Real C++_None;
C++_None = CARLAACCDriverCommand();
```

`C++_None` 是非法 C++ 变量名，说明自定义原子组件返回值或未消费输出的变量名生成仍有问题。

### 4.4 未声明临时变量

生成代码中使用但没有声明：

```text
temp199951
temp196788
temp199342
temp204061
```

其中 `temp199951 / temp196788 / temp199342` 对应外层跟车控制中的全局参数或中间量，`temp204061` 对应 ACCDecision 内部 `FMIN_Gap` 输出。

### 4.5 真值表没有生成函数体

生成代码调用：

```cpp
truth_table_6957bd9f_a281_4c91_a9b5_5b2c83757f58(temp203065, input.commandType);
```

但当前 `temp_codegen_output` 下没有对应的函数声明和函数定义文件。后续如果要把 GAASD 生成代码作为 ACC 决策实现，真值表必须能生成可编译的函数体。

### 4.6 Pangu 模块节点没有接入画布生成代码

生成的 Pangu 模块：

```text
project/newaccpro3/icvos/src/modules/module_empty_d3fdb632_0c78_41ab_a837_3bc336dba2df/module_empty/src/module_empty.cpp
```

核心函数目前为空：

```cpp
bool module_empty::InitIO() {
  PLOG_INFO("in  InitIO");
  return true;
}

bool module_empty::Proc() {
  return true;
}
```

也就是说，即便 `temp_codegen_output` 修到能编译，当前 `module_empty` 也不会周期调用 `FuncStep` 或 `c_process.run()`。

### 4.7 datatype 空列表处理异常

日志中还有：

```text
global_services.json, 错误: 'list' object has no attribute 'get'
global_channels.json, 错误: 'list' object has no attribute 'get'
```

实际文件内容为空列表：

```json
[]
```

生成器应兼容空列表，或者项目数据应统一输出为生成器期望的对象结构。

## 5. 对当前测试方案的影响

当前不建议把 `newaccpro3` 的生成代码直接接入 Pangu ACC 闭环。

短期仍应采用已经跑通的方案：

```text
手写 ACC oracle + ZmqBridgeModule + mock / 真实 Bridge / CARLA
```

待 GAASD 代码生成器修复后，再执行：

```text
GAASD 生成代码 -> 编译通过 -> 与 oracle 同输入序列 diff -> 替换 oracle -> 再跑 mock 和真实 CARLA 闭环
```

## 6. 建议反馈给 GAASD 团队的最小问题清单

1. 生成 `GlobalContext.hpp` / `MainInclude.hpp`，或从生成的 include 中移除无效引用。
2. 复合组件统一接口：生成代码应使用 `run()`，或头文件声明 `composite_block()`，不能两套混用。
3. 根据 JSON 连接关系生成子组件输入赋值，例如 `compositeBlockInstance_2_in.egoV` 和 `commandType`。
4. 修复原子组件返回值未消费时生成 `C++_None` 的问题。
5. 修复临时变量漏声明问题。
6. 生成真值表函数声明和函数体。
7. Pangu `module_empty::Proc()` 需要接入生成的 `c_process.run()` 或等价 callback。
8. `global_services.json/global_channels.json` 为空列表时不要抛异常。

