# source/generated 改写代码规范验证问题实例

验证依据：`docs/C++代码规范.md` v1.0.31

验证范围：

- `source/generated/gaasd_lks_components_src`
- `source/generated/gaasd_p0_acc_min_components_src`
- `source/generated/gaasd_p1_components_src`

总体结论：三套代码目录结构完整，且可以通过 Clang 20 编译；但按当前代码规范做严格校验时，不能视为严格符合规范。主要问题集中在外部适配调用、C 指针式数据流、本地容器缓冲和复合条件表达式没有结构化建模。

## 1. run() 内直接调用外部 CARLA adapter

涉及规则：

- `RUN-001`：`run()` 应表达组件内部可建模逻辑。
- `RUN-004`：`run()` 内不应混入不可结构化建模的外部依赖调用。
- `DEP-006`：组件依赖应显式建模，不应通过外部 C ABI 直接穿透。

问题说明：

当前改写代码在组件头文件中声明 `extern "C"` 适配函数，并在 `run()` 内直接调用 `carla_adapter_*`。这会导致扫描器看到的是外部函数调用和指针参数，而不是组件自身的输入、输出、参数、状态和子模块数据流。

源码实例：

```cpp
// source/generated/gaasd_lks_components_src/include/CARLALKSLateralOffset.hpp
extern "C" int carla_adapter_lks_lateral_offset(double ego_y,
                                                double lane_center_y,
                                                double* lateral_offset,
                                                int* valid);
```

```cpp
// source/generated/gaasd_lks_components_src/src/CARLALKSLateralOffset.cpp
const int rc0 = carla_adapter_lks_lateral_offset(input.egoY,
                                                 input.laneCenterY,
                                                 &lateralOffset0,
                                                 &valid0);
```

```cpp
// source/generated/gaasd_p0_acc_min_components_src/src/CARLAACCEgoSpeed.cpp
const int rc0 = carla_adapter_acc_ego_speed(input.egoVelocity,
                                            &egoSpeed0,
                                            &valid0);
```

```cpp
// source/generated/gaasd_p1_components_src/src/CARLAObjectList.cpp
const int rc0 = carla_adapter_object_list(input.objectCount,
                                          objectIds0.data(),
                                          objectDistances0.data(),
                                          objectSpeeds0.data(),
                                          objectLanes0.data(),
                                          &objectCount0,
                                          &valid0);
```

影响：

- 组件 JSON 中难以把 adapter 调用识别为稳定的模块边界。
- 输入输出关系被隐藏在外部函数和指针出参里。
- 扫描结果容易退化为“函数调用节点”，而不是结构化组件模型。

修复方向：

- 如果 adapter 是系统边界，应先在规范或组件模型中明确“边界适配器”建模方式。
- 组件 `run()` 内应尽量只保留可扫描的字段赋值、条件分支、子模块调用和状态更新。
- 外部 CARLA 调用可迁移到组件外层适配层，或改写为显式输入字段，由组件内部处理结构化数据。

## 2. 指针出参和取地址破坏结构化数据流

涉及规则：

- `BODY-006`：组件数据流应使用结构化字段，不应依赖裸指针、取地址或指针式出参表达。
- `RUN-009`：`run()` 的输入输出应通过 `Input` / `Output` 结构体字段表达。

问题说明：

当前代码在头文件中声明 `double*`、`int*` 等 C 风格出参，并在 `run()` 中使用 `&变量` 和 `.data()` 传递地址。扫描器难以判断这些指针写入对应哪个输出字段，也无法稳定还原数据流方向。

源码实例：

```cpp
// source/generated/gaasd_p0_acc_min_components_src/include/CARLAACCEgoSpeed.hpp
extern "C" int carla_adapter_acc_ego_speed(double ego_velocity,
                                           double* ego_speed,
                                           int* valid);
```

```cpp
// source/generated/gaasd_p0_acc_min_components_src/src/CARLAACCEgoSpeed.cpp
double egoSpeed0 = 0.0;
int valid0 = 0;
const int rc0 = carla_adapter_acc_ego_speed(input.egoVelocity,
                                            &egoSpeed0,
                                            &valid0);
```

```cpp
// source/generated/gaasd_p1_components_src/include/CARLAObjectList.hpp
extern "C" int carla_adapter_object_list(int input_count,
                                         int* object_ids,
                                         double* object_distances,
                                         double* object_speeds,
                                         int* object_lanes,
                                         int* output_count,
                                         int* valid);
```

```cpp
// source/generated/gaasd_p1_components_src/src/CARLAObjectList.cpp
objectIds0.data(),
objectDistances0.data(),
objectSpeeds0.data(),
objectLanes0.data(),
&objectCount0,
&valid0
```

影响：

- 指针写入不是显式字段赋值，无法直接映射到组件 JSON 的 output 字段。
- `.data()` 暴露数组底层内存，弱化了 `ObjectArray` 这类结构化类型的建模意义。
- 取地址和指针参数增加别名分析复杂度，超出当前规范化组件代码的预期范围。

修复方向：

- 将外部返回结果先在边界层转换为结构化对象，再传入组件 `Input`。
- 在组件内部使用直接字段赋值，例如 `output.egoSpeed = egoSpeed0;`。
- 如果必须保留 C ABI，应把该部分排除在组件扫描目标之外，或在规范中新增专门的边界适配规则。

## 3. run() 内声明本地 std::array 缓冲

涉及规则：

- `BODY-022`：`run()` 内不应声明非必要的局部容器缓冲。
- `BODY-006`：数组或集合数据流应通过结构化字段表达。

问题说明：

`CARLAObjectList::run()` 中创建多个本地 `std::array`，再通过 `.data()` 传给外部 adapter 填充。这种写法把组件输出拆成了本地临时缓冲、外部函数写入、再拷贝回输出字段三个阶段，结构化模型难以直接还原。

源码实例：

```cpp
// source/generated/gaasd_p1_components_src/src/CARLAObjectList.cpp
std::array<int, kMaxObjects> objectIds0{};
std::array<double, kMaxObjects> objectDistances0{};
std::array<double, kMaxObjects> objectSpeeds0{};
std::array<int, kMaxObjects> objectLanes0{};
int objectCount0 = 0;
int valid0 = 0;
```

后续又通过指针写入：

```cpp
// source/generated/gaasd_p1_components_src/src/CARLAObjectList.cpp
const int rc0 = carla_adapter_object_list(input.objectCount,
                                          objectIds0.data(),
                                          objectDistances0.data(),
                                          objectSpeeds0.data(),
                                          objectLanes0.data(),
                                          &objectCount0,
                                          &valid0);
```

影响：

- 本地数组缓冲不是组件 `Input` / `Output` / `State` / `Param` 的稳定字段。
- 数据从 adapter 到 output 的关系需要依赖过程语义推断，不能直接由结构体字段建模。
- 一旦数组复制或循环填充逻辑变化，JSON 建模结果容易不稳定。

修复方向：

- 使用 `Output` 中的结构化数组字段作为唯一输出载体。
- 如果需要对象列表建模，应在类型层定义对象、对象数组和有效数量字段的固定映射。
- 避免在 `run()` 内使用 `.data()` 作为数据流入口。

## 4. 复合条件没有拆成命名布尔值

涉及规则：

- `BODY-041`：复合条件应拆分为可命名、可扫描的中间条件。
- `BODY-019`：条件逻辑应保持结构化，避免把多个判断直接揉在一个表达式中。

问题说明：

当前部分代码直接在 `if` 中写 `rc == 0 && valid != 0`。虽然语义简单，但扫描器只能看到一个复合表达式，难以把每个条件分别建模为状态、有效性或执行门控。

源码实例：

```cpp
// source/generated/gaasd_lks_components_src/src/CARLALKSLateralOffset.cpp
if ((rc0 == 0) && (valid0 != 0)) {
    output.lateralOffset = lateralOffset0;
    output.valid = true;
} else {
    output.lateralOffset = 0.0;
    output.valid = false;
}
```

```cpp
// source/generated/gaasd_p0_acc_min_components_src/src/CARLAACCLeadDistance.cpp
if ((rc0 == 0) && (valid0 != 0)) {
    output.leadDistance = leadDistance0;
    output.valid = true;
} else {
    output.leadDistance = 0.0;
    output.valid = false;
}
```

影响：

- `rc0 == 0` 和 `valid0 != 0` 无法分别成为可命名的数据流节点。
- JSON 中只能得到粗粒度条件表达式，缺少“调用成功”和“数据有效”两个独立语义。
- 后续图形化组件展示时，条件来源不够清晰。

修复方向：

```cpp
const bool adapterCallSucceeded0 = (rc0 == 0);
const bool adapterDataValid0 = (valid0 != 0);
const bool lateralOffsetUsable0 = adapterCallSucceeded0 && adapterDataValid0;

if (lateralOffsetUsable0) {
    output.lateralOffset = lateralOffset0;
    output.valid = true;
} else {
    output.lateralOffset = 0.0;
    output.valid = false;
}
```

注意：上面的例子只说明复合条件的结构化方式，不代表当前规范允许继续保留 adapter 调用和指针出参。

## 5. 对象数组类型映射需要确认

涉及规则：

- `TYPE-015`：进入组件接口的数据结构应有明确、稳定的类型映射。

问题说明：

`CARLAObjectListTypes.hpp` 中使用 `std::array<ObjectInfo, kMaxObjects>` 表示对象列表。这种写法本身可以是结构化类型，但当前规范需要确认是否已经明确覆盖“固定长度对象数组 + 有效数量字段”的建模规则。

源码实例：

```cpp
// source/generated/gaasd_p1_components_src/include/CARLAObjectListTypes.hpp
using ObjectArray = std::array<ObjectInfo, kMaxObjects>;
```

影响：

- 如果规范没有明确对象数组映射，扫描器可能无法稳定判断 `ObjectArray` 是普通容器、总线、列表还是固定槽位数组。
- `objectCount` 与 `objects` 的关系需要被显式表达，否则 JSON 中数组有效范围不清晰。

修复方向：

- 在规范中补充固定长度数组、有效数量字段和元素结构的映射约束。
- 或把对象列表拆成更明确的结构字段，例如 `objects`、`objectCount`、`valid`，并要求扫描器按该模式建模。

## 6. 已通过项

以下检查项没有发现明确问题：

- 三套目录均包含 `include/`、`src/`、`CMakeLists.txt`。
- `.cpp` 文件主要只包含同名 `.hpp`。
- 顶层 CMake 未发现第三方 include 注入。
- 模块类使用 `FuncModule<Traits>` 模板形态。
- `run()` 签名为 `void Class::run(const Input& input, Output& output)`。
- 三套工程均可使用 Clang 20 编译通过。

## 7. 严格校验结论

当前 `source/generated/` 下改写代码属于“可编译但未严格规范化”的状态。

若按现有规范执行，以下问题必须修复后才能判定通过：

- 移除或隔离 `run()` 内的外部 `carla_adapter_*` 调用。
- 移除组件头文件中的 `extern "C"` adapter 声明。
- 消除组件扫描范围内的裸指针出参、取地址参数和 `.data()` 数据传递。
- 将复合条件拆成命名布尔值。
- 明确对象数组和有效数量字段的建模规则，或按已有规范改写为更直接的结构化字段。

如果业务上必须保留 CARLA adapter，则应先修改 `docs/组件改写执行提示词.md` 和相关规范，明确“边界适配器例外”的扫描规则；否则当前代码不能按 v1.0.31 规范判定为严格通过。
