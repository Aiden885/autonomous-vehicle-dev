# Pangu GaaSD 节点模块集成开发指南

本文基于 `modules/examples/DemoModule` 示例，说明如何新增一个可被 ICVOS 框架加载的节点模块：工程结构、编译集成、代码约定与运行时配置。

---

## 1. 示例模块概览（DemoModule）

`DemoModule` 展示了一条完整链路：

- **动态库入口**：通过 `REGISTER_NODE_DEFINE` 导出 `init` / `unload` / `loadconfig` 等符号，供框架按 `basic.path` 加载 `.so`。
- **生命周期**：继承 `icvos::adapter::IcvosBaseModule`，实现 `Init`、`InitIO`、`Proc`、`LoadConfig`、`SetVersion`。
- **通信**：`DECLARE_PUBLISHER` / `CREATE_OUTPUT`、`DECLARE_SUBSCRIBER` + `DECLARE_CALLBACK` / `CREATE_INPUT` / `DEFINE_CALLBACK_HEAD`，以及定时任务 `CREATE_TIMER_TASK`。
- **配置**：模块私有参数用本模块 `proto` 生成 `*Params`，整体配置消息内嵌 `icvos.common.NodeModuleConfig basic`；运行时由 `conf/node_module/<模块名>/*.pt` 的文本 protobuf 提供。
- **可选算法库**：`module_lib/` 编译为 `DemoModule_core`，主模块链接该库，用于把算法与适配层分离。

关键源码位置：

| 内容 | 路径 |
|------|------|
| 模块类 | `DemoModule/src/DemoModule.h`、`DemoModule.cc` |
| 配置 proto | `DemoModule/proto/DemoModule_config.proto` |
| 节点级 .pt | `DemoModule/conf/DemoModule.pt`、`default.pt` 等 |
| 构建 | `DemoModule/CMakeLists.txt` |
| 可选核心库 | `DemoModule/module_lib/demo_app.*` |

---

## 2. 目录与命名约定

建议新模块目录名为 **与 `project()` 及类名一致**（如 `MyPlanner`），典型结构：

```text
modules/<分类>/MyPlanner/
├── CMakeLists.txt          # 可从 DemoModule 复制后改 project 名与依赖
├── proto/
│   └── MyPlanner_config.proto
├── src/
│   ├── MyPlanner.h
│   └── MyPlanner.cc
├── module_lib/             # 可选；存在则生成 MyPlanner_core
│   └── ...
└── conf/
    ├── MyPlanner.pt        # 常用主配置
    └── default.pt          # 或其它场景配置
```

**命名约定（与 Demo 一致）：**

- 配置 proto 中参数 message 建议为 `MyPlannerParams`，成员变量命名为 `MyPlanner_params_`。
- 外层配置 message 建议为 `MyPlannerConfig`，包含 `icvos.common.NodeModuleConfig basic = 1` 与 `MyPlannerParams params = 2`。
- `global_module.pt` 里的 `config_type` 使用 **protobuf 全名**，例如 `pangu.modules.MyPlannerConfig`（与 `package` + message 名一致）。

---

## 3. 编译系统集成

### 3.1 自动发现子工程

根目录 `modules/CMakeLists.txt` 会递归查找各子目录下的 `CMakeLists.txt`，并对每个目录调用 `add_subdirectory`（通过宏 `add_module`），**无需**在根 `CMakeLists.txt` 里手工 `add_subdirectory` 新模块路径；只要新模块落在 `modules/` 下且带有自己的 `CMakeLists.txt` 即可参与构建。

注意：路径名匹配 `test` 的目录会被跳过；部分平台会通过 `ENABLE_MDC_610`、`ENABLE_J6`、`IS_X86` 等条件排除特定模块，新增模块若需平台裁剪，可在此文件中按同样模式扩展条件。

### 3.2 只编译固定几个模块（白名单）

顶层 `CMakeLists.txt` 会在包含 `common_pangu.cmake` 之前设置：

- 未开启白名单：`ALL_MODULES` 为 **on**，凡被 `modules/CMakeLists.txt` 扫到的模块都会 `add_subdirectory`。
- 开启白名单：`-DOPEN_WHITE_LIST=ON`（或脚本里的 `on`）时，`ALL_MODULES` 为 **off**，只有出现在 `WHITE_LIST` 里的模块名才会参与编译。

`add_module` 判定式（`dependencies/common_pangu.cmake`）为：

```cmake
if((ALL_MODULES OR (WHITE_LIST MATCHES "(.*);${module_name}:(.*)"))
    AND NOT (BLACK_LIST MATCHES "(.*);${module_name}:(.*)"))
```

其中 **`module_name` 是模块目录的最后一级名字**（例如 `modules/examples/DemoModule` → `DemoModule`），与 `project(DemoModule)` 一致。

**`WHITE_LIST` 的字符串格式必须带分号与尾部分号**，以便与上面的正则匹配：每个模块一项，形如 `;模块A:;模块B:`。

**直接用 CMake 配置示例**（只编 `DemoModule` 与 `MpdmPlanner`）：

```bash
cmake -S /path/to/pangu_gaasd -B build \
  -DOPEN_WHITE_LIST=ON \
  -DWHITE_LIST=";DemoModule:;MpdmPlanner:"
```

多个模块继续在引号内追加 `;名字:` 即可。

**使用仓库脚本** `script/build_local.sh` 时，可用 `-pkg`（内部会设置 `OPEN_WHITE_LIST=on` 并拼接 `WHITE_LIST`）：

```bash
./script/build_local.sh -pkg DemoModule MpdmPlanner
```

脚本会检查 `modules` 下是否存在对应目录名；其余 CMake 参数仍按该脚本惯例追加。

**注意：**

- 白名单只作用于 **`modules/` 树内**通过 `add_module` 加入的子工程；`dependencies/user_message`、`modules/algo/proto`（若存在）等仍按顶层 `CMakeLists.txt` 单独 `add_subdirectory`，不受此白名单控制。
- 若模块 A 链接了模块 B 生成的库，而 B 不在白名单里，可能导致 **链接失败**，需要把依赖链上的模块一并列入白名单，或调整 `target_link_libraries` 改为链接已安装的第三方/预编译库。

### 3.3 屏蔽某个模块（黑名单）

在 **`ALL_MODULES` 为 on**（默认，即未开白名单或 `OPEN_WHITE_LIST` 为 off）时，仍可通过 **`BLACK_LIST`** 排除指定目录名，格式同样为 `;模块名:`：

```bash
cmake -S /path/to/pangu_gaasd -B build \
  -DBLACK_LIST=";HdMapService:;HmiRviz:"
```

**使用 `build_local.sh`：**

```bash
./script/build_local.sh -no_pkg HdMapService HmiRviz
```

逻辑关系：**先**看是否允许加入（`ALL_MODULES` 或白名单命中），**再**看是否被黑名单排除。因此黑名单也可与白名单叠用（仅白名单中的模块里再排除若干项），例如：

```bash
cmake ... -DOPEN_WHITE_LIST=ON -DWHITE_LIST=";A:;B:;C:" -DBLACK_LIST=";B:"
```

只编 `A` 与 `C`。

### 3.4 平台级「硬屏蔽」

部分模块在 `modules/CMakeLists.txt` 里按平台写死 `continue()`（如 MDC610 下跳过 `SimulationAdapter`）。若某个模块在特定芯片/架构上**永远不应参与构建**，除黑名单外也可在该文件中增加路径匹配规则，与现有写法保持一致。

### 3.5 单模块 CMakeLists 模板要点（对照 Demo）

以 `DemoModule/CMakeLists.txt` 为模板时，通常只需改 **顶部**：

- `project(模块目录名)`：与目录名、`lib/lib<名>.so` 一致。
- `set(CORE_LIBS ...)`：填写 `module_lib` 或第三方库依赖。
- `target_link_libraries(${PROJECT_NAME} ...)` 中的额外库（Demo 中链接了 `user_message_msg_pb` 等）。

模板下半段约定：

- **`proto/`**：`generate_proto` 生成 `*_conf_pb` 共享库；若不需要模块私有 proto，可通过工程里 `COMMON_MESSAGE=no` 一类开关跳过（与现有工程选项一致，以实际 `CMake` 变量为准）。
- **`module_lib/`**：存在 `.cc/.c/.cpp` 时生成 `${PROJECT_NAME}_core`；也可仅放置预编译 `module_lib/lib/${TARGET_PLATFORM}/lib${PROJECT_NAME}_core.so`。
- **`src/`**：编译主动态库 `${PROJECT_NAME}.so`，安装到 `lib/`。
- **`conf/`**：`install(DIRECTORY conf/ DESTINATION conf/node_module/${PROJECT_NAME})`，即安装后路径为 `install/conf/node_module/<模块名>/`。

版本号在模板中有 `CMakeLists Version is 1.5.2` 日志，便于对照升级。

---

## 4. 代码开发要点

### 4.1 基类与模块名

类应继承 `icvos::adapter::IcvosBaseModule`，构造函数传入的字符串为 **模块逻辑名**（与配置、目录一致），例如 Demo 中 `DemoModule() : IcvosBaseModule("DemoModule") {}`。

框架在 `SetApi` 后会设置：

- `module_conf_folder_` → `<install>/conf/node_module/<module_name>/`
- `basic_` → 从框架侧注入的 `NodeModuleConfig` 文本解析结果

### 4.2 节点注册与导出符号

- 头文件末尾：`REGISTER_NODE_DECLARE(MyPlanner);`
- 源文件顶部：`REGISTER_NODE_DEFINE(MyPlanner);`

这样生成的 `.so` 才具备框架所需的 `init` 等导出函数。

### 4.3 配置加载

`LoadConfig` 中一般使用 `google::protobuf::TextFormat::ParseFromString(ptstr, &<Module>Config或Params>)` 解析传入字符串。Demo 直接解析 `DemoModule_params_`；完整做法通常是解析带 `basic` + `params` 的外层 `MyPlannerConfig`，再分别使用两部分。

### 4.4 IO 与回调

**头文件：**

- `DECLARE_PUBLISHER(别名, Protobuf类型);`
- `DECLARE_SUBSCRIBER(别名);`（由 `os_api.h` 等提供空宏占位时可只作声明配合）
- `DECLARE_CALLBACK(回调名, Protobuf类型);`

**InitIO：**

- `CREATE_OUTPUT_RETURN_IF_ERROR(别名, 类型);`
- `CREATE_INPUT_RETURN_IF_ERROR(模块类名, 订阅别名, 类型);`

**源文件：**

- 对每个 callback：`REGISTER_CALLBACK(模块类名, 回调名, 类型);`
- 实现：`DEFINE_CALLBACK_HEAD(模块类名, 回调名, 类型) { ... GET_TRIGGER_MSG_DATA_PROTO(...) ... }`

**发布消息：** `PUB_MSG(别名, msg_ptr);`

**定时周期处理：** `CREATE_TIMER_TASK(模块类名, 周期秒(double), Proc);`（Demo 中周期来自配置 `timer_duration`）

回调中的别名、`conf` 里 `tasks` / `subscribers` 的 `name`、`input_trigger` 等需 **一致**，否则框架无法把通道与任务绑定。

### 4.5 与 module_lib 分工

将耗时算法、缓冲、状态机放在 `module_lib`，模块类只做 IO、配置与调度，便于单测与复用（Demo 中 `DemoApp` 缓存轨迹并在 `Proc` 中输出）。

---

## 5. Protobuf 配置

### 5.1 模块配置 proto（示例）

`DemoModule_config.proto` 模式：

- `package pangu.modules;`（或与全局注册一致的其他 package）
- `import "node_module_config.proto";`
- `message DemoModuleParams { ... }`
- `message DemoModuleConfig { icvos.common.NodeModuleConfig basic = 1; DemoModuleParams params = 2; }`

其中 `basic` 与节点 `.pt` 里的 `basic { ... }` 字段对应；`params` 与 `params { ... }` 对应。

### 5.2 与全局注册表关联

在 `configs/global_conf/global_module.pt` 中增加一项：

```text
modules {
  name: "MyPlanner"
  config_type: "pangu.modules.MyPlannerConfig"
}
```

`name` 与节点模块名一致；`config_type` 为 proto 中 **外层 Config message** 的全限定名。框架据此做类型校验与反序列化。

---

## 6. 运行时配置（.pt 文件）

### 6.1 节点级配置：`conf/node_module/<Module>/<场景>.pt`

Demo 中 `DemoModule.pt` 结构分为两段：

1. **`basic`**（类型 `icvos.common.NodeModuleConfig`）
   - `path`: 相对安装前缀的 `.so` 路径，如 `lib/libDemoModule.so`。
   - `module_type`: 如 `APP`、`SENSOR`（见 `node_module_config.proto` 中 `ModuleType`）。
   - `ioconfig`:
     - `subscribers`：`name`（代码里别名）、`channel`（总线通道名）、`bindcode`。
     - `publishers`：同上；同一 **物理 channel** 可有多个 **别名**（如 Demo 里 `local_trajectory` 与 `hmi` 同 channel）。
     - `tasks`：`task_name` 通常与某 subscriber 名一致；`task_option` 内 `task_type`（如 `POLICY_TYPE_MSG_NEWEST`）、`input_trigger`、`timer_trigger_interval`（微秒）等决定触发策略；`input` 列表指明该任务消费哪些输入。

2. **`params`**（模块自定义，与 `*Params` message 一致）

不同场景可复制多份 `.pt`（如 `default.pt`、`DemoModule_12m.pt`），仅改 `params` 或 `ioconfig` 差异部分。

### 6.2 应用/进程编排：`configs/app_module/*.pt`

例如 `configs/app_module/demo_test.pt` 定义 SOC 上启动哪个进程、加载哪个节点配置：

- `node.module_name` / `node_name`
- `path`: 指向 `conf/node_module/DemoModule/DemoModule.pt`（安装后的相对路径）
- `process_name`、`log` 等

新模块接入某条产品线时，在对应 `app_module` 的 `.pt` 里增加 `node` 块，或新建一份 app 配置并在启动脚本/启动参数中选用。

### 6.3 安装布局

构建安装后：

- 库：`install/lib/lib<模块>.so`
- 节点配置：`install/conf/node_module/<模块>/`
- 全局与应用配置：由顶层 `CMakeLists.txt` `install(DIRECTORY configs/ ...)` 统一安装到 `install/conf/`。

运行时应保证工作目录或环境变量指向的 **install 根路径** 与配置里 `path`、`conf/...` 相对关系一致（与现有 `launch.sh` / `run.sh` 用法保持一致）。

---

## 7. 检查清单（新模块上线前）

1. **目录与 `project()`、`IcvosBaseModule("Name")`、`lib/libName.so` 一致。**
2. **`REGISTER_NODE_DEFINE` / `REGISTER_CALLBACK` 与 `DECLARE_*` 成对且名称与 `.pt` 中 task/subscriber 一致。**
3. **`proto` 中 `*Config` 含 `NodeModuleConfig basic`，并在 `global_module.pt` 注册 `config_type`。**
4. **`conf/node_module/<Name>/` 已安装，`basic.path` 指向正确 `.so`。**
5. **业务 `app_module` 的 `.pt` 已加入节点并指向上述节点 `.pt`。**
6. **`CMakeLists.txt` 中 `CORE_LIBS` / `TARGET_LIBS` 已声明全部链接依赖。**
7. **若仅某平台可用**：在 `modules/CMakeLists.txt` 中增加排除逻辑，或通过白名单控制构建。

---

## 8. 参考文件索引

| 主题 | 参考路径 |
|------|-----------|
| 模块 CMake 模板 | `modules/examples/DemoModule/CMakeLists.txt` |
| 模块扫描与 `add_module` | `modules/CMakeLists.txt`、`dependencies/common_pangu.cmake` |
| 基类与注册宏 | `dependencies/os_adapter/icvos/*/sdk/include/icvos_base_module.hpp` |
| `NodeModuleConfig` / IO proto | `dependencies/os_adapter/icvos/*/sdk/proto/os_proto/node_module_config.proto` |
| Demo 全局类型注册 | `configs/global_conf/global_module.pt` |
| Demo 进程编排 | `configs/app_module/demo_test.pt` |

按上述步骤复制 Demo、改名、改 proto 与 `.pt`，即可在不大改框架的前提下完成新模块集成。
