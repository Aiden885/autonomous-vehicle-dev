# APA 测试交接说明

本说明提供接收、构建、测试和问题反馈入口，不将编译结果或历史场景结果视为当前版本验收。接口判定见 [接口契约](INTERFACE.md)，实现职责见 [README](../README.md)。

## 接收版本

源码包须包含 `modules/apa/apa/` 的全部源码、`tests/`、车型生成器及配置，包括新增文件；不能仅导出 Git 已跟踪文件。二进制包至少配套提供：

| 内容 | 安装位置 |
| --- | --- |
| APA 核心库和节点 | `lib/libapa_core.so`、`lib/libapa.so` |
| 独立规划 worker | `bin/apa_planner_worker` |
| JSON 输入输出模块 | `lib/libApaZmqInputModule.so`、`lib/libApaZmqOutputModule.so` |
| APA Protobuf 及依赖 | `lib/libapa_conf_pb.so` 和发布流程收集的 Pangu/Protobuf/ZMQ 依赖 |
| 运行配置和车型 | `conf/node_module/apa/` |
| 三节点拓扑 | `conf/app_module/apa.pt` |
| 本说明及接口文档 | `share/apa/` |

接收记录应包含源码提交及未提交差异摘要、构建工具链、上述产物 SHA-256/Build ID、实际配置、车型指纹，以及配套 CARLA 工程版本和地图摘要。模型/交付副本不是运行模块的替代品。

发布物必须成套安装并重启相关进程后才生效。已加载库不能只用磁盘文件摘要代表；先核对加载路径和运行日志。回退也使用同一完整发布包，不单独覆盖核心库。

## 构建检查

以下生产示例面向已提供工程依赖的 Linux X86/ICVOS 环境，需要 CMake 3.16.1+、Python 3 和支持 C++20 的编译器。Pangu SDK、Protobuf 和 ZMQ 使用工程配套版本；不要混用其他构建的生成头文件或系统 Protobuf 库。

在 Pangu 仓库根目录执行；已有构建目录沿用原平台和依赖参数。

```bash
APA_BUILD=build_apa
cmake -S . -B "$APA_BUILD" -DBUILD_TARGET=X86 -DRUNTIME_OS=ICVOS \
  -DOPEN_WHITE_LIST=ON '-DWHITE_LIST=;apa:' -DUTEST=OFF -DCOMMON_MESSAGE=all
cmake --build "$APA_BUILD" --target apa_core apa apa_planner_worker \
  apa_conf_pb ApaZmqInputModule ApaZmqOutputModule -j4
```

构建输出位于仓库 `build_out/lib/`、`build_out/bin/`。按工程发布流程安装，不以更新构建目录代替部署。本模块安装规则同时提供配置与 `share/apa` 文档；应用拓扑和依赖由 Pangu 发布流程提供。

独立检查入口默认不需要 Pangu 服务或 CARLA，构建 C 算法、实际结构化搜索和 C++ FuncModule 组合层，并编译四项回归：

```bash
APA_TEST_BUILD=/tmp/apa-tests
cmake -S modules/apa/apa/tests -B "$APA_TEST_BUILD" -DCMAKE_BUILD_TYPE=Release
cmake --build "$APA_TEST_BUILD" -j4
```

接口及在线安全用例还需要先生成 `apa_conf_pb`，然后配置：

```bash
cmake -S modules/apa/apa/tests -B "$APA_TEST_BUILD" \
  -DAPA_BUILD_INPUT_CONTRACT_TESTS=ON -DAPA_PROTO_BUILD_DIR="$PWD/build_out"
cmake --build "$APA_TEST_BUILD" -j4
```

可选接口目标当前按工程 X86 SDK/依赖目录装配。独立构建保留断言并启用严格编译检查；生产工程的告警设置不代替这项检查。仅执行上述命令不会运行回归用例。

## 离线回归

测试执行阶段，在独立构建完成后运行：

```bash
(cd "$APA_TEST_BUILD" && ctest --output-on-failure)
```

| 用例或目标 | 覆盖内容 | 证据边界 |
| --- | --- | --- |
| `tracking_regression` | 跟踪状态、换向、段末、偏差和停车约束 | C 控制入口，不代表 CARLA 物理响应 |
| `actuator_regression` | D/R 踏板、起步保持、坡度及停车约束 | 名义执行器模型 |
| `slot_guidance_regression` | 车体中心、退化线段、有限值、库口距离和旋转/入口反向 | 几何边界检查 |
| `parking_search_direction_regression` | 四类车位的泊入/泊出方向、固定实际起点、目标锚点和元数据 | 使用确定性搜索替身检查真实适配及安全门，不覆盖真实图搜索可解率 |
| `input_contract_regression`（可选） | 身份/时钟/别名/容量/车型、非法配置、在线安全场景 | JSON/Protobuf 与安全入口，不启动 APA 服务 |
| `apa_cpp_funcmodule` | 当前 C 接口与 C++20 Traits/组合层编译 | 编译目标，无独立行为验收结论 |

真实搜索和 worker 回放须使用带版本的冻结请求、同帧完整场景及生产控制配置，保留原始输入、完整退出状态和日志。记录规划耗时、展开/标签数、候选拒绝来源、路径长度、换向数、短段和执行预演结果。不得通过删硬障碍、调小车身、放宽验收或改执行响应参数使回归通过。

## CARLA 场景验收

使用配套 CARLA Bridge 与场景面板，先核对协议、车型、地图及目标目录，再从驻车状态启动任务。车辆尺寸、转向模型和地图高度覆盖须匹配当前配置；头入目标与实体车位的别名关系应保留。

| 场景组 | 最低覆盖 | 通过判据 |
| --- | --- | --- |
| 泊入 | VT/HT/DTH/VH；Town15 的 8、15 及头入 108、115 | 到位、持续停稳；无实体接触、协议拒绝或非预期故障 |
| 泊出 | 头入后倒车出库、尾入后前进出库、平行位；使用实际库内起点 | 完整路径通过后才起步，驶至指定库外目标并停稳 |
| 连续任务 | 泊入完成后开始泊出；库内偏移/偏航后重新规划 | 新任务使用当前位姿和占用帧，不复用旧起点或邻位空闲名单 |
| 邻位和障碍 | 空闲、占用、未知邻位；新增静态/动态障碍、库口及轮挡 | 借位来源可追溯；实体、未知及目标三边约束保持有效 |
| 仅规划 | `planning_only=true`，含规划成功和失败 | 路径可查看，车辆保持驻车，无执行起步 |
| 时序和故障 | 同帧重传、旧帧、任务切换、时钟跳变、worker 退出/超时 | 正确复用或拒绝；制动保持，无旧任务控制生效 |
| 跟踪和恢复 | D/R 起步、换挡、短段、坡度、净距不足和恢复无进展 | 阶段与诊断一致；越界/接触被阻止，故障不产生继续驱动 |

每项同时检查 `control_status`、实际位姿、速度、持续停稳、碰撞传感器、恢复次数和最终踏板；`success=true` 只表示存在发布路径。到位容差取请求与实际配置的较严值，不能从旧文档抄录常数。

## 已知限制及结果记录

- 搜索有有限展开、标签和执行候选预算；error 36 应记录为预算内未找到路径，不能直接判为几何无解。
- `attempt_timeout_sec=0` 时不启用 worker 墙钟看门狗，恢复等待时间不包含求解/候选验收；测试总响应时限须单独记录。
- 名义执行预演未覆盖全部 CARLA 轮胎、地面和侧滑响应；通过预演仍需要物理闭环验证。
- Town15/108 库内泊出方向、直出种子、邻位一致性及泊出元数据已作源码修正，需在本次完整发布物上回放原失败输入并做新闭环，不能沿用旧版本结果。

单个结果至少保存：发布包标识、地图/目标及实体车位 ID、动作、起终位姿、配置与车型摘要、会话/任务/帧、场景版本、命令、worker/节点/Bridge 日志、原始请求与故障反馈、路径质量和实际完成指标。区分构建检查、离线算法、名义执行预演和 CARLA 闭环；未执行项明确标记“未执行”。
