# 工作日志 WORKLOG

> **用途**：记录每次工作的修改内容、遗留问题和下一步任务。
> 新开会话或交给 Codex 前，先读这个文件。
> 每次工作结束后，更新"当前状态"和"下一步任务"两节。

---

## 当前状态（2026-06-15 更新）

**newaccpro2 规范组件源码对比**：

- 已对比团队提供的
  `project/accpro2/newaccpro2_component_sources.zip` 与本地
  `deliverables/newaccpro2_components_src_v2.zip`。
- 两套代码均包含相同的六个 `FuncModule<Traits>` 组件，且
  `FuncModule.hpp` 完全一致；主要差异不在框架，而在运行时边界端口契约。
- 团队版读取组件输入为“业务值 + `adapterRc` + `valid`”，输出仅保留业务值；
  本地 v2 输入为“业务值 + `valid`”，并继续输出业务值与 `valid`。
- 团队版纵向命令输出为 `targetSpeed/enable`；本地 v2 输出为
  `speed/enable/valid`。两者均已将实际 ZMQ 发布移出组件 `run()`。
- 团队版保留可配置的 `defaultDistance`，并沿用 `desiredDist` 参数名；
  本地 v2 将无前车距离固定为 `1000000.0`，参数名为
  `desiredDistance`。两版速度上限数值均为 `5.0 m/s`。
- 团队版压缩包的 CMake 未针对本机 GCC 9.4 添加 `-fconcepts`，直接构建
  失败；显式增加 `-fconcepts` 后六个组件可以构建。本地 v2 已在 CMake
  中处理 GCC 9 兼容并以严格警告构建通过。
- `adapter_cpp` 不属于扫描组件，不应与六个 `FuncModule` 放在同一次扫描中。
  但“不扫描”不代表“不需要集成”：GAASD 运行时/节点框架仍须调用 adapter，
  将读取返回值映射到组件输入，并将控制组件输出交给 adapter 发布。
- 当前 adapter 已提供 ACC 所需的读取和发布 C ABI，核心通信逻辑无需因本次
  组件规范化重写；尚缺的是 GAASD 运行时的数据绑定层及双方最终端口契约确认。

**LKS 决策与完整控制方案设计**：

- 已读取并分析 `docs/LKS算法设计原理0206.docx`，以最终修订 V0.6 为准。
- 已逐层解析原算法 `project/lks/LKS_tcp17.slx` 与
  `project/lks/lks_tcp_s17.py`。原链路以 0.1 s 周期通过 TCP 传输
  186 个 `double`，其中包含 `boolLD[60]`、`cx[60]`、`cy[60]`；
  Simulink 将有效车道点压缩并补零为固定 60 维数组，再按欧氏距离选取
  近、中、远三个预瞄点。
- 原控制模型的预瞄距离为
  `ld = (5 + 0.5 * v) / curvatureFactor`，三点权重为
  `0.2/0.3/0.5`，比例增益为 `0.08`，并使用
  `atan(3 * 2.9 / v^2) / 30 deg` 形式限制归一化转向值。
- 确认正式控制算法为三预瞄点加权的位置式比例控制，不再单独使用航向误差；
  启控时需要捕获并保持当前归一化转向值 `theta0`。
- 确认原文档表 18 中弯道预瞄距离缩放系数为数学分数 `2/3`，不是文本提取
  结果中的 `23`。
- 决策层简化为布尔逻辑：低速、制动、主动换道任一成立即进入待命；车道线
  有效状态在无感知测试阶段固定为 `1`，无需使用真值表。
- 已确认新版 GAASD 基础组件库提供“数组取值”和“数组赋值”组件，数组索引
  可作为动态输入端口，且两类组件已纳入代码生成流程。此前“GAASD 画布不能
  处理数组”的假设不成立。
- 已解包并核查当前 `codescan` 与 `gaas_codegen`：扫描器固定使用 C11，
  文件白名单只有 `.c/.h`，因此不支持扫描 `.cpp` 或
  `std::array<double, 60>`。
- 最小扫描验证确认 `double input[60]` / `double output[60]` 会在组件
  JSON 中保留为 `const double[60]` / `double[60]`，属于当前正式支持的
  数组端口表示。
- `typedef double LaneArray60[60]` 虽然函数本身能被扫描，但数组别名不会
  写入 `typedef.json`；扫描器的 typedef 导出仅保留基础类型别名，因此不应
  使用“整个数组的 typedef 别名”作为组件边界类型。
- `typedef double Real; Real values[60]` 可以使用：`Real -> double` 会写入
  `typedef.json`，数组维度仍由端口的 `[60]` 表达。
- 当前 `gaas_codegen` 的函数调用生成逻辑对 `out/inout` 参数统一添加 `&`，
  没有排除数组类型。对于 `void f(double output[60])`，可能生成
  `f(&output)` 而不是 `f(output)`，因此“数组输出端口”仍有代码生成缺陷；
  在修复前不能直接作为 LKS Bridge 数组输出边界。
- 现有方案暂采用 Bridge 输出三次车道多项式和道路曲率，GAASD 画布使用基础
  运算模块计算三个预瞄点误差、动态预瞄距离、加权比例控制和横向加速度动态
  限幅；若固定长度数组最小验证通过，应重新评估是否改为与 Simulink 更一致的
  60 点数组方案。
- 根据原 Simulink 数组实现，若要求严格复现原算法，边界方案应优先调整为
  Bridge 直接输出三个预瞄横向误差标量、道路曲率和有效点计数；多项式方案
  可作为近似实现，但其按纵向位置求值与原模型按欧氏距离选点并不完全等价。
- 当前基础组件库未发现反正切模块。完整实现需要标准 `atan` 数学组件，或新增
  只封装 `std::atan` 的原子数学组件。
- 详细架构、参数、模块实例和连线关系已写入
  `docs/GAASD_LKS_完整控制画布方案.md`。

**待确认**：

- CARLA 首轮测试最低适控速度和固定测试速度。
- LKS 使用键盘激活/取消还是先以常量自动激活。
- 请 GAASD 团队修复数组 OUT/INOUT 参数调用时错误添加 `&` 的生成逻辑，并
  提供固定长度数组跨组件编译用例。
- 固定长度 C 数组完成跨组件连线、动态取值和数组赋值后的生成、编译、运行
  验证。
- 根据数组最小验证结果，在 60 点数组方案与车道多项式方案之间最终选型。

---

## 当前状态（2026-06-12 更新）

**CARLA ACC 最小组件 C++ v1.0.33 规范化**：

- 已依据 `docs/C++代码规范.docx v1.0.33` 重写
  `generated/gaasd_p0_acc_min_components_src/`。
- 六个组件统一采用 `FuncModule<Traits>` 与结构化
  `Input/Output/Param/State/Sub`：
  `CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、
  `CARLAACCLeadDistance`、`CARLAACCDriverCommand`、
  `CARLAACCLongitudinalCmd`、`CARLAACCComputeTargetSpeed`。
- 组件 `run()` 已移除 `carla_adapter_*`、C ABI 指针出参、ZMQ/JSON
  依赖和外部副作用；条件使用命名布尔量，语义块使用 Doxygen
  `@brief` 注释，结果显式写入 `Output`。
- `CARLAACCLongitudinalCmd` 现在输出 `speed/enable/valid`，由 GAASD
  运行时或节点框架负责发布；输入组件由运行时向
  `egoV/leadV/distance/commandType/valid` 端口注入数据。
- `tools/carla_bridge/adapter/src/carla_gaasd_adapter.cpp` 保留为独立
  通信基础设施，不属于 GAASD 扫描组件。将其外部调用放回组件
  `run()` 会违反规范中的指针、依赖和函数体白名单。
- GCC 9.4 C++20 自包含构建通过，并启用
  `-Wall -Wextra -Wpedantic -Werror`；校验记录见
  `generated/gaasd_p0_acc_min_components_src/校验报告.md`。
- 已更新可发送源码目录
  `deliverables/newaccpro2_component_sources/`，并生成
  `deliverables/newaccpro2_components_src_v2.zip`。压缩包分别保留当前
  画布 C 源码、规范版 C++ 组件和独立 adapter C++ 源码。

**下一步任务**：

- 使用团队提供的新版 C++ 扫描器仅扫描
  `cpp_funcmodule_components/`，确认六个组件的端口和元数据可正常入库。
- 由 GAASD 团队确认节点/运行时的数据绑定方式，把 Bridge 消息注入组件
  `Input`，并把控制组件 `Output` 发布回 Bridge。

---

## 当前状态（2026-06-11 更新）

**GAASD 2.7.0.5 代码生成模块更新**：

- 已将 `/home/aiden/文档/temp/gaasd_code_tools_20260610.tar` 安装到隔离版新版 GAASD：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/codeTools`。
- 更新前完整 `codeTools` 已备份到：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/rollback/20260611_094210/codeTools`。
- 对比确认该包相对当前 20260606 环境只有 `gaas_codegen` 二进制发生变化；
  `codescan`、`run_simulation.sh` 及其他脚本内容一致。原有 `.env` 和 `dist/logs` 已保留。
- 新 `gaas_codegen` SHA-256：
  `8480b21606eba811ec235484777618c9eb5446d9e3cbb3389e065ed4f31e6e8c`。
- 已使用 `newaccpro2` 当前画布在 `/tmp/gaasd_codegen_20260610_smoke` 完成隔离代码生成，
  生成器可正常启动、处理画布并产出 C++ 工程，未覆盖项目现有生成代码。
- 隔离编译仍失败，已确认至少存在以下生成器问题：
  - `GlobalParams` 同时生成在 `GlobalContext.hpp` 和 `GlobalContextTypes.hpp`，导致结构体重复定义。
  - 主画布仍生成非法变量名 `C++_None`。
  - 复合组件头文件声明 `run()`，调用端和实现端却使用 `composite_block()`，接口不一致。
  - 复合组件仅初始化 `vMin`，`GapStep/MinGap/MaxGap/SpdStep/MinSpd` 等参数未按画布值完整初始化。
- 结论：20260610 代码生成模块已正确替换，但 `newaccpro2` 当前仍不能直接通过生成代码编译运行；
  后续应把上述最小复现结果提交给 GAASD 团队继续修复。

---

## 当前状态（2026-06-09 更新）

**GAASD 2.7.0.5 隔离版更新完成**：

- 使用 `GAASD_SETUP_20260608.tar.gz`、`gaasd_code_tools_20260606.tar` 和 `清华组件包_0608_1.tar` 更新了 `/home/aiden/gaasd_versions/gaasd-2.7.0.5`。
- 系统旧版 `/opt/gaasd` 和 `/usr/bin/gaasd` 未修改；更新前文件和数据库保存在 `.../rollback/20260609_143740/`。
- 基础组件已更新到 `1.0.8`，清华组件库已导入 28 个顶层组件和 1275 条包内组件详情；SQLite 完整性及 `originId` 唯一性检查通过。
- code tools 环境记录已更新为 `20260606`；`gaas_codegen`、`codescan` 和 `run_simulation.sh` 基础检查通过。
- 隔离启动脚本已处理 IDE 的 `ELECTRON_RUN_AS_NODE` 污染，并补建新版要求的 `cacheFile` 目录；GUI 冒烟启动通过。
- 详细记录和回滚说明见 `docs/GAASD_2.7.0.5_20260608更新记录.md`。
- 已知限制：0606 包内 `codescan` 仍是 C 扫描器，尚不能据此确认新版 C++ `FuncModule` 扫描链路。

**`newaccpro2` 最新 ACC 决策控制画布进展**：

- 本次复核以 `project/newaccpro2/data/cbdes.db` 和 `temp.db` 为事实源；两库内容一致且 SQLite 完整性检查通过。数据库最后更新时间为 2026-06-09 14:36，晚于 11:40 生成的 `FuncStep.c`，因此当前生成代码不能代表最新画布。
- 工程数据库现有 87 个组件、241 条连线。其中 ACC 主画布包含 20 个组件、26 条连线，`ACCDecision` 复合组件内部包含 65 个组件、78 条连线。
- 主画布已完成 CARLA 输入、驾驶指令、决策和纵向控制主链路：
  - `CARLAACCEgoSpeed -> ACCDecision.egoV`
  - `CARLAACCDriverCommand.commandType -> ACCDecision.commandType`
  - `ACCDecision.enable -> CARLAACCLongitudinalCmd.enable`
  - `egoV * timeGap -> max(MinDistance, ...) -> desiredDistance`
  - `leadV + Kdist * (distance - desiredDistance) + Kspeed * (leadV - egoV)` 生成原始目标速度
  - 目标速度经过非负限幅和 `ACCDecision.maxSpeed` 上限后连接 `CARLAACCLongitudinalCmd.speed`
- `ACCDecision` 已不是半成品。当前已配置：
  - 输入：`egoV`、`commandType`
  - 输出：`enable`、`timeGap`、`maxSpeed`
  - 局部状态：`controlEnabled=0`、`hasHistory=0`、`timeGap=1.8`、`maxSpeed=20`
  - 局部参数：`vMin=1`、`GapStep=0.2`、`MinGap=1`、`MaxGap=5`、`SpdStep=4`、`MinSpd=0`
- 状态计算、真值表和反馈链已经连接：低速状态判断、S0/S1/S2/S3 编码、`controlEnabled/hasHistory` 读写、`timeGap/maxSpeed` 读写均已进入画布。`enable` 当前由 `S0 = notLowSpeed && controlEnabled` 输出。
- 真值表已配置 `systemState(u), commandType(v) -> y`，包含降速、增速、降距、增距、无继控制启控、继承控制启控、扭矩仲裁和待命分支。后级已对 `y=1..6` 中当前需要的分支进行解码，用于使能、时距和限速更新。
- 时距更新链已完成：`y=3/4` 分别触发减小/增大时距，并通过 `MinGap/MaxGap` 限幅后写回 `state_.timeGap`。
- 限速更新链已完成：`y=1/2` 分别触发降低/提高速度上限，通过 `MinSpd` 限幅后写回 `state_.maxSpeed`。
- `CARLAACCDriverCommand` 已放入主画布并连接 `ACCDecision.commandType`，不再是“后续待放置”的画布组件。

**本次已补齐的 CARLA 联调边界**：

- 已实现 `watch-carla.py -> Bridge -> adapter -> CARLAACCDriverCommand` 驾驶指令链路：
  - E/Q/T/R/C 作为单周期脉冲，adapter 每条只返回一次。
  - W/S 在按下期间作为持续电平发送，释放后恢复 `commandType=0`；S 对应制动并退出 ACC。
  - W/S 以 10 Hz 发送心跳，Bridge 超过 0.5 s 未收到更新会自动释放，避免窗口异常退出后残留指令。
  - 当前只传输 `commandType`。W 的驾驶员油门扭矩仲裁暂不实现；S 不依赖扭矩仲裁，直接由画布退控链处理。
- Bridge 新增 `gaasd.carla.driver_command.v1` 路由，控制输入和驾驶指令共用 `5702` 入口，但按 topic 分流；Bridge 再通过 `5701` 发布驾驶指令供 adapter 订阅。
- adapter 已订阅驾驶指令 topic，并实现脉冲队列和持续指令缓存；Docker `ubuntu:env` 中构建和 mock loop 测试通过。
- 已参考 `/home/aiden/PycharmProjects/CarlaAcc` 增加动态预瞄横向 PID：当前横向偏差权重 0.7、预瞄偏差权重 0.3，预瞄距离按 `8 + 0.3 * speed_mps` 在 5~15 m 内变化。
- 已新增 `scenarios/newaccpro2_keyboard_carla_20260609` 联调场景，启动时自动打开键盘摄像头窗口，并使用 `newaccpro2`、动态前车和预瞄车道保持配置；重点验收 S 制动退出后保持待命。场景已保存当前 `newaccpro2` 画布快照并提供 UI 一键恢复脚本，覆盖恢复前会自动备份现有工程。

**当前画布仍待联调确认的边界**：

- `project/newaccpro2/icvos/src/oscilloscopeFunctions/.../FuncStep.c` 仍是更新前的固定 `desiredDistance=15`、固定 `enable=1` 版本。需要在新版 GAASD 中基于最新画布重新生成，才能检查完整决策代码。
- 11:40 生成的真值表 C 代码仍暴露已知生成器问题：多条动作被无条件顺序赋值，不能据此运行验证真值表逻辑。画布真值表配置本身已完成，问题位于 GAASD 代码生成环节。
- 当前 R7 只在真值表中表达 W 油门对应的“扭矩仲裁决策”。画布没有驾驶员油门扭矩数值输入，也没有 ACC 扭矩与驾驶员扭矩比较、选择和输出链，因此尚未实现 W 的物理控制量级扭矩仲裁。S 制动退出链已经连接，不属于该缺口。
- 当前示波器仅连接 `egoV`、`leadV`、`distance`，第一个输入端口未连接。完整联调前应补充 `targetSpeed`，并根据需要增加 `commandType/y/enable/timeGap/maxSpeed` 的观测。

**`newaccpro2` 下一步**：

- 用最新画布重新生成代码，优先复核 `ACCDecision`、局部状态读写和真值表生成结果。
- 修复或绕过真值表代码生成问题后，使用 `scenarios/newaccpro2_keyboard_carla_20260609/run.sh` 进行键盘指令、ACC 决策和 CARLA 闭环联调。
- 后续若要求真实扭矩仲裁，需要新增驾驶员踏板或扭矩输入以及对应控制输出接口，不能只依赖 `commandType=5`。

---

## 当前状态（2026-06-04 更新）

**ACC 决策层用「组件」(composite-block) 封装 + 官方范式确认**：

- 确认新版「组件」= `composite-block`（带自有 cppClass，可配局部参数/局部状态），是决策子系统的正确容器；composite-function 不带局部状态。官方 PidController demo 即 composite-block + read-local-state/read-local-param 实现。
- 解析 PidController 内部（34 子块）提炼 4 点范式并写入 `docs/GAASD_ACCDecision_复合函数连线清单.md`：
  1. 子系统端口用组件内部的 `input`/`output` 块表示。
  2. 可调增益用局部参数 Param + `read-local-param`（如 kDist/kSpeed/timeGapStep/限幅值），属性面板可调，不必改图重生成。
  3. 结构性常量（判等 0-8）仍用 `constant`。
  4. 关键中间量可用 `variable` 块命名。
- 当时的 newaccpro2 体检结论（已被 2026-06-09 最新画布进展取代）：CARLA I/O + 基础控制律（仍是固定 15m 公式）已搭；状态计算半成品且 isActive/hasHistory 用常量；真值表已放未接（0 连线）；无任何状态记忆组件。
- 已定方向：控制律升级为时距公式（egoV×timeGap）保留在主画布；全部决策状态机封进 `ACCDecision` 组件(composite-block)，输入 egoV/commandType，局部状态 controlEnabled/hasHistory/timeGap/maxSpeed，输出 enable/timeGap/maxSpeed。
- 决策连线清单（约 75-85 块，分段 A-H）见 `docs/GAASD_ACCDecision_复合函数连线清单.md`；要求先搭“最小验证版（到 enable）”验证 composite-block + 状态组件 + 真值表的 codegen 能用，再加时距/限速。
- 指令脉冲源（边沿检测/驾驶指令边界组件）列为后续，见主方案第 11 节，当前主线按“单周期脉冲”约定，画布不搭边沿检测。

**CARLA ACC 边界组件严格扫描版（TM 问题闭环）**：

- 针对 `docs/source_generated代码规范验证问题实例.md` 中 TM 指出的外部 adapter、C 指针出参、`.data()`、复合条件等问题，新增并行严格版目录：`generated/gaasd_p0_acc_min_components_strict_src/`。
- 严格版保留 4 个 ACC 主线边界组件名：`CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、`CARLAACCLeadDistance`、`CARLAACCLongitudinalCmd`，但移除头文件 `extern "C"` 声明和 `run()` 内 `carla_adapter_*` 调用。
- 严格版把 CARLA 数据改为结构化 `Input` 字段输入，把控制命令改为结构化 `Output` 字段输出；因此它用于新版 GAASD 扫描/入库讨论，不能直接替代当前可运行 adapter 版本。
- 已验证：`cmake -S generated/gaasd_p0_acc_min_components_strict_src -B /tmp/gaasd_p0_acc_min_components_strict_build` 和 `cmake --build /tmp/gaasd_p0_acc_min_components_strict_build -j2` 均通过；源码目录扫描未发现 `carla_adapter_`、`extern "C"`、`.data()`、`double*`、`int*`。
- 若采用严格版联调，GAASD 团队需要在运行时/节点层完成 Bridge 数据绑定：将 `egoV/leadV/distance/valid` 注入组件输入，并从 `CARLAACCLongitudinalCmd` 的 `speed/enable/valid` 输出发布到 Bridge。

---

**新版 GAASD 2.7.0.5 旁路安装与组件迁移**：

- 新版以旁路方式安装，与旧版（`/opt/gaasd`、`~/gaasd_server`）完全隔离：
  - 本体 `~/gaasd_versions/gaasd-2.7.0.5/app`，独立 profile `.../home`，启动脚本 `run-gaasd-2.7.0.5.sh`（隔离 HOME/XDG/`--user-data-dir`）。
  - 安装包只含 GUI，后端 `gaasd_server`（codeTools/new_gaasd、hardwarePlatform、components）需单独导入；已通过 GUI「系统环境管理」导入到新 profile：环境镜像 `ubuntuEnv.tar`、工具包 `gaasd_code_tools_20260529.tar`、清华组件 `THICV`。
  - 2026-06-04 已手动更新 0603 后端和组件包：
    - 工具包：`/home/aiden/文档/temp/gaasd_code_tools_20260603.tar`
    - 组件包：`/home/aiden/文档/temp/清华组件包_0603_3.tar`
    - 更新目标：`/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/codeTools` 和 `.../components/THICV`
    - 更新前备份：`backups/gaasd-2.7.0.5/20260604_103442/`
  - 0603 版 `dist/run_simulation.sh` 已改变 `struct.json` 缺失处理：不再直接退出，而是输出警告并创建空 `{}`。这可能解决新版字典缺少旧 `struct.json` 导致仿真启动失败的问题。
  - 0603 版 `codescan --help` 仍显示为“C代码扫描工具”，未发现 `new_gaasd` 可执行文件；新版 C++ FuncModule 扫描能力仍需实际验证或等待团队确认。

- 新版重建工程 `project/newaccpro2`（此处记录的是初建阶段，当前完整进展见 2026-06-09 章节）：
  - 自定义组件由 `gaasd_carla_p0_acc_min_components.tar.gz` 导入（4 个 CARLA 组件，THICV 供应商）。
  - 适配器 `libcarla_gaasd_adapter.so`（5/26 版）已放入 `objectCode/total/DLL/`。
  - 代码生成正常（FuncStep.c 正确），但示波器仿真报错。

- **已定位的新版关键问题（GAASD 侧）**：
  - 仿真报错根因：旧版 `run_simulation.sh` 第 248 行硬检查 `dictionaryData/struct.json`，而新版 GUI 生成的是新格式字典（`baseTypes.json`、`GlobalContext.json`、分散的 `*Types.json`），无 `struct.json` → 仿真启动即退出，未进编译。**已反馈开发团队（工具包内新旧组件不配套）。**
  - 新版新增「全局/局部 状态与参数」体系：`GlobalVariable.json`(GlobalParams/GlobalStates)、复合组件 Param/State、`GlobalContext`；后端接口 `getGlobalVariableConfig` / `updateCompositeVariableConfig` 等（旧版无）。

- **自定义组件按新版 C++ 规范改写完成**：
  - 依据 `docs/C++代码规范.md`(v1.0.31) 与 `docs/组件改写执行提示词.md`，输出到 `generated/gaasd_p0_acc_min_components_src/`。
  - 5 个模块改写为 `FuncModule<Traits>` C++20 组件（Input/Output/Param/State/Sub + run()），含固定 `FuncModule.hpp`、`CMakeLists.txt`、`校验报告.md`。
  - 在 docker `ubuntu:env`（gcc9.4）自包含编译通过（产物 `libcarla_acc_min_components.a`）；GCC9 concepts 需 `-fconcepts`，已在 CMake 按版本条件追加。
  - `CARLAACCComputeTargetSpeed` 纯计算完全合规；4 个 I/O 模块因 CARLA 适配器为 C ABI（指针出参），`run()` 含指针，与 `BODY-006`/`RUN-009` 冲突，**校验报告中标为「需确认」**（边界适配固有约束，待确认新版是否提供图级边界输入机制）。

- **全部 5 组共 18 个组件已按新规范改写并编译通过**（acc_min 5、acc_decision 2、p0 4、lks 4、p1 3），均输出到 `generated/<工程>/`。

- **组件重要性取舍（2026-06-03 决定，详见 `generated/组件状态与取舍.md`）**：
  - 🟢 核心在用（维护、优先扫描入库）：`gaasd_p0_acc_min_components_src` 的 4 个 I/O（newaccpro2 用）+ `gaasd_lks_components_src` 的 4 个（lkspro1 用）。
  - 🟡 未来扩展（保留待用）：`gaasd_p1_components_src`（ObjectList/LateralCmd/ControlCmd）。
  - 🔴 废弃/已被取代（不再维护）：`gaasd_acc_decision_components_src`（决策已走真值表+基础块路线）、`gaasd_p0_components_src`（多输出版被 acc_min 单输出版取代）、acc_min 内的 `ComputeTargetSpeed`（目标速度用基础块算）。
  - CARLAObjectList 为数组型边界接口，存在 TYPE-013/015、BODY-006/022 多条例外，列为未来扩展暂不使用。

- **ACC 完整决策画布路线修正（2026-06-03）**：
  - 主线明确为“CARLA 边界自定义组件 + ACC 内部基础模块搭建”。自定义组件只用于 `CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、`CARLAACCLeadDistance`、`CARLAACCLongitudinalCmd` 等 Bridge 通信边界。
  - ACC 内部决策和控制尽量使用 GAASD 基础模块：`constant`、`read-local-state/write-local-state`、`add/subtract/multiply/divide`、`equal/not-equal`、`less/greater`、`logic-and/or/not`、`fmin/fmax`、`truth-table`、`oscilloscope`。新版局部状态组件优先，`static-variable` 只作为旧版或状态组件不可用时的兜底。
  - 决策层以 `truth-table` 为主线，按旧版 `accpro2` 的 GAASD 真值表习惯直接输出端口 `y`。推荐定义 `y=0` 为无新动作/保持参数，`y=1..8` 对应 R1..R8；后级用基础比较块生成 `R1=(y==1)`、`R2=(y==2)` 等分支条件。
  - 不再引入 `decisionEvent` / `activeDecisionY` / `lastActiveDecision` 这套额外命名和反馈链。`y=0` 时不触发 `timeGap/maxSpeed` 更新，参数由局部状态 `timeGap/maxSpeed/controlEnabled/hasHistory` 自然保持，避免无新指令时重复触发降距/增距。
  - 真值表先按已验证的 2 输入形态设计：`systemState, commandType -> y`。当前主线把 `commandType` 定义为单周期驾驶指令脉冲，不在画布内搭边沿检测。
  - `docs/GAASD_ACC_完整决策控制画布方案.md` 已补充完整画布连线表，按实例名和端口列出基础跟车、指令脉冲输入、S0-S3 状态、真值表、enable、timeGap、maxSpeed、hasHistory、示波器各阶段连接方式；同时补充 `Mul_/Sub_/Eq_/And_` 和 `RS_/WS_` 等实例名前缀与 GAASD 基础模块类型的对应关系，便于照表拖模块和连线。
  - `commandType` 正常为 0；有驾驶指令时只允许非零保持 1 个 GAASD 仿真周期，然后回到 0。若后续输入源不能保证单周期脉冲，再补 `lastCommandType` 边沿检测模块。
  - CARLA 静止起步测试需要处理低速 S3 死锁：文档一致模式可让 ego 初速高于 `vMin`；CARLA 测试模式建议 `vMin=0.0`。同时 R5 当前速度启控不能在 `egoV=0` 时把 `maxSpeed` 覆盖成 0，需加 `r5CaptureMinSpeed` 保护。
  - 多个跨周期状态量（`controlEnabled`、`hasHistory`、`timeGap`、`maxSpeed`）必须先在新版局部 `State` 中配置，再通过 `read-local-state/write-local-state` 分阶段接入；每接一个状态写入后生成代码检查环路/拓扑排序，不能一次性全部铺开。
  - `tools/carla_bridge/gaasd_acc_full_decision_components/` 已降级为备用验证件，不作为正式画布导入主线；正式方案见 `docs/GAASD_ACC_完整决策控制画布方案.md`。

- **codescan 验证结论**：本地 codescan 是旧版 C 扫描器，扫旧 `.c` 能出 5 组件，但**完全识别不了新版 C++ FuncModule 格式**（0 组件）。即工具包 codescan / run_simulation / gaas_codegen 全是旧版，只有 GUI 是新版。改写后的 C++ 组件须等团队提供配套新版 codescan 才能扫描入库。

**下一步任务**：

- 等开发团队提供与新版 GUI 字典格式配套的 `run_simulation.sh`（或确认新版后端 `new_gaasd`），解决 `struct.json` 报错，跑通 newaccpro2 闭环。
- 用新版 codescan 扫描 `generated/gaasd_p0_acc_min_components_src/`，确认改写后的 C++ 组件能被新版正确扫描入库。
- 确认 I/O 边界模块的处理方式：保留 run() 内适配器调用，还是改为新版图级边界输入。

---

## 当前状态（2026-05-26 更新）

**LKS 单车联合仿真准备**：

- Bridge 新增 `gaasd.carla.lane_tracking.v1`，持续发布 `lateral_offset_m`、`heading_error_rad`、道路标识及有效标志；数据发布不依赖 Bridge 内置车道保持开关。
- adapter 新增 `carla_adapter_read_lane_tracking()`，并用 `ubuntu:env` 容器构建及 mock 测试验证通过；产物位于 `tools/carla_bridge/adapter/dist/ubuntu_env/libcarla_gaasd_adapter.so`。
- 新增 GAASD LKS 单输出组件源码及扫描包：`tools/carla_bridge/gaasd_lks_components_src/`、`tools/carla_bridge/gaasd_lks_components/`、`tools/carla_bridge/gaasd_carla_lks_components.tar.gz`。
- 新增单车测试场景 `scenarios/lkspro1_basic_carla_20260526/`：不生成前车，初始横向偏差 `0.8 m`、航向误差 `5 deg`，配置中显式关闭 `lane_keep_enabled`。
- LKS 首版画布公式确定为 `steerRad = limit(0.072 * lateralOffset + 0.48 * headingError, -0.15, 0.15)`，目标速度固定为 `2.0 m/s`。
- 实际消息探测通过：Bridge 输出 `lateral_offset_m=0.800000`、`heading_error_rad=0.087267`、`valid=true`。
- adapter 真实通信短测通过：`lks_live_result ego_valid=48 lane_valid=48 chassis_valid=48 commands=31 lateral_offset=0.800000 heading_error=0.087267`。

**启动稳定性修复**：

- `tools/carla_bridge/start-gaasd-carla-manual.sh` 不再以裸 TCP 方式检测 CARLA RPC 端口；旧 ACC 场景快照启动脚本同步修复。
- `tools/carla_bridge/health-carla.sh` 改用 CARLA Python API 健康检查；UI 面板只探测 Bridge 端口，不触碰 CARLA RPC 端口。

**下一步任务**：

- 在 GAASD 页面导入 `gaasd_lks_components`，新建 `lkspro1` 画布并按场景 README 连线。
- GAASD 生成工程后，将新版 `libcarla_gaasd_adapter.so` 放入工程 `objectCode/total/DLL/`，再运行示波器验证横向偏差收敛。

---

## 当前状态（2026-05-25 更新）

**Bug 修复：UI 面板健康检查导致 CARLA 崩溃**

- **现象**：联调仿真启动约 90 秒后 CARLA 无故退出，崩溃报告显示 `SecondsSinceStart≈33`、`rpc::detail::server_session::close()` + `close: Bad file descriptor`。
- **根本原因**：UI 面板每 3 秒自动对 CARLA 2000 端口做一次"裸 TCP 连接后立即断开"的健康探测。CARLA 2000 端口是 LibRPC 会话端口（非普通 HTTP），连上后不按协议通信直接断开，会触发 CARLA 0.9.15 服务端会话清理路径的隐藏 bug（文件句柄已关闭再次关闭），**第 29 次**连接时 CARLA 崩溃。复现验证：单独运行 CARLA 稳定；加入 Bridge + 前车场景稳定；对正常运行的 CARLA 重复执行裸 TCP 探测，第 29 次后崩溃，调用栈一致。
- **修复内容**：
  - `tools/gaasd_scenario_panel/app.py:294`：CARLA 健康状态不再调用 `socket_open(2000)`，改为 `bridge_pub_ok and bridge_control_ok` 推断（Bridge 存活即 CARLA 存活）。
  - `scenarios/acc_carla_phase2_20260513/bridge_snapshot/tools/carla_bridge/health-carla.sh`：重写，探测 Bridge ZMQ 5701/5702，不再碰 CARLA 2000 端口。
  - `start-gaasd-carla-manual.sh`：启动时"CARLA 是否已在运行"的判断从 `tcp_open` 改为 `carla_api_ready`（Python CARLA API 方式，安全）。

---

## 当前状态（2026-05-19 更新）

**当前工作目标**：保留已跑通的 ACC 跟车 CARLA-GAASD 联合仿真快照，并在 P0 最小闭环基础上逐步扩展通用 CARLA 接口。

**已确认状态**：
- `http://127.0.0.1:8765/` 本机场景启动面板可直接启动 `ACC 跟车 CARLA-GAASD 联合仿真快照`。
- 点击“启动环境”后，CARLA + Bridge 可启动，CARLA 视角自动进入已确认的跟随视角。
- 前车和自车位置符合当前设计，可作为后续 GAASD 示波器运行 ACC 闭环的基准场景。
- 根目录 `tools/carla_bridge/` 与场景快照 `scenarios/acc_carla_phase2_20260513/bridge_snapshot/tools/carla_bridge/` 的关键启动逻辑已同步：
  - `reset-acc-straight-scene.py` 支持 `ego_spawn_index=198` 越界时使用 Bridge 已放置 ego 位置。
  - `set-spectator-follow.py` 后台跟随不再等待 CARLA tick。
  - `start-gaasd-carla-manual.sh` 后台跟随前先执行一次 `--once` 固定初始视角。
- UI 页面代码保存在 `tools/gaasd_scenario_panel/`。
- 可复现快照记录见 `docs/GAASD_CARLA_场景快照备份记录.md`。
- GAASD 重新生成代码后的 `carla.h` 兼容修复已标准化：`tools/carla_bridge/fix-gaasd-generated-code.py` 可修复 `protobuf-c` 头文件缺失和 `Infopack__TrafficLight__State` 枚举缺失问题，并已接入 UI 面板“修复生成代码”按钮。
- P1 扩展组件包已生成：
  - 源码：`tools/carla_bridge/gaasd_p1_components_src/carlaP1Components.c`
  - 扫描组件包：`tools/carla_bridge/gaasd_p1_components/`
  - 压缩包：`tools/carla_bridge/gaasd_carla_p1_components.tar.gz`
  - 组件：`CARLAObjectList`、`CARLALateralCmd`、`CARLAControlCmd`
  - adapter 新增 C ABI：`carla_adapter_read_object_list`、`carla_adapter_publish_lateral_cmd`、`carla_adapter_publish_control_cmd`
  - 验证：P1 组件源码 `cc -std=c99 -Wall -Wextra -fPIC -c` 通过；`ubuntu:env` 内 adapter CMake 构建通过；`carla_gaasd_adapter_mock_loop` 已覆盖 `object_list`、横向控制和联合控制命令，输出 `mock_loop_ok`。
- 2026-05-20 新增 `accpro2` 基础模块版 ACC 测试场景：
  - `project/accpro2` 画布使用官方基础运算模块拼出 ACC 目标速度公式。
  - 生成代码主链路为 `CARLAACCLeadDistance / CARLAACCLeadSpeed / CARLAACCEgoSpeed -> subtract / multiply / add -> CARLAACCLongitudinalCmd`。
  - 当前生成公式为 `targetSpeed = 0.35 * (distance - 15.0) + 0.8 * (leadV - egoV) + leadV`。
  - 已补充 `project/accpro2/objectCode/total/DLL/` 的 `libcarla_gaasd_adapter.so` 与 ZMQ 运行库。
  - `tools/carla_bridge/fix-gaasd-generated-code.py` 已扩展为修复任意 GAASD 生成头文件，当前已修复 `project/accpro2/icvos/src/functions/accpro2.h`。
  - 新增 UI 场景目录 `scenarios/accpro2_basic_carla_20260520/`，可在 `http://127.0.0.1:8765/` 中选择并启动 CARLA + Bridge 环境。
  - 通过 GAASD 示波器运行按钮已生成 `project/accpro2/icvos/src/oscilloscopeFunctions/.../FuncStep.c`。
  - `FuncStep.c` 已包含 `scope_push_send(...)` 和 `CARLAACCLongitudinalCmd(...)`，示波器观测顺序为 `targetSpeed`、`egoV`、`leadV`、`distance`。

**下一步任务**：
- 后续继续基于该快照做动态前车跟车调参和稳定性验证。
- 在 GAASD 页面导入 P1 组件包，验证 `CARLAObjectList` 的数组/多输出端口是否符合当前代码生成器能力。
- 若数组端口在 GAASD 生成工程中不稳定，补充 P1 标量拆分组件或推动 GAASD 代码生成器支持固定长度数组端口。
- 在 `accpro2` 画布中补充 `limit(rawTarget, 0.0, 5.0)` 后重新生成代码。

---

## 当前状态（2026-05-09 更新）

**当前工作目标**：GAASD 接入 CARLA，完成 ACC 最小闭环的阶段二联合仿真验证。

**当前路线**：继续采用 P0 C 包装组件方案。`CARLA-json.zip` 中的 `carla-sim-input/output` 组件目前只能用于 GAASD 组件显示和连线关系，不能替代当前已跑通的 C 运行组件；真实闭环仍由 `carla_adapter_*` 适配库和 ACC 最小闭环组件完成。

**已完成验证**：
- 阶段一 Python `acc-runner.py` 闭环通过：CARLA -> Bridge -> Python ACC Runner -> Bridge -> CARLA。
- 阶段一 60 秒加固测试通过：Bridge 约 20Hz 发布，`control_cmd_received=1201`，前车持续有效，超时制动正常。
- 阶段二 Docker 前置验证通过：`ubuntu:env` 已补齐 `libzmq3-dev` / `pkg-config`，GAASD `dist/CMakeLists.txt` 可链接外部 `.so`。
- 阶段二 P0 适配库测试通过：
  - `libcarla_gaasd_adapter.so` 可在 `ubuntu:env` 内编译。
  - C ABI 符号 `carla_adapter_read_*` / `carla_adapter_publish_longitudinal_cmd` 已导出。
  - smoke test 通过：无 Bridge 时接口安全返回。
  - mock ZMQ 测试通过：`ego_state` / `lead_vehicle` / `chassis_feedback` / `control_cmd` 字段映射正确。
  - live Bridge 短测通过：`ego_valid=202`、`lead_valid=202`、`chassis_valid=202`、`commands=122`，Bridge 侧 `control_cmd_received=122`。

**Claude 审核结论（2026-04-29）**：
- 认可“阶段二 P0 适配库通信链路已通过”，但边界是“适配库独立通信”，不是“GAASD 画布闭环已通过”。
- 必须修复/验证：
  - `libcarla_gaasd_adapter.so` 的 SONAME 部署风险。
  - `chassis_feedback` 中 `steer_rad` 与 Bridge 实际字段 `steering_angle_rad` 不匹配。
  - 用 `run_simulation.sh` 跑最小 `FuncStep.c`，验证 GAASD 生成工程真实加载并调用适配库。

**下一步任务（CARLA 阶段二）**：
- [x] 修复适配库 SONAME 和 `steering_angle_rad` 字段映射。
- [x] 重跑 `ubuntu:env` 内编译、smoke、mock 测试。
- [x] 准备最小 GAASD FuncStep 测试工程，放置 `objectCode/total/DLL/libcarla_gaasd_adapter.so`。
- [x] 用 `run_simulation.sh` 验证真实 GAASD 运行框架能加载并调用适配库。
- [x] 实现 `CARLAEgoState`、`CARLALeadVehicle`、`CARLALongitudinalCmd` 三个 P0 组件包装函数，并额外提供 `CARLAChassisFeedback` 观测组件。
- [x] 在 GAASD 页面导入 P0 ACC 最小闭环组件包，搭建 ACC 画布闭环并运行真实联调。
- [x] 固定前车场景，完成稳定跟车效果和 60 秒长跑稳定性验证。
- [ ] 动态前车场景测试：前车低速行驶，验证 `leadV` 非零时的跟车响应。

**2026-04-29 至 2026-05-09 后续测试结果**：
- 修复后 `readelf` 显示 `Library soname: [libcarla_gaasd_adapter.so]`。
- `adapter_smoke_test` 通过。
- `adapter_mock_loop_test` 通过，输出 `mock_loop_ok`。
- 最小 `run_simulation.sh` 无 Bridge 测试通过：执行 5 步 `FuncStep`，`egoValid=0`、`leadValid=0`，安全降级。
- 最小 `run_simulation.sh` + 真实 CARLA/Bridge 测试通过：执行 21 步 `FuncStep`，每步 `egoValid=1`、`leadValid=1`，Bridge 侧 `control_cmd_received=21`，`chassis_feedback.last_command_id=21`。
- P0 组件包已生成：`tools/carla_bridge/gaasd_p0_components/`，压缩包为 `tools/carla_bridge/gaasd_carla_p0_components.tar.gz`。
- `codescan` 生成 4 个组件 JSON：`CARLAEgoState`、`CARLALeadVehicle`、`CARLALongitudinalCmd`、`CARLAChassisFeedback`。
- P0 组件源码 `cc -std=c99 -Wall -Wextra -fPIC -c` 编译无警告。
- GAASD 页面已导入 CARLA 组件包，`/home/aiden/.gaasd/imported/component/` 中已有 4 个 CARLA 组件 JSON。
- `project/carla/data/temp.db` 中已保存 ACC 最小闭环画布：8 个节点、5 条必需连线，连线关系正确。
- 已在 `ubuntu:env` 内重新构建 `libcarla_gaasd_adapter.so`，并放置到 `project/carla/objectCode/total/DLL/`；SONAME 为 `libcarla_gaasd_adapter.so`，依赖 `libzmq.so.5` 正常。
- GAASD 页面重新生成代码后，`project/carla/icvos/blocks/functions/c_process_*.json` 已包含画布里的 CARLA/ACC 子组件，`project/carla/icvos/src/functions/` 已生成对应 C 源码。
- 当前新阻塞点：生成的 C 工程暂不能编译。
  - `carla.h` 引用了 `Infopack__TrafficLight__State`，但未生成该枚举定义，导致组件源码编译失败。
  - `main.c` 对多输出 CARLA 组件只传入已连线端口，例如 `CARLALeadVehicle(&leadV, &distance)`，但函数原型要求完整输出参数列表 `leadV/distance/relativeSpeed/ttc/valid`，导致参数数量不匹配。
  - 结论：画布保存与代码生成链路已前进一大步，但当前 P0 多输出组件设计与 GAASD 代码生成器行为不完全匹配。下一步建议拆分/裁剪为 ACC 最小闭环专用单输出或少输出组件，再重新导入并生成代码。
- 已新增 ACC 最小闭环组件包：
  - 源码：`tools/carla_bridge/gaasd_p0_acc_min_components_src/carlaAccMinComponents.c`
  - 扫描包：`tools/carla_bridge/gaasd_p0_acc_min_components/`
  - 压缩包：`tools/carla_bridge/gaasd_carla_p0_acc_min_components.tar.gz`
  - 组件：`CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、`CARLAACCLeadDistance`、`CARLAACCLongitudinalCmd`
  - 验证：组件源码 `cc -std=c99 -Wall -Wextra -fPIC -c` 通过；`codescan --custom` 生成 4 个组件 JSON 和 `cbdes.db`。
  - 下一步：在 GAASD 页面导入该最小组件包，替换原画布中的 `CARLAEgoState`、`CARLALeadVehicle`、`CARLALongitudinalCmd`，重新生成代码后再检查编译。
- GAASD 页面已用 ACC 最小闭环组件重新搭建并生成代码：
  - `project/carla/icvos/blocks/functions/` 中已替换为 `CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、`CARLAACCLeadDistance`、`CARLAACCLongitudinalCmd`。
  - `project/carla/icvos/src/functions/c_process_*/main.c` 调用关系正确：`egoV -> leadV -> distance -> accComputeTargetSpeed -> CARLAACCLongitudinalCmd`。
  - 之前的多输出组件参数不完整问题已消失。
  - 剩余生成问题：
    - `carla.h` 仍漏生成 `Infopack__TrafficLight__State` 枚举。
    - 当前 `accComputeTargetSpeed` 组件仍是旧三入参版本，函数体引用 `ACC_DESIRED_DIST` / `ACC_MAX_SPEED`，但生成头文件没有对应宏。
  - 已对当前生成工程临时补 `carla.h` 枚举和 ACC 宏后验证：组件 `.o`、`libcarla_static.a`、`libcarla_shared.so` 均可生成。
  - 当前独立 CMake 可执行文件链接失败，原因是生成 CMake 未链接 `objectCode/total/DLL/libcarla_gaasd_adapter.so`；该适配库已存在，真实仿真需通过 GAASD/codeTools 运行流程把 `PROJECT_EXTRA_LIBS` 带入。
  - `project/carla` 当前未生成 `FuncStep.c`，直接用 `run_simulation.sh` 还缺少运行入口；下一步需确认 GAASD 页面“启动仿真”是否会生成/调用 `FuncStep.c`，或需要我们手动提供一个 `FuncStep.c` 包装当前画布逻辑。
- 已从源头修复 ACC 宏依赖问题：
  - `tools/carla_bridge/gaasd_p0_acc_min_components_src/carlaAccMinComponents.c` 新增 `CARLAACCComputeTargetSpeed`。
  - 该组件直接在函数体内使用局部常量 `desiredDist=15.0`、`maxSpeed=12.0/3.6`、`kDist=0.5`、`kSpeed=0.5`，不依赖 `ACC_DESIRED_DIST` / `ACC_MAX_SPEED` 宏。
  - 已重新扫描生成 5 个 ACC 最小闭环组件并更新压缩包。
  - 下一次画布应使用 `CARLAACCComputeTargetSpeed` 替代旧 `accComputeTargetSpeed`。
- GAASD 页面已重新导入并使用 `CARLAACCComputeTargetSpeed` 生成代码：
  - `main.c` 调用关系正确：`CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、`CARLAACCLeadDistance`、`CARLAACCComputeTargetSpeed`、`CARLAACCLongitudinalCmd`。
  - 旧 `accComputeTargetSpeed` 和 `ACC_DESIRED_DIST` / `ACC_MAX_SPEED` 宏依赖已从生成工程中消失。
  - 直接编译仍先失败于 `carla.h` 漏生成 `Infopack__TrafficLight__State`；`enum.json` 中存在该枚举，确认是 GAASD 代码生成器未把 enum 写入头文件。
  - 临时补 enum 后，所有组件对象文件、`libcarla_static.a`、`libcarla_shared.so` 可生成。
  - 独立 CMake 可执行文件仍因未链接 `libcarla_gaasd_adapter.so` 失败；手动加入 `project/carla/objectCode/total/DLL/libcarla_gaasd_adapter.so` 后链接成功，`ldd` 可解析 `libcarla_gaasd_adapter.so`、`libzmq.so.5`。
  - 结论：当前画布代码链路已可编译/可链接；剩余问题是 GAASD 生成器 enum bug 和正式仿真启动流程是否传递 `PROJECT_EXTRA_LIBS`。
- 阶段二 GAASD 画布真实闭环已跑通：
  - 手动启动脚本：`tools/carla_bridge/start-gaasd-carla-manual.sh`，默认使用 `tools/carla_bridge/config.phase2.json`。
  - GAASD 示波器周期从 `0.5s` 调整到 `0.1s` 后，30 秒仿真产生 `301` 个采样点。
  - 最新有效日志：`/home/aiden/gaasd_server/codeTools/dist/logs/run_simulation_20260508_143150.log`。
  - `egoV` 从 `0` 上升到最高约 `2.35 m/s`，结束时约 `2.31 m/s`。
  - Bridge 侧 `control_cmd_received=301`、`last_command_id=301`，说明 GAASD 控制命令已持续进入 Bridge。
  - 结论：链路 `CARLA -> Bridge -> GAASD 组件 -> GAASD 控制输出 -> Bridge -> CARLA 自车` 已成立。
  - 剩余现象：仿真后半段 `leadV=0`、`distance=1000000`，说明前车后续未被 Bridge 选为有效同车道前车；下一步需要固定前车行为和车道位置后验证 ACC 跟车收敛效果。
- 前车固定策略已调整，待下一轮 GAASD 页面重跑验证：
  - `tools/carla_bridge/spawn-lead-vehicle.py` 默认从 `ego_forward + constant_velocity` 改为 `ego_forward + traffic_manager`。
  - 前车默认放在 ego 正前方，由 CARLA Traffic Manager 管理，禁止变道、忽略红绿灯。
  - 当前地图出生点下 `lane_waypoint` 会把前车放到侧向约 16m 的位置，暂不作为默认策略。
  - Bridge 已增加 `role_name` 输出，并在阶段二配置中优先选择 `role_name=gaasd_lead` 的测试前车，允许该前车使用更宽的横向筛选阈值。
  - `tools/carla_bridge/start-gaasd-carla-manual.sh` 新增 `--lead-placement`、`--lead-behavior`、`--tm-port` 参数，并默认使用 `ego_forward` / `traffic_manager`。
  - 命令行侧 20 秒探测结果：`ego_forward + traffic_manager` 下 `lead_vehicle.valid=399/399`，`selection_rule=preferred_role_nearest_front`，`clearance_m≈20.21`，`lateral_distance≈0`。
  - 当前 P0 前车可视为静止目标，因此 `leadV=0` 属于预期；下一轮 30 秒日志重点确认 `distance` 不再跳到 `1000000`，`egoV` 和 `targetSpeed` 对静止前车产生响应。
- 固定前车后的 30 秒 GAASD 画布测试通过：
  - 最新有效日志：`/home/aiden/gaasd_server/codeTools/dist/logs/run_simulation_20260509_112022.log`。
  - 示波器周期 `0.1s`，30 秒产生 `301` 个采样点。
  - Bridge 侧 `control_cmd_received=301`、`last_command_id=301`。
  - `distance` 从约 `20.21m` 收敛到约 `15.64m`，全程未跳到 `1000000`。
  - `egoV` 从 `0` 上升到最高约 `1.90m/s`，末值约 `0.06m/s`；说明自车先靠近前车，随后因 ACC 目标速度下降而减速。
  - `targetSpeed` 从约 `2.60m/s` 下降到约 `0.29m/s`，符合接近静止前车时的降速行为。
  - Bridge 实时探测显示 `lead_vehicle.valid=true`、`selection_rule=preferred_role_nearest_front`、`clearance_m≈15.63`、`lateral_distance≈-0.07`。
  - 结论：P0 30 秒 ACC 静止前车闭环验证通过；下一步补跑 60 秒稳定性测试。
- 60 秒稳定性测试通过：
  - 最新有效日志：`/home/aiden/gaasd_server/codeTools/dist/logs/run_simulation_20260511_114220.log`。
  - 示波器周期 `0.1s`，60 秒产生 `601` 个采样点。
  - `distance` 从约 `20.21m` 收敛到约 `15.38m`，全程无 `1000000` 跳变。
  - `egoV` 从 `0` 上升到最高约 `1.86m/s`，末值约 `0.046m/s`。
  - `targetSpeed` 从约 `2.60m/s` 下降到约 `0.17m/s`。
  - 结论：P0 静止前车 60 秒稳定性验证通过；下一步进入动态前车场景。
- 软件团队提供的 `/data/aiden/下载/CARLA-json.zip` 已检查：
  - 包含 `CARLASimClock`、`CARLAEgoState`、`CARLAObjectList`、`CARLALeadVehicle`、`CARLATrafficLightList`、`CARLARoute`、`CARLAControlCmd`、`CARLALongitudinalCmd`、`CARLALateralCmd`、`CARLATrajectoryCmd`、`CARLAChassisFeedback` 共 11 个组件 JSON。
  - topic 和主要端口字段与协议基本一致。
  - 组件类型为 `carla-sim-input/output`，没有 `function_body`；软件团队确认 GAASD 当前只能提供组件显示和连线关系。
  - 结论：该包可作为后续正式 UI 组件定义参考，但当前不能承担真实数据订阅/发布运行逻辑，P0 测试继续使用 C 包装组件。

---

## 当前状态（2026-04-01 更新）

**工作目标**：将 `huanyuan1/thicv-pilot/planningFigure/` 里的规划算法从 C++ 迁移到纯 C。

**当前验证版路径**：`huanyuan1/thicv-pilot/planningFigure/`
- C 文件放在各自子目录（`GaussConvert/`、`GetMinDistanceOfPoint/` 等）
- C++ 框架文件放在 `src/`
- 构建已通过：`[100%] Built target planning`，`start_all.sh` 可运行

**最近完成的工作**：
- 新增 `ACC/accSimStep.c`，实现单步 ACC 仿真推进逻辑（前车更新→测距→目标速度→PID→自车更新）
- 将 `ACC/accSimStep.c` 接入 `planningFigure/CMakeLists.txt`
- 新建了 4 个 C 模块（见下方"已完成记录"第一条）
- 修复了 `localPlanning_m.cpp:3257` 遗留的坐标交叉赋值 Bug
- 在 AGENTS.md 中新增了"坐标系约定"章节

---

## 下一步任务（按优先级）

### P1 — 立即可做

- [ ] **将坐标系约定注释补充到 `gaussConvert.h` 的 Doxygen**
  - 在 `gaussConvert` 的 `@param` 里说明 `dNorth_X=北坐标/dEast_Y=东坐标`
  - 在 `gaussConvertOutput` 结构体字段注释里写清楚含义

- [ ] **提交当前所有修改到 git**
  - 修改的文件：`GaussConvert/`、`GetMinDistanceOfPoint/`、`AssessTrajectory/`、`GetOptimalTrajectoryIndex/`、`include/localPlanningNew.h`、`include/localPlanning_m.hpp`、`src/localPlanning_m.cpp`、`src/planningFigure_m.cpp`、`CMakeLists.txt`、`AGENTS.md`、`../../start_all.sh`

### P2 — 下一个大模块

- [ ] **坐标转换模块 `geometry_m.cpp`**（AGENTS.md 中标记为 🔴）
  - 参考文件：`src/geometry_m.cpp`
  - 输出位置：建议放 `Geometry/` 目录
  - 注意：涉及坐标变换，严格遵守 AGENTS.md 的坐标系约定

- [ ] **线性插值 `interpolate_m.cpp`**（标记为 🔴）
  - 参考文件：`src/interpolate_m.cpp`

### P3 — 工具库（其他模块依赖）

- [ ] **`DataStructure/DynArray/dynArray.c`**（已有 `dynArray.h`，实现未完成）
  - 接口在 `dynArray.h` 已定义，用 `/codex-impl` 生成实现

---

## 已完成工作记录

### [2026-03-30] 最优轨迹选取模块全面重写

**目标**：移植 `localPlanning_m.cpp` 中的障碍物距离评估链路到 C 语言。

**新建文件**：

| 文件 | 来源 C++ | 说明 |
|------|----------|------|
| `GaussConvert/gaussConvert.c/.h` | `localPlanning_m.cpp:62~95` + `115~139` | 经纬度转高斯坐标、全局→局部坐标变换 |
| `GetMinDistanceOfPoint/getMinDistanceOfPoint.c/.h` | `localPlanning_m.cpp:3204~3327` + `3341+` | 计算路径点到障碍物最近距离，含 5 个 static 辅助函数 |
| `AssessTrajectory/assessTrajectory.c/.h` | `localPlanning_m.cpp:3154~3189` | 遍历轨迹各点求最小障碍物距离 |
| `GetOptimalTrajectoryIndex/getOptimalTrajectoryIndex.c` | 原有 C++ 风格文件，完全重写 | 红绿灯处理 + 最优轨迹选取 |

**修改文件**：

| 文件 | 改了什么 |
|------|----------|
| `include/localPlanningNew.h` | 末尾新增 9 个 typedef struct（高斯/坐标变换/findMin/getMinDistanceOfPoint 相关）|
| `include/localPlanning_m.hpp` | 12 个 C++ struct 加 `Cpp` 后缀，避免与 C typedef 名称冲突 |
| `src/localPlanning_m.cpp` | 同步更新 12 个 struct 使用处为 `Cpp` 后缀 |
| `src/planningFigure_m.cpp` | 同步更新 4 个 struct 使用处为 `Cpp` 后缀 |
| `CMakeLists.txt` | 新增 4 个 C 文件到编译目标；include 目录补全；源文件路径改为 `./src/` 前缀 |
| `../../start_all.sh` | 新增：启动前备份 planning 二进制，防止 make 失败删除唯一可运行版本 |

**Bug 修复**：

| 位置 | 问题 | 修复方式 |
|------|------|----------|
| `getMinDistanceOfPoint.c` `appendSingleRoadsideObjectDistances` | 从 `localPlanning_m.cpp:3257` 照抄了坐标交叉赋值（East/North 写反）| 改为正确写法：`gaussNorthTemp = outputGC.dNorth_X; gaussEastTemp = outputGC.dEast_Y` |

**说明**：此 Bug 在原 C++ `localPlanning_m.cpp` 中存在，`planningFigure_m.cpp:1047` 是正确的参考实现。不影响当前运行的原因：后续只算欧氏距离，x/y 对调不改变 `sqrt(dx²+dy²)` 的值。

**注释**：4 个新 C 文件均包含：
- 函数级 Doxygen 注释
- 内联步骤注释（含对应 C++ 文件行号）
- 所有局部变量行内说明

---

**本次工作遗留的编译调试记录**（供参考，非待办）：

- CMakeCache 指向 Docker 路径 `/tmp/Modularization`，本地编译需删除 cache 重新 cmake
- libzmq.so 符号链接缺失，需手动 `ln -s libzmq.so.5 libzmq.so`
- `.cpp` 源文件曾丢失（git stash），从 `stash@{0}` 恢复后更新 CMakeLists.txt 路径前缀
- make 失败会删除 planning 二进制（POSIX 行为），已在 start_all.sh 加备份逻辑

---

## 坐标系约定速查（详见 AGENTS.md）

```
gaussConvert 输出：dNorth_X = 北坐标(y)，dEast_Y = 东坐标(x)
IMU 字段：       gaussx   = 北坐标(y)，gaussy   = 东坐标(x)

正确赋值：
  gaussNorthTemp = outputGC.dNorth_X;
  gaussEastTemp  = outputGC.dEast_Y;
  dX = gaussEastTemp;   /* x = East */
  dY = gaussNorthTemp;  /* y = North */
```
