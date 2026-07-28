# 工作日志 WORKLOG

> **用途**：记录每次工作的修改内容、遗留问题和下一步任务。
> 新开会话或交给 Codex 前，先读这个文件。
> 每次工作结束后，更新"当前状态"和"下一步任务"两节。

---

## 当前状态（2026-07-27 更新）

**GAASD `20260711_2` 双版本安全更新**：

- 更新包：

```text
/home/aiden/文档/temp/GAASD_SETUP_20260711_2.tar.gz
SHA256: 0c462e79812b2c269df56dea5f864a43bebcebde9bee1b5bf7941f5af3efe0eb
```

- 已使用包内官方 `install.sh` 更新系统版前端，并用同一内层前端负载更新隔离版：

```text
系统版: /opt/gaasd
入口:   /usr/bin/gaasd -> /opt/gaasd/gaasd

隔离版: /home/aiden/gaasd_versions/gaasd-2.7.0.5/app
入口:   /home/aiden/gaasd_versions/gaasd-2.7.0.5/run-gaasd-2.7.0.5.sh
```

- 两个安装目录与安装包前端负载逐文件比对均为零差异；`ldd` 未发现缺失动态库，
  两处 `chrome-sandbox` 均保持 `root:root 4755`。
- 两套后端和用户数据未互相覆盖，仍分别使用：

```text
系统版后端: /home/aiden/gaasd_server
隔离版后端: /home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server
隔离版配置: /home/aiden/gaasd_versions/gaasd-2.7.0.5/home/.config/GAASD
```

- 新版不再接受旧启动脚本显式传入的 `--user-data-dir` 参数。已从隔离版启动脚本删除该参数，
  继续通过独立 `HOME` 和 `XDG_CONFIG_HOME/XDG_CACHE_HOME/XDG_DATA_HOME` 保持完整隔离。
- 系统版和隔离版均已完成真实 GUI 冒烟测试：主进程、zygote、GPU、network、renderer 进程
  正常保持运行，数据库和后端路由正常初始化；测试实例完成后已关闭。
- IDE/Codex 终端可能注入 `ELECTRON_RUN_AS_NODE=1`，直接执行 Electron 二进制时会表现为立即退出。
  隔离版启动脚本已主动 `unset ELECTRON_RUN_AS_NODE`；桌面图标启动不受该 IDE 环境变量影响。
- 更新前回滚副本：

```text
系统版: /opt/gaasd.rollback-before-20260711-2-20260727_155903
隔离版: /home/aiden/gaasd_versions/gaasd-2.7.0.5/rollback/gui_before_20260711_2_20260727_155903/app
清单:   /home/aiden/gaasd_versions/gaasd-2.7.0.5/rollback/gui_before_20260711_2_20260727_155903/update_manifest.txt
```

**newaccpro3 手动画布重构与数据库修改基线**：

- 已在 `project/newaccpro3` 的 `MainFlow` 下建立清晰的 `Decision / Control` 两层结构。
  `Control` 内部当前实现的纵向控制关系为：

```text
desiredDistance = max(minDistance, egoV * timeGap)
distanceError = distance - desiredDistance
relativeSpeed = leadV - egoV
rawTargetSpeed = leadV + kDist * distanceError + kSpeed * relativeSpeed
targetSpeed = min(maxSpeed, max(0, rawTargetSpeed)) * enable
```

- 顶层原有的五个观察输出 `valid / timeGap / maxSpeed / decision / systemState`
  按当前要求继续保留，暂未删除或改变语义。
- 已将顶层重复的旧控制计算链清理掉，并完成 `MainFlow` 重新排布；正式算法计算集中在
  `Control` 子系统内。
- 已用用户从当前组件库新拖入的示波器替换旧示波器，并迁移四条观测线：

```text
Control.targetSpeed -> scope.input1
egoV                -> scope.input2
leadV               -> scope.input3
distance            -> scope.input4
```

- 本次示波器问题的根因不是连线，而是组件代际不一致：旧示波器为
  `version=1.0.6, isCustom=1`，当前新示波器为 `version=1.1.7, isCustom=0`。
  旧实例仍能显示在画布上，但不能可靠打开新版属性配置面板；新实例可以正常配置
  `inputNumber / simulateStep / simulateTotalTime`。
- 修改前工程备份：

```text
/home/aiden/文档/Modularization_backups/newaccpro3_before_control_20260727_105735
/home/aiden/文档/Modularization_backups/newaccpro3_before_mainflow_20260727_110853
/home/aiden/文档/Modularization_backups/newaccpro3_before_scope_swap_20260727_111930
/home/aiden/文档/Modularization_backups/newaccpro3_before_decision_refactor_20260727_1725
```

**newaccpro3 `Decision` 内部分层重构**：

- 真值表默认动作已由 `R8/y=8` 改为 `R0/y=0`。`R0` 表示无新动作并保持当前状态；
  `R8` 保留为进入待命、退出当前控制的响应。
- 原 `Decision` 同层包含 66 个节点和 78 条连线，状态识别、决策、使能写回、时距更新和
  限速更新混排。现按物理职责拆为四个非空子模块：

```text
StateClassifier
ControlStateUpdate
TimeGapUpdate
MaxSpeedUpdate
```

- 主真值表没有额外套空壳组件，仅在 `Decision` 顶层重命名为 `DecisionTable`；无连线的
  `真值表_1` 已删除。
- `Decision` 的局部参数和局部状态仍由原层级的 `read-local-param / read-local-state /
  write-local-state` 访问，值通过子模块端口传递，没有改变状态作用域和读写时序。
- 各决策比较放入实际使用它的模块：R1/R2 属于 `MaxSpeedUpdate`，R3/R4 属于
  `TimeGapUpdate`，R5/R6 和 Cmd6/Cmd7 属于 `ControlStateUpdate`。
- 已新增可复现脚本 `tools/refactor_newaccpro3_decision.py`。脚本保留原基础组件 ID，迁移
  `parentId`，并为跨层信号建立明确的输入输出端口。
- 数据库验证结果：SQLite 完整性、连接端点、单输入唯一驱动、子系统边界、组合环和节点重叠
  全部通过；把四个子模块边界展平后得到的 78 条逻辑连线与修改前逐条完全一致；
  `cbdes.db` 与 `temp.db` SHA256 均为
  `ba85577b1a446cf0e0faa3cfdbcc5806656a2400e01322a97dd6a19b991ba96a`。
- 数据库验证不替代 GUI 和代码生成验证。下一步需重新打开 `newaccpro3`，逐级进入四个子模块，
  点击边界端口和真值表属性，并重新生成一次代码。

**同类组件兼容风险审计**：

- 当前 GAASD 组件库数据库：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/.gaasd/gaasd.db
```

- 当前库中的 `input / output / read-local-param / read-local-state /
  write-local-state / constant / oscilloscope / multiply / subtract / add /
  fmax / fmin / truth-table` 均已有 `1.1.7, isCustom=0` 标准版本。
- 端口数量不是判断组件新旧版本的依据。当前 `add` 的
  `base_config.inputNumber` 是可配置属性，库模板默认值为 2，但实例可以配置为三个或更多输入；
  当前 `RawTargetSpeed` 使用三输入 `add` 本身是合理设计。`multiply` 和示波器也具有可配置
  输入数量，必须区分“组件版本元数据”和“实例属性配置”。
- `Control` 子系统共 22 个实例，其中 11 个边界或父级实例已经是当前标准版本，
  另外 11 个计算实例仍为 `1.0.6, isCustom=1`。涉及 `constant / multiply /
  subtract / add / fmax / fmin`，存在与示波器相似的属性面板或代码生成兼容风险，
  但当前不能据此直接判定其计算逻辑错误。
- `Decision` 及其全部后代共 66 个实例，目前均为 `1.0.6, isCustom=1`，包含输入输出、
  参数和状态读写、逻辑比较、数学运算及真值表。该部分必须分阶段迁移，不能整体替换。
- 不能只把数据库里的 `version/isCustom` 改成新值来“升级”组件。新版组件的端口 UUID、
  `extensionProps`、属性结构和动态端口生成方式可能已经变化，这样修改会产生表面版本正确、
  实际属性面板或代码生成仍错误的伪升级。
- 以下动态或可配置组件在修改前必须先读取当前组件库的 `base_config`、端口定义和动态端口规则；
  可以由用户在当前 GAASD 界面中创建并配置，也可以在完全掌握当前属性结构后由数据库脚本构造，
  但不能从不了解配置语义的旧实例盲目复制：

```text
oscilloscope
可变输入数量的 add 等运算组件
truth-table
input / output
read-local-param
read-local-state / write-local-state
constant
```

- `RawTargetSpeed` 应继续使用一个三输入 `add` 完成
  `leadV + distanceFeedback + speedFeedback`。如需迁移到当前版本，应把新版 `add` 的
  `inputNumber` 配置为 3 后再迁移连线，而不是拆成两个二输入加法组件。
- `truth-table` 替换时必须完整保留条件、动作、场景矩阵和端口对应关系；状态组件替换时必须
  保留 `operateKeys`、默认值、类型以及读写时序。因此二者安排在迁移后段处理。

**基础组件能力与画布设计原则**：

- 设计前先确认组件的可配置属性，不再只根据画布外观、默认端口数或旧工程用法推断能力。
  当前已确认：

```text
add / multiply: inputNumber、isReal
oscilloscope: inputNumber、simulateStep、simulateTotalTime
constant: name、dataType、dataValue
input / output: name、dataType、category、shape
read/write local param/state: operateKeys
truth-table: conditions、actions、scenarios
```

- 对同一层级的纯加法项，优先使用一个多输入 `add` 一次完成求和；只有需要中间结果被其他模块
  复用、需要单独观测，或计算顺序具有明确物理意义时，才拆成多个加法模块。
- 同理，可变输入运算组件应优先按公式的自然结构配置，避免为了沿用默认二输入端口而制造
  不必要的节点和连线，降低画布可读性。
- 每次首次使用一种基础组件前，应记录其：数学语义、可配置字段、输入输出端口生成规则、
  数据类型限制、默认值、属性面板行为和代码生成形式。组件包更新后重新抽查这些信息。
- 版本兼容判断应综合比较 `originId / version / isCustom / properties / extensionProps /
  端口 UUID 与代码生成结果`，不能把输入端口数量作为单独判据。

**手动画布经验沉淀与自动审计工具**：

- 已将本次 `newaccpro3` 成功分层经验整理为长期知识库，明确区分稳定设计原则和随 GAASD
  版本变化的实现细节：

```text
docs/GAASD_手动画布构建知识库与复用流程.md
docs/templates/GAASD_画布修改任务模板.md
```

- 新增通用只读工具 `tools/gaasd_canvas_tool.py`，支持：

```text
audit       检查数据库完整性、端点、子系统边界、单输入唯一驱动、组合环、布局、动态端口、
            参数/状态绑定、真值表和 cbdes/temp 一致性
snapshot    在 GAASD 退出后复制可恢复的完整工程快照，并写入环境和审计清单
catalog     从当前 gaasd.db 导出核心基础组件模板
fingerprint 保存前端构建号、组件库哈希、组件版本和工程数据库哈希
compare     将更新后的环境与既有指纹逐项比较
```

- 工具不会自动修改画布；具体重构仍使用针对单一工程、逐条声明迁移关系的专用脚本。这样既能
  复用检查能力，又不会因为 GAASD 更新导致通用修改脚本误接端口。
- 当前环境事实已经更新：系统版和隔离版前端构建号均为 `27.1.3`，当前隔离版组件库核心模板
  已为 `1.2.0`。`newaccpro3` 成功画布仍混用 `1.0.6/1.1.7` 实例；这属于需要记录的模板漂移，
  不能通过直接修改 `version/isCustom` 字段解决。
- 已保存当前基线：

```text
docs/gaasd_canvas_baselines/20260728_newaccpro3_environment.json
docs/gaasd_canvas_baselines/20260728_newaccpro3_audit.json
docs/gaasd_canvas_baselines/20260728_core_component_catalog.md
```

- `newaccpro3` 通用审计结果为 `PASS`：131 个组件、147 条连线、0 errors、0 warnings；
  `cbdes.db/temp.db` 哈希仍为
  `ba85577b1a446cf0e0faa3cfdbcc5806656a2400e01322a97dd6a19b991ba96a`。
- `lks2` 通用审计也为 `PASS`：0 errors，保留 1 条旧实例端口元数据警告，后续进行 LKS
  重构时再结合当前组件模板处理，不在本次任务中改动。
- 当前 GAASD 仍在运行，因此没有强制制作重构后的完整工程快照；工具已验证会拒绝运行中的快照，
  且不会生成半成品目录。GAASD 退出后应执行：

```bash
python3 tools/gaasd_canvas_tool.py snapshot \
  project/newaccpro3 \
  --label decision_refactor_success
```

- 后续画布构建固定采用以下顺序：冻结公式/接口/参数/状态 -> 环境指纹 -> 完整快照 -> 当前 GUI
  创建高风险动态组件 -> 按物理职责增量构建 -> 通用结构审计 -> GUI 属性检查 -> 生成代码/编译 ->
  固定输入与 CARLA 闭环 -> 保存成功基线。

**允许直接修改 GAASD 画布数据库的范围与方法（当前版本暂行）**：

1. 修改前完全退出 GAASD，确认没有进程继续写工程数据库，并整目录备份目标工程。
2. 以 `project/<工程>/data/cbdes.db` 为持久化画布事实源；完成修改后同步覆盖
   `data/temp.db`。`runtimeCanvas.db` 属于运行期数据，不能在未核对用途时盲目覆盖。
3. 仅移动、改显示名或迁移已有连线时，保留组件 `id` 和端口 UUID；连接关系只修改
   `project_connection` 的 `source/target/parentId`。
4. 新建或替换组件时，不能凭外观复制旧实例。应先从当前 GAASD 组件库确认组件能力和属性结构；
   对尚未掌握动态端口规则的组件，优先让用户在画布中拖入并配置好，再由脚本处理布局和接线。
5. 删除节点前先查询所有入边、出边和子节点；只删除确认迁移完成的目标，禁止按名称模糊删除。
6. 每次修改后至少执行：SQLite `integrity_check`、连接端点存在性检查、同一输入端口重复连接检查、
   子系统边界检查、组合环检查、节点重叠检查，以及 `cbdes.db/temp.db` 哈希一致性检查。
7. 数据库检查通过只代表结构完整，不代表 GAASD 属性面板和代码生成一定可用；仍需重新打开工程，
   逐级进入画布、点击关键组件属性，并执行一次生成代码验证。
8. 若界面中新拖入的同类组件与旧实例在版本、`isCustom`、端口或属性结构上不同，默认以新实例为准，
   不再从旧实例克隆新模块。

> 上述组件版本、数据库事实源和可修改边界是针对当前 GAASD 2.7.0.5 环境形成的暂行规则。
> GAASD 前端、组件包、扫描器或代码生成器更新后，应先做小样本对比，再更新本节标准，不能把本节
> 视为永久不变的格式规范。

**下一步建议顺序**：

1. 先逐类核对 `Control` 内 11 个旧版计算实例的当前组件属性；迁移时保持现有公式和端口语义，
   `RawTargetSpeed` 仍采用单个三输入 `add`，不拆分成多个加法节点。
2. 完成 `Control` 固定输入计算复核和一次代码生成检查后，再处理 `Decision`。
3. `Decision` 按“边界输入输出与参数状态 -> 普通逻辑/数学块 -> 真值表”的顺序逐批迁移，
   每批都重新打开属性面板并生成代码，避免一次迁移后无法定位问题。

---

## 当前状态（2026-07-22 更新）

**ACC/LKS 回归手动画布正向搭建和闭环测试**：

- 重要汇报阶段结束后，正式构建主线调整为：ACC 和 LKS 算法内部以
  GAASD 基础组件手动搭建的画布为主，暂不再以 C++ 源码扫描自动生成画布为正式构建入口。
- 手动画布基线确定为：

```text
ACC: project/newaccpro3
LKS: project/lks2
```

- 源码扫描工程 `project/demo1`、`lks_demo` 和
  `generated/modules_refined_20260708` 仅保留为层级、公式、命名和边界端口参考，
  不再作为手动画布内部算法的导入来源。
- 边界接口继续复用已确认的最新契约：
  ACC 输入 `egoV/leadV/distance/commandType`，输出 `targetSpeed/enable`；
  LKS 输入 `egoV/c0/c1/c2/c3/curvature/brakePressed/driverSteerNorm`，
  输出 `steerRad/controlEnabled`。
- CARLA/Bridge 协议字段、Pangu 边界端口和手动画布内部端口统一使用上述简短名称，
  不再设置二次别名；单位保存在接口元数据中。
- 手动搭建不等于绕过代码生成。后续仍需验证：

```text
手动画布 -> GAASD 生成代码 -> Pangu 编译运行 -> CARLA 闭环
```

- 已新增当前状态与四周工作计划：

```text
/home/aiden/文档/Modularization/docs/GAASD_ACC_LKS_手动画布恢复与四周工作计划.md
```

- 现有手动画布的核心功能链基本具备，但层级结构不够清晰，模块、端口、参数和状态命名仍需按物理含义统一整理。
- ACC 和 LKS 是独立工程，重构后的画布模块名不使用 `Acc/Lks` 前缀；顶层统一使用
  `VehicleInputChannel / PerceptionInputChannel / MainFlow / Decision / Control`，内部模块按具体物理功能命名。
- 四周计划按周组织，不列具体日期：第 1 周恢复基线并完成架构和命名设计，第 2 周重构 ACC 手动画布，
  第 3 周重构 LKS 手动画布，第 4 周集中处理生成代码、Pangu/CARLA 联调、数据复测和工程快照。
- 当前尚未复制或修改正式画布工程；待计划文档审核后再执行工程副本创建和画布接线。

---

## 当前状态（2026-07-10 更新）

**lks_demo 工程打包（2026-07-10）**：

- 已将当前 `lks_demo` GAASD 工程按原目录结构直接打包，未额外添加 README 或说明文件：

```text
/home/aiden/文档/Modularization/deliverables/lks_demo_20260710.zip
```

- 包内保留 `lks_demo/project.json`、`lks_demo/data/cbdes.db`、`lks_demo/data/temp.db`
  和 `lks_demo/icvos/` 等工程结构，解压后应可作为 `lks_demo` 工程目录在其他已安装
  GAASD 的电脑上打开。

**demo1 扫描画布结构说明（2026-07-10）**：

- 已解析 `project/demo1/data/cbdes.db` 中的 GAASD 画布组件树和连线结构，并结合扫描源码缓存
  `acc_input_acc_target_speed_c395800a` 核对 ACC 示例工程的真实执行逻辑。
- 已新增说明文档：

```text
/home/aiden/文档/Modularization/docs/demo1_扫描画布结构说明.md
```

- 文档按 `lks_demo_扫描画布结构说明.md` 的形式说明 `demo1`：
  `ACCModule` 输入/输出通道、`AccTargetSpeed` 算法入口、`AccDecisionStage`
  决策状态管理、`AccCruiseSettingStage` 巡航设定值管理、`AccTargetSpeedStage`
  目标速度门控计算，以及 `DetermineAccSystemState`、`EvaluateAccDecision`、
  `DetermineAccEnable`、`SelectAccLastDecision`、`InitializeAccMemory`、
  `DetermineNextTimeGap`、`DetermineNextMaxSpeed`、`CalcAccTargetSpeed` 等子模块功能。
- 已根据 `demo1` 工程真实决策结构更新 `/home/aiden/文档/acc_lks.pptx` 第 4 页：
  将 ACC 决策页改为 `AccDecisionStage`，展示 `DetermineAccSystemState`、
  `EvaluateAccDecision`、`DetermineAccEnable`、`SelectAccLastDecision`
  四个内部子模块，以及 `controlEnabled/hasHistory/lastDecision` 状态写回关系。
- 已同步重写 `acc_lks.pptx` 全部 9 页备注，使备注内容与当前页面结构、公式和验证状态一致。
- 已重排 `/home/aiden/文档/acc_lks.pptx` 第 7 页 LKS 决策页：
  将原简单流程图改为“输入与阈值 / 三类安全判断 / 使能逻辑 / 输出门控”四区块结构，
  突出 `controlEnabled = speedReady && !brakePressed && !driverOverride` 的门控含义。
- 已补充 `demo1_扫描画布结构说明.md` 中 ACC 目标速度核心计算细节：
  解释 `safeLeadDistance`、`desiredDistance`、`distanceError`、`relativeSpeed`、
  `rawTargetSpeed`、`targetSpeed` 的物理含义，以及为什么源码中的 `max/clamp/条件选择`
  在 GAASD 画布中会展开成较多基础节点。
- 已补充 `lks_demo_扫描画布结构说明.md` 中 LKS 四个核心子模块的详细讲解：
  控制使能判断、远预瞄距离计算、三点预瞄误差加权、方向盘转角命令计算，并说明每帧重新计算的原因。
- 已导出两个 PDF 版本，方便汇报或离线查看：

```text
/home/aiden/文档/Modularization/docs/demo1_扫描画布结构说明.pdf
/home/aiden/文档/Modularization/docs/lks_demo_扫描画布结构说明.pdf
```

**系统旧版 GAASD 更新（2026-07-10）**：

- 已使用安装包更新系统旧版 GAASD：

```text
/home/aiden/文档/temp/GAASD_SETUP_20260707_1.tar.gz
```

- 更新目标为系统全局旧版路径：

```text
/opt/gaasd
/usr/bin/gaasd -> /opt/gaasd/gaasd
```

- 安装前曾临时备份旧 `/opt/gaasd`，用户确认旧版无需保留后已删除：

```text
/home/aiden/gaasd_versions/old_opt_gaasd_backup_20260710_101603
```

- 执行方式：先解包到 `/tmp/gaasd_setup_20260707_1_inspect` 检查 `install.sh`，确认脚本只会更新
  `/opt/gaasd`、`/usr/bin/gaasd`、系统桌面入口和 `/home/aiden/.setup_info`；随后执行：

```bash
sudo -n bash /tmp/gaasd_setup_20260707_1_inspect/install.sh gaasd
```

- 安装后校验结果：
  - `/usr/bin/gaasd` 已指向 `/opt/gaasd/gaasd`。
  - `/opt/gaasd/chrome-sandbox` 权限为 `root:root` + `4755`。
  - `ldd /opt/gaasd/gaasd` 未发现缺失动态库。
  - `/home/aiden/.setup_info` 记录为 `GAASD2.7.0.5:/opt/gaasd`。
  - `/opt/gaasd/gaasd` 与隔离新版 `/home/aiden/gaasd_versions/gaasd-2.7.0.5/app/gaasd`
    SHA-256 一致：`723e82279257afe8943fb188c00e4222ed6050dd5f5adfddd572ab16206ac45c`。
- 隔离新版目录未被安装脚本覆盖：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5
```

- 注意：该安装包内部 `/opt/gaasd/version` 文件仍显示 `27.1.3`，但安装脚本和安装记录使用
  `2.7.0.5`；这是包内版本文件和发布版本号不一致的问题，不影响本次旧版路径更新结论。

---

## 当前状态（2026-07-09 更新）

**ACC/LKS 汇报 PPT 更新（2026-07-09）**：

- 已按 `/data/aiden/文档/模板.pptx` 的当前单页模板样式，重新整理并覆盖生成：

```text
/data/aiden/文档/acc_lks.pptx
```

- 2026-07-09 追加修改：将正文和备注中的公式从代码式表达改为更学术的数学表达：
  ACC 使用 `d_des(t)`、`e_d(t)`、`Δv(t)`、`sat_[0,v_max](·)`、`χ_ACC(t)`；
  LKS 使用指示函数 `I_v/I_b/I_δ`、远预瞄距离 `L(t)`、三次车道多项式误差
  `e_i(t)`、加权误差 `e_w(t)`、横向加速度限幅 `δ_lim(t)` 和最终转角
  `δ_LKS(t)`。
- 2026-07-09 继续增强备注页：第 4、5、7、8、9 页备注已补充每个符号的物理含义、
  单位、公式作用和上下游关系，便于汇报时直接解释 ACC 决策、ACC 控制、LKS 使能、
  LKS 三点预瞄和 LKS 转向限幅。
- 2026-07-09 完成公式符号规范化落地：正文和备注中已将 `k_d` 替换为车距误差增益
  `k_s`，将 `v_ACC*` 替换为期望速度 `v_des`，将 `χ_ACC/χ_LKS` 替换为
  指示函数 `I_ACC/I_LKS`，将 LKS 预瞄距离写为 `L_p`，预瞄时间系数写为
  `T_p`，轴距写为 `l_wb`。已抽取校验确认旧符号 `k_d/v_ACC/χ_/L_wb/r_t`
  在正文和备注中均无残留。
- 2026-07-09 已重画 ACC 决策页层级：`AccControlStateUpdate` 明确作为父模块，
  内部包含 `AccSystemStateClassifier`、`AccDecisionTable`、
  `AccControlMemoryUpdate`、`AccHistoryUpdate` 四个子功能；
  `AccTimeGapUpdate` 和 `AccMaxSpeedUpdate` 作为父模块外部的同级参数更新模块，
  均由 `decision` 驱动。
- 2026-07-09 进一步重排 ACC 决策页版式：左侧集中展示 `AccControlStateUpdate`
  父模块及其四个内部子模块，右侧只保留同级的 `AccTimeGapUpdate` 和
  `AccMaxSpeedUpdate`，删除原先拥挤的多层堆叠排布，避免父子层级和同级模块混在一起。
- 2026-07-09 简化 ACC 控制页：保留一个展开后的目标速度公式
  `v_cmd = sat_[0,v_max](v_l + k_s[d_l-d0-T_h v_e] + k_v[v_l-v_e])`，
  将原“输出含义”改为“公式含义”，说明自车速度、前车速度、车距、期望距离、
  车距误差增益、相对速度增益和限幅函数的作用。
- PPT 内容已切换到最新扫描源码架构：
  ACC 顶层使用 `AccMainFlow`，LKS 顶层使用 `LksMainFlow`；
  结构按“车上输入通道 / 感知输入通道 / 决策层 / 控制层”讲解。
- 已补充 ACC/LKS 关键公式和变量含义到每页备注中，便于汇报时直接讲解。
- 已根据模板修正左上角页码样式：去除额外填充色，仅保留白色页码文字，压在模板原有色块上。
- 已清理本次生成过程中产生的临时 PPT 备份文件，仅保留正式 `acc_lks.pptx` 和模板文件。

**ACC.zip 扫描源码本机后端适配验证（2026-07-09）**：

- 测试对象：

```text
/data/aiden/文档/temp/ACC.zip
```

- 该包为 GAASD/Pangu 扫描用 C++ ACC 模块源码，包含 `AccMainFlow`、`AccDecision`、
  `AccSpeedControl` 及其子模块。
- 初始测试发现源码依赖 `FuncModule(Param)` 单参数构造，但本机 GAASD 后端公共头只支持
  默认构造、`FuncModule(param, state)` 和 `FuncModule(sub, param, state)`，导致编译/扫描报
  `no matching conversion`。
- 已对本机后端公共头增加兼容构造函数：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/codeTools/public/FuncModule.hpp
```

- 修改前已备份：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/codeTools/public/FuncModule.hpp.bak_20260709_compat_param_ctor
```

- 兼容补丁只新增 `FuncModule(Param)` 构造，不改变原有构造方式。
- 使用修复后的公共头重新验证：
  18 个 ACC `.cpp` 源文件全部编译通过；
  临时 `AccMainFlow` 测试驱动验证了启控、时距调节、限速调节和取消退出逻辑。
- 使用 `codescan-cpp --mode generate` 重新扫描通过，输出：
  `cbdes.db`、20 个组件 JSON、26 个字典 JSON。
- 当前判断：`ACC.zip` 的算法逻辑和扫描结构可用；本机能否扫描/运行取决于后端公共
  `FuncModule.hpp` 是否包含 `FuncModule(Param)` 兼容构造。

**GAASD 20260707_1 GUI 更新（2026-07-08）**：

- 已导入新的 Pangu Docker 镜像包：

```bash
gunzip -c /home/aiden/文档/temp/pangu_x86_laster_07_06.tar.gz | docker load
```

- 导入结果为：

```text
docker.cbdes.cn:8080/cbdes/x86:latest
```

- 已将 `GAASD_SETUP_20260707_1.tar.gz` 中的 GUI 程序更新到现有旁路安装目录：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5/app
```

- 更新方式为只替换 GUI `app` 目录，保留现有隔离 HOME 和后端：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server
```

- 更新前 GUI 已备份到：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5/rollback/gui_before_20260707_1_20260708_151939/app
```

- 桌面入口仍为：

```text
/home/aiden/.local/share/applications/GAASD-2.7.0.5.desktop
```

- 非侵入检查结果：
  `ldd /home/aiden/gaasd_versions/gaasd-2.7.0.5/app/gaasd` 未发现缺失动态库；
  新版 GAASD 内部 Pangu x86 镜像默认值已经切换为
  `docker.cbdes.cn:8080/cbdes/x86:latest`。

**周报生成流程化工具（2026-07-07）**：

- 已根据 `docs/` 下历史周报格式，建立可复用的 `.docx` 周报生成流程，后续每周只需维护一份 Markdown 输入文件。
- 新增周报生成脚本和依赖说明：

```text
tools/report_generator/generate_weekly_report.py
tools/report_generator/README.md
tools/report_generator/requirements.txt
```

- 新增周报输入模板和版式参考模板：

```text
docs/weekly_inputs/template.md
docs/templates/weekly_report_template.docx
```

- 输入模板支持按“核心任务进展 / 风险与问题 / 下周工作计划 / 需要协调与帮助”填写内容；
  生成脚本会输出与现有周报一致的标题、报告周期、三列表格和分节结构。
- 已安装并验证依赖 `python-docx`，测试命令如下：

```bash
python3 tools/report_generator/generate_weekly_report.py \
  docs/weekly_inputs/template.md \
  -o /tmp/weekly_report_test.docx
```

- 验证结果：`/tmp/weekly_report_test.docx` 成功生成，抽查包含 21 个段落和 6 张表格；
  核心进展、风险、下周计划和协调事项均能正常落入 Word 表格。
- 后续标准用法：

```bash
cp docs/weekly_inputs/template.md docs/weekly_inputs/2026-07-week3.md
python3 tools/report_generator/generate_weekly_report.py \
  docs/weekly_inputs/2026-07-week3.md \
  -o docs/赵煜坤-7月第3周项目进展周报.docx
```

**ACC 前车 waypoint PID 控制器（2026-07-07）**：

- 重新评估 ACC 前车控制方式：`constant_velocity` 只施加固定世界坐标速度向量，不会主动沿道路转弯；
  Traffic Manager 适合交通流，但官方文档说明其路径会动态生成、路口行为可能随机，目标速度也默认与限速相关，
  不适合作为可重复 ACC 标定前车。
- 已新增自定义前车控制器：

```text
tools/carla_bridge/lead-waypoint-pid-controller.py
```

- 控制逻辑：找到 `role_name=gaasd_lead` 的前车后，关闭 `constant_velocity` 和 autopilot；
  每帧读取当前车道 waypoint，取前方预瞄点计算横向误差，使用横向 PID 输出 `steer`；
  同时用速度 PID 将前车速度稳定到 `--target-speed-mps`，输出 `throttle/brake`。
- `tools/carla_bridge/start-gaasd-carla-manual.sh` 已支持第三种前车行为：

```bash
--lead-behavior waypoint_pid
```

  启动流程为：先在 ego 前方 waypoint 生成前车，再启动后台 waypoint PID 控制进程。
- `tools/carla_bridge/stop-gaasd-carla-manual.sh` 已增加前车 PID 控制器清理逻辑，避免残留进程影响下一次测试。
- `scenarios/newaccpro3_pangu_carla_20260701/run.sh` 和 `scenario.yaml` 已将默认前车行为从
  `constant_velocity` 切换为 `waypoint_pid`；该场景现在应显示 ego 跟随约 `2m/s` 的 waypoint-PID 前车。
- 静态验证已通过：

```bash
python3 -m py_compile tools/carla_bridge/lead-waypoint-pid-controller.py
bash -n tools/carla_bridge/start-gaasd-carla-manual.sh
bash -n tools/carla_bridge/stop-gaasd-carla-manual.sh
bash -n scenarios/newaccpro3_pangu_carla_20260701/run.sh
```

- 2026-07-07 对 `newaccpro3` 实测日志完成前车性能抽查：
  `/tmp/newaccpro3-pangu-carla/carla/lead-waypoint-pid.log` 中 96 个样本速度均值
  `2.002m/s`，速度 RMS 误差约 `0.101m/s`；过弯阶段最大转向约 `0.384`，
  道路编号从 `road=1/41/0/11` 过渡到 `road=8`，说明前车已能沿 waypoint 通过弯道，
  不再是直线 `constant_velocity`。
- 发现 ego 侧仍使用 Bridge 旧的 lane keep 横向控制，过弯能力弱于前车。已在
  `tools/carla_bridge/carla_bridge.py` 新增 `lane_keep_mode=waypoint_pid`：
  ego 横向转向复用“前方 waypoint 预瞄角 + PID”的思路，只覆盖 `VehicleControl.steer`，
  不改 `target_speed_mps -> throttle/brake` 的 ACC 纵向控制链路。
- `scenarios/newaccpro3_pangu_carla_20260701/bridge_config.json` 已启用 ego 侧
  `lane_keep_mode: waypoint_pid`，并将 `lane_keep_max_steer` 调整为 `0.45`，匹配前车
  成功过弯时所需转向量级。

**LKS CARLA 可视化稳定性和接管输入处理（2026-07-06）**：

- 单独 Pygame 摄像头窗口只作为调试辅助，不作为 GAASD/Pangu 场景的核心集成链路：
  场景后续由 Pangu Docker 承载，Pygame 若放入容器需要额外 X11、窗口焦点和依赖配置，
  容易增加集成复杂度；更合适的定位是宿主机可选观察器，核心接口仍走
  `gaasd.carla.*` ZMQ 消息。
- CARLA 0.9.15 + RTX 5080 在本机多次出现可视窗口运行一段时间后 `Signal 11` 崩溃。
  Crash 栈定位在 UE4 `VulkanRHI` / `FRHIThread`，不是 Bridge、Pangu 或 LKS 算法问题。
- 已将 LKS 默认 CARLA 启动参数改为：

```text
-windowed -Resx=800 -Resy=600 -fps=20 -nosound -graphicsadapter=0 -norhithread
```

  其中 `-norhithread` 是关键修复项。关闭独立 RHI 线程后，Town04 LKS 可视闭环已稳定
  运行超过 4 分钟，未复现 VulkanRHI 崩溃。
- 原官方 `Town05 road=37 lane=-2` 场景算法数据可通过，但本机可视渲染会在约 2 分钟后
  崩溃；不再作为默认演示路线。
- 已通过 CARLA map API 筛选官方 `Town04` 无路口连续弯道路段，默认 LKS 场景改为中间车道
  候选点：

```text
map: Town04
ego/reference: (-511.738, 242.657, 0.5)
road/lane: road=45, lane=-3
target_speed: 4.0 m/s
LKS Kp: 0.06
```

- `lane=-4` 旧候选点的稳定性验证结果：
  `sample_count=2393`、`sim_duration=119.6s`、`max_abs_lateral_offset=0.219m`、
  `rms_lateral_offset=0.158m`、`curve_sample_count=2210`、无路口、无碰撞、`passed=true`。
- 调试平台 LKS A/D/B 接管输入不再通过每次按键启动短生命周期 Python 发布进程，已改为
  Flask 后端常驻 ZMQ PUB socket 直接向 `gaasd.carla.driver_state_cmd.v1` 发布多帧消息。
  这样和 Bridge `SUB bind :5702` 的真实工作方式一致，避免 PUB/SUB slow-joiner 导致按键
  偶发不生效。
- 2026-07-07 对最新 `Town04 road=45 lane=-3` 场景完成复测：
  `sample_count=3590`、`sim_duration=179.45s`、`max_abs_lateral_offset=0.226m`、
  `rms_lateral_offset=0.143m`、弯道覆盖完成、无碰撞、`passed=true`。
- 2026-07-07 对最新 LKS A/D/B 接管链路完成复测：
  A 得到底盘 `steer_norm=-0.30`，D 得到底盘 `steer_norm=+0.30`，B 得到 `brake=1.0`，
  释放后输入归零，`takeover_latest.json` 判定 `passed=true`。

**LKS 交接场景切换为官方 Town05（2026-07-05）**：

- 发现原交接包默认地图 `acc_30km_new` 不是 CARLA 官方地图，开发团队环境无法保证存在。
- 检查 `~/PycharmProjects/CarlaAcc` 全部 Git 历史后确认：历史中没有 `Town06`；切换到
  自定义地图前的可运行官方地图为 `Town05`。提交 `5667b6d`、`dd934d8` 使用固定道路点
  `(0.663731, -203.651886, 0.5)`。
- CARLA 0.9.15 实测该点投影到 `road=37、lane=-2、yaw=179.758deg`，不在路口；沿当前
  车道向前至少 600 m 无分叉并包含连续缓弯，适合作为 LKS 单车基线。
- LKS 场景、Bridge 配置和 reset 脚本已改为官方 `Town05`；航向由 CARLA waypoint 自动
  获取，不依赖硬编码角度。
- Town05 实际闭环结果：最高车速 `5.1489m/s`，681 个弯道样本，最大横向偏差
  `0.30394m`，RMS 偏差 `0.17159m`，无路口、无碰撞，`passed=true`。

**newaccpro3 ACC 持久化 Pangu 构建修复（2026-07-03）**：

- 调试平台启动 ACC 场景时报错：

```text
[Scenario] missing: /tmp/newaccpro3_pangu_codegen_build/install/setup.bash
```

- 根因不是 ACC 代码或 CARLA 链路损坏，而是场景长期依赖 `/tmp` 中的 Pangu 构建产物；
  系统重启或临时目录清理后，`/tmp/newaccpro3_pangu_codegen_build` 整体消失。
- 已新增可重复执行的持久化构建脚本：

```text
tools/pangu_acc_closed_loop/build_pangu_module.sh
```

- ACC 构建及安装目录迁移为：

```text
/home/aiden/.cache/gaasd-pangu/newaccpro3_codegen_build
```

- `scenarios/newaccpro3_pangu_carla_20260701/run.sh`、`stop.sh` 和 `scenario.yaml`
  已统一使用持久化目录；若 `setup.bash` 或 `libZmqBridgeModule.so` 缺失，场景会先自动构建。
- 构建脚本会从当前 `project/newaccpro3/icvos/src` 复制模块配置并编译修补后的生成代码，
  同时补齐 `install/image/lib`、`app_empty.pt`、节点配置和有效的
  `icvos_machine.pt`。`install/conf` 明确链接到 `install/image/conf`，避免读取项目中的空机器配置。
- 持久化目录 mock 复测通过：`E/T/R/Q/C/E/S/0` 全部到达，启控、取消、再次启控和制动退出
  均符合当前画布逻辑，脚本最终输出 `[mock-seq] PASS`。

**lks2 驾驶接管、运行稳定性与开发团队交接（2026-07-02）**：

- 电脑在 LKS Pangu 重编译期间发生重启，已将构建/安装目录从易失的 `/tmp` 迁移为：

```text
/home/aiden/.cache/gaasd-pangu/lks2_codegen_build
```

- `build_pangu_module.sh` 已限制默认并行任务为 4，并处理容器构建产物属主、运行配置目录和
  场景 `.pt` 文件安装；重启后不需要重新构建即可启动场景。
- 已定位 Pangu 容器运行一段时间后退出的原因：场景缺少 `--privileged`，Pangu 配置路由时
  出现 `SIOCADDRT: Operation not permitted`。LKS 场景现使用 `--net=host --privileged`，
  CARLA、Bridge PUB、Bridge CONTROL、Pangu 业务进程已持续稳定在线。
- 已修复 LKS 驾驶接管输入“页面有日志但车辆无反应”的两层问题：
  - 测试发布器改为连接后等待并重复发送，避免 ZMQ PUB/SUB slow join 丢失单次消息。
  - Pangu 边界层在 LKS 退出时将 `driverSteerNorm` 映射为实际手动转向；制动输入会产生
    有效底盘制动，不再只是把 LKS 转向清零。
- LKS 调试按键最终约定：
  - A 按住：`driverSteerNorm=-0.3`，向左接管。
  - D 按住：`driverSteerNorm=+0.3`，向右接管。
  - B 按住：`brakePressed=1`，制动并退出 LKS。
  - 松键：两项归零，满足车速条件后 LKS 自动恢复；当前 LKS 不设置 E/C 总开关。
- 已新增自动验收脚本和独立 Pygame 键盘发布器：

```text
tools/pangu_lks_closed_loop/verify_driver_takeover.py
tools/pangu_lks_closed_loop/lks_keyboard_input.py
```

- 真实 CARLA 自动验收通过：A/D 分别得到底盘 `steer_norm=-0.30/+0.30`，B 得到
  `brake=1.0` 并将车速降至 0，释放后输入归零且车辆重新起步。测试面板 HTTP 输入路径
  也单独验证通过。
- 已生成开发团队集成说明和交接包：

```text
docs/GAASD_CARLA_ACC_LKS_开发团队集成说明.md
deliverables/gaasd_carla_acc_lks_20260702.zip
deliverables/gaasd_carla_acc_lks_20260702.zip.sha256
```

- 交接包包含：接口变量清单、Python Bridge、ACC/LKS 驾驶输入脚本、Pangu 模块源码与
  配置、ACC/LKS 场景脚本、修复后的 GAASD 生成代码和 LKS 接管测试证据；压缩完整性、
  SHA256 和 LKS 离线算法测试均已通过。

**调试面板物理键盘修复（2026-07-03）**：

- 已确认原状态：LKS 页面注册过 A/D/B，但只在浏览器聚焦时生效，且快速松键可能因请求忙
  丢失归零；ACC 页面只有按钮点击，没有注册物理键盘事件。
- LKS 已改为按键集合和最新状态队列：A/D/B 支持按住、组合键、松开归零，归零请求不会再
  被前一条请求阻塞丢弃；失焦和页面隐藏时强制释放。
- ACC 已增加 E/Q/T/R/C 脉冲键，W/S 持续键和松键自动发送 `commandType=0`；页面新增 W
  油门接管按钮，W/S 鼠标按住和松开与物理键盘语义一致。
- 无界面 Chrome 实测通过：LKS A 产生 `-0.30 -> 0.00`；ACC E、S、S 松开依次产生
  `commandType=1/6/0`。
- 页面键盘仍受浏览器焦点限制，这是浏览器输入模型，不是消息链路问题。CARLA 窗口前台
  测试或 Docker 集成时使用独立的 ACC/LKS Pygame 键盘发布器。

**LKS 场景参数调试面板（2026-07-05）**：

- 已给 `lks2_pangu_carla_20260701` 增加可编辑参数 schema，覆盖场景目标车速、初始横向偏差、
  记录时长、预瞄距离、三点误差权重、转向增益、横向加速度限幅、最低接管车速和驾驶员接管阈值。
- 调试面板后端新增参数读取/保存接口，启动 LKS 场景前会把保存值转换为 16 个 `LKS_*`
  环境变量并传入 `run.sh`；`run.sh` 和 Pangu LKS 模块会读取这些值覆盖默认标定参数。
- 参数保存接口已验证通过，当前持久化默认值包括 `LKS_TARGET_SPEED_MPS=6.0`、
  `LKS_PARAM_KP=0.08`、`LKS_PARAM_W1/W2/W3=0.2/0.3/0.5` 等。
- 修复测试平台页面初始化鲁棒性：固定按钮事件改为安全绑定，缺少某些按钮时不会再出现
  `Cannot set properties of null` 导致场景列表加载中断。
- 为 `style.css` 和 `app.js` 增加静态资源版本参数 `20260705-lks-param`，避免浏览器缓存旧前端脚本。
- 已重启 8765 测试平台，当前宿主机进程 PID `3663120`；页面已确认包含 `lksParameterCard`，
  `/api/scenarios/lks2_pangu_carla_20260701/parameters` 返回 16 个参数。
- 2026-07-06 继续修复测试平台显示不全问题：取消 `body/.dashboard` 全局隐藏溢出，恢复页面纵向滚动；
  主工作区宽度从 `1000px` 扩展到 `1280px`，宽屏下 LKS 参数区改为三列排布；参数区和日志区保留内部滚轮。
  静态资源版本更新为 `20260706-layout-scroll`，当前 8765 面板进程 PID `3716881`。
- 2026-07-06 进一步做产品化视觉统一：侧栏、场景卡、控制卡、健康状态、参数卡和日志终端统一为浅色桌面控制台风格；
  主工作区扩展到 `1380px`，宽屏下控制区采用更稳定的三列布局。LKS 参数区取消内部滚轮，改为自然撑开并由页面整体滚动承载，
  避免参数滚轮和页面滚轮误操作。静态资源版本更新为 `20260706-product-polish`，当前 8765 面板进程 PID `3777491`。

**newaccpro3 Pangu-CARLA 可视化闭环场景**：

- 已新增调试平台场景：

```text
scenarios/newaccpro3_pangu_carla_20260701/
```

- 该场景会一键启动：
  - 本机 CARLA 0.9.15。
  - Python Bridge。
  - 直道 ACC 跟车场景，ego 位于 `spawn_points[198]`，前车约 `25m`，速度 `2m/s`。
  - Pangu Docker 容器 `newaccpro3_pangu_carla`。
  - `run.sh app_empty ZmqBridgeModule -nohup` direct mode 业务进程。
- `run_all.sh app_empty` / topo 链路当前问题：
  - 日志出现 `Failed to connect to aicc_flow_topo`。
  - `run_all.sh` 仍打印 `run_all success`，但不会可靠拉起 `ZmqBridgeModule`。
  - 当前判断是 topo 服务/拓扑拉起链路未 ready 或连接失败，不是 ACC 算法库和 ZMQ 链路本身失败。
- direct mode 已验证能拉起：

```bash
bash /tmp/newaccpro3_pangu_codegen_build/install/run.sh app_empty ZmqBridgeModule -nohup
```

- 真实 CARLA 现象：
  - CARLA/Bridge/Pangu 容器和业务进程均可启动。
  - Bridge 能收到 Pangu 控制命令。
  - 手动重新执行一次 `boost + E` 后，ego 能启动并跟随前车，肉眼观察跟车效果可接受。
- 已定位启动时偶发不启控的主因：
  - `E` 是单周期驾驶指令脉冲。
  - 如果脚本在 Pangu ZMQ 订阅端 ready 之前发送 `E`，该脉冲会被丢失。
  - 后续手动再发一次 `E` 时订阅端已经 ready，所以 ego 能启动。
- 已加固 `scenarios/newaccpro3_pangu_carla_20260701/run.sh`：
  - 等待 Pangu `dataflow_runner --process_name=ZmqBridgeModule` 存在。
  - 再等待 `PANGU_COMMAND_READY_DELAY_SEC`。
  - 然后执行一次 `boost-ego-speed.py` 和 `keyboard_command_publisher.py --once e`。
  - 避免连续刷 `E`，防止在已启控后重复触发“降低设定速度”。
- 调试平台入口：

```bash
cd /home/aiden/文档/Modularization
python3 tools/gaasd_scenario_panel/app.py
# 浏览器打开 http://127.0.0.1:8765/
```

- 调试平台已针对该场景补充手动测试按钮：
  - `辅助启控`：执行一次 `boost + E`，用于 Pangu 订阅端 ready 后手动启控。
  - `E/Q/T/R/C/S/0`：直接发送 ACC 驾驶指令到 Bridge `5702`，再由 Bridge 转发到 Pangu 输入总线。
  - 健康检查新增 `Pangu ACC`，检查 Docker 容器内 `dataflow_runner --process_name=ZmqBridgeModule` 是否存在。
- 2026-07-01 人工测试发现原参数存在碰撞风险，已将 ACC 场景默认参数调整为安全基线：
  - 初始时距 `3.0s`，最小时距 `1.5s`，最小静态距离 `8m`。
  - 初始最高车速 `3.0m/s`，调速步长 `0.5m/s`。
  - 距离增益 `0.15`，相对速度增益 `0.8`。
  - Bridge 最大油门 `0.35`、最大制动 `0.7`、最大加速度 `1.5m/s^2`。
  - 场景启动前默认清理旧 CARLA/Bridge/Pangu，避免旧实例占用 2000 端口。
- 调试平台运行日志已改为内部独立滚动，底部增加留白并新增“复制日志”按钮。

**lks2 Pangu-CARLA 旁路闭环（2026-07-01）**：

- 已扩展 Bridge `gaasd.carla.lane_tracking.v1`，新增 `c0_m/c1/c2_per_m/c3_per_m2/curvature_per_m`，旧的横向偏差和航向误差字段保持兼容。
- 已新增 `gaasd.carla.driver_state.v1`，并增加 `gaasd.carla.driver_state_cmd.v1` 测试输入；
  `brakePressed/driverSteerNorm` 代表驾驶员输入，不使用车辆实际转角冒充驾驶员输入。
- 已修复 `project/lks2/icvos/src/temp_codegen_output` 的确定性生成缺陷：
  - 恢复 LKS 全局参数。
  - 统一复合组件实现为 `run()`。
  - 补数学函数声明、顶层输入输出和三个预瞄点连线。
- 离线生成算法测试通过：`egoV=5m/s、c0=0.8m` 时输出 `preview=7.5m、weightedError=0.8m、steerRad=0.0384rad、enable=1`。
- 新增 Pangu LKS 源码、构建脚本和场景：

```text
tools/pangu_lks_closed_loop/
scenarios/lks2_pangu_carla_20260701/
docs/GAASD_CARLA_LKS_Pangu旁路闭环执行方案.md
```

- LKS ZMQ 适配源码已通过 `-Wall -Wextra -Wpedantic` 单文件编译。
- Pangu ABI 构建、动态库加载和真实 CARLA 闭环已经通过。
- 首轮 Town01 高速长测在约 65 秒进入路口急转，70.65 秒撞护栏。数据表明三点预瞄
  计算和控制输出符合画布公式，但该路线包含路口路径选择和接近 90 度道路转换，不属于
  LKS 连续车道保持工况；因此没有向控制器擅自增加曲率前馈。
- 已按原参考项目 `project/lks/lks_tcp_s17.py` 恢复正式测试条件：
  - 地图：`acc_30km_new`（本机 CARLA 包中已安装）。
  - 起点：参考 `case 0` 的 `(1665.092773, 6334.560059, 0.5)`。
  - 道路朝向参考点：`(1654.013672, 6322.584473, 0.5)`。
  - 初始横向偏置：`0 m`；目标速度：`6 m/s`。
- 参考地图 90 秒闭环通过：
  - 1796 个前台记录样本，自动记录器结果 1816 个样本。
  - 最高车速 `5.1515 m/s`，结束车速 `5.03 m/s`。
  - 有效弯道样本 `479` 个，最大曲率约 `0.001143 1/m`。
  - 全程最大横向偏差 `0.02959 m`，RMS 偏差 `0.01295 m`。
  - 无路口、无碰撞，测试结论 `passed=true`。
- 已新增 LKS 自动记录器：保存 `c0~c3`、曲率、道路编号、横向偏差、实际转角和
  CARLA 碰撞事件；最新结果位于
  `/tmp/lks2-pangu-carla/results/latest_summary.json`。
- 调试平台已增加 LKS 专用控制区：
  - `A/D` 按住发送驾驶员左右转向输入（`driverSteerNorm=±0.3`）。
  - `B` 按住发送制动退出信号。
  - 松键、窗口失焦或点击“释放”后输入归零；Bridge 另有 `0.75s` 超时保护。
  - “测试结果”按钮读取最新弯道指标。
- LKS 决策离线用例已补齐并通过：正常控制 `enable=1`；驾驶员转向接管、制动和
  `egoV < 1m/s` 三种情况均输出 `enable=0/steerRad=0`。

**newaccpro3 Pangu 生成代码版 ACC mock 闭环复测**：

- 已用 `sudo` 补齐 `/tmp/newaccpro3_pangu_codegen_build/install/image/` 下运行所需文件：
  - `lib/libmodule_empty.so`
  - `lib/libmodule_empty_core.so`
  - `lib/libmodule_empty_conf_pb.so`
  - `lib/libZmqBridgeModule.so`
  - `conf/app_module/app_empty.pt`
  - `conf/node_module/module_empty/*.pt`
  - `conf/global_conf/global_module.pt/global_com.pt/tf.pt/icvos_machine.pt`
- 结论一：`run_all.sh app_empty` 当前仍只显示 `run_all success`，但日志没有 `ZmqBridgeModule/module_empty` 被拉起的记录；该路径的问题更像是 `icvos_tools topo start app`/拓扑拉起链路问题，不是算法 `.so` 链路问题。
- 结论二：绕过 topo、直接执行 `run.sh app_empty ZmqBridgeModule -nohup` 可以拉起生成代码版业务节点，`ZmqBridgeModule` 能通过 ZMQ 订阅 mock 输入并发布 `gaasd.carla.control_cmd.v1`。
- 已修改 `tools/pangu_acc_closed_loop/verify_mock_command_sequence.sh`：
  - 支持 `PANGU_START_MODE=direct`，用于直接启动单个 Pangu 进程。
  - 支持 `MOCK_EGO_SPEED` 和 `MOCK_COAST_DECEL`，用于避开或验证 ACC 低速保护。
  - 退控断言改为检查 `enable=0`，不再强制要求 `target=0`；生成代码当前退控后仍会计算 target，但使能为 0。
- 复测命令：

```bash
docker run --rm -v /home:/home -v /data:/data -v /tmp:/tmp --net=host --shm-size=8193m \
  docker.cbdes.cn:8080/cbdes/x86:pangu-2.0.5 \
  /bin/bash -lc 'cd /data/aiden/文档/Modularization && \
  PANGU_BUILD_ROOT=/tmp/newaccpro3_pangu_codegen_build \
  PANGU_INSTALL_DIR=/tmp/newaccpro3_pangu_codegen_build/install \
  LOG_DIR=/tmp/newaccpro3_codegen_mock_runtime_direct \
  PANGU_START_MODE=direct \
  PANGU_PROCESS_NAME=ZmqBridgeModule \
  MOCK_EGO_SPEED=2 \
  MOCK_COAST_DECEL=0 \
  tools/pangu_acc_closed_loop/verify_mock_command_sequence.sh'
```

- 复测结果：
  - `E/T/R/Q/C/E/S/0` 全部到达 mock Bridge。
  - `E` 后生成代码版 ACC 输出 `enable=1`，目标速度从约 `3.75m/s` 逐步调整到约 `2.25m/s`。
  - `C` 后输出 `enable=0`。
  - 再次 `E` 后重新输出 `enable=1`。
  - `S` 后输出 `enable=0`。
  - 脚本最终输出 `[mock-seq] PASS`，退出码为 0。
- 注意：生成代码中 `vMin=1`，低速保护有效。若 mock 初速为 0，或在 `run.sh -nohup` 的 10 秒等待期间自然滑行到 0，则 `enable` 会保持 0，这是符合当前画布设计的行为，不是链路失败。

## 下一步任务（2026-07-01，按当前主流程）

### P0：完成 newaccpro3 真实 CARLA 人工验收

- [ ] 从调试平台启动 `newaccpro3_pangu_carla_20260701`，确认 CARLA、Bridge PUB、Bridge CONTROL、Pangu ACC 四项均在线。
- [ ] 验证自动启控；若未自动起步，使用“辅助启控”复测，并记录是否仍存在首个 `E` 脉冲丢失。
- [ ] 逐项验证 `E/Q/T/R/C/S/0`：启控、调速、调时距、取消和制动退出均应符合 `commandType` 约定。
- [ ] 至少运行 60 秒，确认自车不碰撞、不丢前车、Pangu 业务进程和 CARLA 不异常退出。

### P1：把“肉眼可用”升级为可量化验收

- [ ] 持久化记录 `egoV/leadV/distance/targetSpeed/enable/commandType`，不只依赖 CARLA 画面。
- [ ] 增加场景结果摘要：启控是否成功、最小车距、末端车距、速度稳态误差、退出响应和消息计数。
- [ ] 在调试平台提供最新结果或日志入口，使一次测试能够留下可复核证据。

### P2：固化可复现场景和开发团队交接包

- [ ] 将已验证的 Pangu 构建产物从临时 `/tmp/newaccpro3_pangu_codegen_build` 迁移到仓库可构建/可恢复的位置；场景不能长期依赖 `/tmp`。
- [ ] 固化 Docker 镜像、Pangu 配置、CARLA 地图/出生点、Bridge 配置、前车行为、相机参数和启动顺序。
- [ ] 测试稳定后整理 GAASD 团队交接包：CARLA 场景脚本、Python Bridge、键盘指令脚本、Pangu 模块源码/配置、构建安装脚本和最小说明。
- [ ] 交接前在干净目录或新容器中执行一次从构建到闭环的复现测试。

### P3：回到 GAASD 生成代码验证轨道

- [ ] 每次 GAASD/codegen 更新后重新生成 `newaccpro3`，先确认编译通过，再运行统一输入序列。
- [ ] 使用与手写 oracle 相同的 `egoV/leadV/distance/commandType` 做逐周期 diff，区分算法差异和生成器缺陷。
- [ ] 生成代码通过 diff 后，用生成代码替换当前临时修补/手写基准，再重复 mock 与真实 CARLA 60 秒测试。
- [ ] `run_all/topo` 问题单独向 GAASD/Pangu 团队反馈；在修复前继续使用已验证的 direct `run.sh`，不让它阻塞算法闭环。

### P4：完成 lks2 Pangu-CARLA 联调

- [x] 复用当前 Pangu、Bridge、场景和调试平台框架，扩展已确认的 LKS 输入输出接口。
- [x] 修复 `lks2` 生成代码并完成离线算法基准测试。
- [x] 执行 `tools/pangu_lks_closed_loop/build_pangu_module.sh`，完成 Pangu ABI 构建并迁移到持久化缓存。
- [x] 从调试平台启动 `lks2_pangu_carla_20260701`，完成参考地图无初始偏置连续弯道闭环。
- [x] 记录 `c0~c3/curvature/steerRad/controlEnabled`，验证转向符号、收敛表现和无碰撞。
- [x] 验证 A/D 驾驶转向接管、B 制动退出以及松键自动恢复的完整物理响应。
- [x] 整理 ACC/LKS Bridge、Pangu、场景、键盘脚本和接口说明开发团队交接包。
- [ ] 如需扩大工况覆盖，再单独验证初始横向偏差和初始航向误差；不阻塞当前最小闭环交接。
- [ ] 最后再考虑 ACC+LKS 组合测试，避免同时引入纵向和横向问题导致无法定位。

---

## 当前状态（2026-06-26 更新）

**已审核的 ACC/LKS CARLA 联调接口变量（2026-06-29）**：

- 最终确认文档：

```text
docs/GAASD_CARLA_ACC_LKS_接口变量清单.md
```

- 后续 ACC/LKS 与 CARLA/Pangu 通道、Bridge、mock 测试方案均以该文档为基准；不要再把已删除的扩展变量重新加入接口需求。

- ACC 必须保留的接口：
  - 输入：`egoV`，`double`，m/s，自车速度。
  - 输入：`leadV`，`double`，m/s，前车速度。
  - 输入：`distance`，`double`，m，自车到前车距离。
  - 输入：`commandType`，`int`，驾驶指令事件。
  - 输出：`targetSpeed` / `speed`，`double`，m/s，ACC 目标速度。
  - 输出：`enable`，`int`，ACC 控制使能。

- `commandType` 约定：
  - `0`：无指令。
  - `1`：降低设定速度；待命无历史时表示当前速度启控。
  - `2`：提高设定速度；待命有历史时表示继承参数启控。
  - `3`：减小时距。
  - `4`：增大时距。
  - `5`：驾驶员油门。
  - `6`：驾驶员制动并退出 ACC。
  - `7`：取消 ACC。
  - 当前画布按单周期事件脉冲设计：无指令周期为 `0`，有指令时 `1..7` 只保持一个仿真周期，下一周期回到 `0`。如果输入源无法保证单周期脉冲，再考虑边沿检测。

- LKS 必须保留的输入接口：
  - `egoV`，`double`，m/s，自车速度，用于预瞄距离、横向加速度限幅、低速判断。
  - `c0`，`double`，m，车道中心线多项式常数项，近似当前横向偏差。
  - `c1`，`double`，1，车道中心线一阶项。
  - `c2`，`double`，1/m，车道中心线二阶项。
  - `c3`，`double`，1/m^2，车道中心线三阶项。
  - `curvature`，`double`，1/m，道路曲率，用于弯道预瞄距离修正。
  - `brakePressed`，`int`，是否制动，用于退出 LKS。
  - `driverSteerNorm`，`double`，驾驶员方向盘输入归一化值，用于判断主动转向/换道。

- LKS 必须保留的输出接口：
  - `steerRad` / `lksSteerRad`，`double`，rad，LKS 横向转角命令。
  - `controlEnabled`，`int`，LKS 横向控制使能。

- LKS 特别注意：
  - `driverSteerNorm` 必须表示驾驶员输入，不能使用 LKS 输出后的车辆实际转角，否则 LKS 自己产生的转向会被误判为驾驶员主动转向。
  - 第一阶段测试可以默认 `laneValid=1`，但 `laneValid` 当前不作为必须接口变量列入最终精简清单。
  - LKS 本身只负责横向控制；单车 LKS 仿真需要车辆持续运动时，固定纵向速度或外部纵向控制命令属于仿真场景支持，不属于 LKS 算法输出。

**官方 accreference 示例项目复核**：

**Pangu ACC 最小闭环执行产物（2026-06-29）**：

- 已按最终方案从参考包复制并改造独立模块，位置：

```text
tools/pangu_acc_closed_loop/ACCClosedLoopModule
```

- 该目录来自 `/home/aiden/文档/temp/ACCModule.tar.gz`，未覆盖原始参考包。
- `proto/ACCModule_config.proto` 已改为：
  - `AccInput` 输入 `ego_speed_mps / lead_speed_mps / lead_distance_m / command_type`。
  - `AccOutput` 输出 `target_speed_mps / enable / valid`，并额外带 `time_gap_s / max_speed_mps / decision / system_state` 作为调试字段。
- `module_lib/AccTargetSpeed.*` 已从参考包的简单距离控制改为手写 ACC oracle：
  - `commandType=1` 可从待命无历史状态按当前速度启控。
  - `commandType=2` 支持待命有历史状态继承参数启控。
  - `commandType=3/4` 调整时距。
  - `commandType=6/7` 退出 ACC。
  - 输出 `targetSpeed` 和 `enable`，并保留内部 `timeGap/maxSpeed/decision/systemState` 方便后续与 GAASD 生成代码做 diff。
- `module_lib/acc_zmq_bridge.*` 已新增订阅：

```text
gaasd.carla.driver_command.v1
```

- 端口方向保持最终联调语义：
  - `5701`：Bridge 对外发布 `ego_state / lead_vehicle / driver_command`。
  - `5702`：Bridge 接收 `control_cmd / driver_command`。
  - 键盘输入仍先进入 Bridge，再由 Bridge 归一化转发给 Pangu，不让键盘程序直接私连 Pangu。
- 新增 mock 和键盘工具：

```text
tools/pangu_acc_closed_loop/mock_carla_bridge.py
tools/pangu_acc_closed_loop/keyboard_command_publisher.py
tools/pangu_acc_closed_loop/README.md
```

- 已完成的本地验证：
  - Python 脚本 AST 语法检查通过。
  - `AccTargetSpeed.cpp` 宿主机单文件编译通过。
  - ACC oracle smoke test 通过：`commandType=1` 能启控，后续无指令时 `enable` 保持，目标速度为正。
  - `acc_zmq_bridge.cpp` 使用 Pangu 第三方路径单独编译通过：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/panguFramework/pangu_gaasd/dependencies/thirdparty/X86/include
```

- Claude 审核后已修正 oracle 与当前 `newaccpro3` 真值表的差异：
  - 当前真值表包含 `u==0&&(v==6||v==7)->R8`，且 catch-all 默认输出 R8。
  - `ComputeDecision()` 默认输出已改为 R8，不再把未匹配格输出为 0。
  - `hasHistory` 写回已按 `ACCHasHistoryNext` 语义收窄，只在已在控或真正启控时置历史。
- 已完成完整 Pangu 模块临时构建验证：

```text
/tmp/pangu_acc_closed_loop_build_20260629_202705
```

- 构建方式：
  - 在 `/tmp` 搭建临时 Pangu 构建树。
  - 软链官方 `dependencies/configs`，复制官方 `script` 和当前 `ACCClosedLoopModule`。
  - 手动 CMake 配置 whitelist，只构建 `ACCClosedLoopModule`。
  - 构建前需要加载 Pangu thirdparty：

```bash
source dependencies/thirdparty/X86/setup.bash
```

- 已生成并通过依赖检查的产物：

```text
build_out/lib/libACCModule.so
build_out/lib/libZmqBridgeModule.so
build_out/lib/libACCModule_core.so
build_out/lib/libACCModule_conf_pb.so
```

- `ldd` 检查未发现 `not found`，`libzmq.so.5` 和 `libprotobuf.so.26` 均来自 Pangu thirdparty。
- 已安装最小运行目录并从 `project/accreference/icvos/src/configs` 补齐 `app_empty` 运行配置：
  - `conf/app_module/app_empty.pt`
  - `conf/global_conf/global_module.pt`
  - `conf/global_conf/global_com.pt`
  - `conf/global_conf/icvos_machine.pt`
  - `conf/global_conf/tf.pt`
- 已通过 `run_all.sh app_empty` 启动过 Pangu runtime，日志显示 `ACCModule` 和 `ZmqBridgeModule` 被加载到 `app_empty`。
- `keyboard_command_publisher.py` 已新增 `--once` 单次指令模式，便于自动验证：

```bash
python3 tools/pangu_acc_closed_loop/keyboard_command_publisher.py --once e
```

- 已新增 mock 闭环复测脚本：

```bash
tools/pangu_acc_closed_loop/verify_mock_closed_loop.sh
```

  默认使用 `/tmp/pangu_acc_closed_loop_build_20260629_202705`，如构建目录变化可用
  `PANGU_BUILD_ROOT=/path/to/build_root` 覆盖。
- 复测脚本已修复两个运行坑：
  - colcon/Pangu `setup.bash` 会引用未定义环境变量，脚本在 source 期间临时关闭 `nounset`。
  - `app_empty.pt` 原始 `soc_name: "app_empty"` 与本机 `icvos_machine.pt` 的
    `local_machine_name: "soc1"` 不一致，Pangu 不会真正拉起模块；脚本启动前会自动改为 `soc1`。
- 已完成 mock 最小闭环验证：
  - 键盘脚本发送 `commandType=1(E)` 后，mock 日志显示 `cmd=1`。
  - 按当前 `newaccpro3` 画布语义，启控在下一周期生效；随后 Pangu 输出控制命令回到 mock，mock 日志显示 `target=5.00 enable=1`，自车速度开始上升。
  - 键盘脚本发送 `commandType=7(C)` 后，退控在下一周期生效；随后 mock 日志显示 `target=0.00 enable=0`。
  - 结论：`keyboard -> 5702 -> mock Bridge -> 5701 -> ZmqBridgeModule -> ACCModule -> ZmqBridgeModule -> 5702 -> mock`
    这条最小闭环链路已跑通。
- 已新增并通过 ACC oracle 计算序列测试：

```bash
tools/pangu_acc_closed_loop/verify_oracle_sequence.sh
```

  固定输入 `ego=2m/s、lead=2m/s、distance=20m` 下，已验证：
  - `E`：无历史待命启控，`decision=5`，`systemState=2`，当前周期 `enable=0`，`targetSpeed=0`。
  - 无指令保持：`systemState=0`，`decision=8`，下一周期 `enable=1`，`targetSpeed=5.0000`。
  - `T`：`timeGap=1.8 -> 1.6`。
  - `R`：`timeGap=1.6 -> 1.8`。
  - `Q`：当前配置已在 `maxSpeedCap=5.0000` 上限，`maxSpeed` 保持 `5.0000`。
  - 在控 `E`：`maxSpeed=5.0000 -> 3.6111`，`targetSpeed=3.6111`。
  - `C`：取消 ACC，当前周期仍按旧状态输出，下一周期 `enable=0`，`targetSpeed=0`。
  - 历史恢复 `Q`：`systemState=1`，`decision=6`，下一周期重新启控。
  - `S`：制动退出，按下一周期退控处理。
- 已新增并通过 mock 多指令链路测试：

```bash
tools/pangu_acc_closed_loop/verify_mock_command_sequence.sh
```

  默认发送 `E/T/R/Q/C/E/S/0`，已验证：
  - 全部驾驶指令都到达 mock Bridge。
  - Pangu 持续输出 `enable=1` 控制命令。
  - `C/S` 后 Pangu 按下一周期退控，随后输出 `target=0.0000 enable=0`。
  - mock 动态日志中目标速度随距离/速度变化从 `5.0000` 逐步下降到约 `3.1m/s`，说明目标速度计算在闭环中持续生效。
- 已新增真实 CARLA/Bridge/Pangu 复测脚本：

```bash
tools/pangu_acc_closed_loop/verify_real_bridge_closed_loop.sh
```

- 已完成真实 Bridge 多指令闭环验证：
  - `start-gaasd-carla-manual.sh` 能启动 CARLA、Bridge、Town01 ACC 直道场景。
  - `verify_real_bridge_closed_loop.sh` 启动 Pangu `app_empty` 后默认发送 `E/T/R/Q/C/E/S/0`。
  - Bridge 状态探针显示 `driver_command_received=8`。
  - Bridge 状态探针显示 `control_cmd_received` 增长到 `257`，证明 Pangu 控制输出持续回到真实 Bridge。
  - 结论：`keyboard -> 5702 -> carla_bridge.py -> 5701 -> ZmqBridgeModule -> ACCModule -> ZmqBridgeModule -> 5702 -> carla_bridge.py -> CARLA`
    这条真实 Bridge 闭环链路已跑通。
- 修复真实场景启动竞态：
  - `reset-acc-straight-scene.py` 原来只查一次 `role_name=hero`，Bridge 刚生成 ego 时可能查不到。
  - 已改为按 `--timeout-sec` 等待 ego 出现，避免场景 reset 偶发失败。
- 当前真实验证中的 Pangu `disk_fault:loop*` 报警来自 snap loop 只读挂载使用率 100%，本次不影响 `run_all success` 和控制闭环验证。

**newaccpro3 GAASD 生成代码修补验证（2026-06-30）**：

- 当前生成代码目录：

```text
project/newaccpro3/icvos/src/temp_codegen_output
```

- 已确认生成代码的大体框架可用：
  - `run_2f3edfa5_...` 顶层包含 CARLA 输入、`ACCDecision` 复合子块、目标速度计算、CARLA 纵向命令输出。
  - `composite_block_empty_ea2170e7_...` 已生成 `Input/Output/Param/State`，可表达 ACC 决策状态保持。
  - `callback_d3fdb632_..._FuncStep()` 能周期调用 `c_process -> run`。
- 已手动修补的生成代码小问题：
  - 新增 `GlobalContext.hpp`，补齐 `GlobalParams{MinDistance,Kdist,Kspeed}` 和 `global::params`。
  - 新增 `MainInclude.hpp`，补齐 CARLA 边界函数包装和当前画布真值表函数。
  - 修复复合组件实现名 `composite_block()` 与头文件 `run()` 不一致的问题。
  - 修复复合组件漏声明 `temp204061`、`output.timeGap` 自赋值导致未输出时距的问题。
  - 修复顶层非法变量名 `C++_None`，改为 `tempDriverCommand`。
  - 修复顶层漏接线问题：`CARLAACCDriverCommand -> commandType`、`CARLAACCEgoSpeed -> egoV`。
  - 修复顶层漏读取全局参数问题：`MinDistance/Kdist/Kspeed`。
- 已完成构建验证：

```bash
cmake -S project/newaccpro3/icvos/src/temp_codegen_output -B /tmp/newaccpro3_codegen_build
cmake --build /tmp/newaccpro3_codegen_build -j$(nproc)
```

  构建通过，生成 `libMainInclude_static.a` 和 `libMainInclude_shared.so`，当前无 `-Wall -Wextra` 警告。
- 已完成 `ACCDecision` 子块离线序列验证：
  - `E(commandType=1)` 后下一周期 `enable=1`，符合当前画布的一周期状态延迟语义。
  - `T/R(commandType=3/4)` 可更新 `timeGap`。
  - `Q(commandType=2)` 可提高 `maxSpeed`。
  - `C/S(commandType=7/6)` 后下一周期 `enable=0`。
- 已完成顶层 `run_2f3edfa5_...` 离线测试桩验证：
  - 用本地测试桩模拟 `carla_adapter_read_driver_command/read_ego_state/read_lead_vehicle/publish_longitudinal_cmd`。
  - `CARLA input -> ACCDecision -> targetSpeed -> CARLAACCLongitudinalCmd` 链路能串通。
  - 固定 `egoV=2m/s、leadV=2m/s、distance=20m` 时，当前画布参数输出 `target=5.75m/s`。
  - `target=5.75m/s` 的原因是当前生成画布中 `maxSpeed` 初值为 `20` 且没有额外 `5m/s` 上限；这是画布参数/结构差异，不是补丁引入的问题。
- 已完成 Pangu 节点级最小补丁：
  - `acc_zmq_bridge` 新增订阅 `gaasd.carla.driver_command.v1`，并缓存 `command_type/command_valid`。
  - `ZmqBridgeModule` 新增生成 `ACCDecision` 子块实例。
  - `ZmqBridgeModule::Proc()` 现在按 `ego_state + lead_vehicle + driver_command` 调用生成决策子块，计算 `targetSpeed` 并通过 `PublishControl()` 发布 `gaasd.carla.control_cmd.v1`。
  - `module_empty/CMakeLists.txt` 已把 `temp_codegen_output/composite_block_empty_...cpp` 编入 `libZmqBridgeModule.so`。
- 已完成 Pangu Docker ABI 编译验证：
  - 临时构建树：`/tmp/newaccpro3_pangu_codegen_build`。
  - 使用镜像：`docker.cbdes.cn:8080/cbdes/x86:pangu-2.0.5`。
  - 容器内构建通过，生成：

```text
/tmp/newaccpro3_pangu_codegen_build/build_out/lib/libmodule_empty.so
/tmp/newaccpro3_pangu_codegen_build/build_out/lib/libmodule_empty_core.so
/tmp/newaccpro3_pangu_codegen_build/build_out/lib/libmodule_empty_conf_pb.so
/tmp/newaccpro3_pangu_codegen_build/build_out/lib/libZmqBridgeModule.so
```

  - 容器内 `ldd` 检查无 `not found` 和 GLIBC 版本错误。
  - 宿主机直接运行会因 Pangu `install/lib/libc.so.6` 与宿主机工具链冲突，不应在宿主机直接跑 Pangu runtime；运行验证应在 Pangu Docker 内执行。
- 已尝试 mock 运行验证：
  - 第一次失败原因：`run_all.sh` 使用 `install/image/conf`，而临时安装后 `app_empty` 和 `node_module/module_empty` 只在 `install/conf`，导致实际没有拉起 `ZmqBridgeModule`。
  - 需要下一步把以下内容同步到 `/tmp/newaccpro3_pangu_codegen_build/install/image` 后重跑：
    - `install/lib/libmodule_empty*.so` 和 `libZmqBridgeModule.so` -> `install/image/lib/`
    - `project/newaccpro3/icvos/src/configs/app_module/app_empty.pt` -> `install/image/conf/app_module/`
    - `install/conf/node_module/module_empty` -> `install/image/conf/node_module/`
    - `project/newaccpro3/icvos/src/configs/global_conf/{global_module.pt,global_com.pt,tf.pt}` -> `install/image/conf/global_conf/`
    - 已验证的 `icvos_machine.pt` -> `install/image/conf/global_conf/icvos_machine.pt`
  - 当前会话继续同步时需要 root 权限，因为 `install/image` 由 Docker root 创建；本次 sudo/escalation 被系统拒绝，未继续绕过。
- 仍未完成：
  - 重跑 Docker 内 mock 闭环，确认 `driver_command -> generated ACCDecision -> control_cmd`。
  - 真实 CARLA/Bridge 联调。
  - 与 GAASD 团队确认：正式代码生成应直接把 `app_empty/node_module` 安装到 `install/image/conf`，否则 `run_all.sh` 会跑不到生成节点。

- 后续放入官方 Docker 场景时，建议将键盘脚本随 CARLA/Bridge 工具放到：

```text
/opt/carla/carla_tools/keyboard_command_publisher.py
```

- 场景脚本可以从 `/opt/carla/carla_tools/scenario/` 调用该脚本；端口方向保持 `keyboard -> 5702 -> Bridge -> 5701 -> Pangu`。

- 注意：
  - 宿主机和裸 Pangu Docker 镜像默认系统路径下都没有 `zmq.h`，完整构建必须走 Pangu 的 `THIRD_PATH` 或显式加入上述 thirdparty include/lib。
  - 当前已经验证 `.so` 能构建，Pangu runtime 能启动，mock 闭环已跑通；尚未接入 GAASD 生成代码 diff。

**newaccpro3 最新 GAASD 生成代码复核（2026-06-30）**：

- 复核记录已写入：

```text
docs/GAASD_newaccpro3_代码生成问题复核_20260630.md
```

- 本次用户在 GAASD 中重新生成 `newaccpro3` 代码，输出路径为：

```text
project/newaccpro3/icvos/src/temp_codegen_output
```

- 已确认画布关键连线在 JSON 中存在：
  - `CARLA自车速度 -> ACCDecision.egoV`
  - `CARLA驾驶指令 -> ACCDecision.commandType`
  - `ACCDecision.enable -> CARLAACCLongitudinalCmd.enable`
  - `ACCDecision.timeGap/maxSpeed -> 后级时距控制和限幅链路`
- 当前生成代码仍不能进入 oracle diff：
  - `GlobalContext.hpp` / `MainInclude.hpp` 未生成，编译第一层即失败。
  - 复合组件头文件声明 `run()`，cpp 实现和外层调用使用 `composite_block()`，接口不一致。
  - 子组件输入结构体没有根据 JSON 连线赋值，例如 `compositeBlockInstance_2_in.egoV`、`commandType` 未生成。
  - 仍存在非法变量名 `C++_None`。
  - 仍存在未声明临时变量：`temp199951 / temp196788 / temp199342 / temp204061`。
  - 真值表函数被调用但没有生成函数体。
  - Pangu `module_empty::Proc()` 为空，没有接入 `FuncStep` 或 `c_process.run()`。
  - `global_services.json/global_channels.json` 为空列表时生成器报 `'list' object has no attribute 'get'`。
- 结论：这不是 ACC 公式问题；短期仍以手写 Pangu ACC oracle 作为闭环联调基准。GAASD 生成代码需等生成器修复后，再执行同输入序列 diff。

**Pangu ACC oracle 按生成代码语义对齐（2026-06-30）**：

- 根据 `newaccpro3/icvos/src/temp_codegen_output` 中已生成的 `ACCDecision` 片段，已将
  `tools/pangu_acc_closed_loop/ACCClosedLoopModule/module_lib/AccTargetSpeed.*`
  调整为当前画布语义，而不是按算法直觉补充额外决策格。
- 当前对齐后的关键语义：
  - `enable = notLowSpeed && old controlEnabled`，即输出使用当前周期读出的旧状态。
  - `E/Q/S/C` 等命令只更新下一周期状态，因此启控和退控都有 1 个周期延迟。
  - `controlEnabledNext = (old controlEnabled || R5 || R6) && !(cmd6 || cmd7)`，低速状态本身不清除内部在控状态。
  - 当前简化画布未实现 R5 当前车速捕获；`maxSpeed` 只由 R1/R2 调整，启控后默认保持初始 `maxSpeed=5.0m/s`。
  - `C/S` 退控同样下一周期生效，当前周期仍可能输出最后一次有效控制命令。
- 已重跑 `tools/pangu_acc_closed_loop/verify_oracle_sequence.sh`，固定序列通过：
  - `E` 当前周期 `decision=5`、`enable=0`、`targetSpeed=0`。
  - 下一周期无指令保持时 `enable=1`、`targetSpeed=5.0000`。
  - `T/R` 正常调整 `timeGap`。
  - `Q` 在 `maxSpeedCap=5.0m/s` 下保持上限。
  - 在控 `E` 将 `maxSpeed` 从 `5.0000m/s` 降到 `3.6111m/s`。
  - `C/S` 按下一周期退控验证通过。
  - 新增低速保护用例：`egoV < vMin` 时当前 `enable=0`，但内部 `controlEnabled` 保持；速度恢复后自动续上。
- 已重跑 `tools/pangu_acc_closed_loop/verify_mock_command_sequence.sh`，由于 ZMQ/socket 在沙箱内受限，使用提权执行；结果通过：
  - `E/T/R/Q/C/E/S/0` 全部到达 mock Bridge。
  - mock 日志中能看到 `enable=1` 控制命令和 `enable=0` 退出命令。
  - E/C/S 的一周期延迟与当前画布生成语义一致。
- 根据生成日志进一步修正低速保持语义后，已再次重跑
  `tools/pangu_acc_closed_loop/verify_mock_command_sequence.sh`，结果仍为 `PASS`。
  mock 日志显示 `target` 随距离/速度连续变化，`C/S` 后稳定输出 `target=0 enable=0`。
- 已重跑真实 Bridge/CARLA/Pangu 多指令闭环复测：

```bash
tools/pangu_acc_closed_loop/verify_real_bridge_closed_loop.sh
```

  结果通过：
  - CARLA/Bridge/Town01 ACC 直道场景启动成功，Bridge 状态为 `running`，地图为 `Carla/Maps/Town01`。
  - Pangu `run_all.sh app_empty` 返回 `run_all success`，`ACCModule` 和 `ZmqBridgeModule` 均被拉起。
  - 键盘序列 `E/T/R/Q/C/E/S/0` 全部发送成功，Bridge 状态中 `driver_command_received=8`。
  - Bridge 状态中 `control_cmd_received` 从 `234` 增长到 `254`，证明 Pangu 控制命令持续返回真实 Bridge。
  - `ego_state/object_list` 计数持续增长到 600+，说明 CARLA -> Bridge 输入链路稳定。
  - Pangu 日志仍有 snap loop 只读挂载 `disk_fault:loop*` 报警，该报警此前已确认不影响本次 `run_all success` 和闭环链路。
  - 脚本结束后自动清理 CARLA/Bridge/Pangu 栈，`tools/carla_bridge/health-carla.sh` 显示 CARLA 不可达，符合清理预期。
- 后续如果画布补回 R5 当前车速捕获、或调整真值表退出格，需要同步更新 oracle 和测试期望。

- 新增参考工程位置：

```text
/home/aiden/文档/Modularization/project/accreference
```

- `gen_code.json` 显示该工程走新版 Pangu 集成路径：
  - `is_oscilloscope=false`
  - `is_integration=true`
  - `pangu_code_path=/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/panguFramework/pangu_gaasd`
- 该工程不是旧版“示波器 + FuncStep”仿真路径，也不是大规模基础模块函数图生成路径，而是新版
  **App / Module / Channel / Task** 节点级集成路径。

**accreference 画布层级结构**：

- `app_empty` 空应用下挂 1 个模块：`ACCModule`。
- `ACCModule` 内部只有 2 个通道组件：
  - 输入通道 `acc_input`，消息类型 `pangu.modules.AccInput`。
  - 输出通道 `acc_output`，消息类型 `pangu.modules.AccOutput`。
- 当前画布连接只有 1 条：

```text
acc_input.frame_id -> acc_output.frame_id
```

- `task` 表中只有 1 个任务：
  - `task_name=acc_input`
  - `task_type=POLICY_TYPE_MSG_NEWEST`
  - `timer_trigger_interval=20`
  - `input_trigger=["acc_input"]`
- 结论：官方示例主要验证“节点订阅输入消息、回调后发布输出消息”的机制；算法逻辑并没有在画布里用基础模块铺开。

**accreference 生成代码与模板源码差异**：

- 工程内生成源码位置：

```text
/home/aiden/文档/Modularization/project/accreference/icvos/src/modules/ACCModule_5fbcb590_2620_4070_bc51_d7f985dc3561/ACCModule
```

- 其中 `ACCModule.cpp` 当前只做 `frame_id` 透传：

```cpp
output->set_frame_id(input.frame_id());
PUB_MSG(acc_output, output);
```

- Pangu 框架原始模板位置：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/panguFramework/pangu_gaasd/modules/ACCModule/ACCModule
```

- 原始模板中的 `ACCModule.cc` 才真正调用 `AccTargetSpeed::run()`，并输出：
  - `target_speed_mps`
  - `enable`
  - `valid`
- `AccTargetSpeed.cpp` 中的目标速度公式为：

```text
distanceSafe = max(leadDistance, 0)
distDiff = distanceSafe - desiredDistance
speedDiff = leadSpeed - egoSpeed
rawTargetSpeed = leadSpeed + distanceGain * distDiff + speedGain * speedDiff
targetSpeed = enable ? clamp(rawTargetSpeed, 0, maxSpeed) : 0
valid = enable
```

- 默认参数：
  - `desired_distance_m=15`
  - `max_speed_mps=5`
  - `distance_gain=0.35`
  - `speed_gain=0.8`

**accreference CARLA/ZMQ 桥接方式**：

- Pangu 模板提供 `ZmqBridgeModule` 和 `acc_zmq_bridge`。
- `acc_zmq_bridge` 从 CARLA Bridge 的 ZMQ PUB 端订阅：
  - `gaasd.carla.ego_state.v1`
  - `gaasd.carla.lead_vehicle.v1`
- 默认输入端点：

```text
tcp://127.0.0.1:5701
```

- 默认控制输出端点：

```text
tcp://127.0.0.1:5702
```

- `ZmqBridgeModule` 将 ZMQ 数据转为内部 `acc_input` 消息，`ACCModule` 输出
  `acc_output` 后，再由 `ZmqBridgeModule` 转回 CARLA 控制命令。
- 这说明官方方向和我们前期设计的 Bridge 思路并不冲突，只是把 ZMQ 适配层放进 Pangu 节点模块内部，而不是由 GAASD 画布扫描组件直接承担。

**accreference 编译与运行状态判断**：

- 日志显示代码编译阶段通过：

```text
/home/aiden/文档/Modularization/project/accreference-代码编译成功
compile success
```

- 同一日志后续运行阶段出现：

```text
[app_empty]--ACCModule, FAILED
[app_empty]--ZmqBridgeModule, FAILED
run_all failed
```

- 当前 `icvos/output/install/lib` 下未看到对应 `libACCModule.so` / `libZmqBridgeModule.so` 安装产物。
- 因此当前结论要分开写：
  - **代码生成和编译阶段已通过**。
  - **运行阶段仍需要继续排查模块库安装、运行包路径或启动依赖问题**。

**对 ACC/LKS 后续方案的影响**：

- `accreference` 证明新版 GAASD/Pangu 的推荐集成方向是：

```text
App -> Module -> Channel Input/Output -> Task Callback -> Pangu 节点运行
```

- 对后续 ACC/LKS，建议采用两层路线：
  1. **节点层**：按官方示例使用 `ACCModule/LKSModule + input/output channel + task callback`。
  2. **算法层**：继续尽量用 GAASD 基础模块搭建 ACC/LKS 逻辑；如果生成器短期不稳定，可先把算法封装到模块源码中，等函数图生成稳定后再替换为画布生成逻辑。
- 官方示例没有证明“大规模基础模块画布生成复合函数”已经稳定；它实际上绕开了这条路径，所以不能据此否定我们之前发现的复合组件、全局参数、状态变量、函数绑定等生成器问题。
- 后续和 GAASD 团队沟通时，应把需求拆成：
  - 节点/通道/任务机制：参考 `accreference`，这条路线已经基本明确。
  - 函数图/基础模块生成机制：仍需要继续修复，才能承载我们完整 ACC/LKS 画布算法。

**newaccpro3 新版节点级代码生成复核**：

- 用户在 GAASD 页面点击“生成代码”后，当前事实源工程为：

```text
/home/aiden/文档/Modularization/project/newaccpro3
```

- 代码生成入口记录在：

```text
/home/aiden/文档/Modularization/project/newaccpro3/log/2026-06-26.log
```

- 本次实际执行命令为：

```bash
/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/codeTools/new_gaasd gen_code /home/aiden/文档/Modularization/project/newaccpro3/gen_code.json
```

- `gen_code.json` 中关键配置：
  - `project_path=/home/aiden/文档/Modularization/project/newaccpro3`
  - `platform=x86`
  - `pangu_code_path=/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/panguFramework/pangu_gaasd`
  - `scan_path=/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/components/THICV`
  - `is_oscilloscope=false`
  - `is_integration=true`
- 因此本次不是旧版示波器/函数级仿真生成，而是新版 **节点级/模块集成代码生成路径**。

**本次生成代码位置**：

- 函数级/复合组件中间产物：

```text
/home/aiden/文档/Modularization/project/newaccpro3/icvos/src/temp_codegen_output
```

- 关键文件：
  - `run_2f3edfa5_8227_43eb_859d_1a6db0afd957/run_2f3edfa5_8227_43eb_859d_1a6db0afd957.cpp`
  - `run_2f3edfa5_8227_43eb_859d_1a6db0afd957.hpp`
  - `composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad/composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad.cpp`
  - `composite_block_empty_ea2170e7_297c_4433_b70a_9c66e48988ad.hpp`
  - `callback_d3fdb632_0c78_41ab_a837_3bc336dba2df/c_process_c1ecaf88_00f5_488b_8f5d_9d0af56264d0.cpp`
  - `callback_d3fdb632_0c78_41ab_a837_3bc336dba2df/callback_d3fdb632_0c78_41ab_a837_3bc336dba2df_FuncStep.cpp`
- 节点级回调信息：

```text
/home/aiden/文档/Modularization/project/newaccpro3/icvos/src/temp_callback_info.json
```

**本次确定的生成器问题**：

- `global_services.json` 与 `global_channels.json` 当前内容均为 `[]`，但第一阶段代码生成器按对象调用
  `.get()`，日志报：
  `'list' object has no attribute 'get'`。这是空 service/channel 数据结构兼容问题。
- `run_*.cpp` 仍生成非法 C++ 变量名 `C++_None`，来源是 `CARLAACCDriverCommand`
  这类无输出/未正确绑定输出组件被生成器硬编码成 `C++_None`。
- `run_*.cpp` 中存在未声明临时变量，例如 `temp273497`、`temp270562`、`temp272941`；
  说明部分常量/参数/连接源没有被纳入局部变量声明或参数读取。
- `composite_block_empty_*.hpp` 声明 `run(const Input&, Output&)`，但对应 cpp 实现为
  `composite_block(const Input&, Output&)`，调用端也使用 `composite_block()`，复合组件接口仍存在
  `run/composite_block` 不一致问题。
- `composite_block_empty_*.cpp` 中 `output.timeGap = output.timeGap;`，没有把
  `temp277178` 或 `state_.timeGap` 正确赋给输出端口，说明 output 节点的源连接解析仍有缺陷。
- `c_process_*.cpp` 中调用：

```cpp
sub_.compositeBlockInstance.run(compositeBlockInstance_in, compositeBlockInstance_out);
```

  但 `compositeBlockInstance_in` 未声明，进程层 wrapper 生成不完整。
- `temp_callback_info.json` 中 `function_members=[]`、`source_files=[]`、
  `channel_to_function: {"空模块": []}`，说明节点级 callback 没有正确把 composite-block
  绑定为周期调用函数；当前 `module_empty` 更像空节点模板，不是完整 ACC 节点。
- 日志最后虽然显示 `result success`，但这是任务流程层面的 success，不代表生成源码可编译、可运行。

**代码生成方向判断**：

- GAASD 当前目标方向已经明确：函数画布作为算法逻辑，外层通过 `app/process/module`
  形成节点级工程，再由 Pangu 框架拉起模块运行。
- `new_gaasd` 现在分两段做事：
  1. 先按 functions/composite-block 生成 `FuncModule` 风格 C++ 中间产物；
  2. 再按 module/process/app 生成节点级 Pangu 工程，并通过 callback/channel/message 把函数绑定到节点。
- 这个方向是正确的，也符合后续 CARLA/实车联调需要；问题在于第二段“模块 callback 与函数画布的绑定”
  还没打通，第一段复合组件细节也还有若干代码生成 bug。

**对 ACC/LKS 实现方法的影响**：

- ACC/LKS 画布设计继续保留“基础模块 + 少量 CARLA 边界自定义组件”的路线，不建议因为当前生成器 bug
  回退到大量手写业务代码。
- 画布内部仍优先使用基础模块、局部参数、局部状态、真值表/比较逻辑搭建；这是 GAASD 应该支持的正式方向。
- CARLA 输入/输出边界组件仍保持单输出、少端口、严格 FuncModule 化，避免把 ZMQ/adapter 逻辑塞进扫描组件。
- 在 GAASD 修复前，ACC/LKS 的验收要分成两层：
  - **画布结构验收**：检查连线、参数、状态、真值表配置是否正确。
  - **代码生成验收**：每次新版工具更新后检查 `temp_codegen_output` 和节点级 `modules/` 是否消除上述 bug。
- 后续判断 GAASD 是否可进入联调的最低标准：
  1. 不再生成 `C++_None`；
  2. 不再生成未声明临时变量；
  3. 复合组件统一使用 `run()` 或生成完整适配调用；
  4. output 端口能正确赋值；
  5. `c_process` 能声明并填充 composite input；
  6. `temp_callback_info.json` 中 `function_members/channel_to_function/source_files`
     能体现实际 ACC/LKS 组件，而不是空数组；
  7. 空 `global_services/global_channels` 能被正常跳过或按统一对象结构输出。

**全局参数/全局状态生成位置与 Pangu 迁移策略**：

- 当前 `newaccpro3` 使用的函数级 `FuncModule` 中间产物已经生成全局参数和全局状态代码，位置为：

```text
/home/aiden/文档/Modularization/project/newaccpro3/icvos/src/temp_codegen_output/GlobalContext.hpp
/home/aiden/文档/Modularization/project/newaccpro3/icvos/src/temp_codegen_output/GlobalContextTypes.hpp
/home/aiden/文档/Modularization/project/newaccpro3/icvos/src/temp_codegen_output/GlobalContext/GlobalContext.cpp
```

- 当前生成出的全局参数：

```cpp
struct GlobalParams {
  Real controlTs = 0.05;
};
```

- 当前生成出的全局状态：

```cpp
struct FeedbackState {
  Real lonVelFb;
  Real latPosFb;
};

struct GlobalStates {
  FeedbackState feedback = {.lonVelFb = 0.0, .latPosFb = 0.0};
};
```

- 全局变量实例定义在：

```text
/home/aiden/文档/Modularization/project/newaccpro3/icvos/src/temp_codegen_output/GlobalContext/GlobalContext.cpp
```

  对应：

```cpp
namespace control::global {
GlobalParams params {.controlTs = 0.05};
GlobalStates states {.feedback = {.lonVelFb = 0.0, .latPosFb = 0.0}};
}
```

- 这些代码属于清华 `FuncModule` 函数层。函数层组件通过：

```cpp
using Global = GlobalParams;
...
global::params
```

  把全局参数传给 `FuncModule` 基类。

- 当前 Pangu 节点级生成物位置为：

```text
/home/aiden/文档/Modularization/project/newaccpro3/icvos/src/modules/module_empty_d3fdb632_0c78_41ab_a837_3bc336dba2df/module_empty
```

- Pangu 模块当前只有模块配置参数：

```text
src/module_empty.h
src/module_empty.cpp
proto/module_empty_config.proto
conf/module_empty.pt
```

  其中 `module_emptyParams` 只包含 `timer_duration/work_root/desired_distance_m/max_speed_mps`
  等模块级配置，未包含函数画布里的 `GlobalParams/GlobalStates`，也没有生成 `GlobalContext`。

- 因此当前事实是：
  - 清华函数框架已有全局参数/状态能力。
  - Pangu 节点框架当前生成物尚未接入全局参数/状态。
  - 后续正式联调优先走 Pangu 框架，GAASD 团队应补齐 Pangu 框架下的全局参数/状态生成与持久化能力。

**临时补丁策略（等切到 Pangu 并重新生成后执行）**：

- 如果短期需要在 Pangu 框架下继续测试 ACC/LKS，而 GAASD 尚未补齐全局参数/状态，可以由 Codex 手动补一层兼容代码。
- 最小补丁原则：
  1. 保留 `temp_codegen_output/GlobalContext*` 作为事实源，不重新设计参数结构。
  2. 将 `GlobalContext.hpp`、`GlobalContextTypes.hpp`、`GlobalContext.cpp` 纳入 Pangu 模块构建。
  3. 在 Pangu 模块 `CMakeLists.txt` 中加入对应源文件或复制到 `module_lib/` 后编译。
  4. 在 `module_empty::LoadConfig()` 或 `Init()` 中把 Pangu `module_emptyParams` 映射到
     `control::global::params`，例如 `controlTs/timer_duration`。
  5. 对跨周期变量优先保留在函数组件 `state_` 内；只有确实需要跨模块共享时，才临时写入
     `control::global::states`。
  6. 不把这类补丁当正式架构，只作为 GAASD 代码生成器修复前的测试兼容层。
- 重要限制：当前 `module_empty` 仍没有把 composite-block 真实绑定到 callback（`function_members=[]`），
  所以现在只补 `GlobalContext` 还不能让 ACC 运行。必须先等或修到 Pangu 模块能周期调用
  ACC/LKS 函数画布后，再补全局参数/状态才有实际测试价值。

**Pangu Docker 手动启动方式（GAASD 自动运行不可用时使用）**：

- 背景：新版 GAASD 正常流程应为“生成代码 → 点击运行 → 软件自动启动 Pangu/Docker 仿真场景”。
  但当前 ACC/LKS 工程仍卡在代码生成/节点绑定问题，GAASD 自动运行链路暂不可用；因此需要先手动启动
  Pangu Docker 场景容器，用于验证 GAASD 内置仿真场景、bridge 脚本和 CARLA 场景启动链路。
- 已确认 `ubuntuEnv_0.0.5_v2.tar` 必须使用 `docker load -i` 正确加载，不能用错误的 import 方式。
  正确镜像名为：

```text
docker.cbdes.cn:8080/cbdes/x86:pangu-2.0.5
```

- 已确认本机 Docker 当前未配置 NVIDIA Container Toolkit，`--gpus all` 会报：

```text
could not select device driver "" with capabilities: [[gpu]]
```

  因此当前手动启动容器时先不要带 `--gpus all`。
- 如果已有同名残留容器，应先清理：

```bash
docker rm -f pangu_x86
```

- 当前已验证可用的手动启动命令：

```bash
docker run --restart=always -it \
  -e DISPLAY=$DISPLAY \
  -e TERM \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=/tmp/.dockerg2b8mikt.xauth \
  -v /tmp/.dockerg2b8mikt.xauth:/tmp/.dockerg2b8mikt.xauth \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /etc/localtime:/etc/localtime:ro \
  --shm-size=8193m \
  --name=pangu_x86 \
  --privileged \
  --net=host \
  -v /home:/home \
  docker.cbdes.cn:8080/cbdes/x86:pangu-2.0.5 \
  /bin/bash
```

- 启动后应在容器内优先检查 GAASD 启动场景依赖脚本是否存在：

```bash
ls /opt/carla/carla_tools/scenario/load_scene_bridge.sh
```

- 已确认 `pangu_x86` 容器内包含 GAASD 团队集成的 ACC 场景和我们前期整理的
  CARLA Bridge 工具链，核心目录为：

```text
/opt/carla/carla_tools/scenario
/opt/carla/carla_tools/tools/carla_bridge
```

- GAASD 页面点击“启动场景”时，当前 ACC 场景实际执行入口为：

```text
/opt/carla/carla_tools/scenario/load_scene_bridge.sh
```

- `load_scene_bridge.sh` 的职责不是启动 CARLA 本体，而是在 CARLA 已由
  GAASD “启动仿真器”打开后，执行以下动作：
  1. 选择 `/opt/carla/carla_tools/scenario/bridge_config.json` 作为 Bridge 配置。
  2. 调用 `/opt/carla/carla_tools/tools/carla_bridge/start-gaasd-carla-manual.sh`。
  3. 使用 `--no-start-carla` 连接已有 CARLA。
  4. 设置 ACC 直道场景：`ego_spawn_index=198`、前车距离 `25m`、前车速度 `2m/s`。
  5. 启动 Bridge，绑定 `PUB=127.0.0.1:5701`、`CONTROL=127.0.0.1:5702`。
  6. 设置 spectator 跟随视角：后方 `8m`、高度 `6m`、俯仰角 `-25deg`。
- 注意：`load_scene_bridge.sh` 虽然传入了 `--lead-behavior traffic_manager`，
  但 `start-gaasd-carla-manual.sh` 默认 `RESET_SCENE=1`，会优先调用
  `reset-acc-straight-scene.py`；该脚本内部直接使用 `enable_constant_velocity()`
  控制前车。因此当前 ACC 场景实际是 **lane waypoint 放置 + constant velocity 前车**，
  不是 Traffic Manager 前车。

- `bridge_config.json` 中当前 ACC 场景关键配置：

```text
CARLA host/port: 127.0.0.1:2000
CARLA root: /opt/carla
PythonAPI: /opt/carla/PythonAPI/carla
map: Town01
ego role: hero
ego spawn index: 198
fixed_delta_seconds: 0.05
ZMQ PUB: tcp://127.0.0.1:5701
ZMQ CONTROL: tcp://127.0.0.1:5702
speed PID: Kp=1.4, Ki=0.18, Kd=0.05
lane_keep_enabled: true
lane_keep_override_zero_steer: true
```

- Bridge 发布的数据链路保持为：

```text
CARLA -> carla_bridge.py -> ZMQ PUB(5701)
topics: ego_state / object_list / lead_vehicle / chassis_feedback / bridge_status
```

- GAASD/adapter 控制链路保持为：

```text
GAASD 组件输出 target_speed_mps / enable / steer_rad
-> ZMQ CONTROL(5702)
-> carla_bridge.py
-> CARLA VehicleControl(throttle, brake, steer)
```

- 当前容器内 `carla_bridge.py` 已包含之前修复过的关键逻辑：
  - `control_cmd_received` 使用真实计数器。
  - `lead_vehicle.relative_speed_mps` 使用纵向相对速度。
  - 控制命令支持目标速度 PID 映射油门/刹车。
  - 当 `lane_keep_enabled=true` 且输入转角接近 0 时，Bridge 可自动用 CARLA 车道中心计算横向保持转角。

- 新增场景的建议方式：
  1. 在 `/opt/carla/carla_tools/scenario/` 下新增独立脚本，例如
     `load_scene_lks.sh` 或 `load_scene_acc_dynamic.sh`。
  2. 为新场景准备独立配置，例如 `bridge_config_lks.json`，避免直接覆盖 ACC 默认配置。
  3. 新脚本继续复用 `tools/carla_bridge/start-gaasd-carla-manual.sh`，只调整参数：
     `--ego-spawn-index`、`--lead-distance`、`--lead-speed`、`--no-lead`、
     `--spectator-back/up/pitch`、`--config`。
  4. 如果要让 GAASD 页面直接选择新场景，还需要在 GAASD 前端场景列表中新增对应项，
     使其执行 `/opt/carla/carla_tools/scenario/<场景名>.sh`。
  5. 在 GAASD 暂未开放配置前，可先复用 `load_scene_bridge.sh` 或手动在容器内执行新脚本验证。

- 该方式的定位：只作为当前阶段“手动启动 Docker 仿真场景”的临时方案。等 GAASD 代码生成和运行链路稳定后，
  仍应回到 GAASD 软件内自动生成、自动启动、自动运行的正式流程。

---

## 当前状态（2026-06-24 更新）

**隔离版 GAASD 2.7.0.5 更新**：

- 已检查 `/home/aiden/文档/temp` 中 2026-06-24 新增的三个包：
  - `GAASD_SETUP_20260623_1.tar.gz`：完整 GAASD 前端安装包，内部包含
    `gaasd_1_build-2.7.0.5.tar.gz`、安装脚本和图标。
  - `gaasd_code_tools_build_2.14.0.20.tar`：新版后端代码工具包，包含
    `gaas_codegen`、`new_gaasd`、`codescan-cpp`、`gen_code.json`、
    `scan.json`、`public/FuncModule.hpp` 及 clang 资源目录。
  - `codescan-cpp-release.tar.gz`：单独发布的 C++ 组件扫描器自包含包，
    包含 `codescan-cpp` 和 `lib/clang/20` 资源目录。
- 已将隔离版新版 GAASD 更新到
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5`，未修改系统全局 `/opt/gaasd`
  和 `/usr/bin/gaasd`。
- 更新方式：先将旧 `app` 和旧 `home/gaasd_server/codeTools` 移入回滚目录
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/rollback/20260624_100551`，
  再安装新版前端和新版 codeTools。
- `codeTools` 采用 `gaasd_code_tools_build_2.14.0.20.tar` 为主体，并用
  `codescan-cpp-release.tar.gz` 中的 `codescan-cpp` 与 `lib/clang/20`
  覆盖，以使用单独发布的 C++ 扫描器版本。
- 由于 20260623 前端仍引用旧入口 `codescan`、`run_x86.sh` 和
  `codeTools/dist/run_simulation.sh`，已从回滚备份补回以下兼容文件：
  `.env`、`build.sh`、`codescan`、`dist/`、`exec_order.conf`、
  `math_codegen`、`run_executables.sh`、`run_x86.sh`、`wicv_build.sh`、
  `wicv_run.sh`。这些文件不覆盖新版 `new_gaasd`、`gaas_codegen`、
  `codescan-cpp`。
- 安装后校验：
  - 新前端可执行文件：`app/gaasd` SHA-256
    `723e82279257afe8943fb188c00e4222ed6050dd5f5adfddd572ab16206ac45c`。
  - 新 `gaas_codegen` SHA-256
    `bc855304e5f625919d2802931bc0cae6482d44947cf7c0edccfbcf383f978d7f`，
    `--help` 可正常输出。
  - 新 `new_gaasd` SHA-256
    `8edcafd48215ca1c70521ccd5364e4dd92f1a4818e3ec9fdafa9c82e83037380`，
    能启动并在隔离 profile 下创建 `data.db`；该工具无标准 `--help`
    输出，传 `--help` 时记录参数错误。
  - 当前 `codescan-cpp` SHA-256
    `26949ab22a1ca62e6560bcd87d0d3219a3601e25d3f009b617727819166bcc31`，
    可正常输出 usage。
  - 旧 C 扫描器 `codescan` 已保留并可正常输出 usage。
- 启动方式仍为：

```bash
/home/aiden/gaasd_versions/gaasd-2.7.0.5/run-gaasd-2.7.0.5.sh
```

**待验证**：

- 使用新版前端打开副本工程，验证 LKS/ACC 画布是否能正常读取。
- 使用新版 `new_gaasd` / `codescan-cpp` 扫描团队确认的 C++ FuncModule
  组件包，验证 C++ 组件能否正常入库。
- 使用当前 `project/lks` 或工程副本重新生成代码，检查复合组件、
  示波器、全局参数和数组端口相关问题是否已修复。

**组件包导入（2026-06-24）**：

- 已检查并导入 `/home/aiden/文档/temp` 新增的两个 THICV 组件包：
  - `清华组件包PID_0606.tar`：包含 `PidController`、`VehicleModel`、
    `TopCanvas` 3 个顶层组件。
  - `清华组件包_CC_0609_2.tar`：包含 28 个定速巡航/决策状态机/SPPVT 控制相关
    顶层组件。
- 导入目标为隔离版 GAASD：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5`，未修改旧版全局 GAASD。
- 由于两个组件包各自带独立 `cbdes.db`，直接顺序导入会互相覆盖顶层组件列表；
  已新增脚本 `tools/gaasd_import_component_packages.py`，先合并当前 THICV 组件和两个
  新包，再更新组件文件与 `.gaasd/gaasd.db`。
- 导入前已备份当前组件目录和数据库到：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/rollback/component_import_20260624_103222`。
- 导入后校验：
  - `home/gaasd_server/components/THICV/cbdes.db` 顶层组件数：31。
  - `.gaasd/gaasd.db` 中 `vendor=THICV` 的 `component` 记录：31。
  - `.gaasd/gaasd.db` 中 `vendor=THICV` 的 `component_detail` 记录：1403。
  - 已确认 `PidController_b42c0e67.json`、`VehicleModel_4c08435f.json`、
    `TopCanvas_237dcf0f.json` 文件存在。

**前端回退（2026-06-24）**：

- 因新版前端默认图层规则导出 `cpp_rule`，导致旧 C 工程的
  `c-process` 空进程无法进入下级 function 画布，已先将隔离版 GAASD
  前端回退到 20260608 版本。
- 回退范围仅限：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/app`。
- 未回退、未修改：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/codeTools`
  及后端工具链。
- 当前 20260623 前端备份位置：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/rollback/frontend_current_before_rollback_20260624_111029/app`。
- 回退来源：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/rollback/20260609_143740/app`
  （对应 `GAASD_SETUP_20260608.tar.gz`）。
- 回退后校验：
  - 前端资源文件已恢复为 `parseData-B2Q3KD_7.js`、
    `index-B_I3Hw40.js` 等 20260608 版本。
  - 后端工具仍为新版：
    `new_gaasd` SHA-256
    `8edcafd48215ca1c70521ccd5364e4dd92f1a4818e3ec9fdafa9c82e83037380`，
    `gaas_codegen` SHA-256
    `bc855304e5f625919d2802931bc0cae6482d44947cf7c0edccfbcf383f978d7f`，
    `codescan-cpp` SHA-256
    `26949ab22a1ca62e6560bcd87d0d3219a3601e25d3f009b617727819166bcc31`。

**前端重新更新（2026-06-24）**：

- 已按要求将隔离版 GAASD 前端重新恢复为 20260623 最新版本。
- 更新范围仅限：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/app`。
- 未回退、未修改：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/codeTools`
  及后端工具链。
- 本次恢复来源：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/rollback/frontend_current_before_rollback_20260624_111029/app`。
- 更新前的 20260608 前端备份位置：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/rollback/frontend_20260608_before_reupdate_20260624_142730/app`。
- 更新后校验：
  - 前端资源文件已恢复为 `parseData-Bv4l81Ao.js`、
    `index-C6DC-xSh.js` 等 20260623 版本。
  - 当前 `app` 与
    `rollback/frontend_current_before_rollback_20260624_111029/app`
    目录 `diff -qr` 无差异。
  - 后端工具仍为新版：
    `new_gaasd` SHA-256
    `8edcafd48215ca1c70521ccd5364e4dd92f1a4818e3ec9fdafa9c82e83037380`，
    `gaas_codegen` SHA-256
    `bc855304e5f625919d2802931bc0cae6482d44947cf7c0edccfbcf383f978d7f`，
    `codescan-cpp` SHA-256
    `26949ab22a1ca62e6560bcd87d0d3219a3601e25d3f009b617727819166bcc31`。

**图形化工具版本迭代会议结论（2026-06-24）**：

- 已提取并核对 `docs/文字记录：图形化工具版本迭代问题协调会 2026年6月24日.docx`
  与 `docs/智能纪要：图形化工具版本迭代问题协调会 2026年6月24日.docx`。
- 新版 GAASD 当前主线转向“节点级”运行：应用下面是进程和模块，模块更接近
  通信 node；节点级主要面向 CARLA、实车和量产式部署，输入输出依赖通道和
  消息触发。
- 旧“函数级”画布仍需保留，主要用于单函数/算法块验证，例如常量输入、变量输入
  和示波器曲线查看。当前新版把节点级和函数级能力合并后，函数级兼容性仍不完整。
- 节点级运行必须有消息通道触发，不能简单默认生成一个空 trigger；没有输入通道的
  空模块更适合走函数级仿真，而不是节点级运行。
- 示波器当前仍与函数级生成强绑定。节点级运行时，示波器暂时不能像旧版一样直接显示
  内部曲线。短期方案是补全“仿真示波器里的函数级功能”；节点级若要看数据，可能先通过
  log 文件或后续简化版 topic/message 示波器展示。
- 全局参数和全局状态入口因 `c-process` 变化暂时缺失。会议倾向先把全局参数/状态配置
  入口加到“空进程”上；但当前 ACC/LKS 单应用场景也可以先使用局部参数，效果基本等同，
  画布读取方式对应 `read-local-param`。
- `core-block` 是新版重要类型：能配置参数/状态，但不能进入下级画布；它类似“原件/原子组件”。
  当前 codegen 尚未完全兼容，刘洋需优先处理，因为这会影响 ACC/LKS 组件测试。
- 通道是节点间消息中间件。当前通道依赖预置 PB 消息和代码仓导入，导入后在图形化界面选择通道，
  再把消息字段拖到函数级画布中使用。
- 对 ACC/LKS 的直接影响：
  - 最终 CARLA 联调应走“节点级 + 通道/message”路线。
  - 当前算法画布搭建和局部验证仍优先走函数级/示波器/常量或局部参数路线。
  - 暂不把 CARLA 演示完全依赖示波器；先保留 Bridge/CARLA 画面验证，示波器只作为函数级或
    后续 message 观测补充。
  - ACC/LKS 参数短期优先用局部参数，等全局参数入口和 codegen 稳定后再决定是否迁移为全局参数。

**隔离版 GAASD 前端更新（2026-06-25）**：

- 已检查 `/home/aiden/文档/temp/GAASD_SETUP_20260624_2.tar.gz`：
  - 包结构与 20260623 前端包一致，包含 `gaasd_1_build-2.7.0.5.tar.gz`、
    `install.sh`、图标和 `sshpass_1.06-1_amd64.deb`。
  - 未包含 `gaasd_code_tools`，因此本次按“前端更新包”处理。
- 已将隔离版 GAASD 前端更新到：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/app`。
- 未修改：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/codeTools`、
  系统全局 `/opt/gaasd` 和 `/usr/bin/gaasd`。
- 更新前旧前端已备份到：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/rollback/frontend_before_20260624_2_20260625_095111/app`。
- 更新后校验：
  - `app/gaasd` SHA-256：
    `723e82279257afe8943fb188c00e4222ed6050dd5f5adfddd572ab16206ac45c`。
  - 与上一前端相比，主要差异集中在：
    `Generate.js/ts`、`ProjectComponent.js/ts`、`WsServer.js/ts`、
    `linuxEnv.js/ts` 和 `fe/assets` 前端构建产物。
  - 后端 codeTools 未变：
    `new_gaasd` SHA-256
    `8edcafd48215ca1c70521ccd5364e4dd92f1a4818e3ec9fdafa9c82e83037380`，
    `gaas_codegen` SHA-256
    `bc855304e5f625919d2802931bc0cae6482d44947cf7c0edccfbcf383f978d7f`，
    `codescan-cpp` SHA-256
    `26949ab22a1ca62e6560bcd87d0d3219a3601e25d3f009b617727819166bcc31`。
- 启动方式仍为：

```bash
/home/aiden/gaasd_versions/gaasd-2.7.0.5/run-gaasd-2.7.0.5.sh
```

**GAASD 内置 CARLA 仿真器启动适配（2026-06-25）**：

- 新版 GAASD 仿真管理页面的 CARLA 0.9.15 启动逻辑位于：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/app/resources/app/src/config/config.ts`。
- 源码中 `simulatorStartup()` 对 0.9.15 固定执行：
  `cd "${path}" && DISPLAY=:1 nohup ./CarlaUE4.sh -quality-level=Low ...`。
- 本机 CARLA 只能稳定通过
  `/home/aiden/snap/code/app/carla-package/start-carla.sh` 启动，因为该脚本修正了
  Vulkan/`LD_LIBRARY_PATH` 冲突并默认低画质启动。
- 已新增 GAASD 专用 CARLA 启动包装目录：
  `/home/aiden/gaasd_carla_launcher`。
- 该目录下的 `CarlaUE4.sh` 是 wrapper，不改原始 CARLA 包：

```bash
/home/aiden/gaasd_carla_launcher/CarlaUE4.sh
```

- wrapper 实际调用：

```bash
/home/aiden/snap/code/app/carla-package/start-carla.sh
```

- 因 GAASD 硬编码 `DISPLAY=:1`，而本机桌面实际为 `:0` 且只有
  `/tmp/.X11-unix/X0`，已在 wrapper 中强制覆盖：

```bash
export DISPLAY=":0"
export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
```

- GAASD 设置页中 CARLA 15 路径应填写绝对路径：

```text
/home/aiden/gaasd_carla_launcher
```

- 已验证：通过 GAASD 页面点击“启动仿真器”后，CARLA 窗口可正常打开。
- 注意：这只解决“启动仿真器”。“启动场景”仍走 GAASD 当前源码中的
  `simulationSceneBridgeStartup()`，会在运行中的 Docker 容器里查找：

```text
/opt/carla/carla_tools/scenario/load_scene_bridge.sh
```

- 当前已定位 `ubuntuEnv_0.0.5_v2.tar` 被 GAASD 保存到：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/hardwarePlatform/ubuntuEnv_0.0.5_v2.tar
```

- Docker 中存在镜像 `ubuntuenv_0.0.5_v2:env`，但该镜像当前没有
  `Cmd/Entrypoint/Env`，也不能直接用 `bash` 或 `/bin/sh` 运行；原始 tar 内部
  `repositories` 声明的真实标签为：

```text
docker.cbdes.cn:8080/cbdes/x86:pangu-2.0.5
```

- 后续若要测试 GAASD 团队的 bridge/场景脚本，应优先确认该 tar 是否应通过
  `docker load -i` 正确加载出原始镜像标签，或由团队提供包含
  `/opt/carla/carla_tools/scenario/load_scene_bridge.sh` 的容器启动方式。

---

## 当前状态（2026-06-17 更新）

**LKS 画布搭建进度（项目 `project/lks`，离线阶段一）**：

已按 `docs/GAASD_LKS_完整控制画布方案.md` 逐盒搭建并逐块核对（事实源
`project/lks/data/cbdes.db`）：

- ✅ **盒子1 LKSPreviewDistance**（复合组件）：算动态预瞄距离
  `ld = αL·(l0+rt·egoV)`，`αL=1−isCurve·(1−rAlpha)`。2 输入（egoV/curvature）、
  4 全局参数（lks_l0/lks_rt/lks_rAlpha/lks_curvatureThreshold）、1 输出
  previewDistance。11 块 16 连线全部核对正确；fabs（非 abs）、常量=1、全局参数绑定均确认。
- ✅ **盒子2 LanePolynomialEval**（复合组件）：Horner 求单点偏差
  `e(x)=((c3·x+c2)·x+c1)·x+c0`。5 输入（c0/c1/c2/c3/x）、1 输出 e。13 连线核对正确。
- ✅ **顶层组装完成**：A 块（6 常量 + egoV/curvature 接入）、B 块（X1_near=读全局参数
  lks_nearPreviewDistance、X2_mid=ld÷2、x3=ld 直接用）、C 块（3 个 Eval 实例
  Eval_P1/P2/P3 + 15 根输入连线，c0~c3 各扇出 3 路、x 分别接 X1_near/X2_mid/ld）。
- ✅ **盒子3 LKSErrorFusion**：`weightedError=0.2·e1+0.3·e2+0.5·e3`，3 输入、3 读参数
  (lks_w1/w2/w3)、3 乘 + 1 个多口加法、1 输出。核对正确（曾发现 w2 误绑 lks_w3，已修）。
- ✅ **盒子4 LKSSteerControl**（阶段一核心）：`thetaRaw=lks_Kp·weightedError`、
  `steerRad=thetaRaw·lks_steerScale`。theta0 启控捕获与横向加速度限幅(atan)留阶段二。
- ✅ **盒子5 LKSDecision**（最简版）：`controlEnabled = NOT(brakePressed OR
  driverSteerHigh OR NOT speedOk)`。3 输入(egoV/brakePressed/driverSteerNorm)、
  greater-equal/fabs/3口 logic-or/logic-not、1 输出。**不设总开关、不要转向灯、不输出三态**。
  用全局参数 lks_vMin=1.0、lks_driverSteerThreshold=0.1。
- ✅ **门控 + 示波器**：顶层 Gate 乘法 `lksSteerRad = controlEnabled · steerRad`
  （退出时转角清零），lksSteerRad/controlEnabled 接示波器。

**阶段一离线 LKS 画布全部搭完，30 条顶层连线端到端核对正确**（事实源
`project/lks/data/cbdes.db`，7 个复合盒子）。整链预期值：previewDistance=7.5、
e1=e2=e3=0.8、weightedError=0.8、steerRad=0.0384、controlEnabled=1、lksSteerRad=0.0384
（按 Kp=0.08 原始 Simulink 实测值；早先误用 0.025 算得 0.012，已更正）。

**信号语义约定（决策输入）**：brakePressed/driverSteerNorm/lksSwitch 等一律按
**电平/持续**语义——Bridge 每周期发当前值、adapter 缓存上一次值，故信号全周期有效，
画布决策每周期组合判一次、无需边沿检测或 latch；开关类的"边沿转电平"放 Bridge 端做。

**下一步**：
- 离线跑仿真验证上述数值（注意复合组件 codegen 可能仍卡，团队在修）。
- 阶段二：给 LKSSteerControl 加 theta0 启控捕获（局部状态 + 决策 controlEnabled 上升沿）
  和横向加速度限幅（atan/fmin/fmax，需新增 wheelBase/frontWheelMaxRad/ayMax 等参数）。

**已确认的关键事实**：

- GAASD 基础算子库实测（`gaasd-2.7.0.5/app/resources/app/preload/defaultComponent`）：
  运算/比较/逻辑类端口 `a/b→result`；一元 `logic-not/negate` 端口 `operand→result`；
  数学函数 `fabs/sqrt/atan/fmin/fmax` 等端口 `x(,y)→return`，**`atan` 存在**（无需新增
  AtanVal）；无 clamp 算子（用 `fmax(lo,fmin(val,hi))`）。已修进方案 §5.4。
- 参数承载：算法整定参数用**全局参数** `lks_*`（10 个，见方案 §8.3）；c0~c3 等模拟输入
  信号阶段一用**常量**。
- 全局参数值不存项目库（存 GUI/生成的 GlobalContext.json），无法从文件核验，靠 GUI 确认或验算。

**自定义边界组件（严格 C++ FuncModule，参照 `newaccpro2_component_sources.zip`）**：

- ✅ 已写 **CARLALKSLaneModel**：`generated/gaasd_lks_lane_components_src/`，一个模块产出
  c0/c1/c2/c3/curvature，含 adapterRc/valid 有效性保护 + Param 回退默认值（离线 valid=0
  时输出默认值 c0=0.8，在线 adapter 注入时输出实测值，**画布连线两阶段不变、永不重连**）。
  GCC9.4 + `-Wall -Wextra -Wpedantic -Werror` 编译通过。
- ⛔ **暂不使用**：GAASD 的 C++ FuncModule 扫描工具尚未更新，自定义组件 scan 不进去。
  因此 c0~c3 **当前仍用常量搭建**；待扫描工具更新后再用 CARLALKSLaneModel 替换常量
  （它的输出直接接 3 个 Eval 和 LKSPreviewDistance，替换无需改其他连线）。

**LKS 后续仍需的自定义组件（待扫描工具更新后做）**：

- egoV → 复用现有 `CARLAACCEgoSpeed`（严格版已有）。
- `CARLALKSCurrentSteer`（currentSteerNorm，阶段二 theta0 启控捕获用）—— 待写。
- `CARLALKSControlCmd`（转角输出到 Bridge）—— 现有 `generated/gaasd_lks_components_src`
  版本是**旧非严格写法**（run 里调 `carla_adapter_publish_control_cmd`），输出阶段需按严格标准重写。
- 决策输入边界（lksSwitchOn/brakePressed/turnSignalOn/driverSteerNorm）—— 阶段五再写。

**下一步**：

- 接完 C 块 15 根连线（c0~c3 四个常量各扇出到 3 个 Eval，x 分别接 X1_near/X2_mid/ld），
  验算 c0=0.8 时 e1=e2=e3=0.8。
- 然后搭盒子3 LKSErrorFusion（e1/e2/e3 加权 → weightedError）。

---

## 当前状态（2026-06-16 更新）

**GAASD 2.7.0.5 节点功能源码核查**：

- 已检查隔离版新版 GAASD：
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/app/resources/app` 与
  `/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server`。
- 前端默认组件已经包含 `app_empty`、`process_empty`、`c_process_empty`、
  `module_empty`，数据模型层面支持“应用-进程-模块”结构。
- `src/Action/Generate.ts` 已能按 `app/process/module/function` 等类型导出
  `icvos/blocks/*`，并为 `module` 写入 `node_name` 扩展字段。
- `src/webWorker/compileWorker/GaasdCompiler.ts` 已包含 Pangu 编译链路：
  复制模块到 `pangu_gaasd/modules/`，拷贝 `configs/node_module`，
  更新 `install/conf/node_module` 和 `app_module`，并通过 `launch.sh`/`run.sh`
  启动应用。
- `src/Action/WsServer.ts` 已包含本地/远程仿真场景启动、算法转发节点启动、
  工程 `launch.sh`/`stop_all.sh` 运行接口。
- 当前本机新版 `gaasd_server/codeTools` 目录没有 `new_gaasd`，而前端仍在
  `linuxEnv.ts`、`PanguFramework.ts`、`ComponentEdit.ts` 中调用
  `new_gaasd scan/submodule/submodule_gen_code`；同时当前目录也没有
  `gaasd_server/panguFramework/pangu_gaasd`。
- 因此当前安装状态下只能确认“节点功能的前端模型和运行入口已实现”，不能
  确认“节点级代码生成、编译、运行完整可用”。完整可用还依赖配套
  `new_gaasd` 与 `panguFramework/pangu_gaasd` 包导入并验证。
- 当前 `project/newaccpro2` 生成物仍是函数模型路径：主要落在
  `icvos/blocks/functions`、`icvos/src/functions` 和
  `icvos/src/oscilloscopeFunctions`，不是节点级 `modules/process/app`
  工程。因此现阶段 ACC 画布还没有接入新版节点运行链路。

**newaccpro2 规范组件源码对比**：

- 已对比团队提供的
  `project/accpro2/newaccpro2_component_sources.zip` 与本地
  `deliverables/newaccpro2_components_src_v2.zip`。
- 已确认 `project/accpro2/newaccpro2_component_sources.zip` 是 GAASD 团队确认的
  正式扫描输入，不应再修改其中 5 个 CARLA 边界组件。
- GAASD 团队确认：`@type atomic` 的 `run()` 内部允许按具体业务自由实现。
  该口径用于后续“adapter 相关代码是否能按组件标准改写”的讨论，不代表要
  覆盖已经确认的 `newaccpro2_component_sources.zip`。
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
- `adapter_cpp` 不属于 `newaccpro2_component_sources.zip` 这批正式扫描组件。
  如果软件团队希望“adapter 这块也按 atomic 原件标准改写”，应单独设计
  adapter/边界封装组件或运行时基础设施，不应回头修改已确认的 ACC 组件包。
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
- LKS 第一阶段画布暂不搭任何限幅，不使用 `atan`，先验证
  `previewDistance -> e1/e2/e3 -> weightedError -> steerRad` 主控制链。
  顶层预瞄点连接已明确为：`x1=nearPreviewDistance` 常量，
  `x2=previewDistance*0.5`，`x3=previewDistance`。
- LKS 比例增益默认采用原 Simulink 解析值 `Kp=0.08`；此前方案文档中的
  `0.025` 来源待复核，不作为第一阶段默认值。`Kp` 应作为可调参数保留。
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
