# GAASD 手动画布构建知识库与复用流程

> 目的：把本次 `newaccpro3` 成功重构形成的经验沉淀为可重复流程，使后续 ACC、LKS 或其他算法画布在 GAASD 持续升级时仍能高效、准确地构建。
>
> 本文区分两类信息：**稳定设计原则**可以长期复用；**组件版本、UUID、属性结构和代码生成行为**必须在每次 GAASD 更新后重新核对。

## 1. 长期复用入口

后续开始任何 GAASD 画布工作前，依次读取：

1. `WORKLOG.md`：当前项目状态、最近一次修改和待办事项。
2. 本文：构建方法、基础组件使用规则和数据库修改边界。
3. `docs/templates/GAASD_画布修改任务模板.md`：把本次修改目标、公式和保留项写清楚。
4. 对应算法方案文档：确认公式、信号语义、参数和状态，而不是根据现有连线猜测需求。

通用辅助工具：

```text
tools/gaasd_canvas_tool.py
```

专用重构脚本只处理已明确验证的工程和结构，例如：

```text
tools/refactor_newaccpro3_decision.py
```

通用工具负责审计、快照和版本对比，不负责自动修改画布。具体改图继续使用经过逐条审查的专用脚本，避免“万能脚本”误改工程。

## 2. 本次成功构建基线

### 2.1 工程和层级

当前 ACC 手动画布基线：

```text
project/newaccpro3
```

核心层级：

```text
MainFlow
├── Decision
│   ├── StateClassifier
│   ├── DecisionTable
│   ├── ControlStateUpdate
│   ├── TimeGapUpdate
│   └── MaxSpeedUpdate
└── Control
    ├── DesiredDistance
    ├── DistanceError
    ├── RelativeSpeed
    ├── DistanceFeedback
    ├── SpeedFeedback
    ├── RawTargetSpeed
    ├── NonNegativeTargetSpeed
    ├── TargetSpeedLimiter
    └── EnableGate
```

`DecisionTable` 直接保留在 `Decision` 层。它本身已经是一项完整、单一的物理职责，外面不再增加只包含一个真值表的空壳子模块。

### 2.2 当前控制关系

```text
desiredDistance = max(minDistance, egoV * timeGap)
distanceError = distance - desiredDistance
relativeSpeed = leadV - egoV
rawTargetSpeed = leadV + kDist * distanceError + kSpeed * relativeSpeed
targetSpeed = min(maxSpeed, max(0, rawTargetSpeed)) * enable
```

这条公式链决定了 `Control` 内部基础模块的物理命名和连线方向。后续调整布局或替换组件版本时，不得改变减法操作数顺序和三输入求和关系。

### 2.3 当前决策和状态关系

```text
isLowSpeed = egoV < vMin
enable = controlEnabled AND NOT isLowSpeed

turnOn = (decision == 5) OR (decision == 6)
cancel = (commandType == 6) OR (commandType == 7)
controlEnabledNext = (controlEnabled OR turnOn) AND NOT cancel
hasHistoryNext = hasHistory OR controlEnabledNext
```

三个量必须明确区分：

| 信号 | 含义 |
| --- | --- |
| `controlEnabled` | 跨周期保存的当前在控记忆，由 `read-local-state` 读取。 |
| `enable` | 本周期真正允许控制输出的有效使能；当前还受低速条件门控。 |
| `controlEnabledNext` | 根据本周期指令计算、由 `write-local-state` 写入的下一周期在控记忆。 |

真值表默认动作是 `R0/y=0`，表示本周期没有新决策动作。`R8/y=8` 表示进入待命、退出当前控制，不能再作为所有未匹配条件的默认动作。

### 2.4 可恢复依据

本次修改前完整备份：

```text
/home/aiden/文档/Modularization_backups/newaccpro3_before_decision_refactor_20260727_1725
```

本次分层重构脚本：

```text
tools/refactor_newaccpro3_decision.py
```

当前版本快照保存在：

```text
docs/gaasd_canvas_baselines/
```

其中环境指纹用于判断 GAASD 和组件库是否变化，审计结果用于判断画布结构是否变化，组件目录用于核对当前基础组件模板。

## 3. 哪些经验是稳定的

以下规则不依赖某个具体 GAASD 小版本：

1. 先确定公式、状态机、信号语义和物理层级，再拖模块和连线。
2. 一个复合模块只负责一类可以完整命名的物理职责。
3. 不为单个基础模块增加没有信息增量的空壳子模块。
4. 可调标定量使用复合模块局部参数，通过 `read-local-param` 读取；结构性固定编号才使用 `constant`。
5. 跨周期记忆使用局部状态，通过 `read-local-state` 读取当前值、`write-local-state` 写入下一周期值。
6. 局部参数和局部状态由定义它们的复合模块持有。需要传入下级子模块时通过端口传递，不在下级重复定义同名参数或状态。
7. 同层多个纯加法项优先使用一个可配置多输入 `add`，除非中间结果需要复用、观测或具有独立物理意义。
8. 模块实例名表达物理功能或输出物理量，不使用 `add_1`、`multiply_2`、`Module01` 等编号式名称。
9. 边界协议字段、复合模块端口和内部主信号尽量使用同一个简短名称，单位写在元数据中。
10. 数据库结构检查、GUI 属性检查、代码生成检查和闭环行为检查是四种不同层面的验证，不能互相替代。

## 4. 哪些信息会随版本变化

每次 GAASD、组件包、扫描器或代码生成器更新后，都必须重新检查：

1. 组件 `version`、`originId`、`isCustom` 和 `vendor`。
2. `extensionProps` 中 `base_config` 的字段结构。
3. 动态输入端口的 UUID、名称和生成规则。
4. `input/output` 边界端口的数据类型和 shape 表达。
5. `read/write-local-state`、`read-local-param` 的 `operateKeys` 格式。
6. 真值表的条件、动作、场景和输入输出端口格式。
7. 示波器的输入数量、仿真步长、仿真时长和属性面板行为。
8. 复合模块的参数、状态、实例名和代码生成签名。
9. GAASD 生成代码的目录、函数命名和 Pangu 构建入口。

当前环境已经说明了这个问题：组件库核心模板为 `1.2.0`，而成功画布仍包含 `1.0.6` 和 `1.1.7` 实例。版本不同本身不是逻辑错误，也不能通过直接改数据库版本字段来升级。

## 5. 基础组件知识库

### 5.1 边界和数据来源

| 组件 | 作用 | 关键配置 | 使用规则 |
| --- | --- | --- | --- |
| `input` | 定义复合模块输入端口 | `name/dataType/category/shape` | 应使用当前 GUI 新建实例；端口名称和物理量一致。 |
| `output` | 定义复合模块输出端口 | `name/dataType/category/shape/isReturnFlag` | 输出类型必须与上游信号一致。 |
| `constant` | 提供结构性固定值 | `name/dataType/dataValue` | 只用于决策编号、0、1 等不会标定的值。 |
| `read-local-param` | 读取当前复合模块的可调参数 | `operateKeys` | 每个实例只绑定一个字段；参数在父复合模块统一配置。 |
| `read-local-state` | 读取当前周期状态 | `operateKeys` | 与同字段 `write-local-state` 成对审查。 |
| `write-local-state` | 写入下一周期状态 | `operateKeys` | 写入线不是普通组合反馈，不得误接到本周期读取端。 |

### 5.2 数学运算

| 组件 | 数学语义 | 关键注意事项 |
| --- | --- | --- |
| `add` | 多项求和 | `inputNumber` 可配置；同层纯求和优先一次完成。 |
| `subtract` | `a-b` | 不满足交换律，修改布局或接线后必须核对操作数方向。 |
| `multiply` | 多项相乘 | `inputNumber` 可配置；门控可用“物理量 × 0/1”。 |
| `divide` | `a/b` | 检查分母零保护；不能只依赖正常场景数据。 |
| `fmax` | 取最大值 | 常用于下限保护，例如 `max(0, targetSpeed)`。 |
| `fmin` | 取最小值 | 常用于上限保护；上下限应按 `fmax` 后 `fmin` 的顺序串联。 |

物理命名示例：

| 不推荐 | 推荐 |
| --- | --- |
| `multiply_1` | `TimeGapDistance` |
| `subtract_2` | `DistanceError` |
| `multiply_3` | `DistanceFeedback` |
| `add_1` | `RawTargetSpeed` |
| `fmax_1` | `NonNegativeTargetSpeed` |

### 5.3 比较和逻辑

| 组件 | 作用 | 使用规则 |
| --- | --- | --- |
| `less-than/greater-than/equal` | 生成条件结果 | 名称应表达条件，例如 `IsLowSpeed`、`IsEngageDecision`。 |
| `logic-not` | 条件取反 | 名称表达取反后的物理含义，例如 `IsSpeedValid`。 |
| `logic-and/logic-or` | 组合条件 | 可配置输入数量；同一物理条件组优先一次合并。 |

比较和逻辑输出在当前设计中作为 0/1 使用。与 `double` 物理量相乘进行门控前，仍需检查生成代码中的类型转换是否正确。

### 5.4 真值表

`truth-table` 由三部分组成：

```text
conditions  条件表达式
actions     输出动作
scenarios   条件匹配矩阵与动作映射
```

使用规则：

1. 每个场景必须覆盖当前全部条件 ID。
2. `condition_matching` 只使用 `1/0/-1`，分别表示真、假、不关心。
3. 每个场景引用的 action 必须存在。
4. 应有且只有一个全 `-1` 的默认场景。
5. ACC 当前默认场景必须输出 `R0/y=0`。
6. 真值表输出是本周期决策事件，不等同于持续的系统状态。
7. 替换组件版本时必须整体迁移 conditions、actions、scenarios 和端口，不能只复制显示内容。

### 5.5 示波器

`oscilloscope` 的当前关键配置为：

```text
inputNumber
simulateStep
simulateTotalTime
```

示波器属于高风险动态组件。旧实例即使仍能显示，也可能无法打开新版属性面板。更新后优先由用户在当前 GUI 中新拖入、配置，再迁移已有观测线，不从旧实例克隆。

### 5.6 复合模块

复合模块用于表达物理职责，不是单纯为了减少顶层节点数。是否拆分主要看：

1. 能否用一个准确名称描述模块职责。
2. 是否具有相对稳定的输入输出契约。
3. 是否能隐藏一组内部实现细节。
4. 是否便于单独检查、测试或调参。

如果内部只有一个真值表、一个加法或一个比较块，且没有独立契约或复用价值，通常不应额外包一层。

## 6. 参数、状态和本周期信号

| 类别 | 示例 | 生命周期 | 画布实现 |
| --- | --- | --- | --- |
| 可调参数 | `kDist`、`kSpeed`、`vMin`、`gapStep` | 配置期间保持，人工调节 | 复合模块 Param + `read-local-param` |
| 跨周期状态 | `controlEnabled`、`hasHistory`、`timeGap`、`maxSpeed` | 每个仿真周期更新 | State + `read/write-local-state` |
| 本周期中间量 | `distanceError`、`relativeSpeed` | 当前调用内有效 | 普通基础模块连线 |
| 结构性常量 | R1~R8 编号、0、1 | 算法结构固定 | `constant` |

判断方法：需要人工标定的是参数，需要记住上一周期的是状态，只用于当前计算的是普通信号。

## 7. 布局和命名标准

### 7.1 布局

1. 信号总体从左向右流动：输入在左、输出在右。
2. 同一公式链放在同一水平带，按计算顺序排列。
3. 参数读取放在对应使用链附近的上方或下方，不跨越整个画布连线。
4. 状态读取靠近状态消费者，状态写入放在该更新链末端。
5. 同一信号多路分发时保留清晰主干，避免多根长距离平行线。
6. 复合模块顶层只显示对外契约，不把内部中间量无必要地暴露到父层。
7. 不允许组件矩形重叠；连线交叉过多时优先调整层级，而不是只拉长画布。

### 7.2 命名

1. 复合模块使用职责名：`StateClassifier`、`TimeGapUpdate`。
2. 计算块使用输出物理量或动作名：`DistanceError`、`ApplyMaxGap`。
3. 条件块以 `Is/Has/Can/No/Next` 表意：`IsLowSpeed`、`HasCancelRequest`。
4. 边界端口使用简短小驼峰：`egoV`、`targetSpeed`、`commandType`。
5. 单位保存在接口说明中，不在名称中附加 `_mps`、`_rad`，除非外部正式协议已经如此规定。

## 8. 数据库事实源和安全修改边界

### 8.1 当前事实源

```text
project/<工程>/data/cbdes.db    持久化画布事实源
project/<工程>/data/temp.db     当前工程同步副本
project/<工程>/data/runtimeCanvas.db  运行期数据，不默认覆盖
```

### 8.2 可以由专用脚本处理的内容

在 GAASD 完全关闭且已有完整工程备份时，可以处理：

1. 已有组件的显示名和描述。
2. 已有组件的坐标、大小和层级位置。
3. 保留原组件 ID 和端口 UUID 的连线迁移。
4. 已明确掌握模板结构后创建复合模块边界。
5. 删除已经证明无子节点、无入边、无出边的孤立节点。

### 8.3 不能盲目处理的内容

1. 只修改 `version/isCustom` 伪装成新版组件。
2. 从旧版实例克隆动态组件并假设属性面板可用。
3. 猜测真值表、示波器、状态组件或动态端口的 UUID。
4. 按模糊名称批量删除节点。
5. 未核对用途就覆盖 `runtimeCanvas.db`。
6. GAASD 正在运行时直接复制整个工程或写数据库。

## 9. 标准构建流程

### 阶段 A：冻结需求

先形成以下四张表：

1. 输入输出信号表：名称、类型、单位、方向、有效语义。
2. 参数表：名称、默认值、范围、所属复合模块。
3. 状态表：名称、初值、更新公式、读写周期。
4. 公式和决策表：每个中间物理量的定义和操作数顺序。

### 阶段 B：记录环境

```bash
python3 tools/gaasd_canvas_tool.py fingerprint \
  --project project/newaccpro3 \
  --output /tmp/newaccpro3_before.json

python3 tools/gaasd_canvas_tool.py catalog \
  --output /tmp/gaasd_core_components.md
```

这一步确认当前前端构建号、组件库哈希、核心组件版本和工程数据库哈希。

### 阶段 C：建立可恢复快照

先完全退出 GAASD，再执行：

```bash
python3 tools/gaasd_canvas_tool.py snapshot \
  project/newaccpro3 \
  --label before_control_refactor
```

工具默认拒绝在 GAASD 运行时复制完整工程。

### 阶段 D：优先由 GUI 创建高风险组件

以下组件在版本变化后优先由用户从当前组件库拖入并完成属性配置：

```text
input / output
truth-table
oscilloscope
read/write-local-state
read-local-param
动态输入数量的 add / multiply / logic-and / logic-or
```

专用脚本随后可以安全处理已创建实例的布局和接线。

### 阶段 E：按物理子系统增量构建

推荐顺序：

```text
边界端口
-> 纯组合计算
-> 参数读取
-> 状态读取和观测
-> 单个状态写回
-> 真值表
-> 其余状态写回
-> 示波器
```

不要一次新增多个复杂状态环和真值表后再统一排错。

### 阶段 F：结构审计

```bash
python3 tools/gaasd_canvas_tool.py audit project/newaccpro3
```

当前审计项目包括：

```text
SQLite 完整性
连接端点存在性
子系统边界一致性
单输入唯一驱动
组合环
节点布局重叠
动态端口数量
边界、常量、参数和状态配置
真值表引用和默认场景
cbdes.db / temp.db 一致性
当前组件库模板差异
```

组件模板差异是版本提示，不自动判定为结构错误。

### 阶段 G：GUI 验证

数据库审计通过后，仍需在 GAASD 中逐项检查：

1. 工程能正常打开。
2. 每一级复合模块都能进入和返回。
3. 输入输出端口显示、类型和顺序正确。
4. 参数、状态、真值表和示波器属性面板能打开。
5. 拖动和保存后重新打开，结构不丢失。

### 阶段 H：代码和行为验证

```text
生成代码
-> 检查生成目录和函数签名
-> 编译
-> 固定输入计算对照
-> mock 通信
-> Pangu 运行
-> CARLA 闭环
```

每层通过后再进入下一层，不能用 CARLA 车辆“看起来能动”替代公式和状态机验证。

### 阶段 I：保存成功基线

成功后保存：

1. 完整工程快照。
2. 环境指纹 JSON。
3. 画布审计 JSON。
4. 组件目录快照。
5. 专用修改脚本和脚本参数。
6. GUI、代码生成、编译和闭环验证结果。
7. `WORKLOG.md` 中的本次结论和下一步。

## 10. GAASD 更新后的处理方式

更新前保存指纹，更新后执行：

```bash
python3 tools/gaasd_canvas_tool.py compare \
  docs/gaasd_canvas_baselines/20260728_newaccpro3_environment.json \
  --project project/newaccpro3
```

根据差异分三类处理：

| 变化类型 | 处理方式 |
| --- | --- |
| 只有前端构建号变化，组件库和工程哈希不变 | 打开工程并做 GUI 冒烟检查。 |
| 组件版本变化，但端口和配置结构不变 | 建立小样本工程验证属性面板和代码生成，再决定是否迁移。 |
| `originId`、端口 UUID、`extensionProps` 或生成代码结构变化 | 视为结构升级，不运行旧版数据库修改脚本；先更新知识库和脚本模板。 |

更新后建议先建立一个最小“哨兵画布”，包含：

```text
input / output
constant
三输入 add
可配置 multiply 或 logic-and
read/write-local-state
read-local-param
truth-table
oscilloscope
一个 composite-block
```

只有哨兵画布的属性面板、保存、代码生成和编译通过后，才在正式 ACC/LKS 工程迁移组件。

## 11. 提高后续协作效率的方法

后续提出画布修改任务时，至少给出：

1. 目标工程路径和目标层级。
2. 本次只修改什么、明确不能修改什么。
3. 公式、决策表或状态转移关系。
4. 需要保留的外部端口、参数、状态和观察量。
5. 当前 GAASD 是否已经完全退出。
6. 是否已经从当前组件库新拖入动态组件。
7. 期望验证到哪一级：数据库、GUI、代码生成、编译或 CARLA。

有了这些信息，我可以按固定流程先审计和备份，再生成针对本次工程的专用迁移脚本，并自动完成结构检查；需要人工确认的部分只剩 GUI 属性和最终视觉布局，效率和准确性都会明显高于每次重新探索数据库结构。

## 12. 当前仍需完成的验证

本次 `newaccpro3 Decision` 分层已经通过数据库结构审计和展平连线等价检查，但以下步骤仍需在 GAASD 中完成后，才能称为完整可运行基线：

1. 逐级打开 `StateClassifier / ControlStateUpdate / TimeGapUpdate / MaxSpeedUpdate`。
2. 检查边界端口、真值表、参数和状态属性面板。
3. 保存并重新打开工程。
4. 重新生成代码并检查生成结果。
5. 编译并执行固定输入和 CARLA 闭环复测。

