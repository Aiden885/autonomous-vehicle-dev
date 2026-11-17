# FuncModule C++ 代码生成规范

**版本**：v3.8
**更新日期**：2026-11-16
**适用范围**：任意功能描述 → 符合规范的 FuncModule C++ 组件代码  
**规范依据**：`C++代码规范.md`（v1.0.35，CTOR/RUN/BODY/FILE/ENG/META 规则的唯一权威来源）

> **文档分工与项目适配**：本文面向 AI 执行 MBD 改写，集中规定生成步骤、代码模板、具体写法与检查项。开发人员理解模块职责、数据边界和评审要求时，配套阅读[《MBD 改写规范（开发人员阅读版）》](C++面向过程实现与MBD改写规范（QingMbdExample版）.md)。本目录中的底层规则文件为[《C++代码规范.md》](C++代码规范.md)。
>
> **QingMbdExample 项目执行要求**：在该项目中使用本指南时，同时应用开发人员阅读版的项目约束：沿用 `include/mbd`、`src/mbd` 和 `qing_mbd`；`block` 的 `run()` 有效代码不超过 120 行，`atomic` 的 `run()` 可以超过 120 行。每个模块独立管理自己的 Param/State；父模块只能通过子模块 Input 传入本次执行的业务数据，通过 Output 获取业务结果，禁止动态配置子模块、注入或回传子模块内部状态，也禁止通过接口包装绕过边界。原算法存在这类交互时必须重构职责和数据归属，不能作为例外保留，无法合规且保持行为时应记录改写受阻。命名、排版和注释遵守开发人员阅读版第 8 节。这些项目要求优先于下文通用示例和例外；其余具体代码生成规则仍按本指南执行。

> **v3.8 新增硬约束：component/block 的 run() 禁止 return**：当模块 `type` 为 `component` 或 `block`（如 `@type component`、`@type block`）时，`run()` 函数体内禁止使用任何 `return` 语句，包括分支中的提前返回和函数末尾的 `return;`。函数必须通过结构化控制流程自然执行至结束。本条覆盖本指南所有模板与示例，且适用于有无子模块的全部 `block`；不将此项禁令扩大到 `atomic`、`element` 或测试入口 `main()`。原有提前返回必须改写为条件执行与分支合流，保持输出、状态更新、循环退出语义及子模块调用条件和顺序，不能仅删除 `return`。具体写法与检查项见第五步和生成检查清单。

> **v3.5 附加硬约束**（覆盖本规范所有模板与示例，冲突时以本节为准）：
> 1. **形态可人为指定，且人为指定优先**：调用方显式声明「原件 / 组件」时，直接采纳该判定，不再用行数阈值反推。
> 2. **类型别名统一在 `FuncModule.hpp`**：`Real`、`Integer`、`UInteger` 等基础别名只在 `FuncModule.hpp` 中定义一次，其余头文件包含后直接使用，不再重复定义。
> 3. **头文件禁止宏定义与 `constexpr`**：`.hpp` 中不出现 `#define` 常量/函数宏，也不使用 `constexpr`；固定数值改用 `Param` 默认值或 `const` 局部量表达（`#pragma once` 属编译指示，允许）。
> 4. **字段尽量为基础数据类型，避免结构体嵌套**：五元结构字段优先展平为 `Real`/`Integer` 等标量；确需复用业务结构体时才嵌套，并在检查清单说明理由。
> 5. **限制 C++ 容器，固定长度 C 数组按维转换**（v3.5）：不使用 `std::vector`/`std::map` 等容器承载业务数据流。一维 `T values[N]` 转换为一个等长的一维 `std::array<T, N>`；二维及以上 `T array[D0]...[Dn]` 必须将除最后一维外的固定索引展开到字段名，生成 `D0 × ... × D(n-1)` 个长度为 `Dn` 的一维 `std::array`。禁止嵌套 `std::array`、禁止展平为单个长数组，也禁止用指针表、引用表、辅助选择函数或其他容器重新组合。类型定义与 `run()`、父子端口、State、测试中的全部读取、写入、初始化、清零、复制、比较、循环和映射必须同步改写；数组只允许下标访问，不调用任何 `std::array` 成员方法。
> 6. **同一功能统一存放一个目录**：功能目录含 `include/`、`src/`、`test/`、`CMakeLists.txt`；同一功能的所有子模块头文件统一放在 `include/` 下，所有子模块源文件统一放在 `src/` 下，不为每个子模块单独建立子目录。`test/` 下测试文件遵守本规范：`test_类名.cpp` 只含 `run()`，`main()` 统一写在 `test/main.cpp`，不追加到任何 `test_类名.cpp`，也不生成独立的 `main_类名.cpp`；测试逻辑封装在测试模块的 `run()` 内、用固定值构造输入。
> 7. **命名空间按业务指定**：`namespace` 依具体业务确定，同一输出工程内保持统一；本规范示例统一用 `apa` 仅作占位。
> 8. **数值字面量禁止携带类型后缀**（BODY-036）：`.hpp` Traits 字段默认值与 `run()` 函数体内的所有数值字面量，禁止添加 `f`/`F`（float）、`u`/`U`（unsigned）、`l`/`L`（long）等后缀。类型已由 `Real`/`Integer`/`UInteger`/`Boolean` 别名承载，后缀既冗余又与别名单一来源原则相悖：写 `Real x = 0.0;`、`Integer n = 0;`，而非 `0.0f`/`0U`。

---

## 本规范解决什么问题

**输入**：一段功能描述（自然语言算法说明、MATLAB/C 源函数、接口清单，或既有超长 cpp 文件）。

**输出**：一套符合 `C++代码规范.md` 的 FuncModule 组件代码——`Traits` 五元结构、继承 `FuncModule<Traits>` 的类、固定签名 `run()` 实现、必要的子模块拆分和 `Types.hpp`。

**核心承诺**：无论功能描述来自哪种形态，按本规范的流程都能产出：
- 每个 `.cpp` 只实现一个 `run()`（RUN-003），不含其他业务函数（RUN-004）；
- `run()` 函数体采用 SSA 式显式赋值、具名局部变量、白名单语法（BODY-003 至 BODY-107）；
- `block` 的 `run()` 超过 120 行有效代码时拆分为子模块，通过 `sub_.子模块名.run(...)` 级联；`atomic` 的 `run()` 可超过 120 行；
- 命名空间、文件组织、注释字段完整（FILE-*、ENG-*、CMT-*）。

> 若目标工程使用较新的 `@type element|component` 约定（见 `prompts/C++20-FuncModule组件生成规范.md` v2.2.0，`namespace control`、`models/*.json` 蓝图），请以那份规范为准；本规范面向当前 apa 工程实际采用的 `@type block` 约定，命名空间按业务指定（示例统一用 `apa` 占位）。采用该约定时，`component` 的 `run()` 仍须遵守本指南 v3.8 的禁用 `return` 约束。

---

## 强制参考：GeoCircle2lineR 的 hpp/cpp 结构与风格

本规范末尾的 **「参考案例：GeoCircle2lineR」是组件代码的强制参考实现，不是可选示意图**。生成任何含 `Sub` 的组件时，必须以该案例的 hpp 与 cpp 为直接模板；只允许按目标源码替换类名、命名空间、字段、子模块数量、子模块端口和业务计算，不能自行发明不同的文件组织、构造方式或 `run()` 编排。

- **hpp 必须参考案例**：直接包含 `FuncModule.hpp` 与每个子模块头文件；定义完整五元 `Traits`；在 `Sub` 中按首次调用顺序声明值语义子模块；主模块在 hpp 中使用案例相同的固定构造结构，即 `FuncModule{Sub{Child(ChildTraits::Param{...}, ChildTraits::State{...})}, std::move(param), std::move(state)}`。每个子模块都显式传入其对应的 `Param` 与 `State`，即使两者为空。
- **cpp 必须参考案例**：只包含主模块自身 hpp；在统一业务命名空间中保留紧邻 `run()` 的 Doxygen；主模块 cpp 只实现一个 `run()`；按照案例的“构造子模块输入 -> 声明子模块输出 -> `sub_.child.run(...)` -> 读取下游输出 -> 顶层输出映射”顺序展开。每个子模块调用均使用独立、具名的 `Traits::Input` 与 `Traits::Output` 变量。
- **不得用等价但不同形态的写法替代案例**：不得将子模块实例放入 `run()`，不得省略子模块 `Param`/`State` 实参，不得以辅助函数、聚合实参、局部临时子模块、不同的构造基类写法或压缩的调用链替代案例结构。业务语义需要的分支、状态和额外子模块必须在该结构内扩展。

原件与叶子子模块没有 `Sub` 时，仍按本规范的无子模块模板生成；一旦模块包含子模块实例，以上 GeoCircle2lineR 参考要求必须满足。

---

## 📚 目录

1. [核心原则](#核心原则)
2. [第一步：判定模块形态](#第一步判定模块形态)
3. [第二步：设计五元 Traits](#第二步设计五元-traits)
4. [第三步（可选）：拆分子模块并设计数据流](#第三步可选拆分子模块并设计数据流)
5. [第四步：生成文件骨架](#第四步生成文件骨架)
6. [第五步：编写 run() 函数体](#第五步编写-run-函数体)
7. [固定长度 C 数组转换规则（v3.5）](#固定长度-c-数组转换规则v35)
8. [Types.hpp 使用规则](#typeshpp-使用规则)
9. [test 目录与测试文件规则](#test-目录与测试文件规则)
10. [常见问题](#常见问题)
11. [生成检查清单](#生成检查清单)
12. [参考案例：GeoCircle2lineR](#参考案例geocircle2liner)

---

## 核心原则

无论功能描述如何，生成代码都必须满足以下原则：

### 1. 五元正交
每个模块拥有一个 `XxxxTraits`，且**有且仅有** `Input`、`Output`、`Param`、`State`、`Sub` 五个嵌套结构。字段按业务角色归位：外部周期输入→`Input`，本周期结果→`Output`，静态标定→`Param`，跨周期记忆→`State`，级联子模块实体→`Sub`。空则保留空结构 `struct X {};`。
字段**尽量为基础数据类型**（`Real`/`Integer`/`UInteger` 等标量），最大程度避免结构体嵌套结构体；确需复用业务结构体时才嵌套，并说明理由（v3.1 约束④）。

### 2. 职责单一
每个模块（或子模块）负责一个明确功能。`block` 的 `run()` 有效代码超过 120 行时，必须按职责拆分为子模块；`atomic` 的 `run()` 可超过 120 行，仍需保持职责明确。**调用方可人为指定形态（原件/组件）；形态与扫描方式分别判断，本项目人为指定形态不能豁免 block 的行数上限。**

### 3. 数据流清晰
模块间只通过 `Input`/`Output` 传值，呈现 `Input → Sub1 → Sub2 → … → Output` 的流水线。子模块之间不共享隐式状态，所有依赖走显式接口。

### 4. 规范优先
`run()` 函数体严格遵守：RUN-003（一个 run）、RUN-004（无其他业务函数）、BODY-003（SSA 显式赋值）、BODY-023（`sub_.xxx.run()` 调用）、BODY-104（禁三目）、BODY-105/106（禁位运算/switch/lambda 等白名单外写法）。

### 5. Types.hpp 最小化
只在需要 `Input/Output/Param/State/Sub` 之外的业务结构体时创建 `XxxxTypes.hpp`；仅含基础类型（`Real`/`Integer` 等）时直接在 `.hpp` 中定义 `Traits`。

### 6. 类型别名单一来源
`Real`、`Integer`、`UInteger` 等基础别名**只在 `FuncModule.hpp` 中定义一次**，其余头文件包含后直接使用，任何模块头文件与 `Types.hpp` 不得重复定义（v3.1 约束②）。

### 7. 头文件洁净
`.hpp` 中**禁止宏定义（`#define`）与 `constexpr`**：固定数值用 `Param` 默认值或函数体内 `const` 局部量表达；定长容量用 `Param` 中的 `Integer` 字段承载（v3.1 约束③）。`#pragma once` 属编译指示，允许保留。

### 8. 受限容器 / 基础类型优先
不使用 `std::vector`/`std::map` 等容器承载业务数据流。`std::array` 仅用于原始固定长度 C 数组转换：一维数组转换为一个等长的一维 `std::array`；二维及以上数组按 v3.5 规则拆分为多个仅保留最后一维的一维 `std::array`，前导索引按原维度顺序进入字段名。禁止嵌套数组、单个长数组展平、指针/引用选择表和选择辅助函数。转换后只使用 `array[index]`，不调用 `.at()`、`.size()`、`.data()`、`.front()`、`.back()`、`.begin()`、`.end()`、`.cbegin()`、`.cend()`、`.fill()`、`.swap()` 等任何成员方法。原始代码没有 C 数组时，不得新增数组或把编号标量字段主动合并为 `std::array`（v3.5 约束⑤）。

### 9. 模块工程目录
同一功能统一存放一个目录，含 `include/`、`src/`、`test/`、`CMakeLists.txt`；同一功能下所有子模块的头文件汇集在 `include/`、源文件汇集在 `src/`，不为每个子模块单独建立子目录（v3.1 约束⑥）。`test/` 下测试文件同样合规：`test_类名.cpp` 只含 `run()`，`main()` 统一写在 `test/main.cpp`，不追加到任何 `test_类名.cpp`，也不生成独立的 `main_类名.cpp`；测试逻辑封装在测试模块的 `run()` 内。命名空间按业务指定、工程内统一（v3.1 约束⑦）。

---

## 第一步：判定模块形态

判断该模块是**原件**（单体算法）还是**组件**（调度子模块）。

### 判定优先级

1. **人为指定优先**：若调用方/需求已显式声明「本模块为原件」或「本模块为组件（含哪些子模块）」，**直接采纳**，跳过下面的行数推断。人为指定只决定「原件 / 组件」形态，不豁免 `block` 的 120 行上限。例如可显式要求某 80 行算法拆成组件；某 150 行流程若整体保留为原件，必须符合 `atomic` 的适用条件并标记 `@type atomic`，不能继续作为 `block`。
2. **无人为指定时**，再按下表依据 `run()` 的有效代码规模与结构自动判定。

### 自动判定依据（按 run() 的有效代码规模与结构）

| 情形 | 形态 | 处理 |
|------|------|------|
| 算法自成一体，`run()` 有效代码 ≤ 120 行，不调度其他模块 | 原件 | 单个 hpp+cpp，`Sub` 为空 |
| `block` 的 `run()` 有效代码 > 120 行 | 组件 | 拆分为多个子模块，主模块只做协调 |
| 描述中天然包含多个可分离阶段（预处理→计算→输出映射） | 组件 | 按阶段拆分子模块 |
| 描述中包含大循环体（循环内逻辑独立且可复用） | 组件 | 循环体提为子模块，主模块循环调用 |

**有效代码行统计口径**（用于判断是否超 120 行）：不计空行、纯注释行、块注释内部行；计入变量声明、赋值、条件、循环、函数调用、输出写入、状态读写、子模块调用。行尾带注释的代码行按 1 行计。

### 规模→子模块数量参考

| 预估 run() 规模 | 建议子模块数 |
|----------------|-------------|
| ≤ 120 行 | 0（单原件） |
| 120–300 行 | 2–3 |
| 300–600 行 | 3–4 |
| 600–1000 行 | 4–6 |
| 1000+ 行 | 6+ |

> 拆分数量以「每个 block 子模块 `run()` ≤ 120 行且职责单一」为收敛目标，不是为了凑拓扑数量；atomic 不受该行数上限约束。若某段逻辑本就短小且不可复用，不必强行独立成子模块。**该表仅在无人为指定时作参考**；本项目指定拆分方案仍须满足对应扫描方式的要求。

---

## 第二步：设计五元 Traits

从功能描述提取五元要素。这是接口契约，图形化建模工具据此确定节点边界、端口、参数、状态和拓扑依赖。

### 提取映射表

| 功能描述中的成分 | 归入 | 说明 |
|-----------------|------|------|
| 外部周期性输入、控制流入口 | `Input` | 每周期刷新的量 |
| 计算结果、可观察输出 | `Output` | 本周期产生 |
| 阈值、系数、标定常量、限幅 | `Param` | 初始化时确定，运行期基本不变 |
| 积分累加、上一帧缓存、跨周期记忆 | `State` | 无则 `struct State {};` |
| 级联子模块实例 | `Sub` | 无则 `struct Sub {};` |

### 字段级要求

- 每个字段带默认值和 `///<` 业务注释（含单位/角色），禁止「输入值」「占位字段」这类无法区分语义的描述（CMT 系列 + BODY-007/008）。
- **字段类型优先为基础数据类型**：`Real`、`Integer`（=int）、`UInteger`。这些别名统一来自 `FuncModule.hpp`（v3.1 约束②），头文件包含后直接使用，不重复定义。
- **尽量展平，避免嵌套结构体**（v3.1 约束④）：能用标量表达的字段就不要包一层 `struct`。例如需要一条线段，直接写 `Real line0X1/line0Y1/line0X2/line0Y2`，而不是内嵌一个 `Segment` 结构；确需复用工程公共结构体（如 `ApaPathPointSet`）时才嵌套，并在检查清单注明复用理由。
- **限制 C++ 容器，固定长度 C 数组按维转换**（v3.5 约束⑤）：不在 Traits 字段中使用 `std::vector`/`std::map`。一维固定长度 C 数组转换为等长的一维 `std::array`；二维及以上数组将除最后一维外的索引展开为多个字段，每个字段是长度等于最后一维的一维 `std::array`。声明处显式包含 `<array>`，只使用下标访问。拆分字段必须保留在原 `Input`/`Output`/`Param`/`State` 或业务 Types 角色中并注明原索引；原始编号字段仍保留编号标量 + `count`。
- 不要把状态误放进 `Input`，不要把配置放进局部变量，不要把子模块当临时对象。

### Traits 模板

```cpp
/**
 * @brief XxxxTraits 是 Xxxx 模块的接口结构集合。
 */
struct XxxxTraits {
  struct Input {
    Real vstX = 0.0;   ///< 起点世界 X 坐标 (m)
    Real vstY = 0.0;   ///< 起点世界 Y 坐标 (m)
  };

  struct Output {
    Real arcLen = 0.0; ///< 圆弧长度 (m)
  };

  struct Param {};     ///< 无静态配置时保留空结构
  struct State {};     ///< 无跨周期状态时保留空结构
  struct Sub {};       ///< 无子模块时保留空结构
};
```

---

## 第三步（可选）：拆分子模块并设计数据流

仅当第一步判定为**组件**时执行。原件跳过本步。

### 1. 划分子模块边界

按功能描述中的自然阶段切分，保持每段逻辑完整、职责单一。例如一个几何规划算法：

```
阶段A：坐标转换 / 航向规范化 / 参数预计算   → SubPrep
阶段B：核心几何量计算（圆心、端点）        → SubCircle
阶段C：结果参数计算 + 输出映射             → SubArc
```

### 2. 确定数据流与端口

对每个子模块，列出它消费什么、产出什么、流向谁：

| 子模块 | 输入（来源） | 输出（去向） |
|--------|-------------|-------------|
| SubPrep | input.vstX/vstY/…（主 Input） | lineA1/B1/C1（→SubCircle、SubArc） |
| SubCircle | prep_out.lineA1/…、input.radius | edPtX/edPtY、cptX/cptY（→SubArc、Output） |
| SubArc | circle_out.edPtX/…、prep_out.cosVtgt | arcLen、arcAngle（→Output） |

数据流原则：
- 主 `Input` 只在需要它的子模块上出现；上游子模块的 `Output` 逐字段喂给下游子模块的 `Input`。
- 每个主 `Output` 字段最终由某个子模块 `Output` 或局部量显式写出。
- 原接口含二维及以上固定长度数组时，父模块和每个相关子模块端口都按 v3.5 规则拆分；父模块 `run()` 必须映射全部一维字段，不得遗漏、交换或重新组合为嵌套/长数组。整体一维数组赋值若不符合 BODY/SSA 规则，则逐元素下标复制。

---

## 第四步：生成文件骨架

按标准工程目录组织（ENG-001）。**同一功能统一存放一个目录**，含 `include/`、`src/`、`test/`、`CMakeLists.txt`（v3.1 约束⑥）。组件与其所有子模块共用同一目录的 `include/` 和 `src/`：

```
Xxxx/                          # 功能目录（组件 + 子模块共用）
├── include/
│   ├── FuncModule.hpp         # 固定框架文件，定义 Real/Integer 等别名（唯一来源）
│   ├── Xxxx.hpp               # 主模块：Traits + 类声明 + 构造函数实现（显式赋值）
│   ├── XxxxTypes.hpp          # 仅当存在必须复用的业务结构体时
│   ├── SubA.hpp               # 子模块头（与主模块共用 include/）
│   └── SubB.hpp
├── src/
│   ├── Xxxx.cpp               # 主模块 run()
│   ├── SubA.cpp               # 子模块 run()（与主模块共用 src/）
│   └── SubB.cpp
├── test/
│   ├── test_Xxxx.cpp          # 主模块测试（可选）
│   ├── test_SubA.cpp          # 子模块测试（可选）
│   └── test_SubB.cpp
└── CMakeLists.txt
```

> 子模块与主模块共用同一工程目录的 `include/` 和 `src/`；不为每个子模块单独建立子目录。`.hpp` 引用直接写文件名，无需路径前缀。

### include / 命名空间规则（FILE-*）

- 模块头文件 `类名.hpp`，源文件 `类名.cpp`，业务类型 `类名Types.hpp`，测试文件 `test_类名.cpp`。
- `.cpp` 只直接包含自己的同名 `.hpp`（若需 `<cmath>` 等标准库，按工程既有惯例引入）。
- `.hpp` 只包含自定义 `.hpp` 和标准库头文件，直接写文件名，不用相对/绝对路径，不含第三方库头文件。原始固定长度 C 数组转换为 `std::array` 时，声明所在头文件必须显式包含 `<array>`。
- **`.hpp` 内禁止 `#define` 宏与 `constexpr`**（v3.1 约束③）：固定容量/阈值放入 `Param` 默认值，用 `Integer`/`Real` 字段承载；`#pragma once` 允许保留。
- **`Real`/`Integer`/`UInteger` 等别名只在 `FuncModule.hpp` 定义**（v3.1 约束②），其余头文件包含 `FuncModule.hpp`（或包含它的公共头）后直接使用。
- 所有业务代码位于**按业务指定的统一命名空间**（v3.1 约束⑦），本规范示例用 `namespace apa` 占位。

### 固定模板：无子模块实例模块（原件形态，也适用于叶子子模块）

```cpp
#pragma once
#include "FuncModule.hpp"   // 仅标量字段时，别名 Real/Integer 均来自此文件

namespace apa {

/**
 * @brief SubATraits 是子模块 SubA 的接口结构集合。
 */
struct SubATraits {
  struct Input {
    Real var1 = 0.0;   ///< 变量1说明 (单位)
    Real var2 = 0.0;   ///< 变量2说明 (单位)
  };
  struct Output {
    Real result1 = 0.0; ///< 结果1说明 (单位)
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

/**
 * @brief SubA 是 SubA 子模块。
 */
class SubA : public FuncModule<SubATraits> {
 public:
  using FuncModule::FuncModule;   // 无子模块，引入基类构造函数
  void run(const Input& input, Output& output) override;
};

}  // namespace apa
```

> 仅当字段需复用工程公共业务结构体（如 `ApaPathPointSet`）时，才额外 `#include "ApaPathTypes.hpp"`；否则只包含 `FuncModule.hpp`。别名不在本文件重复定义（v3.1 约束②）。

无子模块实例模块的构造写法到此为止：只能使用 `using FuncModule::FuncModule;`。不得显式编写空构造函数、只转发 `Param`/`State` 的构造函数，也不得使用 `using Base::Base;` 或显式模板实参形式替代。

### 固定模板：包含子模块实例模块（组件形态）

```cpp
#pragma once
#include "FuncModule.hpp"   // 别名唯一来源；如需公共业务结构体再加 ApaPathTypes.hpp
#include "ChildModule.hpp"  // 引入子模块头文件

namespace apa {

/**
 * @brief XxxxTraits 是 Xxxx 模块的接口结构集合。
 */
struct XxxxTraits {
  struct Input  { /* … 基础类型字段，尽量展平 */ };
  struct Output { /* … */ };
  struct Param  {};  ///< 只保存 Xxxx 自身参数
  struct State  {};  ///< 只保存 Xxxx 自身状态
  struct Sub {
    ChildModule child;  ///< 子模块实例
  };
};

class Xxxx : public FuncModule<XxxxTraits> {
 public:
  // 构造函数在 hpp 中完整实现，显式注入 Sub/Param/State（CTOR-005 至 CTOR-014）
  Xxxx(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                ChildModule(
                    ChildModuleTraits::Param{
                        .field = {
                            .nestedFieldA = {
                                .leafFieldA = 1.2,
                                .leafFieldB = 0.8},
                            .nestedFieldB = {
                                .leafFieldC = 0.05}}},
                    ChildModuleTraits::State{
                        .memory = {
                            .lastValue = 0.0,
                            .initialized = false}})},
            std::move(param),
            std::move(state)} {}

  void run(const Input& input, Output& output) override;
};

}  // namespace apa
```

> **显式构造**指：`Sub{...}` 中逐个列出子模块实例（顺序与 `Sub` 字段声明一致），每个子模块均使用 `ChildModule(ChildModuleTraits::Param{...}, ChildModuleTraits::State{...})`。两个实参必须使用当前子模块对应的 Traits 临时对象，**即使 Param/State 为空也不省略**；不得传入裸字段、局部变量、其他 Traits 类型或完整参数/状态对象。

组件构造函数必须保留固定签名、固定初始化顺序和空函数体。`Sub{...}` 直接构造父模块持有的子模块实例，展开后的 `ChildModuleTraits::Param{...}` 和 `ChildModuleTraits::State{...}` 由该子模块实例自身保存。`XxxxTraits::Param`、`XxxxTraits::State` 只描述当前 `Xxxx` 模块自身的参数和状态，不得在其中定义或保存子模块 `Param`/`State`；`std::move(param)`、`std::move(state)` 也只初始化当前模块自身的 `param_`、`state_`。包含子模块实例模块不得使用 `using FuncModule::FuncModule;`，也不得用 `FuncModule(...)`、`FuncModule<XxxxTraits>{...}`、基类别名、委托构造或构造函数体内赋值替代固定初始化列表。

固定模板已经同时展示子模块参数和状态的递归展开方式：每一层结构字段都使用 `.field = value` 指定初始化，直到基础叶子。原规范模板中的 `valueA`、`valueB`、`valueC` 仅表示叶子位置，生成代码时必须像模板中的 `1.2`、`0.8`、`0.05` 一样替换为明确的 C++ 字面值；子模块状态叶子同样必须直接使用 `0.0`、`false` 等字面值。禁止读取 `param.xxx`、`state.xxx`、`global::params.xxx`、`global::states.xxx`，也禁止使用局部变量、结构体对象或函数调用。

> **子模块 Param/State 运行期隔离硬约束**：子模块 `Param`、`State` 只允许在其所属模块的构造函数 `Sub{...}` 表达式中初始化，父模块 `run()` 不得复现原始 C/C++ 中“复制子模块参数/状态 → 覆盖字段 → 调用子模块”的交互，必须通过职责重构消除该依赖。不得把父模块 `param_`/`state_`、源代码中的 `childParam`/`childState`，或任何等价值包装为子模块 `Input` 字段；不得新增 `effectiveThreshold`、`overrideXxx`、`useXxxParam` 一类桥接端口，亦不得调用 setter、重新构造子模块或以其他方式动态设置其 Param/State。子模块 `Input` 只能承载本周期真实业务运行数据，来源仅限父模块 `Input`、上游子模块 `Output` 或父模块本周期计算结果；模块参数和内部状态不得跨模块读取、写入或传递。该责任边界同时适用于 `block` 和 `atomic`。

当原始实现通过父模块动态配置子模块、注入或回传其内部状态时，该交互禁止保留在改写结果中。必须重新划分模块职责和数据归属，使子模块独立管理 Param/State，父模块仅通过 Input/Output 交换业务数据。子模块在 `Sub{...}` 中使用自身确定的字面量初值，不能以伪造业务端口、setter、全局变量或状态副本恢复外部管理。业务 Output 可以表达计算结果，但不得用作导出内部 State 并交由父模块修改、再注入的通道。若无法同时满足隔离约束和原算法行为，应记录改写受阻及原因，不得仅删除原流程并宣称完成等价改写。

当原始实现存在 `child_in.field = param_.field`（或等价的父模块参数字段）且该字段仅表示子模块固定配置、不是本周期业务数据时，必须将 `field` 从子模块 `Input` 移至该子模块 `Param`，并在父模块构造函数的 `Sub{...}` 中以字面值指定初始化。`run()` 中不得保留这类参数到输入的赋值。

```cpp
struct ClampTraits {
  struct Input {
    Real value{};
  };
  struct Param {
    Real lowerLimit{};
    Real upperLimit{};
  };
  struct State {};
};

// 构造函数的 Sub{...} 中：
Clamp(ClampTraits::Param{.lowerLimit = 0.0, .upperLimit = 0.0},
      ClampTraits::State{})
```

| 子模块构造场景 | 规范形式 | 禁止写法 |
|---------------|---------|---------|
| Param/State 均空 | `ChildModule(ChildModuleTraits::Param{}, ChildModuleTraits::State{})` | `ChildModule()` |
| 仅 Param 非空 | `ChildModule(ChildModuleTraits::Param{.x = 0.5}, ChildModuleTraits::State{})` | 只传 Param、不传 State |
| State 非空 | `ChildModule(ChildModuleTraits::Param{}, ChildModuleTraits::State{.x = 0.0})` | 直接传 `state.childState` |
| 嵌套 Param/State | 每层 `.field = {...}`，叶子直接写字面值 | `.field = param.childParam`、`.field = childState0` |
| 叶子初始化值 | `1.2`、`-8.0`、`10`、`true`、`false` | `valueA`、`param.x`、`global::params.x`、局部变量、函数调用 |
| `run()` 中的子模块 Param/State | 重构原始动态复制、覆盖和回写依赖；只传递真实运行数据 | `child_in.effectiveX = param_.x`、`child_in.useOverride = true`、`sub_.child.setParam(...)` |

以下形式一律禁止：

```cpp
ChildModule();
ChildModule(param.childParam, state.childState);
ChildModule(ChildModuleTraits::Param{param.childParam},
            ChildModuleTraits::State{});
ChildModule(ChildModuleTraits::Param{.field = param.childParam},
            ChildModuleTraits::State{.memory = childState0});
ChildModule(ChildModuleTraits::Param{.threshold = valueA},
            ChildModuleTraits::State{});
ChildModule(ChildModuleTraits::Param{.threshold = global::params.threshold},
            ChildModuleTraits::State{});
child_in.effectiveThreshold = param_.threshold;
child_in.useChildParamOverride = true;
sub_.child.setParam(ChildModuleTraits::Param{.threshold = 0.5});
```

---

## 第五步：编写 run() 函数体

这是产出「符合规范代码」的核心。无论功能描述来自何处，`run()` 函数体都必须按以下规则书写。`run()` 声明与实现均需紧邻 Doxygen 注释。

### 固定签名与注释头（RUN-006、CMT）

```cpp
#include "SubA.hpp"

namespace apa {

/**
 * @brief 一句话描述该模块做什么。
 * @cn_name 模块中文名
 * @type block
 * @tag_level0 一级分类标签（XXX库）
 * @tag_level1 二级分类标签（XXX包）
 * @tag_level2 三级分类标签（XXX组）
 * @version 1.0
 * @date 2026-07-31
 * @author aide
 */
void SubA::run(const Input& input, Output& output) {
  /* 函数体见下述规则 */
}

}  // namespace apa
```

签名固定为 `void 类名::run(const Input& input, Output& output)`：只用引用不用指针（RUN-009），禁止增参（RUN-010）、禁止重载（RUN-011）。

### component/block 的 run() 禁止 return（v3.8）

- 当 `type` 为 `component` 或 `block` 时，整个 `run()` 函数体内不得出现任何 `return` 语句；异常分支、循环内部及函数末尾均无例外。即使 `void run()` 末尾的 `return;` 在 C++ 语法上合法，也不符合本指南。
- 限制依据模块的 `type` 判断，不依据是否含 `Sub` 判断：没有子模块的 `block` 同样受限。本条不改变 `atomic`、`element` 或 `main()` 的既有规则。
- 将提前返回改写为具名条件控制的 `if/else`、分支合流或循环执行条件，使原本不会执行的后续计算、子模块调用及状态更新仍不执行。每条路径的输出、状态更新、循环退出语义及调用条件和顺序必须与原逻辑一致，不得仅删除 `return` 后让后续语句无条件执行；无法证明等价时应记录改写受阻及原因。

以下为同一 `run()` 的函数体片段，适用于 `component` 和 `block`。示例约定 `input.valid` 表示本次输入是否有效，`input.value` 为业务输入，`output.value` 为结果；本例无跨周期状态。

**❌ 违规示例：分支提前返回及末尾 return 均禁止。**

```cpp
// 判断本次输入是否无效，决定是否跳过后续计算。
const Boolean isInvalid0 = !input.valid;
if (isInvalid0) {
  // 无效输入按原业务约定输出零。
  output.value = 0.0;
  return;  // 禁止：从分支提前返回。
}
// 仅有效输入应执行此处的结果映射。
const Real result0 = input.value;
output.value = result0;
return;  // 禁止：函数末尾也不能使用 return。
```

**✅ 合规示例：通过 if/else 保持原分支语义，自然执行至函数结束。**

```cpp
// 判断本次输入是否无效，控制结果映射分支。
const Boolean isInvalid0 = !input.valid;
if (isInvalid0) {
  // 无效输入输出零，不执行有效输入的后续计算。
  output.value = 0.0;
} else {
  // 有效输入映射业务结果。
  const Real result0 = input.value;
  output.value = result0;
}
```

### A. 原件 run() 的固定语句顺序（BODY-012）

无子模块的模块按此顺序：

1. 读取输入、参数、状态；
2. 计算中间物理量/数学量；
3. 更新模块状态；
4. 写输出。

### B. 组件 run() 的固定语句顺序（BODY-011）

含子模块的模块按此顺序：

1. 读取并锁存全局/模块状态；
2. 构造第一个子模块输入；
3. 调用第一个子模块；
4. 构造下游子模块输入；
5. 调用下游子模块；
6. 写顶层输出；
7. 更新跨周期状态。

### C. SSA 式显式赋值与命名（BODY-003、BODY-013 至 BODY-022）

- 一条语句只表达一个可扫描的计算/赋值/状态读写/输出映射/子模块调用（BODY-018）。
- 用有业务含义的变量名；禁止 `tmp`/`ret`/`res`/`val`/`a`/`b` 及其编号变体（BODY-014）。`a`/`a0` 仅表加速度时可用，`x`/`x0` 仅表坐标时可用（BODY-015）。
- 同一语义量连续变换：保持同一 baseName、递增数字后缀（BODY-016）。产生新物理/数学含义：换新 baseName、后缀从 `0` 重新开始（BODY-017）。
- 同一局部变量不得二次赋值表达新的计算结果（BODY-020、BODY-034）。

```cpp
// ✅ 正确：同一语义量连续变换
Real integral0 = state_.integral;
Real integral1 = integral0 + error0 * dt0;
Real integral2 = std::min(integral1, maxIntegral);

// ✅ 正确：跨语义换 baseName
Real dx0 = targetX - currentX;
Real dy0 = targetY - currentY;
Real distSq0 = dx0 * dx0 + dy0 * dy0;
Real dist0 = std::sqrt(distSq0);

// ❌ 错误：二次赋值 + 无语义名
Real value0 = input.a;
value0 = value0 + input.b;
Real tmp0 = input.x - input.y;
```

### D. 四则运算与表达式（BODY-030 至 BODY-036）

- 支持 `+ - * /`、比较、`&& || !`；每个表达式结果先赋给具名局部变量，再用于后续计算/输出/状态回写。
- 禁止在输出赋值、状态回写、子模块输入赋值、函数实参中嵌入复合表达式（BODY-033）。
- **数值字面量禁止携带 `f`/`F`/`u`/`U`/`l`/`L` 等类型后缀**（BODY-036）：`Real`/`Integer`/`UInteger` 别名已确定类型，写 `0.0` 而非 `0.0f`，写 `0` 而非 `0U`（见 v3.5 附加硬约束第 8 条）。

```cpp
// ❌ output.value = std::min(input.a + input.b, param_.limit);
// ✅
Real sum0 = input.a + input.b;
Real clamped0 = std::min(sum0, param_.limit);
output.value = clamped0;
```

### E. 条件与分支合流（BODY-104、v2.2.0 第 13/14 条）

- **禁止三目 `?:`** 表达业务分支合流；改写为 `if/else`，每个分支都对同一个具名 merge 变量显式赋值。
- 禁止把 `&&`/`||`/比较/算术表达式直接作为 `if` 条件或输出值；先拆成具名布尔/数值局部变量再组合。
- 每个分支都要对后续使用的变量赋值，不得依赖默认初始化作为某分支的隐式输出。

```cpp
/** @brief 判断输入 a 是否为正值。 */
const bool isPositiveA = input.a > 0.0;
/** @brief 判断输入 b 是否低于上限。 */
const bool isBelowLimitB = input.b < param_.limit;
/** @brief 合成分支执行条件。 */
const bool shouldUseChild = isPositiveA && isBelowLimitB;

/** @brief 分支合流值，所有分支都显式赋值。 */
Real mergedValue;
if (shouldUseChild) {
  mergedValue = input.a;
} else {
  mergedValue = 0.0;
}
output.value = mergedValue;
```

### F. 子模块调用固定写法（BODY-023 至 BODY-029）

- 必须 `sub_.子模块名.run(子模块名_in, 子模块名_out)`；输入变量名 `实例名_in`，输出变量名 `实例名_out`（BODY-025/026）。
- 先声明 `Traits::Input`，逐字段赋值；再声明 `Traits::Output`，传入 `run()`（BODY-027/028）。
- 调用后先把用到的输出字段取成具名局部变量，再参与后续计算或输出。
- 原始代码在子模块调用后紧邻 `if (child_out.valid)`（或等价输出字段判定）时，生成代码必须在该 `sub_.child.run(...)` 后、构造或调用下一子模块前保留该分支。分支中对 `valid`、`flag`、`value`、`x`、`y`、`status`、`index` 等字段的更新必须逐项保留；不得延后、合并到后续子模块结果，或改写为无条件赋值。

```cpp
SubATraits::Input prep_in;
prep_in.vstX = input.vstX;
prep_in.vstY = input.vstY;

SubATraits::Output prep_out;
sub_.prep.run(prep_in, prep_out);

// ❌ sub_.prep.run({input.vstX, input.vstY}, prep_out);  // 禁止聚合初始化实参
```

### G. 状态与输出映射（BODY-010、BODY-057 至 BODY-068）

- 输出只通过 `output.field = value` 写出；跨周期状态只通过 `state_`（或全局 `global::states`）显式读写。
- 涉及物理/数学/控制含义、状态读写、分支、循环、子模块调用、输入/输出映射的语句必须写语义注释，说明其在模型中的作用，而非复述语法（BODY-007/008）。

### H. 白名单边界（BODY-100 至 BODY-107）

`@type block` 的 `run()` 只允许白名单语法：基础类型/枚举/别名局部变量、子模块 Input/Output 结构体变量、四则运算、比较、逻辑表达式、`if`/`if-else`/`for`/`while`（固定模板）、白名单标准库函数与子模块 `run()`、`state_`/`output` 显式读写。**禁止**任何 `return` 语句（v3.8，含提前返回和末尾 `return;`）、三目、位运算/移位/枚举强转、`switch`、`goto`、异常、lambda、宏技巧、模板技巧、I/O、线程、网络。原始逻辑含这些写法时，先等价改写为白名单形式；无法证明等价的标记为「需确认」，不得生成看似等效但越界的代码（BODY-102/107）。

v3.8 的禁用 `return` 约束同样适用于 `@type component` 的 `run()`，与是否含子模块无关，详见本节前述统一规则。

> 若某模块内部确实无法按 `block` 的函数体规则稳定展开，或 `run()` 有效代码超过 120 行且不可再拆，在满足依赖、接口类型及其他框架约束的前提下，可整体标记 `@type atomic`（RUN-012/013/014）。`atomic` 的 `run()` 可以超过 120 行，只豁免规定的函数体内部写法规则；不得用于绕过第三方依赖或不支持的容器数据流限制，签名、Traits、文件组织、注释字段仍须合规。

### 组件主模块 run() 完整模板

```cpp
#include "Xxxx.hpp"

namespace apa {

/**
 * @brief 协调子模块完成 Xxxx 功能。
 * @cn_name Xxxx 中文名
 * @type block
 * @version 1.0
 * @author aide
 */
void Xxxx::run(const Input& input, Output& output) {
  // 1. 调用预处理子模块
  SubATraits::Input prep_in;
  prep_in.vstX = input.vstX;
  prep_in.vstY = input.vstY;

  SubATraits::Output prep_out;
  sub_.prep.run(prep_in, prep_out);

  // 2. 调用圆心计算子模块（消费 prep 输出）
  SubBTraits::Input circle_in;
  circle_in.lineA1 = prep_out.lineA1;
  circle_in.radius = input.radius;

  SubBTraits::Output circle_out;
  sub_.circle.run(circle_in, circle_out);

  // 3. 汇总输出
  output.cptX = circle_out.cptX;
  output.cptY = circle_out.cptY;
}

}  // namespace apa
```

**主模块 cpp 只允许**：`run()` 实现（RUN-003）。**禁止**：构造函数实现（在 hpp）、私有成员方法、静态辅助函数、其他业务函数（RUN-004）。

| 内容 | hpp | cpp |
|------|-----|-----|
| 构造函数实现 | ✅ | ❌ |
| 子模块注入 Sub | ✅（构造函数中） | ❌ |
| run() 声明 | ✅ | — |
| run() 实现（业务逻辑） | — | ✅ |

**若 block 主模块 run() 仍超 120 行**：说明子模块拆分不够细。增加子模块 / 进一步下切逻辑，保持主模块只做「协调 + 数据映射」。atomic 主模块不受 120 行上限约束。

---

## 固定长度 C 数组转换规则（v3.5）

本节是 v3.5 强制规则，适用于 Traits、业务 Types、局部固定长度数组、原始函数参数、父子模块端口、State 和测试代码。修改数组声明后，必须搜索并改写 `run()` 及测试中的所有使用点；只修改 Traits 或 `Types.hpp` 声明视为未完成。

### 1. 一维数组保持一个等长一维 `std::array`

原始一维固定长度数组：

```cpp
Real values[10];
```

转换为：

```cpp
std::array<Real, 10> values{};
```

继续使用 C 数组风格下标：

```cpp
Real value = values[index];
values[index] = newValue;
```

不得调用 `std::array` 成员方法。

### 2. 二维数组拆分为多个一维字段

原始二维数组：

```cpp
Real matrix[3][4];
```

必须将第一维展开：

```cpp
std::array<Real, 4> matrix0{};
std::array<Real, 4> matrix1{};
std::array<Real, 4> matrix2{};
```

映射关系：

```text
matrix[0][column] -> matrix0[column]
matrix[1][column] -> matrix1[column]
matrix[2][column] -> matrix2[column]
```

禁止转换为嵌套 `std::array`，也禁止展平为一个长度为 12 的一维数组。

### 3. 三维及以上数组只保留最后一维

对于 `T array[D0][D1]...[Dn]`，生成 `D0 × D1 × ... × D(n-1)` 个 `std::array<T, Dn>`；最后一维是唯一保留的数组下标维度，其他维度按原顺序成为字段名中的固定索引。

```text
array[i0][i1]...[i(n-1)][in]
->
array<i0>_<i1>_..._<i(n-1)>[in]
```

原始三维数组：

```cpp
Real table[2][3][4];
```

转换为：

```cpp
std::array<Real, 4> table0_0{};
std::array<Real, 4> table0_1{};
std::array<Real, 4> table0_2{};
std::array<Real, 4> table1_0{};
std::array<Real, 4> table1_1{};
std::array<Real, 4> table1_2{};
```

原始 `Integer grid[2][3][4][5]` 应按行优先维度顺序生成 `2 × 3 × 4` 个长度为 5 的字段，例如 `grid0_0_0`、`grid0_0_1`，直至 `grid1_2_3`。不得交换行列或改变维度含义。

### 4. 字段命名与业务角色

命名采用“原数组名 + 除最后一维外的各维固定索引”。二维可用 `matrix0`、`matrix1`；三维及以上使用稳定分隔符，如 `table0_0`。目标工程可调整分隔符或大小写，但字段必须与原数组前导索引存在唯一、稳定、可追溯的对应关系。

多维数组位于 `Input`、`Output`、`Param`、`State` 或业务 Types 时，全部拆分字段必须留在原业务角色中，并注明对应索引：

```cpp
struct Input {
  std::array<Real, 4> matrix0{};  ///< 原 matrix[0][0..3]
  std::array<Real, 4> matrix1{};  ///< 原 matrix[1][0..3]
  std::array<Real, 4> matrix2{};  ///< 原 matrix[2][0..3]
};
```

声明所在头文件必须显式包含 `<array>`。不得把原数组的一部分移到其他 Traits 角色或局部变量。

### 5. `run()` 必须同步保持业务语义

必须搜索并改写该数组的全部使用点，包括：元素读取和写入、多层循环、聚合/默认初始化、每周期清零、复制、比较、条件访问、Input/Output 映射、父子模块端口映射、State 读取和回写、Param 访问、原始函数参数以及测试输入和预期输出。

转换后必须保持原始数组容量、索引顺序、执行顺序、循环次数、条件判断、状态更新顺序、输出语义和子模块调用顺序。不得只改字段声明，不得遗漏任一拆分字段。

固定前导索引直接访问对应字段：

```cpp
Real value0 = input.matrix1[column];
output.matrix2[column] = value0;
Real value1 = input.table1_2[column];
state_.table0_1[column] = value1;
```

最后一维继续使用下标访问。

### 6. 多层循环展开

完整遍历多维数组时，展开除最后一维以外的循环，只保留最后一维循环：

```cpp
for (Integer column0 = 0; column0 < 4; column0 = column0 + 1) {
  matrix0[column0] = 0.0;
}
for (Integer column1 = 0; column1 < 4; column1 = column1 + 1) {
  matrix1[column1] = 0.0;
}
for (Integer column2 = 0; column2 < 4; column2 = column2 + 1) {
  matrix2[column2] = 0.0;
}
```

若外层循环变量还参与条件、累加、状态、输出、子模块调用、错误标志、有效数量或提前退出，必须按每个固定前导索引复制完整业务块，并以对应固定值替换外层变量。提前退出必须做可证明等价的显式处理；无法证明时标记“需确认”。

不得为保留外层循环引入嵌套数组、指针/引用数组、`std::array*` 选择表、`std::vector`、`std::reference_wrapper`、选择辅助函数、lambda 或容器成员方法。

### 7. 运行期前导索引显式路由

运行期前导索引不能动态选择字段，必须按原行优先顺序使用显式 `if / else if / else`。二维读取示例：

```cpp
Real selectedValue;
if (row == 0) {
  selectedValue = input.matrix0[column];
} else if (row == 1) {
  selectedValue = input.matrix1[column];
} else if (row == 2) {
  selectedValue = input.matrix2[column];
} else {
  selectedValue = 0.0;  // 示例假设原始越界语义为返回零。
}
```

写入采用同样分支：

```cpp
if (row == 0) {
  output.matrix0[column] = selectedValue;
} else if (row == 1) {
  output.matrix1[column] = selectedValue;
} else if (row == 2) {
  output.matrix2[column] = selectedValue;
} else {
  // 按原始代码的越界语义处理。
}
```

三维以上按前导索引组合建立显式分支，例如 `(layer == 0) && (row == 0)` 映射 `table0_0[column]`，依次覆盖全部组合。分支条件和合流变量继续遵守 BODY 的具名条件与 SSA 规则。禁止三目、`switch`、lambda、辅助选择函数、指针表、引用表或容器成员方法。

原始代码不能证明索引有效时，保留其边界处理；原代码无明确越界语义时标记“需确认”，不得擅自取模、截断、钳位或默认选择首字段。

### 8. 初始化、清零、复制与比较

原始每周期清零（包括 `memset(matrix, 0, sizeof(matrix))`）必须在 `run()` 中按每个拆分字段使用最后一维下标循环保留，不得改用 `.fill()`，也不得仅依赖字段 `{}` 默认值。只有原始初始化语义与定义时零初始化完全一致且不是每周期操作时，才可使用 `{}`。

数组复制必须逐拆分字段处理：

```cpp
for (Integer column0 = 0; column0 < 4; column0 = column0 + 1) {
  output0[column0] = input0[column0];
}
for (Integer column1 = 0; column1 < 4; column1 = column1 + 1) {
  output1[column1] = input1[column1];
}
for (Integer column2 = 0; column2 < 4; column2 = column2 + 1) {
  output2[column2] = input2[column2];
}
```

比较也按原索引顺序逐字段、逐元素执行，并保持短路、结果更新和循环退出语义。不得使用 `memcpy`、`std::copy`、`.data()`、`.begin()` 或 `.end()`。若既有 BODY/SSA 规则明确允许同类型一维 `std::array` 整体赋值，可逐字段整体赋值；否则逐元素复制。

### 9. 父子模块端口与 State

父子模块原本传递整个多维数组时，双方端口都拆分并映射全部字段：

```cpp
childInput.matrix0 = input.matrix0;
childInput.matrix1 = input.matrix1;
childInput.matrix2 = input.matrix2;
```

若整体一维数组赋值不符合 BODY/SSA，则逐元素下标复制。子模块 Output 返回多维数组时同样逐字段或逐元素映射，不能遗漏或交换。

跨周期多维状态必须全部保留在 `State`：固定前导索引直接使用 `state_.history0[column]` 等字段，动态索引使用显式分支。保持原始状态读取、计算、回写顺序，不得改成局部临时数组或遗漏任何状态行。

### 10. `std::array` 使用边界与不适用情况

允许默认初始化、聚合初始化、`array[index]` 读取/写入、最后一维下标循环，以及现有规范明确允许的同类型一维数组赋值。禁止调用 `.at()`、`.size()`、`.data()`、`.front()`、`.back()`、`.begin()`、`.end()`、`.cbegin()`、`.cend()`、`.fill()`、`.swap()` 等成员方法；不得把拆分字段重新组合为嵌套数组、单个长数组、指针表、引用表或其他容器。

本规则不直接适用于运行期长度数组、C 变长数组、动态分配多维数组、多级指针模拟数组、每行长度不一致的数据、维度容量不明数组、仅凭指针访问推测的数组、依赖连续整块内存进行外部二进制交互的数组，以及依赖地址退化或指针算术的逻辑。这些情况标记“需确认”，不得擅自推断维度、容量或布局。依赖连续布局时必须记录拆分可能改变地址连续性的语义风险，不得宣称已完成等价转换。

若前导组合展开使单个 `block` 的 `run()` 超过 120 行，按既有 FuncModule 子模块规则拆分完整业务逻辑；不得恢复嵌套/展平数组，也不得用辅助业务函数规避 RUN-004。`atomic` 的行数豁免不改变接口数组的类型约束。

---

## Types.hpp 使用规则

**Types.hpp 只存业务结构体**（`Input/Output/Param/State/Sub` 之外的自定义类型），不含任何 `Traits` 定义，也**不重复定义 `Real`/`Integer` 别名**（这些来自 `FuncModule.hpp`，v3.1 约束②）。

**优先使用基础标量，避免动态容器与深层嵌套**（v3.1 约束④、v3.5 约束⑤）。原始接口为编号字段时保留编号标量；一维固定长度 C 数组转换为等长一维 `std::array`，二维及以上数组拆分为多个只保留最后一维的一维 `std::array`。Types 中的拆分字段仍属于原业务结构，不得改变角色。仅当同一组数据在多个模块间反复复用、直接定义会造成大量重复时，才提取为业务结构体：

```
功能描述里是否出现必须复用的自定义业务结构体（非五元）？
  否 → 不创建 Types.hpp，Traits 直接用基础类型标量字段写在 .hpp（多数情况）
  是 → 创建 XxxxTypes.hpp 存这些结构体，.hpp 引入后在 Traits 字段中引用（仅此一层嵌套）
```

```cpp
// XxxxTypes.hpp —— 只放必须复用的业务结构体；原始编号字段继续保持编号形态
#pragma once
#include "FuncModule.hpp"   // 取用 Real/Integer 别名，不重复定义

namespace apa {

struct BoundaryLineSet {
  Integer lineCount = 0;  ///< 有效线段数（原始接口本身提供 count）
  Real line0X1 = 0.0;     ///< 线段0起点X
  Real line0Y1 = 0.0;     ///< 线段0起点Y
  Real line0X2 = 0.0;     ///< 线段0终点X
  Real line0Y2 = 0.0;     ///< 线段0终点Y
  // 原始接口的 line1X1 等编号字段继续按原形态声明，不主动合并为 std::array
};

}  // namespace apa

// Xxxx.hpp —— Traits 字段引用业务结构体（此处才允许一层嵌套）
struct XxxxTraits {
  struct Input {
    BoundaryLineSet boundaries; ///< 边界线集合
  };
  /* … */
};
```

原始固定长度 C 数组转换必须显式包含 `<array>`，并遵守 v3.5 按维转换规则：

```cpp
#include <array>

struct Input {
  std::array<Real, 10> points{};  ///< 原 points[0..9]
  std::array<Real, 4> matrix0{};  ///< 原 matrix[0][0..3]
  std::array<Real, 4> matrix1{};  ///< 原 matrix[1][0..3]
  std::array<Real, 4> matrix2{};  ///< 原 matrix[2][0..3]
};

Real pointI = input.points[index];
Real matrixValue = input.matrix1[column];
```

二维及以上数组不得在 Types 中声明为嵌套 `std::array`，也不得展平为单个长数组。动态前导索引、循环、初始化、复制、比较、端口映射和状态回写按“固定长度 C 数组转换规则（v3.5）”同步改写。

禁止将转换后的 `std::array` 当作一般容器使用；不得调用 `.size()`、`.data()`、`.front()`、`.fill()` 或其他成员方法。

工程级公共业务类型（如 `ApaPathPointSet`、`ApaStObsArraySet`）已集中在 `ApaPathTypes.hpp`，直接引入复用，不要在各模块重复定义。若既有公共类型含多维 C 数组或嵌套数组且本次需要改写，必须按 v3.5 拆分规则同步调整类型及全部业务访问；不能仅沿用旧嵌套形态。新增 `std::array` 字段必须能追溯到原始固定长度 C 数组，不得因便利性主动引入。

---

## test 目录与测试文件规则

功能目录下的 `test/` 存放测试文件（v3.1 约束⑥）。测试文件按 **头文件 + 源文件** 分离：

- `test_类名.hpp`：测试 Traits（五元结构）+ 测试类声明 + 构造函数实现（被测模块以 `Sub` 字段注入）。
- `test_类名.cpp`：仅含一个 `run()` 实现，通过 `sub_` 调用被测模块，遵守所有 BODY-* 规则；**不追加 `main()`**。
- `test/main.cpp`（唯一）：统一入口，声明并依次运行所有测试模块，包含全部 `main()` 逻辑；**不与任何 `test_类名.cpp` 同名**。

测试文件遵守以下规则：

- **被测模块通过 Sub 注入，run() 内用 `sub_` 调用**：`sub_.模块名.run(模块名_in, 模块名_out)`（BODY-023）；不在 `run()` 内局部实例化被测模块。
- **只包含被测模块的头文件**（直接写文件名），通过工程指定命名空间访问模块。
- **遵守一个 run() 标准**：`test_类名.cpp` 中只有一个 `run()` 实现，不出现其他独立函数（包括 `main()`）。
- **`main()` 集中在 `test/main.cpp`**：不追加到任何 `test_类名.cpp`，也不生成独立的 `main_类名.cpp` 文件。
- **输入结构体可为空**：`Input` 若为空结构，固定值在 `run()` 内直接构造；不从外部注入。
- 断言用基础比较表达；浮点比较用容差 `const Real tol = ...`，不使用第三方断言库；不引入宏与 `constexpr`（固定值用 `const` 局部量）。
- 测试文件本身不定义业务接口结构体，只构造 `Traits::Input`、接收 `Traits::Output`。原测试涉及二维及以上固定长度数组时，测试输入、State 初值、子模块端口和预期输出必须同步拆分为全部一维字段，并按原索引顺序逐元素赋值与比较；不得遗漏字段或调用 `std::array` 成员方法。

### 文件布局（示例：SubA 的测试）

```
test/
├── test_SubA.hpp      # TestSubATraits + TestSubA 类声明（含构造函数，Sub 中注入 SubA）
├── test_SubA.cpp      # TestSubA::run() 实现，仅此一函数
├── test_SubB.hpp
├── test_SubB.cpp
└── main.cpp           # 统一 main()，依次运行所有测试模块
```

### test_SubA.hpp 模板

```cpp
#pragma once
#include "FuncModule.hpp"
#include "SubA.hpp"   // 被测模块头文件

namespace apa {

struct TestSubATraits {
  struct Input  {};                  ///< 固定值在 run() 内定义，可为空
  struct Output { bool passed = false; };
  struct Param  {};
  struct State  {};
  struct Sub {
    SubA subA;   ///< 被测子模块：SubA
  };
};

/**
 * @brief TestSubA 是 SubA 的测试模块，run() 包含全部测试逻辑。
 */
class TestSubA : public FuncModule<TestSubATraits> {
 public:
  TestSubA(Param param = Param{},
           State state = State{})
      : FuncModule{
            Sub{SubA(SubATraits::Param{}, SubATraits::State{})},
            std::move(param), std::move(state)} {}

  void run(const Input& input, Output& output) override;
};

}  // namespace apa
```

### test_SubA.cpp 模板

```cpp
#include "test_SubA.hpp"

#include <cmath>
#include <cstdio>

namespace apa {

/**
 * @brief 用固定值输入验证 SubA 计算结果。
 * @cn_name SubA 测试
 * @type block
 * @version 1.0
 * @date 2026-07-31
 * @author aide
 */
void TestSubA::run(const Input&, Output& output) {
  // 构造被测模块输入（固定值）。
  SubATraits::Input subA_in;
  subA_in.var1 = 3.0;
  subA_in.var2 = 4.0;

  // 通过 sub_ 调用被测子模块（BODY-023）。
  SubATraits::Output subA_out;
  sub_.subA.run(subA_in, subA_out);

  // 取出被测输出为具名局部量，再与期望值比较。
  const Real actual   = subA_out.result1;   ///< 实测结果
  const Real expected = 5.0;                ///< 期望结果（固定值）
  const Real tol      = 1e-4;               ///< 浮点比较容差
  const Real diff     = std::fabs(actual - expected);
  const bool passedOk = diff < tol;         ///< 通过标志
  output.passed = passedOk;

  if (output.passed) {
    std::printf("[PASS] SubA\n");
  } else {
    std::printf("[FAIL] SubA: got %f, expected %f\n",
                static_cast<double>(actual),
                static_cast<double>(expected));
  }
}

}  // namespace apa
// ⚠️ 此文件末尾不追加 main()。main() 统一在 test/main.cpp 中。
```

### test/main.cpp 模板

```cpp
/**
 * @brief 测试统一入口：main() 集中在此文件，各 test_*.cpp 只含 run()。
 */
#include "test_SubA.hpp"
#include "test_SubB.hpp"

int main() {
  apa::TestSubATraits::Input  subA_in;
  apa::TestSubATraits::Output subA_out;
  apa::TestSubA subA(apa::TestSubATraits::Param{}, apa::TestSubATraits::State{});
  subA.run(subA_in, subA_out);

  apa::TestSubBTraits::Input  subB_in;
  apa::TestSubBTraits::Output subB_out;
  apa::TestSubB subB(apa::TestSubBTraits::Param{}, apa::TestSubBTraits::State{});
  subB.run(subB_in, subB_out);

  const bool allPassed = subA_out.passed && subB_out.passed;
  return allPassed ? 0 : 1;
}
```

### CMakeLists.txt 片段

```cmake
add_executable(test_All
    test/test_SubA.cpp
    test/test_SubB.cpp
    test/main.cpp)
target_include_directories(test_All PRIVATE test)
target_link_libraries(test_All PRIVATE Xxxx)
add_test(NAME test_All COMMAND test_All)
```

> `Input` 为空时（`struct Input {};`），`run()` 签名中的 `const Input&` 参数用 `const Input&` 占位即可，固定值体现在被测逻辑对 `Param`/`State` 或常量的依赖上。

---

## 常见问题

### Q1：多个逻辑块更新同一累积变量怎么办？
把累积变量同时作为子模块的 `Input`（读入当前值）和 `Output`（写回更新值），在主模块中链式传递，避免隐式共享状态。

```cpp
SubATraits::Input  { Real currentBest = 0.0; };
SubATraits::Output { Real updatedBest = 0.0; };
```

### Q2：一个循环体就有 200 行怎么办？
把循环体提为子模块，主模块中循环调用（`for`/`while` 属白名单控制结构）：

```cpp
Real accumulated0 = 0.0;
for (Integer i = 0; i < count; i = i + 1) {
  LoopBodyTraits::Input loop_in;
  loop_in.index = i;
  loop_in.carry = accumulated0;

  LoopBodyTraits::Output loop_out;
  sub_.loopBody.run(loop_in, loop_out);

  accumulated0 = loop_out.carry;  // 跨迭代携带，Next 模式回写
}
```

### Q3：子模块需要使用父模块的数据、参数或状态怎么办？

先按语义分类，不要让子模块直接访问父模块 `param_`/`state_`：

- 子模块每周期消费的运行数据，通过子模块 `Input` 逐字段传入。
- 子模块自身的静态配置，必须在父模块构造函数的 `Sub{...}` 中用字面值绑定到对应 `SubTraits::Param{...}`；不得读取父模块参数或在 `run()` 中动态设置。
- 子模块初始状态，必须在同一子模块构造表达式中用字面值绑定到对应 `SubTraits::State{...}`；后续状态由子模块自身维护。
- 父模块 `Param`/`State` 只保存父模块自身业务字段，不得机械增加 `childParam`/`childState` 作为子模块参数或状态的固定容器。
- 父模块 `run()` 不得将 `param_`/`state_` 或原始实现中的子模块 Param/State 映射为子模块 `Input`；不得为此新增 `effectiveXxx`、`overrideXxx`、`useXxxParam` 等桥接字段。原始实现存在该类动态配置时，必须重构职责和数据归属，不得直接删除影响算法行为的逻辑；无法同时满足规范并保持原算法行为时，记录改写受阻。

每周期运行数据示例：

```cpp
struct Input {
  Real threshold = 0.0;
};

// 主模块 run() 中：
SubATraits::Input prep_in;
prep_in.threshold = input.threshold;
```

若 `threshold` 的业务语义实际是子模块静态参数，则必须在父模块构造函数中直接写字面值，而不是读取 `param.someThreshold`；原始实现若在父模块 `run()` 中覆盖该值，必须重构职责和数据归属，不得直接删除影响算法行为的逻辑；无法同时满足规范并保持原算法行为时，记录改写受阻。以下仅示意静态参数的字面值构造方式，不代表可以直接替代原始动态配置逻辑：

```cpp
SubA(
    SubATraits::Param{
        .threshold = 0.5},
    SubATraits::State{
        .lastValue = 0.0})
```

### Q4：功能描述来自 MATLAB / C 源码，含指针、数组、结构体拷贝、位运算怎么办？
先做等价改写：指针参数→值/引用字段；C 结构体→五元或 `Types.hpp` 业务结构体；位运算/三目→白名单显式语句。固定长度一维 C 数组转换为等长一维 `std::array`；二维及以上数组按 v3.5 将全部前导索引拆入字段名，仅最后一维保留下标。必须同步改写 `run()` 的读写、循环、初始化/清零、复制/比较、条件、State、父子端口、输出和测试映射，只用下标访问。运行期长度、变长、动态分配、维度不明、指针模拟或依赖连续内存布局的数组不得直接套用；无法证明等价时标记「需确认」。

---

## 生成检查清单

### 📋 设计阶段
- [ ] 已判定原件 / 组件形态（**人为指定优先**；无指定时才按 run() 有效代码是否 > 120 行）
- [ ] 五元 Traits 字段按业务角色归位，每字段有默认值 + 业务注释
- [ ] 字段尽量为基础数据类型、避免业务结构嵌套；无 `std::vector`/`std::map` 等容器；`std::array` 仅来自原始固定长度 C 数组，且未主动合并编号标量字段
- [ ] （组件）子模块边界、数据流、端口已列表确定

### 📝 文件生成（同一功能统一目录）
- [ ] 目录含 `include/`、`src/`、`test/`、`CMakeLists.txt`；同功能所有子模块头文件在 `include/`、源文件在 `src/`，不建子目录
- [ ] （含 `Sub` 的组件）已逐项对照「参考案例：GeoCircle2lineR」的 hpp/cpp：仅替换业务相关的类名、字段、子模块和计算，未改变案例的结构与编码风格
- [ ] hpp：Traits 五元完整（空结构显式保留）+ 类继承 `FuncModule<XxxxTraits>`
- [ ] 无子模块实例模块只使用 `using FuncModule::FuncModule;`，未显式编写空构造或 Param/State 转发构造
- [ ] 含子模块实例模块使用固定签名和 `FuncModule{Sub{...}, std::move(param), std::move(state)}` 固定顺序，构造函数体为空
- [ ] `Sub{...}` 内每个子模块均使用对应 `SubTraits::Param{...}`、`SubTraits::State{...}` 两个临时对象显式构造
- [ ] 非空子模块 Param/State 逐层 `.field = value` 递归到基础叶子，所有叶子直接写字面值，无 `valueA/valueB/valueC` 占位符、父模块字段、全局对象、局部变量或函数调用
- [ ] 子模块 Param/State 由子模块实例自身持有，未机械写入父模块 `Param.childParam` / `State.childState`
- [ ] 父模块 `run()` 未复制、覆盖、回写或动态设置子模块 Param/State，未通过 `effectiveXxx`/`overrideXxx`/`useXxxParam` 等 Input 桥接字段规避该限制
- [ ] 仅承载父模块参数的子模块 Input 字段已迁移至对应子模块 Param，并在 `Sub{...}` 中以字面值初始化；`run()` 中无 `child_in.field = param_.field` 一类赋值
- [ ] hpp 内无 `#define` 宏、无 `constexpr`（`#pragma once` 除外）
- [ ] `Real`/`Integer` 等别名仅来自 `FuncModule.hpp`，模块头/Types.hpp 未重复定义
- [ ] 是否需要 Types.hpp 判断正确（仅必须复用的业务结构体才建）
- [ ] 原始固定长度 C 数组已按 v3.5 转换并显式包含 `<array>`：一维保持等长，二维及以上拆分为多个仅保留最后一维的一维字段
- [ ] 运行期长度数组、变长数组、动态数组及长度不明的指针未擅自转换为 `std::array`，不明确项已标记「需确认」
- [ ] cpp：只 `#include` 自己的同名 hpp；命名空间按业务指定、工程内统一
- [ ] （含 `Sub` 的组件）cpp 按 GeoCircle2lineR 的调用骨架展开：每一阶段都先构造具名子模块输入、声明具名子模块输出、调用 `sub_.child.run(...)`，再显式完成顶层输出映射
- [ ] test：`test_类名.hpp`（Traits + 类声明，Sub 注入被测模块）+ `test_类名.cpp`（仅 `run()`，不追加 `main()`）+ `test/main.cpp`（统一入口）三文件分离；被测模块通过 `sub_` 调用；输入用固定值构造

### 🔨 run() 函数体
- [ ] 一维 C 数组已转换为等长的一维 `std::array`
- [ ] 多维 C 数组未转换为嵌套 `std::array`
- [ ] 多维 C 数组未展平为单个长数组
- [ ] 多维数组已按除最后一维外的索引拆分成多个一维数组
- [ ] 每个一维数组的长度等于原始数组最后一维的长度
- [ ] 拆分字段名称可以唯一追溯到原始数组及其固定索引
- [ ] 拆分字段仍位于原始 Input、Output、Param、State 或业务 Types 角色中
- [ ] `run()` 中所有多维数组读取均已同步修改
- [ ] `run()` 中所有多维数组写入均已同步修改
- [ ] 前导维度循环已正确展开
- [ ] 循环展开未遗漏条件、计算、状态、输出或子模块调用
- [ ] 运行期前导索引已使用显式 `if / else if / else` 路由
- [ ] 动态索引分支保留了原始边界处理语义
- [ ] 数组初始化、清零、复制和比较已同步调整
- [ ] 父子模块端口未遗漏任何拆分字段
- [ ] State 中的拆分数组均正确读取并回写
- [ ] 测试输入和预期输出已同步修改
- [ ] 所有数组只使用下标访问
- [ ] 未调用任何 `std::array` 成员方法
- [ ] 未引入嵌套数组、展平数组、指针表、引用表或选择辅助函数
- [ ] 转换前后的容量、索引顺序、循环次数和业务语义一致
- [ ] 依赖连续内存布局但无法证明等价的情况已标记为“需确认”
- [ ] 签名固定 `void 类名::run(const Input&, Output&)`，紧邻 Doxygen（`@brief/@cn_name/@type block/@version/@author`）
- [ ] v3.8：已按 `type` 识别全部 `component`、`block`，其 `run()` 函数体内无任何 `return` 语句（含分支、循环和函数末尾）；无子模块的 `block` 也已检查
- [ ] 原有提前返回已改写为条件执行与分支合流，保留各路径的输出、状态更新、循环退出语义及子模块调用条件和顺序，未仅删除 `return`；无法证明等价时已记录改写受阻及原因
- [ ] 语句按固定顺序（原件 BODY-012 / 组件 BODY-011）
- [ ] SSA 式：一句一事，具名变量，同语义递增后缀、跨语义换 baseName，无二次赋值
- [ ] 无三目、无位运算/switch/lambda/异常等白名单外写法
- [ ] 分支合流用具名 merge 变量，每分支显式赋值
- [ ] 复合表达式已拆为具名局部变量（输出/子模块输入/实参处不嵌复合表达式）
- [ ] BODY-036：全部 `.hpp` 默认值与 `run()` 内的数值字面量无 `f`/`F`/`u`/`U`/`l`/`L` 后缀（可用 `grep -nrE '[0-9][fFuUlL]\b' include src test` 自查）
- [ ] 子模块调用为 `sub_.名.run(名_in, 名_out)`，输入先声明逐字段赋值、输出先声明再传入
- [ ] 原始子模块调用后的输出判定分支紧邻对应 `sub_.名.run(...)`，且在下一子模块构造或调用前完整保留
- [ ] 输出用 `output.field=`，状态用 `state_` 显式读写；关键语句有语义注释

### ✅ 规范符合性（关键）
- [ ] RUN-003：每个 cpp 只有一个 run()
- [ ] RUN-004：主模块 cpp 无构造函数实现、无私有方法、无静态辅助函数、无其他业务函数
- [ ] BODY-023：子模块只通过 `sub_.xxx.run()` 调用
- [ ] FILE-007/008：无多余 Types.hpp / Types.hpp 只含业务结构体
- [ ] 所有 `block` 模块的 `run()` 有效代码 ≤ 120 行；`atomic` 的 `run()` 可超过 120 行，仍需检查接口、类型、依赖和参数状态边界
- [ ] v3.1：头文件无宏/`constexpr`；别名单一来源；字段基础类型优先；同一功能统一目录含 test；命名空间按业务指定
- [ ] v3.5：一维 C 数组转换为等长一维 `std::array`；多维 C 数组拆分为多个仅保留最后一维的一维字段；类型、`run()`、端口、State 和测试同步改写；无嵌套/长数组展平/成员方法

### ✅ 编译验证
- [ ] `rm -rf build && mkdir build && cd build && cmake ..`
- [ ] `make -j4` 无错误、无警告
- [ ] `test/` 目标编译并运行通过（各模块测试用固定值输入）

**快速核查命令**：

```bash
# 每个 cpp 的非 run 函数定义（期望：主模块无输出，子模块无输出）
for f in src/*.cpp; do
  echo "=== $(basename "$f") ==="
  grep -n "^[a-zA-Z_].*::" "$f" | grep -v "run("
done

# 文件物理行数仅用于查看规模，不用于判定 run() 的有效代码是否超限。
# 另行识别 @type：block 统计函数体有效代码并检查 120 行上限，atomic 豁免。
for f in src/*.cpp; do
  lines=$(wc -l < "$f"); name=$(basename "$f")
  echo "$name: $lines 个文件物理行（非 run 有效行数）"
done

# 头文件洁净：不应有 #define 宏 / constexpr（#pragma once 除外）
grep -rnE "^\s*#\s*define|constexpr" include/ | grep -v "#pragma once" \
  && echo "❌ 头文件存在宏/constexpr" || echo "✅ 头文件洁净"

# 别名唯一来源：Real/Integer 只应在 FuncModule.hpp 定义
grep -rn "using Real\|using Integer\|using UInteger" include/ | grep -v "FuncModule.hpp" \
  && echo "❌ 别名被重复定义" || echo "✅ 别名单一来源"

# 禁止的业务容器：std::array 仅允许用于固定长度 C 数组的 v3.5 按维转换
grep -rnE "std::(vector|map|unordered_map|set|deque|list|span)" include/ src/ \
  && echo "❌ 检出受限容器" || echo "✅ 未检出受限容器"

# std::array 声明逐项追溯来源；一维核对等长，多维核对拆分字段及 <array>
grep -rn "std::array" include/ src/ test/ \
  && echo "⚠️ 检出 std::array，请核对来源、最后一维长度、拆分字段和 <array>" || echo "✅ 未检出 std::array"

# 禁止嵌套 std::array；若命中必须改为多个一维字段
grep -rnE "std::array[[:space:]]*<[[:space:]]*std::array" include/ src/ test/ \
  && echo "❌ 检出嵌套 std::array" || echo "✅ 未检出嵌套 std::array"

# std::array 仅允许下标访问；下列成员调用一律禁止（同时人工核对无展平长数组、指针/引用选择表和选择辅助函数）
grep -rnE "\.(at|size|data|front|back|begin|end|cbegin|cend|fill|swap)\s*\(" include/ src/ test/ \
  && echo "❌ 检出禁止的容器成员方法" || echo "✅ 未检出禁止的容器成员方法"
```

---

## 参考案例：GeoCircle2lineR

224 行单 `run()` → 组件（1 主模块 + 3 子模块），每个 cpp ≤ 120 行，编译无警告，符合 RUN-003/004、BODY-023、FILE-007/008。

**拆分与数据流**：

```
Input → GC2LRPrep → GC2LRCircle → GC2LRArc → Output
        (预处理:      (圆心/端点    (圆弧参数
         坐标转换、    计算)         + 输出映射)
         直线参数)
```

| 子模块 | 行数 | 职责 |
|--------|------|------|
| GC2LRPrep | 113 | 坐标转换、航向规范化、直线参数计算 |
| GC2LRCircle | 90 | 圆心和端点计算 |
| GC2LRArc | 90 | 圆弧参数计算和输出映射 |

**主模块 hpp（构造函数注入 Sub，子模块 Param/State 均空）**：

```cpp
class GeoCircle2lineR : public FuncModule<GeoCircle2lineRTraits> {
 public:
  GeoCircle2lineR(Param param = Param{},
                  State state = State{})
      : FuncModule{
            Sub{
                GC2LRPrep(GC2LRPrepTraits::Param{}, GC2LRPrepTraits::State{}),
                GC2LRCircle(GC2LRCircleTraits::Param{}, GC2LRCircleTraits::State{}),
                GC2LRArc(GC2LRArcTraits::Param{}, GC2LRArcTraits::State{})},
            std::move(param),
            std::move(state)} {}

  void run(const Input& input, Output& output) override;
};
```

**主模块 cpp（只有 run()，级联三子模块 + 汇总输出）**：

```cpp
#include "GeoCircle2lineR.hpp"

namespace apa {

void GeoCircle2lineR::run(const Input& input, Output& output) {
  // 1. 预处理
  GC2LRPrepTraits::Input prep_in;
  prep_in.vstX = input.vstX;
  prep_in.vstY = input.vstY;
  prep_in.vstTheta = input.vstTheta;
  prep_in.vtgtX = input.vtgtX;
  prep_in.vtgtY = input.vtgtY;
  prep_in.vtgtTheta = input.vtgtTheta;
  prep_in.ptyp = input.ptyp;

  GC2LRPrepTraits::Output prep_out;
  sub_.prep.run(prep_in, prep_out);

  // 2. 圆心计算（消费 prep 输出）
  GC2LRCircleTraits::Input circle_in;
  circle_in.radius = input.radius;
  circle_in.lineA1 = prep_out.lineA1;
  circle_in.lineB1 = prep_out.lineB1;
  circle_in.lineC1 = prep_out.lineC1;
  circle_in.lineA2 = prep_out.lineA2;
  circle_in.lineB2 = prep_out.lineB2;
  circle_in.lineC2 = prep_out.lineC2;
  circle_in.trnDir = prep_out.trnDir;
  circle_in.vstTheta = input.vstTheta;
  circle_in.vtgtTheta = input.vtgtTheta;

  GC2LRCircleTraits::Output circle_out;
  sub_.circle.run(circle_in, circle_out);

  // 3. 圆弧参数（消费 prep + circle 输出）
  GC2LRArcTraits::Input arc_in;
  arc_in.radius = input.radius;
  arc_in.edPtX = circle_out.edPtX;
  arc_in.edPtY = circle_out.edPtY;
  arc_in.vtgtX = input.vtgtX;
  arc_in.vtgtY = input.vtgtY;
  arc_in.vtgtTheta = input.vtgtTheta;
  arc_in.vstTheta = input.vstTheta;
  arc_in.cosVtgt = prep_out.cosVtgt;
  arc_in.sinVtgt = prep_out.sinVtgt;
  arc_in.ptyp = input.ptyp;
  arc_in.dir = input.dir;

  GC2LRArcTraits::Output arc_out;
  sub_.arc.run(arc_in, arc_out);

  // 4. 汇总输出
  output.cptX = circle_out.cptX;
  output.cptY = circle_out.cptY;
  output.stPtX = circle_out.stPtX;
  output.stPtY = circle_out.stPtY;
  output.stPtTheta = input.vstTheta;
  output.edPtX = circle_out.edPtX;
  output.edPtY = circle_out.edPtY;
  output.edPtTheta = input.vtgtTheta;
  output.edPtrX = arc_out.edPtrX;
  output.edPtrY = arc_out.edPtrY;
  output.edPtrTheta = arc_out.edPtrTheta;
  output.trnDir = prep_out.trnDir;
  output.goDir = prep_out.goDir;
  output.arcLen = arc_out.arcLen;
  output.arcAngle = arc_out.arcAngle;
}

}  // namespace apa
```

---

**最后更新**：2026-11-16 · **版本**：v3.8 · **状态**：✅ 新增 component/block 的 run() 禁用 return 硬约束，保留既有代码生成规则
