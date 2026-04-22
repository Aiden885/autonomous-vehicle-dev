# ACC 系统 GAASD 实现方案

> 文档状态：进行中（画布已搭建，受 GAASD 代码生成 Bug 阻塞）
> 最后更新：2026-04-17
> 作者：liuruyu

---

## 一、目标与范围

在 GAASD 画布上，用现有 THICV 预置模块 + 新扫描的 ACC 专用模块，搭建一个**纵向一维跟车 ACC**：

- 前车：匀速直线行驶（accel 恒为 0）
- 自车：ACC 控制律 → PID → 限幅 → 运动学模型更新
- 不依赖真实传感器，不需要地图

---

## 二、模块盘点

### 2.1 THICV 库中已有（无需额外操作）

以下模块已在 GAASD 中可用，端口信息已核实：

| 模块名 | 类型 | 输入端口 | 输出端口 | 用途 |
|---|---|---|---|---|
| `vehicleModelUpdate` | atomic-function | `accel:double`, `dt:double`, `state:VehicleState*` | `state:VehicleState*` | 前车/自车状态推进 |
| `pid` | atomic-block | `state:PIDState*`, `error:double` | `state:PIDState*`, `output:double` | 速度误差→加速度指令 |
| `limit` | atomic-function | `v:double`, `min:double`, `max:double` | `output:double` | 加速度限幅 |

> `pid` 的 kp/ki/kd/dt/integral_max/derivative_max 通过 `base_config` 在画布属性面板填写，不走连线。

### 2.2 需要新建扫描包（3 个函数）

| 函数名 | 作用 | 源文件 | 状态 |
|---|---|---|---|
| `accComputeTargetSpeed` | ACC 控制律：(egoV, leadV, distance) → targetSpeed | `ACC/accComputeTargetSpeed.c` | ✅ 已导入 |
| `computeDistance1D` | 纵向车距：(leadX, egoX) → distance | `ACC/computeDistance1D.c` | ✅ 已导入 |
| `computeSpeedError` | 速度误差：(targetSpeed, egoV) → error | `ACC/computeSpeedError.c` | ✅ 已导入 |

> **为什么需要 `computeDistance1D` 和 `computeSpeedError`？**
> GAASD 画布只能连接模块端口，不支持直接写算术表达式。`lead.x - ego.x` 和 `targetSpeed - egoV` 必须封装成独立函数才能在画布上连线。

---

## 三、准备工作：新建 C 函数

### 3.1 `computeDistance1D.h / .c`

在 `planningFigure/ACC/` 目录下新建：

**computeDistance1D.h**
```c
#ifndef COMPUTE_DISTANCE_1D_H
#define COMPUTE_DISTANCE_1D_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 计算纵向一维车间距（前车 x 减自车 x）
 * @en_name computeDistance1D
 * @cn_name 纵向车间距计算
 * @type widget
 * @param[IN] double leadX  前车纵向位置（m）
 * @param[IN] double egoX   自车纵向位置（m）
 * @retval double 车间距（m），正值表示前车在自车前方
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 规划与决策
 * @tag_level2 速度规划
 * @version 1.0
 * @date 2026-04-13
 * @author liuruyu
 */
double computeDistance1D(double leadX, double egoX);

#ifdef __cplusplus
}
#endif

#endif /* COMPUTE_DISTANCE_1D_H */
```

**computeDistance1D.c**
```c
#include "computeDistance1D.h"

double computeDistance1D(double leadX, double egoX)
{
    double distance = 0.0;
    distance = leadX - egoX;
    return distance;
}
```

将 `./ACC/computeDistance1D.c` 加入 CMakeLists.txt 的 SRC_FILES。

---

## 四、制作扫描包

扫描包目录结构（必须严格符合）：
```
acc_modules/
├── cbdes.db          ← SQLite，component 表需有对应行
└── component/
    ├── {uuid-A}.json ← accComputeTargetSpeed
    └── {uuid-B}.json ← computeDistance1D
```

### 4.1 生成两个 UUID

```bash
python3 -c "import uuid; print(uuid.uuid4()); print(uuid.uuid4())"
# 示例输出（每次不同，记下这两个值）:
# a1b2c3d4-e5f6-7890-abcd-ef1234567890   ← UUID_A（accComputeTargetSpeed）
# b2c3d4e5-f6a7-8901-bcde-f12345678901   ← UUID_B（computeDistance1D）
```

### 4.2 编写 `{UUID_A}.json`（accComputeTargetSpeed）

将下方 JSON 保存为 `component/{UUID_A}.json`，**把三处 `{UUID_A}` 替换为实际 UUID**：

```json
{
  "name": "accComputeTargetSpeed",
  "componentType": "atomic-function",
  "alias": "ACC目标速度计算",
  "description": "根据自车速度、前车速度、车间距，计算ACC目标跟车速度",
  "author": "liuruyu",
  "version": "1.0",
  "instanceId": "",
  "abstractId": "{UUID_A}",
  "originId": "{UUID_A}",
  "isCustom": false,
  "customType": 0,
  "isAtomic": true,
  "isSourceCode": 0,
  "children": [],
  "parent": "",
  "tags": ["功能模块库", "规划与决策", "速度规划"],
  "vendor": "ACC",
  "extensionProps": [
    {
      "key": "function_input_ports",
      "value": [
        {
          "id": "accComputeTargetSpeed_egoV_In",
          "name": "egoV",
          "alias": "egoV",
          "direction": "In",
          "description": "自车当前速度(m/s)",
          "props": [
            { "key": "dataType", "value": "double", "props": [] },
            { "key": "idx",      "value": 1,        "props": [] },
            { "key": "isReturnFlag", "value": false, "props": [] }
          ]
        },
        {
          "id": "accComputeTargetSpeed_leadV_In",
          "name": "leadV",
          "alias": "leadV",
          "direction": "In",
          "description": "前车当前速度(m/s)",
          "props": [
            { "key": "dataType", "value": "double", "props": [] },
            { "key": "idx",      "value": 2,        "props": [] },
            { "key": "isReturnFlag", "value": false, "props": [] }
          ]
        },
        {
          "id": "accComputeTargetSpeed_distance_In",
          "name": "distance",
          "alias": "distance",
          "direction": "In",
          "description": "纵向车间距(m)",
          "props": [
            { "key": "dataType", "value": "double", "props": [] },
            { "key": "idx",      "value": 3,        "props": [] },
            { "key": "isReturnFlag", "value": false, "props": [] }
          ]
        }
      ]
    },
    {
      "key": "function_output_ports",
      "value": [
        {
          "id": "accComputeTargetSpeed_output",
          "name": "output",
          "alias": "targetSpeed",
          "direction": "Out",
          "description": "目标跟车速度(m/s)，已限幅至[0, ACC_MAX_SPEED]",
          "props": [
            { "key": "dataType",     "value": "double", "props": [] },
            { "key": "idx",          "value": 4,        "props": [] },
            { "key": "isReturnFlag", "value": true,     "props": [] }
          ]
        }
      ]
    }
  ],
  "childConnections": [],
  "childComponents": []
}
```

### 4.3 编写 `{UUID_B}.json`（computeDistance1D）

```json
{
  "name": "computeDistance1D",
  "componentType": "atomic-function",
  "alias": "纵向车间距计算",
  "description": "计算前车与自车的纵向一维距离(leadX - egoX)",
  "author": "liuruyu",
  "version": "1.0",
  "instanceId": "",
  "abstractId": "{UUID_B}",
  "originId": "{UUID_B}",
  "isCustom": false,
  "customType": 0,
  "isAtomic": true,
  "isSourceCode": 0,
  "children": [],
  "parent": "",
  "tags": ["功能模块库", "规划与决策", "速度规划"],
  "vendor": "ACC",
  "extensionProps": [
    {
      "key": "function_input_ports",
      "value": [
        {
          "id": "computeDistance1D_leadX_In",
          "name": "leadX",
          "alias": "leadX",
          "direction": "In",
          "description": "前车纵向位置(m)",
          "props": [
            { "key": "dataType", "value": "double", "props": [] },
            { "key": "idx",      "value": 1,        "props": [] },
            { "key": "isReturnFlag", "value": false, "props": [] }
          ]
        },
        {
          "id": "computeDistance1D_egoX_In",
          "name": "egoX",
          "alias": "egoX",
          "direction": "In",
          "description": "自车纵向位置(m)",
          "props": [
            { "key": "dataType", "value": "double", "props": [] },
            { "key": "idx",      "value": 2,        "props": [] },
            { "key": "isReturnFlag", "value": false, "props": [] }
          ]
        }
      ]
    },
    {
      "key": "function_output_ports",
      "value": [
        {
          "id": "computeDistance1D_output",
          "name": "output",
          "alias": "distance",
          "direction": "Out",
          "description": "纵向车间距(m)",
          "props": [
            { "key": "dataType",     "value": "double", "props": [] },
            { "key": "idx",          "value": 3,        "props": [] },
            { "key": "isReturnFlag", "value": true,     "props": [] }
          ]
        }
      ]
    }
  ],
  "childConnections": [],
  "childComponents": []
}
```

### 4.4 创建 cbdes.db

```bash
cd acc_modules
sqlite3 cbdes.db "
CREATE TABLE component (
  id integer not null PRIMARY KEY AUTOINCREMENT,
  originId text not null unique,
  name text not null DEFAULT 'Unknown',
  alias text not null DEFAULT 'Unknown',
  version text not null DEFAULT 'Unknown',
  author text not null DEFAULT 'Unknown',
  componentType text not null DEFAULT 'module',
  description text not null DEFAULT 'Unknown',
  tags text not null DEFAULT 'Unknown',
  isSourceCode integer not null DEFAULT 0,
  isAtomic integer not null DEFAULT 0,
  isCustom integer not null DEFAULT 0,
  customType integer not null DEFAULT 0,
  configProto text not null DEFAULT 'Unknown',
  namespace text not null DEFAULT 'Unknown',
  instanceId text,
  children text,
  parent text,
  vendor text not null DEFAULT 'Unknown'
);

INSERT INTO component
  (originId, name, alias, version, author, componentType, description,
   tags, isSourceCode, isAtomic, isCustom, customType,
   configProto, namespace, vendor)
VALUES
  ('{UUID_A}', 'accComputeTargetSpeed', 'ACC目标速度计算', '1.0', 'liuruyu',
   'atomic-function', '根据自车速度、前车速度、车间距，计算ACC目标跟车速度',
   '功能模块库,规划与决策,速度规划', 0, 1, 0, 0,
   'Unknown', 'Unknown', 'ACC'),

  ('{UUID_B}', 'computeDistance1D', '纵向车间距计算', '1.0', 'liuruyu',
   'atomic-function', '计算前车与自车的纵向一维距离(leadX - egoX)',
   '功能模块库,规划与决策,速度规划', 0, 1, 0, 0,
   'Unknown', 'Unknown', 'ACC');
"
```

### 4.5 打包

```bash
tar -czf acc_modules.tar.gz acc_modules/
```

---

## 五、导入 GAASD

1. 启动 GAASD：`gaasd`
2. 菜单 → **设置** → **组件库管理**
3. 点击「导入组件包」，选择 `acc_modules.tar.gz`
4. GAASD 校验通过后，左侧组件面板会出现 `ACC` 厂商分类，包含两个新模块

> **如果导入报错**：
> - `originId和当前文件名不一致`：检查 JSON 文件名是否与 `originId` 字段完全一致（不含 `.json`）
> - `缺少必要字段`：对照第四节检查 16 个必填字段是否齐全

---

## 六、GAASD 画布连线

### 6.1 数据流图

```
常量 0.0 ──────────────────────────────────────────────────────────┐
                                                                    ↓
VehicleState(Lead) ──→ vehicleModelUpdate(Lead) ──→ leadState.x ──→ computeDistance1D ──→ distance ──→ accComputeTargetSpeed ──→ targetSpeed
                                                                                                           ↑                           ↓
VehicleState(Ego)  ──────────────────────────────────→ egoState.x ─┘          egoState.v ────────────────┘              speedError = targetSpeed - egoState.v
                    ↑                                                                                                                  ↓
                    └──── vehicleModelUpdate(Ego) ←── clampedAccel ←── limit(-3, 2) ←── pid(kp=1.0, ki=0.1, kd=0.05) ←── speedError
```

### 6.2 模块实例与端口连线表

| 步骤 | 模块实例 | 输入端口 | 来源 |
|---|---|---|---|
| 1 | `vehicleModelUpdate_Lead` | `accel` | 常量 `0.0` |
| 1 | `vehicleModelUpdate_Lead` | `dt` | 常量（如 `0.1`） |
| 1 | `vehicleModelUpdate_Lead` | `state` | 全局 `leadState` |
| 2 | `computeDistance1D` | `leadX` | `vehicleModelUpdate_Lead.state.x` |
| 2 | `computeDistance1D` | `egoX` | 全局 `egoState.x` |
| 3 | `accComputeTargetSpeed` | `egoV` | 全局 `egoState.v` |
| 3 | `accComputeTargetSpeed` | `leadV` | `vehicleModelUpdate_Lead.state.v` |
| 3 | `accComputeTargetSpeed` | `distance` | `computeDistance1D.output` |
| 4 | `pid` | `error` | `accComputeTargetSpeed.output - egoState.v`（见注意）|
| 4 | `pid` | `state` | 全局 `pidState` |
| 4 | `pid` **base_config** | kp=1.0, ki=0.1, kd=0.05, dt=0.1, integral_max=5.0, derivative_max=3.0 | 属性面板填写 |
| 5 | `limit` | `v` | `pid.output` |
| 5 | `limit` | `min` | 常量 `-3.0` |
| 5 | `limit` | `max` | 常量 `2.0` |
| 6 | `vehicleModelUpdate_Ego` | `accel` | `limit.output` |
| 6 | `vehicleModelUpdate_Ego` | `dt` | 常量 `0.1` |
| 6 | `vehicleModelUpdate_Ego` | `state` | 全局 `egoState` |

> **注意 — `targetSpeed - egoState.v` 的处理**：
> GAASD 不支持直接写减法表达式。若画布不提供 subtract 原语，需再扫描一个 `computeSpeedError(targetSpeed, egoV) → error` 函数（类似 computeDistance1D，逻辑为 `return targetSpeed - egoV`）。
> 先尝试连线看 GAASD 是否支持内联表达式；若不支持再新建该函数。

### 6.3 全局状态变量初始值

在 GAASD 项目配置中设置：
```
leadState.x = 30.0   // 前车初始位置（m），比自车超前30m
leadState.v = 5.0    // 前车初速度（m/s ≈ 18km/h）
leadState.a = 0.0
egoState.x  = 0.0   // 自车初始位置
egoState.v  = 0.0   // 自车初速度
egoState.a  = 0.0
pidState.integral  = 0.0
pidState.lastError = 0.0
```

---

## 七、已知限制与风险

| 问题 | 当前状态 | 规避方案 |
|---|---|---|
| `targetSpeed - egoState.v` 不能直接连线 | ✅ 已解决 | 使用 `computeSpeedError.c` 封装，已扫描导入 |
| GAASD 是否支持同一 `VehicleState*` 同时被读写 | 已验证可用 | 生成代码未见数据竞争 |
| `pid` 的 kp/ki/kd 在 base_config 里是常量字符串，不能动态调参 | 已知 | 仿真阶段固定参数，可接受 |
| `accSimStep.c` 和 GAASD 画布是两套实现 | 当前并存 | C 层用于快速验证逻辑；GAASD 层用于可视化展示，两者保持一致 |

---

## 九、实际执行中发现的 GAASD Bug（2026-04-17 新增）

### Bug 1：Docker 编译环境缺少 `protobuf-c`

**现象**：点击「运行仿真」后，Docker 内编译报错：
```
fatal error: protobuf-c/protobuf-c.h: No such file or directory
```

**原因**：GAASD 把项目 frame 识别为 THICV，在 `accpro1.h` 里自动插入所有 THICV 依赖（含 protobuf-c），但仿真 Docker 镜像里没有这个库。

**规避方案**：每次「生成代码」后运行 `fix_accpro1.sh` 打补丁（注释掉 include，添加 stub typedef）。

---

### Bug 2：struct-reference 组件生成错误的字段访问语法

**现象**：生成的 `execute_*.c` 中，`vehiclePoint->x` 被生成为 `vehiclePoint->.x` 或 `vehiclePoint->..x`，导致编译报错：
```
error: expected identifier before '.' token
```

**原因**：GAASD struct-reference 组件在存储字段路径时带了多余的前缀 `.`，代码生成器直接拼接后产生非法 C 语法。

**规避方案**：`fix_accpro1.sh` 中用 sed 修正。

---

### Bug 3：自定义函数之间的连线不生成正确的 temp 变量（核心阻塞问题）

**现象**：画布连线已正确（数据库 `project_connection` 表验证），但生成的代码里，同一根线的"输出端"和"输入端"被分配了**不同编号**的 temp 变量，且两者均未声明：

```c
// 期望：两端用同一个 temp 变量
temp724931 = computeDistance1D(leadX, egoX);        // 输出端 temp724931
accComputeTargetSpeed(egoV, leadV, temp724669);     // 输入端 temp724669 ← 不同！且未声明
```

编译报错：`error: 'temp724931' undeclared`、`error: 'temp724669' undeclared`

**影响范围**：仅发生在 for_body execute 块内，**codescan 导入的自定义函数**互相串联时。预置组件（pid、vehicleModelUpdate、address-of）之间的连线不受影响。

**根本原因**：`gaas_codegen` 代码生成器对自定义函数输出端口的 temp 变量 ID 分配逻辑与输入端口不一致（已向开发者报告）。

**当前状态**：🔴 阻塞中，待开发者修复或使用 `fix_accpro1.sh` 直接重写函数体绕过。

---

### 每次运行仿真的标准流程（因 Bug 1/2 存在）

```bash
# 步骤 1：在 GAASD 画布调整完成后，点「生成代码」

# 步骤 2：打补丁（修复 protobuf、字段访问语法、宏缺失）
bash /data/aiden/文档/Modularization/fix_accpro1.sh

# 步骤 3：在 GAASD 示波器点「运行」（不要再点「生成代码」）
```

> ⚠️ 重要：步骤 2 必须在步骤 1 之后、步骤 3 之前执行。
> 每次点「生成代码」都会覆盖补丁，必须重新运行脚本。

---

## 八、执行顺序总结（2026-04-17 更新）

```
[已完成]
1. 编写 accComputeTargetSpeed.c/.h           ✅
2. 编写 accSimStep.c/.h（C层仿真验证用）     ✅
3. 更新 CMakeLists.txt                       ✅
4. 新建 computeDistance1D.c/.h               ✅
5. 新建 computeSpeedError.c/.h               ✅
6. 制作 acc_modules 扫描包并导入 GAASD      ✅
7. 画布连线（pidTestMain for_body 内）       ✅ 连线已正确（DB验证）
8. 编写 fix_accpro1.sh 补丁脚本             ✅

[阻塞中]
9. 生成代码 / 运行仿真                      🔴 因 GAASD Bug 3（temp变量错误）编译失败
   → 待选方案 A：等待 GAASD 开发者修复
   → 待选方案 B：在 fix_accpro1.sh 中直接重写 execute_dd594018.c 函数体，绕过代码生成器

[未开始]
10. 观察示波器中 egoV 收敛曲线              ⏳
11. 调参（kp/ki/kd）                        ⏳
```
