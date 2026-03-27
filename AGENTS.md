# AGENTS.md — 规划控制模块化项目（Claude Code & Codex 共享规范）

## 项目概述

自动驾驶小车（Scout底盘 + Livox激光雷达）软件模块化重构。
将 C++ 算法代码转换为 C 语言，C++ 保留基础设施（线程/ZMQ/Protobuf）。
多个独立进程通过 ZMQ + Protobuf 通信。

## 技术栈

- 语言：C（算法层）+ C++17（框架层）
- 构建：CMake 3.16+
- 通信：ZMQ 4.3 + Protobuf 3.6.1（C++）/ protobuf-c 1.3.3（C）
- 依赖：OpenCV 4.2、Eigen3、Qt5、yaml-cpp、Boost、libxml2
- 编译环境：Docker 容器 `modularization-dev:latest`，进入方式：`./dev.sh start`

## 目录结构

```
planningFigure（复件）/   ← 原始C++参考，只读，不修改
planning/                 ← 规划模块最终目标（Gitee仓库，只放C文件）
huanyuan1/                ← 规划中间验证版（C+C++共存，测试用）
controller/               ← 控制原始C++参考，只读，不修改
Controller/               ← 控制模块最终目标（C算法+C++封装）
DataStructure/            ← C工具库（dynArray/minHeap/hashMap，待创建）
proto/                    ← Protobuf消息定义
mmap/                     ← 纯C地图数据结构
```

## 构建命令

```bash
# 进入Docker开发环境
./dev.sh start

# 编译验证（在容器内）
cd /workspace/huanyuan1/thicv-pilot/planningFigure
mkdir -p build && cd build && cmake .. && make -j$(nproc)
```

---

## 代码规范（来源：函数规范.xlsx，以xlsx为准）

### 1. 命名规范

| 对象 | 规则 | 示例 |
|------|------|------|
| 函数名 | 小驼峰 | `generateBezierPath` |
| 变量名 | 小驼峰 | `startIndex` |
| 结构体字段名 | **大驼峰** | `WheelBase`、`VehicleSpeedRange` |
| 文件夹 | 大驼峰 | `BezierSpline` |
| 源文件/头文件 | 小驼峰 | `bezierSpline.c` |
| C++类转C结构体 | 原名加 `Cpp` 后缀 | `TrajectoryPointCpp` |
| 函数中文名 | 不超过12个汉字 | — |
| 编码 | 统一 UTF-8 | — |

### 2. 禁止项

- 禁止 `goto`
- 禁止函数中途 `return`（只允许函数末尾一个 `return`）
- 禁止 `break`
- 禁止全局变量
- 编译警告必须全部解决（警告等级调到最高：`-Wall -Wextra`）

### 3. 函数规模

- 单个函数行数 ≤ 200 行
- 函数功能应单一

### 4. 控制流格式（严格遵守）

**if/else 规范：**
- 输入条件必须是单一变量（由函数或比较赋值得到）
- 执行体必须是单一函数调用
- 不允许在 if/else 内提前 return

```c
bool cond;

cond = compare(a, b);

if (cond) {
    ifBody(x);
} else {
    elseBody(x);
}
```

**for 规范：**
- 起始值、步进值、终止值均须在 for 外声明为独立变量
- 执行体必须是单一函数调用
- for 语句写法固定为 `for (r1; r1 < r3; r1 = r1 + r2)`

```c
int r1;    /* 初始值 */
int r2;    /* 步进值 */
int r3;    /* 终止值 */

r1 = 0;
r2 = 1;
r3 = n;

for (r1; r1 < r3; r1 = r1 + r2) {
    forBody(r1, x);
}
```

---

## 注释规范（来源：函数规范.xlsx，以xlsx为准）

### 函数注释模板

```c
/**
 * @brief 模块详细描述
 * @en_name 函数英文名称
 * @cn_name 函数中文名称
 * @type widget | module | widget&module
 * @param cn_name=参数中文名称|none, en_name=参数英文名称|none, desc=参数描述, unit=单位|1, type=数据类型|none, range=[理论取值范围], default=默认值
 * @param[IN] 参数数据类型 参数名称 参数描述
 * @param[OUT] 参数数据类型 参数名称 参数描述
 * @var 变量数据类型 变量名称 变量描述
 * @retval 返回值类型 返回值名称 返回值描述
 * @granularity atomic | composite
 * @tag_level1 库类别
 * @tag_level2 库类别
 * @formula 公式描述（使用 LaTeX 语法，用 \f[ \f] 包裹）
 * @version 版本信息
 * @date YYYY-MM-DD
 * @author 作者
 */
```

> **字段说明：**
> - `@type`：`widget` = 原子函数（控件）；`module` = 复合函数；两者兼有写 `widget&module`
> - `@param`（第一行）：当 `@type` 包含 `widget` 时必填，描述可配置参数；`module` 类型不需要
> - `@param[IN]/[OUT]`：描述函数的输入输出参数
> - `@var`：描述函数内部关键局部变量
> - `@granularity`：`atomic` = 原子粒度；`composite` = 复合粒度
> - `@formula`：有数学公式时填写，使用 LaTeX 语法

**函数注释示例（PID控制器）：**

```c
/**
 * @brief 单自由度 PID 控制器模块
 * @en_name PID_Controller
 * @cn_name 单自由度 PID 控制器
 * @type widget
 * @param cn_name=比例增益, en_name=Kp, desc=比例环节增益, unit=1, type=Real, range=[0,100], default=1.0
 * @param cn_name=积分增益, en_name=Ki, desc=积分环节增益, unit=1, type=Real, range=[0,50], default=0.1
 * @param cn_name=微分增益, en_name=Kd, desc=微分环节增益, unit=1, type=Real, range=[0,20], default=0.01
 * @param[IN] Real setpoint 期望值
 * @param[IN] Real feedback 实际反馈值
 * @param[OUT] Real controlOutput PID控制输出
 * @var Real prevError 上一采样周期误差值
 * @retval None 无返回值，控制输出通过参数返回
 * @granularity atomic
 * @tag_level1 control
 * @formula PID公式：
 * \f[
 * u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}
 * \f]
 * @version 1.0
 * @date 2026-03-27
 * @author 作者
 */
```

---

### 结构体注释模板

```c
/**
 * @brief 功能描述
 * @en_name 结构体英文名称
 * @cn_name 结构体中文名称
 * @tag STRUCT
 * @field name=字段名称, type=类型, unit=单位|1, desc=字段描述
 * @version 版本
 * @date YYYY-MM-DD
 * @author 作者
 */
typedef struct {
    TypeA FieldNameA;    /* 描述 单位 */
    TypeB FieldNameB;    /* 描述 单位 */
} ExampleStruct;
```

> **注意：**
> - 结构体字段名使用**大驼峰**（`WheelBase`，不是小驼峰）
> - 字段行内注释格式：`/* 描述 单位 */`
> - 如字段是数组，`@field` 的 type 写 `Array<元素类型, [维度]>`，例如 `Array<Real, [2]>`

**结构体注释示例：**

```c
/**
 * @brief 存储轴距和车速范围结构体
 * @en_name VehicleParams
 * @cn_name 车辆参数结构体
 * @tag STRUCT
 * @field name=WheelBase, type=Real, unit=m, desc=轴距
 * @field name=VehicleSpeedRange, type=Array<Real, [2]>, unit=m/s, desc=车速范围
 * @version 1.0
 * @date 2026-03-27
 * @author 作者
 */
typedef struct {
    double WheelBase;             /* 轴距 m */
    double VehicleSpeedRange[2];  /* 车速范围 m/s */
} VehicleParams;
```

---

### 枚举注释模板

```c
/**
 * @brief 功能描述
 * @en_name 枚举英文名称
 * @cn_name 枚举中文名称
 * @tag ENUM
 * @field name=字段名称, unit=1, desc=字段描述
 * @version 版本
 * @date YYYY-MM-DD
 * @author 作者
 */
typedef enum {
    FieldA = 0,    /* 描述 */
    FieldB,        /* 描述 */
} ExampleEnum;
```

**枚举注释示例：**

```c
/**
 * @brief 枚举进程的各个状态
 * @en_name E_ProcState
 * @cn_name 进程状态枚举
 * @tag ENUM
 * @field name=InitialProcState, unit=1, desc=进程状态初值
 * @field name=Working, unit=1, desc=进程正常启动/进程处于运行状态
 * @version 1.0
 * @date 2026-03-27
 * @author 作者
 */
typedef enum {
    InitialProcState = 0,    /* 进程状态初值 */
    Working,                 /* 进程正常启动/进程处于运行状态 */
} E_ProcState;
```

---

### 宏定义注释

宏定义不呈现组件，按日常代码习惯即可：

```c
#define DMAMOD 0xF0002000    /* DMA对象地址 */
```

---

## C++ → C 转换规则

1. 类 → 结构体（名称加 `Cpp` 后缀）+ 独立函数
2. `std::vector` → `DynArray`（来自 `DataStructure/dynArray.h`）
3. `std::priority_queue` → `MinHeap`（来自 `DataStructure/minHeap.h`）
4. `std::unordered_map` → `HashMap`（来自 `DataStructure/hashMap.h`）
5. `std::string` → `char*`（手动管理内存）
6. 虚函数/继承 → 函数指针（存入结构体）
7. try-catch → 错误码返回
8. 所有C头文件加 `extern "C"` 桥接宏
9. Protobuf C++ `.pb.cc/.pb.h` → C版 `.pb-c.c/.pb-c.h`

---

## 当前转换进度

### 规划模块（planning/）
- ✅ Dijkstra全局规划
- ✅ 贝塞尔曲线
- ✅ 速度规划 generateSpeedList
- ✅ 最优轨迹选取
- ✅ 停止点判断
- 🔴 A*全局规划（10个函数待转换）
- 🔴 线性插值 interpolate_m.cpp
- 🔴 坐标转换 geometry_m.cpp / convert.cpp
- 🔴 局部规划主逻辑 planningFigure_m.cpp
- 🔴 速度优化 CTSpeedTrajectory / CTSpeedOptimization

### 控制模块（Controller/）
- ✅ PID控制器
- ✅ 纯跟踪（Pure Pursuit）
- ✅ 车辆运动学模型
- 🔴 LQR控制器（需要矩阵库）
- 🔴 MPC控制器（需要矩阵库）
- 🔴 LonController纵向控制
- 🔴 平滑处理

### 工具库（DataStructure/）
- 🔴 dynArray（动态数组）
- 🔴 minHeap（最小堆，A*用）
- 🔴 hashMap（哈希表，A*用）
- 🔴 matrixOps（矩阵运算，LQR/MPC用）
