# 自动驾驶模块化系统 (Modularization)

> **项目背景**：面向自动驾驶小车（Scout 底盘 + Livox 激光雷达）的软件模块化重构项目。
> **核心任务**：将 `planningFigure（复件）/` 中的原始 C++ 规划代码、`controller/` 中的原始 C++ 控制代码，逐一转换为 C 语言，分别提交到 `planning/` 和 `Controller/`。
> **架构策略**：C 语言实现算法，C++ 负责线程/ZMQ/Protobuf 基础设施，多个独立进程通过 ZMQ 通信。

---

## 系统架构

```
传感器层 (driver/)
  ├── GPS/IMU          → Localization/
  ├── Livox 激光雷达   → perception/
  ├── 摄像头           → perception/
  └── Scout 底盘串口   → driver/coms/
          ↓ ZMQ + Protobuf
感知层 (perception/)
  ├── 计算机视觉 (OpenCV)
  ├── 点云处理
  └── 多目标追踪 (Kalman 滤波)
          ↓
定位层 (Localization/)
  ├── GPS/IMU 融合
  └── 坐标系转换 (ENU ↔ WGS84)
          ↓
规划层  ← 当前工作重点
  ├── 全局规划：A*
  ├── 局部规划：贝塞尔曲线 + 最优轨迹选取
  ├── 速度规划：速度列表生成 + 优化
  └── 停止点判断
          ↓
控制层  ← 当前工作重点
  ├── 纵向控制：PID
  ├── 横向控制：纯跟踪 / LQR / MPC
  └── 速度轨迹平滑
          ↓
路侧通信 (v2xserver/)  ← 暂不动
  └── V2X 路侧服务器
```

---

## 目录说明

### 核心工作目录

| 目录 | 角色 | 说明 |
|------|------|------|
| `planningFigure（复件）/` | **规划：原始C++参考** | 原始可运行的C++规划代码，转换时的参照基准，**不修改** |
| `planning/` | **规划：最终C语言目标** | GitLab仓库，只放已转换的C文件，最终交付在这里 |
| `huanyuan1/` | **规划：中间验证版本** | C++和C共存，用来验证转换后C代码能否跑通，过渡产物 |
| `controller/`（小写） | **控制：原始C++参考** | 原始可运行的C++控制代码，转换时的参照基准，**不修改** |
| `Controller/`（大写） | **控制：最终C语言目标** | C算法+C++封装，已部分转换，继续在这里开发 |

### 转换工作流程

```
【规划】
planningFigure（复件）/src/*.cpp
        ↓ 转换为 C
planning/thicv-pilot/planningFigure/对应子目录/*.c
        ↓ 集成验证
huanyuan1/ 中与C++一起编译运行

【控制】
controller/trajectory_tracking/src/*.cpp
        ↓ 转换为 C
Controller/src/*.c
```

### 重要命名规则

> **原始C++的结构体名必须加 `Cpp` 后缀**，避免C和C++代码共存时命名冲突。
> 例：C++ 的 `struct TrajectoryPoint` → C 中改为 `struct TrajectoryPointCpp`

### 其他目录（暂不转换）

| 目录 | 语言 | 功能 |
|------|------|------|
| `v2xserver/` | C++ | V2X路侧服务器，ZMQ+Protobuf通信 |
| `perception/` | C++ | 感知（视觉/点云/Kalman追踪） |
| `Localization/` | C++ | 定位与坐标转换 |
| `driver/` | C++ | 硬件驱动（激光雷达/GPS/Scout底盘） |
| `Logic/` | C++ | 决策树（CART/ID3） |
| `proto/` | Protobuf | 所有模块共用的消息定义 |
| `mmap/` | 纯C | 道路地图内存结构（OpenDRIVE格式） |
| `Thread/` | C++ | 多线程仿真定位框架 |

### 辅助文件

| 文件 | 说明 |
|------|------|
| `protobuf-all-3.6.1.gz` | Protobuf C++ 源码包（**Docker构建用，勿删**） |
| `protobuf-c-1.3.3.gz` | Protobuf C 源码包（**Docker构建用，勿删**） |
| `函数规范.xlsx` | 函数API规范表（C转换时遵守的接口标准） |
| `规划控制模块梳理-1.pdf` | 规划控制模块思维导图（飞书白板导出） |

---

## 规划模块详细拆解

> 颜色规则（来自飞书白板）：**红色 = C++ 待转换，蓝色 = 已转为C**

### 一、地图读取（RoadMap）

| 函数 | 说明 | 状态 |
|------|------|------|
| `MapAnalysis` | 解析地图 | — |
| `neighborLaneCont` | 相邻lane搜索 | — |
| `mapRoadPCCheck` | 地图检索 | — |
| `GetRoadByRoadID` | 根据roadId获取road对象 | — |
| `GetLaneByLaneID` | 根据roadId和laneId获取lane对象 | — |
| `GetStartPointByRoadID` | 根据roadId获取起点坐标 | — |
| `GetEndPointByRoadID` | 根据roadId获取终点坐标 | — |
| `GetLaneByRoadIDLaneID` | 根据roadId、laneId获取lane对象 | — |
| `GetLaneByRoadIDLaneIDpointID` | 根据roadId、laneId、pointId获取point对象 | — |

### 二、全局规划（processGlobalPlanning） — 两套算法并存，均需保留

#### 2a. Dijkstra 算法
| 函数 | 说明 | 状态 |
|------|------|------|
| `dijkstra` 系列 | Dijkstra 全局路径规划 | 🔵 已完成（`dijkstra/dijkstra.c`） |

#### 2b. A* 算法
| 函数 | 说明 | 状态 |
|------|------|------|
| `AstarMapSolve` | A* 主入口 | 🔴 待转换 |
| `AstarInit` | A* 初始化 | 🔴 待转换 |
| `AstarNext` | A* 下一步扩展 | 🔴 待转换 |
| `AstarLink` | A* 连接节点 | 🔴 待转换 |
| `AstarRoad` | A* 道路搜索 | 🔴 待转换 |
| `AstarGetPath` | A* 获取路径 | 🔴 待转换 |
| `AstarFindLane` | A* 查找车道 | 🔴 待转换 |
| `AstarFindLaneID` | A* 查找车道ID | 🔴 待转换 |
| `AstarRecursiveFindLane` | A* 递归查找车道 | 🔴 待转换 |
| `AstarNextSelfCheck` | A* 自检 | 🔴 待转换 |

### 三、局部规划（localPlanning）

| 函数 | 说明 | 状态 |
|------|------|------|
| `getOptimalTrajectoryIndex` | 最优轨迹选取 | 🔵 已完成（`GetOptimalTrajectoryIndex/getOptimalTrajectoryIndex.c`） |
| `getDistance` | 距离计算 | 🔵 已完成 |
| Bezier曲线生成系列 | 局部轨迹生成 | 🔵 已完成（`BezierSpline/Bezier.c`） |
| `stopPointJudge` | 停止点判断 | 🔵 已完成（`StopPointJudge/stopPointJudge.c`） |
| 局部规划主逻辑 | `planningFigure_m.cpp` 对应C实现 | 🔴 待转换 |
| 坐标转换 | `geometry_m.cpp` / `convert.cpp` | 🔴 待转换 |
| 线性插值 | `interpolate_m.cpp` | 🔴 待转换 |

### 四、速度规划

| 函数 | 说明 | 状态 |
|------|------|------|
| `generateSpeedList` | 速度列表生成 | 🔵 已完成（`GenerateSpeedList/generateSpeedList.c`） |
| `generateBezierPath` | 贝塞尔路径生成 | 🔵 已完成 |
| `generateOptimalSpeedPath` | 最优速度路径 | 🔴 待转换 |
| `getSpeedFromTraj` | 从轨迹获取速度 | 🔴 待转换 |
| `CTSpeedTrajectory` | 速度轨迹 | 🔴 待转换 |
| `CTSpeedOptimization` | 速度优化 | 🔴 待转换 |
| `calculateAcceleration` | 加速度计算 | 🔴 待转换 |
| `calculateCurvature` | 曲率计算 | 🔴 待转换 |

---

## 控制模块详细拆解

### 一、LatControllerBase（横向控制基类）→ 纵向PID控制

| 函数/模块 | 说明 | 状态 |
|-----------|------|------|
| 多步规化 | — | 🔴 待转换 |
| 控制前向预测 | — | 🔴 待转换 |
| 间隔点选择 | `getPreviewIndex` | 🔵 已完成（`Controller/src/getPreviewIndex.c`） |
| 前瞻距离计算 | — | 🔴 待转换 |
| 速度点数量 | — | 🔴 待转换 |
| 计算方程（前向追踪+MPC估算） | — | 🔴 待转换 |
| PID输出更新 | `pidController` | 🔵 已完成（`Controller/src/pidController.c`） |

### 二、横向控制器

| 控制器 | 状态 |
|--------|------|
| 纯跟踪（Pure Pursuit） | 🔵 已完成（`Controller/src/purePursuit.c`） |
| LQR控制器 | 🔴 待转换（原始在 `controller/src/controlLQRController.cc`） |
| MPC控制器 | 🔴 待转换 |

### 三、LonController（纵向控制器）

| 函数/模块 | 说明 | 状态 |
|-----------|------|------|
| 速度轨迹与平滑速度 | — | 🔴 待转换 |
| 纵向控制量输出 | — | 🔴 待转换 |

### 四、其他控制模块

| 模块 | 状态 |
|------|------|
| 碰撞检测和障碍物处理 | 🔴 待转换 |
| 动态障碍物坐标系转换 | 🔴 待转换 |
| 发布车辆控制指令（ZMQ） | 保留在C++ |

---

## 开发环境（Docker）

项目使用 Docker 隔离编译环境，**不影响宿主机已有的 Anaconda/Python 等环境**。

### 镜像依赖

| 库 | 版本 |
|----|------|
| gcc / g++ | 9.4.0 |
| cmake | 3.16.3 |
| Protobuf C++ | 3.6.1（项目源码包编译）|
| Protobuf-C | 1.3.3（项目源码包编译）|
| ZMQ | 4.3.2 |
| OpenCV | 4.2.0 |
| Eigen3 | 3.3.7 |
| Qt5 Widgets | 5.12.8 |
| yaml-cpp | 0.6.2 |
| Boost | 1.71（date_time / timer）|
| libxml2 | 2.9.10 |

### 常用命令

```bash
./dev.sh build   # 首次构建镜像（约10-20分钟）
./dev.sh start   # 进入容器（项目挂载到 /workspace）
./dev.sh stop    # 停止容器
```

### 容器内编译（已验证通过）

```bash
cd /workspace/huanyuan1/thicv-pilot/planningFigure
mkdir -p build && cd build
cmake .. && make -j$(nproc)
# [100%] Built target planning ✓
```

---

## CLion 开发环境配置（已完成）

### Toolchain（工具链）
- **Docker toolchain**：名称 `Docker`，镜像 `modularization-dev:latest`
- Volume binding：`/data/aiden/文档/Modularization` → `/workspace`
- 作用：**编译专用**，所有缺失的头文件（OpenCV等）都在容器内

### CMake Profile
- 名称：`Docker-Debug`，Toolchain：`Docker`
- Build directory：`/data/aiden/文档/Modularization/huanyuan1/thicv-pilot/planningFigure/build`

### 工作流
```
改代码
  → Ctrl+F9（锤子按钮）      ← Docker 编译
  → Tools → External Tools   ← 主机运行（有GUI）
```

> **注意**：CLion 的 Docker toolchain 编译和运行都在容器内执行，无法通过 ▶ 按钮在主机运行。
> 运行请使用 External Tool 或直接在系统终端执行。

### 主机运行前提（已配置）
主机缺少 OpenCV 编译头文件，但运行库已安装：
```bash
# 已安装（勿重复安装）
sudo apt-get install libopencv-highgui4.2 libopencv-imgproc4.2 libopencv-core4.2
```

---

## 可运行的程序

### 1. planning（规划主程序）

**位置**：`huanyuan1/thicv-pilot/planningFigure/build/planning`

**运行方式**（必须从 build 目录运行）：
```bash
cd /data/aiden/文档/Modularization/huanyuan1/thicv-pilot/planningFigure/build
./planning
```

**依赖**（运行时需要，缺失则阻塞等待）：
| ZMQ 端口 | 数据来源 |
|----------|---------|
| `tcp://127.0.0.1:5003` | IMU 数据 |
| `tcp://127.0.0.1:5009` | 预测/障碍物数据 |
| `tcp://127.0.0.1:3151` | 底盘驱动数据 |

**验证结果**：地图加载成功（读取 `mapMsg/roadMap_TsingHua0823Final.xodr`），停车点解析正常，全局规划线程启动。无外部数据源时程序阻塞在ZMQ接收循环（正常现象）。

---

### 2. strThreadDemo（地图定位仿真程序）

**位置**：`Thread/build/strThreadDemo`

**功能**：GPS/IMU 坐标转换 + 地图解析 + 定位仿真，读取 `config.yaml` 指定地图文件路径。

**运行方式**（必须从 build 目录运行，否则找不到 `../config.yaml`）：
```bash
cd /data/aiden/文档/Modularization/Thread/build
./strThreadDemo
```

**配置文件**：`Thread/config.yaml`（指定地图路径和起止点参数）

**依赖库**：ZMQ、Protobuf、yaml-cpp（主机已满足，无需Docker）

---

## 版本管理规范

项目有两个独立的 Git 仓库，分别用于不同目的，提交时严格区分。

### 仓库说明

| 仓库 | 地址 | 用途 | 管理范围 |
|------|------|------|---------|
| **个人 GitHub**（remote: `personal`） | `github.com/Aiden885/autonomous-vehicle-dev` | 个人学习记录，完整存档 | 整个 `Modularization/` 主目录 |
| **Gitee 审核库**（remote: `origin`） | `gitee.com/sensizlik-lou/thicv-pilot` | 提交上级审核的代码 | 仅 `huanyuan1/thicv-pilot/` 子目录 |

> `huanyuan1/thicv-pilot/` 是一个**独立的 Git 子仓库**，有自己的 `.git`，`origin` 直接指向 Gitee。

---

### 日常提交流程

#### 方式一：CLion 直接 Push（推荐）

CLion 会自动识别两个仓库并分别推送：
- 粉红色 `Modularization` → 推向 GitHub（`personal`）
- 绿色 `huanyuan1/thicv-pilot` → 推向 Gitee（`origin`）

操作步骤：
```
1. Ctrl+K  打开 Commit 窗口
2. 在 Changes 里勾选要提交的文件（注意区分属于哪个仓库）
3. 填写 Commit Message
4. 点击 "Commit and Push"
```

#### 方式二：命令行操作

```bash
# ---- 提交到 Gitee（给上级审核）----
cd /data/aiden/文档/Modularization/huanyuan1/thicv-pilot
git add <修改的文件>
git commit -m "说明修改内容"
git push origin master

# ---- 提交到 GitHub（个人记录）----
cd /data/aiden/文档/Modularization
git add huanyuan1/thicv-pilot   # 同步子仓库指针
git add <其他修改的文件>
git commit -m "说明修改内容"
git push personal master
```

---

### 注意事项

1. **先提交子仓库，再提交主仓库**：修改 `huanyuan1/thicv-pilot/` 内的文件时，先在子仓库 commit + push 到 Gitee，再回到主仓库更新子模块指针并 push 到 GitHub。

2. **`planningFigure/build/` 已加入 `.gitignore`**：编译产物不会被提交，无需手动排除。

3. **`trajectory_tracking/` 和 `planning/thicv-pilot/`** 是工作目录中额外的独立 Git 仓库（参考代码），不向任何远程推送，CLion 提交时不要勾选这两个仓库的文件。

4. **Gitee 认证**：子仓库 `origin` 的 URL 已内置 token，直接 push 无需额外输入密码。token 过期后需更新：
   ```bash
   cd huanyuan1/thicv-pilot
   git remote set-url origin https://Zhao_Yukun0912:<新token>@gitee.com/sensizlik-lou/thicv-pilot.git
   ```

---

## C++ → C 转换规范

### 1. 类 → 结构体 + 函数（结构体名加 Cpp 后缀）

```c
// C++ 原版
class PIDController {
    double integral, lastError;
    double compute(double error);
};

// C 转换版
typedef struct { double integral; double lastError; } PIDControllerCpp;
double pidController_compute(double error, PIDControllerCpp* state);
```

### 2. std::vector → 动态数组

```c
typedef struct {
    Point* data;
    int count;
    int capacity;
} PointArray;

void pointArray_push(PointArray* arr, Point p) {
    if (arr->count >= arr->capacity) {
        arr->capacity = arr->capacity ? arr->capacity * 2 : 16;
        arr->data = realloc(arr->data, sizeof(Point) * arr->capacity);
    }
    arr->data[arr->count++] = p;
}
```

### 3. 所有 C 头文件加桥接宏

```c
#ifdef __cplusplus
extern "C" {
#endif
// 函数声明
#ifdef __cplusplus
}
#endif
```

### 4. 分层架构

```
C++ 层（main / 线程 / ZMQ / Protobuf 收发）
        ↓  extern "C"
C 算法层（规划 / 控制 / 数学算法）
        ↓
C 标准库（math.h / stdlib.h / string.h）
```

### 5. Protobuf 双版本共存

```bash
protoc   --cpp_out=./ *.proto   # C++ 模块用（生成 .pb.cc / .pb.h）
protoc-c --c_out=./   *.proto   # C 模块用（生成 .pb-c.c / .pb-c.h）
```

### 6. 其他替换规则

| C++ | C |
|-----|---|
| `std::string` | `char*`（手动管理内存）|
| `std::map` | 线性查找结构体 / 自定义哈希表 |
| 虚函数 / 继承 | 函数指针（存入结构体）|
| `try-catch` | 返回错误码 |
| 模板函数 | 保留在 C++ 封装层，不转换 |

---

## 进程间通信（ZMQ + Protobuf）

```
感知模块  →[pub topic:perception]→  规划模块
定位模块  →[pub topic:imu/gps]→     规划模块
规划模块  →[pub topic:planning]→    控制模块
控制模块  →[pub topic:control]→     底盘驱动
```

消息格式定义在 `proto/` 目录，修改后重新生成：

```bash
cd proto && python3 compile_proto.py
```

---

## 原始代码中已知的 Warning（无需修复）

- `CURVE_NUM` / `CURVE_DISTANCE` 等宏重定义
- C 文件中 `const` 指针赋值给非 `const` 成员（`-Wdiscarded-qualifiers`）
- `Bezier.c` 中 `radToDeg` 函数隐式声明
