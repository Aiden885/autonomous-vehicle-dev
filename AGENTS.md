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
planning/                 ← 规划模块最终目标（GitLab仓库，只放C文件）
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

## 代码规范（函数规范.xlsx 摘要）

### 命名
- 函数/变量：小驼峰（`generateBezierPath`）
- 文件夹：大驼峰（`BezierSpline`）
- 源文件/头文件：小驼峰（`bezierSpline.c`）
- C++结构体转C后必须加 `Cpp` 后缀（`TrajectoryPointCpp`）

### 禁止
- 禁止 `goto`
- 禁止函数中途 `return`（只允许最后一个return）
- 禁止 `break`

### 控制流格式
```c
// if/else：条件必须是单一变量
bool cond;
cond = checkCondition(x);
if (cond) {
    doSomethingA(x);
} else {
    doSomethingB(x);
}

// for：初始/步进/终止必须是变量
int start = 0, step = 1, end = n;
for (start; start < end; start = start + step) {
    processItem(start, data);
}
```

### 其他
- 函数行数 ≤ 200 行
- 禁止全局变量
- 注释：Doxygen格式，中文标点，UTF-8编码
- 编译警告：必须全部解决（-Wall -Wextra）

## 函数注释模板

```c
/**
 * @brief 函数功能描述
 * @en_name englishFunctionName
 * @cn_name 中文名称
 * @type widget（原子函数）| module（复合函数）
 * @param[IN] 类型 参数名 描述
 * @param[OUT] 类型 参数名 描述
 * @retval 类型 名称 描述
 * @granularity atomic | composite
 * @version 1.0
 * @date YYYY-MM-DD
 * @author 作者
 */
```

## C++ → C 转换规则

1. 类 → 结构体（名称加`Cpp`后缀）+ 独立函数
2. `std::vector` → `DynArray`（来自`DataStructure/dynArray.h`）
3. `std::priority_queue` → `MinHeap`（来自`DataStructure/minHeap.h`）
4. `std::unordered_map` → `HashMap`（来自`DataStructure/hashMap.h`）
5. `std::string` → `char*`（手动管理内存）
6. 虚函数/继承 → 函数指针（存入结构体）
7. try-catch → 错误码返回
8. 所有C头文件加 `extern "C"` 桥接宏
9. Protobuf C++ `.pb.cc/.pb.h` → C版 `.pb-c.c/.pb-c.h`

## 当前转换进度

### 规划模块（planning/）
- ✅ Dijkstra全局规划
- ✅ 贝塞尔曲线
- ✅ 速度规划generateSpeedList
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
