# 工作日志 WORKLOG

> **用途**：记录每次工作的修改内容、遗留问题和下一步任务。
> 新开会话或交给 Codex 前，先读这个文件。
> 每次工作结束后，更新"当前状态"和"下一步任务"两节。

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
