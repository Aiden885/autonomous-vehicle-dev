# GAASD-CARLA ACC Pangu 最小闭环执行方案

## 1. 目标

本方案用于验证 ACC 在新版 GAASD/Pangu 架构下的最小闭环链路。

目标不是一次性把完整 CARLA 场景、画布生成、Docker 运行全部打通，而是先验证一条后续可以继续复用的主链路：

```text
键盘/模拟输入
    -> commandType
    -> Pangu AccInput
    -> ACCModule
    -> targetSpeed / enable
    -> Pangu AccOutput
    -> control_cmd
```

核心原则：

- `commandType` 必须进入 `AccInput`，作为 ACC 决策输入字段。
- Python 键盘程序只是输入源，不能把键盘逻辑写死在 ACC 算法模块内部。
- 最小测试可以绕过 Docker 和 CARLA，但测试字段、消息流向、模块结构必须和最终联调一致。
- 当前参考包 `ACCModule.tar.gz` 作为 Pangu 模块结构参考，不直接等同于最终 ACC。

## 1.1 当前落地状态

已从参考包复制并改造独立验证目录：

```text
tools/pangu_acc_closed_loop/ACCClosedLoopModule
```

已新增测试工具：

```text
tools/pangu_acc_closed_loop/mock_carla_bridge.py
tools/pangu_acc_closed_loop/keyboard_command_publisher.py
tools/pangu_acc_closed_loop/verify_oracle_sequence.sh
tools/pangu_acc_closed_loop/verify_mock_closed_loop.sh
tools/pangu_acc_closed_loop/verify_mock_command_sequence.sh
tools/pangu_acc_closed_loop/verify_real_bridge_closed_loop.sh
tools/pangu_acc_closed_loop/README.md
```

当前已完成：

- `AccInput` 增加 `command_type`。
- `AccOutput` 输出 `target_speed_mps / enable / valid`，并增加 `time_gap_s / max_speed_mps / decision / system_state` 调试字段。
- `AccTargetSpeed` 已实现手写 ACC oracle，用于先固定 Pangu 节点通信链路和后续生成代码对照。
- `ZmqBridgeModule` 已订阅 `driver_command`，并将其写入 `AccInput.command_type`。
- mock Bridge 已按最终端口语义实现：5701 发布输入，5702 接收控制和驾驶指令。

当前已验证：

- Python 测试脚本语法通过。
- `AccTargetSpeed.cpp` 可单独编译并通过 smoke test。
- `acc_zmq_bridge.cpp` 使用 Pangu 第三方 ZMQ include 路径可单独编译。
- `ACCClosedLoopModule` 已在临时 Pangu 构建树中完整编译通过，生成：
  - `libACCModule.so`
  - `libZmqBridgeModule.so`
  - `libACCModule_core.so`
  - `libACCModule_conf_pb.so`
- `ldd` 检查未发现 `not found`，ZMQ/Protobuf 依赖来自 Pangu thirdparty。
- 已安装最小运行目录并补齐 `app_empty` 运行配置：
  - `conf/app_module/app_empty.pt`
  - `conf/global_conf/global_module.pt`
  - `conf/global_conf/global_com.pt`
  - `conf/global_conf/icvos_machine.pt`
  - `conf/global_conf/tf.pt`
- 已通过 `run_all.sh app_empty` 启动 Pangu runtime，日志显示 `ACCModule` 和 `ZmqBridgeModule` 被加载到 `app_empty`。
- 已通过 `verify_mock_closed_loop.sh` 跑通 mock 最小闭环：
  - `commandType=1(E)` 后，mock 收到驾驶指令；按当前 `newaccpro3` 画布语义，启控在下一周期生效，随后收到 `target=5.00 enable=1`。
  - `commandType=7(C)` 后，退控在下一周期生效；日志中会先出现一个过渡控制命令，随后收到 `target=0.00 enable=0`。
  - 说明 `keyboard -> Bridge -> Pangu -> Bridge` 的最小消息链路成立。
- 已通过 `verify_oracle_sequence.sh` 验证固定输入计算结果：
  - `E`：无历史待命启控，`decision=5`，当前周期 `enable=0`、`targetSpeed=0`，下一周期无指令保持时 `enable=1`、`targetSpeed=5.0000`。
  - `T`：时距从 `1.8s` 降到 `1.6s`。
  - `R`：时距从 `1.6s` 回到 `1.8s`。
  - `Q`：当前测试配置中 `maxSpeedCap=5.0m/s`，设定速度已在上限时保持 `5.0000m/s`。
  - `E`：在控降速时，设定速度从 `5.0000m/s` 降到 `3.6111m/s`。
  - `C`：取消 ACC，当前周期仍按旧 `controlEnabled` 输出，下一周期退出到 `enable=0`、`targetSpeed=0`。
  - 历史恢复 `Q`：`systemState=1`，`decision=6`，恢复命令影响下一周期。
  - `S`：制动退出，和 `C` 一样按下一周期退控处理。
  - 低速状态：`egoV < vMin` 时当前 `enable=0`，但内部 `controlEnabled` 不被低速清除；速度恢复后可自动续上。
- 已通过 `verify_mock_command_sequence.sh` 验证 mock 多指令链路：
  - `E/T/R/Q/C/E/S/0` 全部到达 mock Bridge。
  - Pangu 输出过 `enable=1` 的控制命令和 `enable=0` 的退出命令，且 E/C/S 的一周期延迟与当前画布生成语义一致。
- 复测脚本会自动修正 `app_empty.pt` 的 `soc_name`，使其匹配 `icvos_machine.pt` 中的本机 `soc1`。
- 已通过 `verify_real_bridge_closed_loop.sh` 跑通真实 Bridge 多指令闭环：
  - CARLA、Bridge、Town01 ACC 直道场景能自动启动。
  - 默认发送 `E/T/R/Q/C/E/S/0`。
  - Bridge 状态中 `driver_command_received=8`。
  - Bridge 状态中 `control_cmd_received` 增长到 `257`，证明 Pangu 控制输出持续回到真实 Bridge。
- 已修复 `reset-acc-straight-scene.py` 只查一次 ego 的竞态，改为在 `--timeout-sec` 内等待 `role_name=hero` 出现。

尚未完成：

- 将 GAASD 画布生成代码接入同一输入输出接口，并和 oracle 做 diff。

当前临时构建位置：

```text
/tmp/pangu_acc_closed_loop_build_20260629_202705
```

构建前需要加载 Pangu thirdparty：

```bash
source dependencies/thirdparty/X86/setup.bash
```

## 2. 已确认接口变量

ACC 最终联调必须保持以下接口。

### 2.1 输入

| 字段 | 类型 | 单位 | 来源 |
| --- | --- | --- | --- |
| `egoV` | `double` | m/s | CARLA 自车速度 |
| `leadV` | `double` | m/s | CARLA 前车速度 |
| `distance` | `double` | m | CARLA 前车距离 |
| `commandType` | `int` | 1 | 键盘/驾驶员指令 |

### 2.2 输出

| 字段 | 类型 | 单位 | 去向 |
| --- | --- | --- | --- |
| `targetSpeed` / `speed` | `double` | m/s | CARLA 控制命令 |
| `enable` | `int` | 1 | CARLA 控制命令 |

类型约定：

- Pangu proto 中 `enable` 建议使用 `bool`。
- 算法内部和画布接口中 `enable` 可以使用 `int`。
- callback 边界负责做 `0/1` 与 `bool` 的转换。

## 3. `commandType` 放置位置

`commandType` 最终应放在 Pangu 输入消息 `AccInput` 中。

建议 proto 结构：

```proto
message AccInput {
    uint32 frame_id = 1;
    double ego_speed_mps = 2;
    double lead_speed_mps = 3;
    double lead_distance_m = 4;
    int32 command_type = 5;
}

message AccOutput {
    uint32 frame_id = 1;
    double target_speed_mps = 2;
    bool enable = 3;
    bool valid = 4;
}
```

说明：

- 参考包原始 `AccInput` 中的 `enable` 不适合作为最终主输入。
- 咱们的 `enable` 是 ACC 决策输出，不是外部直接给的控制开关。
- 外部输入应该是驾驶员事件 `commandType`，由 ACC 决策逻辑生成 `enable`。

## 4. 键盘映射

键盘映射参考 `~/PycharmProjects/CarlaAcc`，并与当前接口变量清单一致。

| 键 | `commandType` | 含义 | 触发方式 |
| --- | --- | --- | --- |
| `E` | `1` | 降低设定速度；待命无历史时表示当前速度启控 | 脉冲 |
| `Q` | `2` | 提高设定速度；待命有历史时表示继承参数启控 | 脉冲 |
| `T` | `3` | 减小时距 | 脉冲 |
| `R` | `4` | 增大时距 | 脉冲 |
| `W` | `5` | 驾驶员油门 | 按下为有效，释放回 0 |
| `S` | `6` | 驾驶员制动并退出 ACC | 按下为有效，释放回 0 |
| `C` | `7` | 取消 ACC | 脉冲 |

脉冲类按键约定：

```text
无指令周期：commandType = 0
按键触发周期：commandType = 1/2/3/4/7
下一周期：commandType 回到 0
```

`W/S` 属于持续驾驶员输入，当前测试可按 `mode=level` 处理：

```text
按下 W：commandType = 5
释放 W：commandType = 0
按下 S：commandType = 6
释放 S：commandType = 0
```

## 5. 总体链路

### 5.1 最小 mock 测试链路

不启动 CARLA，不启动 Docker。

```text
keyboard_command_publisher.py
        |
        v
mock_carla_bridge.py
        |
        |  PUB ego_state / lead_vehicle / driver_command
        v
Pangu ZmqBridgeModule
        |
        |  pub acc_input
        v
Pangu ACCModule
        |
        |  pub acc_output
        v
Pangu ZmqBridgeModule
        |
        |  PUB control_cmd
        v
mock_carla_bridge.py 打印 targetSpeed / enable
```

此阶段验证：

- 键盘输入能变成 `commandType`。
- `commandType` 能进入 Pangu `AccInput`。
- `ACCModule` 能收到 `egoV/leadV/distance/commandType`。
- `ACCModule` 能输出 `targetSpeed/enable`。当前 oracle 已按 `newaccpro3` 画布语义对齐：`enable` 输出使用当前周期旧 `controlEnabled`，启控/退控命令影响下一周期，因此 E/C/S 等指令存在 1 个周期延迟。
- 控制命令能被发布到 ZMQ。

此阶段不验证：

- CARLA 是否启动。
- Docker 场景是否正确。
- 车辆是否真实运动。

### 5.2 真实 CARLA 联调链路

mock 通过后再切换到真实 CARLA。

```text
watch-carla.py
        |
        |  driver_command -> 5702
        v
carla_bridge.py
        |
        |  ego_state / lead_vehicle / driver_command -> 5701
        v
Pangu ZmqBridgeModule
        |
        |  acc_input
        v
Pangu ACCModule
        |
        |  acc_output
        v
Pangu ZmqBridgeModule
        |
        |  control_cmd -> 5702
        v
carla_bridge.py
        |
        v
CARLA ego vehicle
```

这里需要明确端口语义：

- `5701` 是 CARLA Bridge 对外发布输入状态的 PUB 总线，Pangu `ZmqBridgeModule` 从这里订阅 `ego_state/lead_vehicle/driver_command`。
- `5702` 是 CARLA Bridge 接收外部命令的 SUB 入口，既接收 Pangu 输出的 `control_cmd`，也接收键盘侧发来的 `driver_command`。
- `watch-carla.py -> 5702 -> carla_bridge.py -> 5701` 这条链路不是把驾驶指令当成车辆控制输出，而是让 Bridge 统一接收、归一化、再转发驾驶指令。
- 如果 mock 阶段不启动真实 `carla_bridge.py`，则 `mock_carla_bridge.py` 必须承担同样的聚合职责：接收键盘 `driver_command`，再在 `5701` 上发布统一后的 `driver_command`。不要让键盘程序直接和 Pangu 模块私连，否则会绕开最终联调的 Bridge 语义。

## 6. 基于参考包需要修改的内容

参考包位置：

```text
/home/aiden/文档/temp/ACCModule.tar.gz
```

建议复制为新模块包，例如：

```text
ACCClosedLoopModule/
```

不要直接覆盖原始参考包。

### 6.1 `proto/ACCModule_config.proto`

修改 `AccInput`：

```proto
message AccInput {
    uint32 frame_id = 1;
    double ego_speed_mps = 2;
    double lead_speed_mps = 3;
    double lead_distance_m = 4;
    int32 command_type = 5;
}
```

修改 `AccOutput`：

```proto
message AccOutput {
    uint32 frame_id = 1;
    double target_speed_mps = 2;
    bool enable = 3;
    bool valid = 4;
}
```

### 6.2 `module_lib/acc_zmq_bridge.h`

在状态结构中增加：

```cpp
int command_type = 0;
bool command_valid = false;
```

### 6.3 `module_lib/acc_zmq_bridge.cpp`

新增订阅 topic：

```cpp
constexpr const char* kDriverCommandTopic = "gaasd.carla.driver_command.v1";
```

订阅：

```cpp
Subscribe(sub_, kDriverCommandTopic)
```

解析：

```text
payload.command_type -> state_.command_type
```

如果一定时间内没有新驾驶指令，保持：

```text
command_type = 0
```

### 6.4 `src/ZmqBridgeModule.cc`

发布 `AccInput` 时增加：

```cpp
input->set_command_type(zmq_state.command_type);
```

原参考包中：

```cpp
input->set_enable(ZmqBridgeModule_params_.zmq_control_enable());
```

最终方案中应删除或忽略这类外部 `enable` 输入。

### 6.5 `src/ACCModule.cc`

callback 中读取：

```cpp
acc_input.egoV = input.ego_speed_mps();
acc_input.leadV = input.lead_speed_mps();
acc_input.distance = input.lead_distance_m();
acc_input.commandType = input.command_type();
```

输出：

```cpp
output->set_target_speed_mps(acc_output.targetSpeed);
output->set_enable(acc_output.enable);
output->set_valid(acc_output.valid);
```

### 6.6 `module_lib/AccTargetSpeed`

参考包中的 `AccTargetSpeed` 只是简单目标速度计算。

最终应替换为咱们 ACC 画布对应逻辑，至少包含：

- `commandType` 决策输入。
- `timeGap` 状态保持。
- `maxSpeed` 状态保持。
- `enable` 决策输出。
- `targetSpeed` 控制输出。

为了先验证链路，可以分两步：

1. 第一版：先保留简单目标速度公式，只加 `commandType` 字段透传和日志，确认链路通。
2. 第二版：加入手写完整 ACC 决策+控制逻辑，作为 oracle 标尺。
3. 第三版：把 GAASD 画布生成函数接入同一输入输出接口，并与 oracle 做同输入对比。

第一版由于已经删除 `AccInput.enable`，需要明确 `enable` 来源：

- 链路验证阶段可临时设置 `enable = (commandType != 6 && commandType != 7)`，或固定 `enable = true`。
- 推荐使用 `enable = (commandType != 6 && commandType != 7)`，这样第一版即可验证 `S/C` 退出命令的控制路径。
- 完整 `enable` 状态机只能在第二版 oracle 中实现，不能把第一版临时逻辑当成最终算法。

第二版 oracle 必须包含查找表语义。

当前 `newaccpro3` 生成代码中可看到真值表调用：

```cpp
truth_table_6957bd9f_a281_4c91_a9b5_5b2c83757f58(systemState, commandType)
```

但当前生成产物只生成调用，没有稳定生成真值表函数体。因此 oracle 需要手写该查找表逻辑，作为后续 GAASD 生成代码的对照标准。

## 7. Mock 测试设计

### 7.1 `mock_carla_bridge.py`

功能：

- PUB 端口模拟 CARLA Bridge 输出。
- SUB 端口接收 Pangu 模块输出控制命令。
- 不依赖 CARLA。

建议端口：

```text
PUB: tcp://127.0.0.1:5701
SUB: tcp://127.0.0.1:5702
```

周期发布：

```text
gaasd.carla.ego_state.v1:
  egoV

gaasd.carla.lead_vehicle.v1:
  leadV
  distance

gaasd.carla.driver_command.v1:
  commandType
```

接收并打印：

```text
gaasd.carla.control_cmd.v1:
  target_speed_mps
  enable
```

### 7.2 `keyboard_command_publisher.py`

功能：

- 读取键盘按键。
- 按映射发送 `driver_command`。

可以直接复用或裁剪现有：

```text
tools/carla_bridge/watch-carla.py
```

只保留键盘到 ZMQ 的部分，不需要 CARLA 摄像头和 HUD。

mock 阶段建议使用与真实 Bridge 相同的方向：

```text
keyboard_command_publisher.py -> 5702
mock_carla_bridge.py SUB 5702 接收 driver_command
mock_carla_bridge.py PUB 5701 发布 ego_state / lead_vehicle / driver_command
Pangu ZmqBridgeModule SUB 5701
```

这样后续从 mock 切换到真实 `carla_bridge.py` 时，不需要修改 Pangu 模块。

自动化测试时可以直接发送一次指令：

```bash
python3 tools/pangu_acc_closed_loop/keyboard_command_publisher.py --once e
python3 tools/pangu_acc_closed_loop/keyboard_command_publisher.py --once c
```

完整 mock 闭环复测脚本：

```bash
tools/pangu_acc_closed_loop/verify_mock_closed_loop.sh
```

默认使用临时构建目录：

```text
/tmp/pangu_acc_closed_loop_build_20260629_202705
```

如果构建目录变化，设置：

```bash
PANGU_BUILD_ROOT=/path/to/build_root tools/pangu_acc_closed_loop/verify_mock_closed_loop.sh
```

算法计算序列测试：

```bash
tools/pangu_acc_closed_loop/verify_oracle_sequence.sh
```

mock 多指令链路测试：

```bash
tools/pangu_acc_closed_loop/verify_mock_command_sequence.sh
```

真实 CARLA/Bridge 闭环复测脚本：

```bash
tools/pangu_acc_closed_loop/verify_real_bridge_closed_loop.sh
```

该脚本会启动 CARLA/Bridge/Pangu，默认发送 `E/T/R/Q/C/E/S/0` 指令，并通过
`gaasd.carla.bridge_status.v1` 检查 `driver_command_received` 和
`control_cmd_received` 计数。默认结束时会停止 CARLA/Bridge/Pangu；如需保留场景：

```bash
KEEP_STACK=1 tools/pangu_acc_closed_loop/verify_real_bridge_closed_loop.sh
```

注意：`conf/app_module/app_empty.pt` 中的 `soc_name` 必须与
`conf/global_conf/icvos_machine.pt` 的 `local_machine_name` 一致。当前本机是 `soc1`。
如果仍为 `app_empty`，`run_all` 可能显示 success，但 `ACCModule/ZmqBridgeModule`
不会真正被本机拉起。

后续放入官方 Docker 场景时，建议将脚本随 CARLA/Bridge 工具放到：

```text
/opt/carla/carla_tools/keyboard_command_publisher.py
```

如果场景入口脚本在 `/opt/carla/carla_tools/scenario/` 下，则由场景脚本调用该工具。脚本仍向 `5702` 发送 `driver_command`，不直接连接 Pangu。

## 8. 验收标准

### 8.1 Mock 阶段验收

必须看到：

1. mock 持续发布 `egoV/leadV/distance`。
2. 按 `E/Q/T/R/W/S/C` 后，Pangu 模块日志中能看到对应 `commandType`。
3. `ACCModule` 能持续输出 `targetSpeed/enable`。
4. mock 能收到 `control_cmd`。
5. `commandType=0` 时不会重复触发调速/调距动作。
6. `commandType=6` 或 `7` 后，`enable` 应退出。

### 8.2 CARLA 阶段验收

必须看到：

1. CARLA ego 车辆能收到控制命令。
2. `targetSpeed` 变化能影响车辆速度。
3. `distance` 变化能影响目标速度。
4. 键盘 `E/Q/T/R/W/S/C` 对应决策行为能生效。
5. `S/C` 能退出 ACC 控制。

### 8.3 代码生成验证验收

除了手写 oracle 路线，还必须并行验证 GAASD 画布生成代码。

最低验收要求：

1. 能从 `newaccpro3` 拉取当前 `temp_codegen_output`。
2. 能记录当前生成代码存在的问题，例如真值表函数体缺失、临时变量缺失、接口声明不一致等。
3. 能用同一组 `egoV/leadV/distance/commandType` 输入分别运行 oracle 和生成代码。
4. 能输出 diff 结果，说明生成代码当前与 oracle 的差距。
5. 当 GAASD 代码生成工具修复后，可以直接复用这套 diff 测试确认生成代码是否可替代 oracle。

## 9. 风险和处理

### 9.1 Pangu 模块运行问题

参考包能提供结构，但具体运行依赖 GAASD/Pangu 安装环境。

如果出现模块库加载失败，需要检查：

```text
libACCModule.so
libZmqBridgeModule.so
conf/node_module/ACCModule/*.pt
```

是否被正确安装到 Pangu install 目录。

### 9.2 画布生成函数暂不稳定

当前闭环主链路不依赖画布生成函数直接可用，但必须保留生成代码验证轨道。

短期做法：

```text
Pangu 模块手写 ACC 逻辑，作为 oracle 标尺
```

后续替换：

```text
Pangu 模块 callback -> 调用 GAASD 画布生成函数
```

这样测试链路不会因为当前代码生成器问题被完全阻塞。

注意：

- 手写 oracle 不是最终交付目标，只是为了先验证 Pangu channel、ZMQ、CARLA 控制链路。
- GAASD 生成代码必须单独验证，不能因为 oracle 跑通就认为 GAASD 画布生成已经可用。

### 9.3 `commandType` 事件持续时间

如果输入源输出持续非零，ACC 会重复执行调速/调距。

处理原则：

- 键盘脉冲类按键只发一个周期。
- `W/S` 按键按 level 处理。
- 如果后续来源无法保证单周期脉冲，则在输入侧增加边沿检测，不优先在 ACC 算法中增加。

## 10. 推荐执行顺序

0. 在 `pangu_x86` 容器内先构建并运行未修改的 `ACCModule.tar.gz` 参考包，确认 Pangu 构建、安装、模块加载路径可用。
1. 从 `ACCModule.tar.gz` 复制出新模块包，不修改原参考包。
2. 修改 proto，新增 `command_type`，并统一 `enable` 的 proto/算法类型转换。
3. 修改 `acc_zmq_bridge`，订阅并解析 `driver_command`。
4. 修改 `ZmqBridgeModule`，把 `command_type` 写入 `AccInput`。
5. 修改 `ACCModule`，把 `command_type` 传入算法。
6. 第一版使用简单目标速度算法，`enable` 临时由 `commandType` 或常量给出，跑通 mock 链路。
7. 第二版加入手写完整 ACC 决策+控制逻辑，作为 oracle。
8. 并行建立 GAASD 生成代码验证轨道：拉取 `newaccpro3/temp_codegen_output`，补丁编译，和 oracle 做同输入 diff。
9. 再跑 mock，验证 `E/Q/T/R/W/S/C`。
10. 接真实 `carla_bridge.py`。
11. 接真实 CARLA 场景。
12. 当 GAASD 代码生成稳定后，把 oracle 替换为画布生成函数，重新跑 diff 和真实 CARLA 测试。

## 11. GAASD 生成代码验证轨道

这条轨道和 Pangu 手写 oracle 主链路并行推进。

### 11.1 输入来源

使用当前画布工程：

```text
project/newaccpro3
```

当前生成代码位置：

```text
project/newaccpro3/icvos/src/temp_codegen_output
```

### 11.2 被测对象

优先验证：

```text
run_*/run_*.cpp
composite_block_empty_*/composite_block_empty_*.cpp
truth_table_*.json
```

### 11.3 当前已知问题

当前生成代码可能存在：

- 真值表函数只生成调用，不生成函数体。
- 复合组件 `run()` 声明、实现、调用不一致。
- 临时变量未声明。
- 输出端口自赋值，例如 `output.enable = output.enable`。
- callback wrapper 中输入结构体未正确生成。

这些问题不阻塞 Pangu oracle 链路，但必须记录为 GAASD 代码生成修复项。

### 11.4 验证方法

准备一组固定输入序列：

```text
egoV
leadV
distance
commandType
```

同时喂给：

```text
手写 oracle ACC
GAASD 生成 ACC 函数
```

比较输出：

```text
targetSpeed
enable
timeGap
maxSpeed
```

其中 `targetSpeed/enable` 是必须一致的外部输出；`timeGap/maxSpeed` 是内部状态，用于定位差异。

## 12. 结论

当前最稳妥方案是：

```text
仿照 ACCModule.tar.gz 构建 Pangu ACC 模块
    +
AccInput 增加 commandType
    +
先用 mock ZMQ 验证完整消息链路
    +
手写 oracle 验证 ACC 逻辑
    +
并行验证 GAASD 生成代码
    +
再替换真实 CARLA Bridge / CARLA
```

该方案验证的内容后续可以直接复用，不是一次性临时测试。

画布中没有现成输入端口连线不是当前主阻塞点。新版 Pangu 的稳定集成主线应是：

```text
channel message -> callback -> 算法输入结构 -> 算法函数 -> 输出消息
```

后续 GAASD 代码生成稳定后，再把算法函数替换为画布生成函数即可。
