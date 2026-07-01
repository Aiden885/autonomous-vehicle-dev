# Pangu ACC 最小闭环测试模块

该目录用于验证新版 GAASD/Pangu 架构下 ACC 与 CARLA/Bridge 的最小闭环链路。它从官方 `ACCModule.tar.gz` 参考包复制而来，只在独立目录内改造，不覆盖原参考包。

## 目标

验证以下链路是否可运行：

```text
CARLA/Mock Bridge -> ZMQ 5701 -> ZmqBridgeModule -> acc_input
acc_input -> ACCModule -> acc_output
acc_output -> ZmqBridgeModule -> ZMQ 5702 -> CARLA/Mock Bridge
Keyboard -> ZMQ 5702 -> Bridge -> ZMQ 5701 -> commandType -> ACCModule
```

## 核心接口

`AccInput`：

- `ego_speed_mps`：自车速度，m/s。
- `lead_speed_mps`：前车速度，m/s。
- `lead_distance_m`：前车距离，m。
- `command_type`：驾驶指令，0 表示无指令。

`AccOutput`：

- `target_speed_mps`：ACC 目标速度，m/s。
- `enable`：ACC 控制使能。
- `valid`：输出是否有效。
- `time_gap_s`、`max_speed_mps`、`decision`、`system_state`：调试字段，用于对照 GAASD 画布生成代码。

`command_type`：

- `0`：无指令。
- `1`：E，降低设定速度；待命无历史时按当前速度启控。
- `2`：Q，提高设定速度；待命有历史时继承参数启控。
- `3`：T，减小时距。
- `4`：R，增大时距。
- `5`：W，驾驶员油门。
- `6`：S，驾驶员制动并退出 ACC。
- `7`：C，取消 ACC。

## Mock 测试

先做纯算法序列测试：

```bash
tools/pangu_acc_closed_loop/verify_oracle_sequence.sh
```

该脚本验证固定输入下的 `E/T/R/Q/C/S` 计算结果，包括 `decision`、`systemState`、
`timeGap`、`maxSpeed`、`targetSpeed` 和 `enable`。

当前 oracle 已按 `newaccpro3` 画布生成代码反推语义对齐：`enable` 输出使用当前周期旧
`controlEnabled`，`E/C/S` 等启控或退控指令影响下一周期；当前简化画布未实现 R5
捕获当前车速，`maxSpeed` 只由 R1/R2 调整。低速状态只让当前周期 `enable=0`，
不清空内部 `controlEnabled`；车速恢复后可自动回到在控输出。

先启动 mock Bridge：

```bash
python3 tools/pangu_acc_closed_loop/mock_carla_bridge.py
```

再启动键盘指令发送器：

```bash
python3 tools/pangu_acc_closed_loop/keyboard_command_publisher.py
```

如果没有图形环境或没有 `pygame`：

```bash
python3 tools/pangu_acc_closed_loop/keyboard_command_publisher.py --text
```

自动化测试时可以只发送一次指令后退出：

```bash
python3 tools/pangu_acc_closed_loop/keyboard_command_publisher.py --once e
python3 tools/pangu_acc_closed_loop/keyboard_command_publisher.py --once c
```

如果已经完成 Pangu 临时构建和安装，可直接运行整套 mock 闭环复测脚本：

```bash
tools/pangu_acc_closed_loop/verify_mock_closed_loop.sh
```

默认读取：

```text
/tmp/pangu_acc_closed_loop_build_20260629_202705/install
```

如果构建目录不同，使用 `PANGU_BUILD_ROOT=/path/to/build_root` 覆盖。

更完整的 mock 多指令测试：

```bash
tools/pangu_acc_closed_loop/verify_mock_command_sequence.sh
```

默认发送 `E/T/R/Q/C/E/S/0`，检查每个 `driver_command` 都能到达 mock Bridge，并检查
Pangu 输出过 `enable=1` 的控制命令和 `enable=0` 的退出命令。由于当前画布采用旧状态输出
`enable`，测试会按一周期延迟判断 E/C/S，而不是要求指令同周期立即生效。

复测脚本会自动把 `conf/app_module/app_empty.pt` 的 `soc_name` 修正为
`conf/global_conf/icvos_machine.pt` 中的本机名。当前本机是 `soc1`；如果 `soc_name`
仍是 `app_empty`，`run_all` 可能返回 success，但模块不会真正被拉起。

真实 CARLA/Bridge 复测：

```bash
tools/pangu_acc_closed_loop/verify_real_bridge_closed_loop.sh
```

该脚本会启动 CARLA、Bridge、Pangu ACC，默认发送 `E/T/R/Q/C/E/S/0` 指令，并读取
`bridge_status` 中的 `driver_command_received/control_cmd_received` 计数。默认结束后会停止
CARLA/Bridge/Pangu；如果需要保留场景，使用：

```bash
KEEP_STACK=1 tools/pangu_acc_closed_loop/verify_real_bridge_closed_loop.sh
```

Pangu 模块启动后，按 `E` 可以从静止启控，`T/R` 调整时距，`C` 或 `S` 退出 ACC。

后续放入官方 Docker 场景时，建议将该脚本随 CARLA/Bridge 工具放到：

```text
/opt/carla/carla_tools/keyboard_command_publisher.py
```

如果场景脚本位于 `/opt/carla/carla_tools/scenario/`，由场景脚本调用该工具即可。端口仍保持 `5702`，由 Bridge 统一转发 `driver_command` 到 `5701`，Pangu 模块只订阅 `5701`。

## 与 GAASD 生成代码的关系

当前 `module_lib/AccTargetSpeed.*` 是手写 oracle，用于验证 Pangu 节点、ZMQ、键盘指令和控制命令链路。后续 GAASD 代码生成稳定后，应将画布生成的 ACC 算法接入同一套 `AccInput/AccOutput`，并用同输入数据对 oracle 与生成代码做差异对比。

这意味着：手写模块跑通不等于 GAASD 生成算法已经验证通过，但它能先固定通信协议、运行结构和测试场景。
