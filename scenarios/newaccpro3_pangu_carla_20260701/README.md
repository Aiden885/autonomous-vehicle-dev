# newaccpro3 Pangu-CARLA 可视化闭环测试

用途：启动本机 CARLA、Python Bridge、newaccpro3 生成代码版 `ZmqBridgeModule`，并自动发送 E 指令进入 ACC 跟车。

关键说明：

- 当前 `run_all.sh app_empty` 会出现 `Failed to connect to aicc_flow_topo`，但脚本仍打印 success，业务进程没有被拉起。
- 本场景使用 `run.sh app_empty ZmqBridgeModule -nohup` 直接启动业务进程，已在 mock 复测中验证可用。
- 当前生成代码默认 `vMin=1`，静止状态下 ACC 不接管。本场景通过 `ACC_DECISION_VMIN=0` 做演示覆盖，允许静止启控；不影响默认算法参数。
- 启动脚本会等待 Pangu `dataflow_runner` 进程存在，并额外等待 `PANGU_COMMAND_READY_DELAY_SEC` 后再发送一次 `E` 指令，避免驾驶指令脉冲早于订阅端 ready 而丢失。
- 若要手动复现启控动作，可在场景启动后执行：

```bash
python3 tools/pangu_acc_closed_loop/keyboard_command_publisher.py --once e
```

启动：

```bash
bash scenarios/newaccpro3_pangu_carla_20260701/run.sh
```

停止：

```bash
bash scenarios/newaccpro3_pangu_carla_20260701/stop.sh
```

调试平台会自动扫描该目录，打开 `http://127.0.0.1:8765/` 后可直接选择该场景。

## 调试平台手动测试流程

1. 启动调试平台：

```bash
cd /home/aiden/文档/Modularization
python3 tools/gaasd_scenario_panel/app.py
```

2. 浏览器打开：

```text
http://127.0.0.1:8765/
```

3. 在左侧选择 `newaccpro3_pangu_carla_20260701`。

4. 点击 `启动联调环境`。该按钮会同时启动：
   - CARLA 窗口。
   - Python Bridge。
   - ACC 跟车场景。
   - 前车 waypoint PID 控制器：按 CARLA waypoint 跟踪车道，并用速度 PID 保持约 `2m/s`。
   - Pangu Docker 容器。
   - `ZmqBridgeModule` 业务进程。

5. 等待 `服务状态` 中以下项目在线：
   - `CARLA Server`
   - `Bridge PUB`
   - `Bridge CONTROL`
   - `Pangu ACC`

6. 如果车辆未自动起步，点击 `辅助启控`。该按钮会执行一次 `boost + E`：
   - 临时给 ego 一个起步速度。
   - 发送 `E` 驾驶指令，让 ACC 进入启控状态。

7. 观察 CARLA 窗口：
   - ego 应跟随约 `2m/s` 的前车。
   - 前车由 waypoint PID 控制，不再使用 `constant_velocity` 直线速度向量。
   - 点击 `T/R` 可观察时距调整效果。
   - 点击 `C` 或 `S` 可观察 ACC 退出。

8. 测试结束点击 `停止`。
