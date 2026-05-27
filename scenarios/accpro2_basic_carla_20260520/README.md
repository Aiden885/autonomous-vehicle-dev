# ACC 基础模块版 CARLA-GAASD 联合仿真场景

本场景用于测试 `project/accpro2` 中手动搭建的 ACC 基础模块画布。CARLA 场景、Bridge 配置、前车位置和视角参数沿用已验证的 `acc_carla_phase2_20260513` 场景。

## 当前检查结果

- 画布已生成 `project/accpro2/icvos/src/functions`。
- 生成的主链路为：`CARLAACCLeadDistance / CARLAACCLeadSpeed / CARLAACCEgoSpeed -> subtract / multiply / add -> CARLAACCLongitudinalCmd`。
- 当前生成公式为：`targetSpeed = 0.35 * (distance - 15.0) + 0.8 * (leadV - egoV) + leadV`。
- 示波器运行流程已生成 `project/accpro2/icvos/src/oscilloscopeFunctions/.../FuncStep.c`。
- `FuncStep.c` 已包含 `scope_push_send(...)`，观测信号顺序为 `targetSpeed`、`egoV`、`leadV`、`distance`。
- 已补充 `objectCode/total/DLL` 中的 `libcarla_gaasd_adapter.so` 及 ZMQ 运行库。
- 已用 `tools/carla_bridge/fix-gaasd-generated-code.py` 修复当前生成头文件中的 `protobuf-c` / 枚举兼容问题。

## 启动方式

通过 UI 面板启动：

```bash
python3 tools/gaasd_scenario_panel/app.py
```

打开 `http://127.0.0.1:8765/`，选择 `ACC 基础模块版 CARLA-GAASD 联合仿真场景`，点击“启动联调环境”。

也可以直接命令行启动：

```bash
scenarios/accpro2_basic_carla_20260520/run.sh
```

启动 CARLA + Bridge 后，在 GAASD 中打开 `project/accpro2`，通过示波器运行仿真。

## 注意

- 当前 `main.c` 中未看到 `limit(rawTarget, 0, 5.0)`，如需严格复现已验证参数，建议在画布中补上 `limit` 后重新生成代码。
