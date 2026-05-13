# ACC CARLA-GAASD 联合仿真快照 20260513

本目录保存 2026-05-13 前后的 ACC 跟车联合仿真可复现快照，用于后续快速恢复、演示和对比新版本修改。

## 内容

| 路径 | 说明 |
|---|---|
| `scenario.yaml` | 工况、软件版本、关键参数和预期结果 |
| `run.sh` | 使用本快照 Bridge 配置启动 CARLA + Bridge + 测试前车 |
| `restore_gaasd_project.sh` | 将保存的 GAASD 工程快照恢复到 `project/` 下 |
| `bridge_config.json` | 本次测试使用的 Bridge 配置 |
| `bridge_snapshot/tools/carla_bridge/` | 当前 Bridge 脚本、Python 主程序、组件包源码、adapter 源码 |
| `gaasd_project_snapshot/carla/` | CARLA 联调用 GAASD 工程快照，包含画布数据库、组件 JSON、生成代码、运行 DLL |
| `gaasd_project_snapshot/accpro1_reference/` | `project/accpro1` 参考快照，用于保留原 ACC 画布工程 |
| `prebuilt/ubuntu-env/` | 当前可用的 `libcarla_gaasd_adapter.so` 及 ZMQ 相关运行库 |

## 恢复 GAASD 画布

默认恢复到 `project/carla_restored_20260513`，避免覆盖当前正在工作的 `project/carla`：

```bash
scenarios/acc_carla_phase2_20260513/restore_gaasd_project.sh
```

恢复后在 GAASD 中打开 `project/carla_restored_20260513`，即可查看当时保存的画布、组件和生成代码。

如果确认要覆盖已有恢复目录：

```bash
scenarios/acc_carla_phase2_20260513/restore_gaasd_project.sh --force
```

## 复现实验启动

先恢复或确认 GAASD 工程，再启动 CARLA 与 Bridge：

```bash
scenarios/acc_carla_phase2_20260513/run.sh
```

脚本启动完成后，进入 GAASD 页面，打开对应工程并通过示波器“开始”运行仿真。建议示波器周期为 `0.1s`，观测信号包括 `targetSpeed`、`egoV`、`leadV`、`distance`。

## 注意

- 本快照保留了 `.so`，目的是快速演示；长期维护仍应优先根据 `bridge_snapshot/tools/carla_bridge/adapter/` 源码重新构建。
- `gaasd_project_snapshot/carla/objectCode/total/DLL/` 中保留了运行时 DLL 和必要 symlink，便于恢复后直接运行。
- 本目录不是 CARLA 或 GAASD 软件本体备份；CARLA 安装路径仍默认使用 `/home/aiden/snap/code/app/carla-package`。
