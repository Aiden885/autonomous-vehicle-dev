# lks2 Pangu-CARLA 闭环场景

该场景使用修复后的 `lks2` 生成代码计算 LKS 转角。Bridge 发布最终接口清单中的
`egoV/c0/c1/c2/c3/curvature/brakePressed/driverSteerNorm`，Pangu 输出
`steerRad/controlEnabled`。

运行前先构建一次：

```bash
tools/pangu_lks_closed_loop/build_pangu_module.sh
```

之后可从调试页面选择本场景并点击“启动环境”，或直接运行：

```bash
scenarios/lks2_pangu_carla_20260701/run.sh
```

页面操作：

- `A` / `D`：按住模拟驾驶员左右转向接管，松开恢复。
- `B`：按住模拟制动退出 LKS，松开恢复。
- `释放`：立即将 `brakePressed` 和 `driverSteerNorm` 归零。
- `测试结果`：读取 `/tmp/lks2-pangu-carla/results/latest_summary.json`。

当前场景使用 CARLA 官方地图 `Town04`。起点为地图 API 筛选出的连续弯道路段候选点
`(-511.738, 242.657, 0.5)`，对应 `road=45、lane=-3`，航向由 CARLA waypoint 自动
确定。该路段在 300 m 内无路口且包含连续缓弯，适合 LKS 单车验证。初始横向偏置为
0，目标速度为 `4 m/s`。

CARLA 启动参数默认包含 `-norhithread`。本机 RTX 5080 + CARLA 0.9.15 在 Vulkan RHI
线程中多次出现 `Signal 11`，关闭独立 RHI 线程后已完成 120 s 可视闭环并保持 CARLA
窗口继续在线。
