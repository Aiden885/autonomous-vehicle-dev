# GAASD-CARLA ACC/LKS 交接包

## 1. 包内容



```text
docs/                 接口清单、集成说明、LKS旁路闭环方案
repo_payload/          需要合入或参考的工程文件
repo_payload/tools/    CARLA Bridge、Pangu ACC/LKS模块、调试面板
repo_payload/scenarios ACC/LKS可复现场景脚本
repo_payload/project   lks2/newaccpro3 当前修复版生成代码
evidence/              最近一次可用测试结果（如存在）
MANIFEST.txt           文件清单
```

## 2. 关键说明

- Bridge 端口约定：`5701` 为 CARLA 状态发布，`5702` 为控制与驾驶输入接收。
- ACC 关键输入：`egoV`、`leadV`、`distance`、`commandType`。
- ACC 关键输出：`targetSpeed/speed`、`enable`。
- LKS 关键输入：`egoV`、`c0`、`c1`、`c2`、`c3`、`curvature`、`brakePressed`、`driverSteerNorm`。
- LKS 关键输出：`steerRad/lksSteerRad`、`controlEnabled`。
- LKS 默认演示场景使用 CARLA 官方 `Town04 road=45 lane=-3`，起点为
`(-511.738, 242.657, 0.5)`。

## 3. 优先阅读

1. `docs/GAASD_CARLA_ACC_LKS_接口变量清单.md`
2. `docs/GAASD_CARLA_ACC_LKS_开发团队集成说明.md`
3. `repo_payload/scenarios/lks2_pangu_carla_20260701/README.md`
4. `repo_payload/scenarios/newaccpro3_pangu_carla_20260701/README.md`

## 4. 当前状态

- ACC 和 LKS 均按 Pangu 节点方式准备了旁路闭环测试代码。
- LKS 调试页面 A/D/B 输入已改为后端常驻 ZMQ 发布，避免短进程发布造成按键消息丢失。
- CARLA 可视化启动参数包含 `-norhithread`，用于规避本机 VulkanRHI 线程崩溃。

