# GAASD-CARLA 最小 Demo 交付说明

## 1. 交付内容

本交付包用于复现当前最小 CARLA 联调 Demo。该 Demo 以 `project/carla` 为示例工程，验证 CARLA 数据进入 GAASD 组件、GAASD 输出控制命令、Bridge 控制 CARLA 车辆的闭环链路。

交付包包含：

```text
tools/carla_bridge/                         CARLA Bridge 与启动脚本
component_package/gaasd_carla_p0_acc_min_components.tar.gz
                                             GAASD 最小 ACC 组件包
project/carla/                               carlatest 示例工程
scenario/                                    最小 Demo 启动配置
README_交付说明.md                           本说明文档
```

不包含历史设计阶段文档，避免把已经调整过的方案描述误当作最终接口。

## 2. 组件包选择

本 Demo 使用：

```text
gaasd_carla_p0_acc_min_components.tar.gz
```

原因是 `project/carla` 中使用的是最小 ACC 组件：

```text
CARLAACCEgoSpeed
CARLAACCLeadSpeed
CARLAACCLeadDistance
CARLAACCComputeTargetSpeed
CARLAACCLongitudinalCmd
```

`gaasd_carla_p0_components.tar.gz` 偏通用 CARLA 状态组件，`gaasd_carla_p1_components.tar.gz` 面向后续 object_list / lateral / full control 扩展。本次最小 Demo 不需要这两个包。

## 3. 运行前提

本机需要已有 CARLA 0.9.15，默认路径：

```text
/home/aiden/snap/code/app/carla-package
```

如路径不同，需要修改：

```text
scenario/bridge_config.json
scenario/run.sh
```

GAASD 运行工程需要能加载：

```text
project/carla/objectCode/total/DLL/libcarla_gaasd_adapter.so
project/carla/objectCode/total/DLL/libzmq.so.5.2.2
project/carla/objectCode/total/DLL/libnorm.so.1.5.8
project/carla/objectCode/total/DLL/libpgm-5.2.so.0.0.122
project/carla/objectCode/total/DLL/libsodium.so.23.3.0
```

这些文件已随 `project/carla` 一起保留。

## 4. 启动方式

在交付包根目录执行：

```bash
cd GAASD_CARLA_min_demo
scenario/run.sh
```

脚本会启动：

```text
本机 CARLA
CARLA Bridge
自车 hero
前车 gaasd_lead
CARLA spectator 视角
```

启动成功后，在 GAASD 中打开 `project/carla`，通过示波器运行仿真。

建议观测信号：

```text
targetSpeed
egoV
leadV
distance
```

## 5. 停止方式

停止 CARLA 与 Bridge：

```bash
tools/carla_bridge/stop-gaasd-carla-manual.sh
```

如果仍有残留进程，可检查：

```bash
pgrep -af 'CarlaUE4|carla_bridge.py|set-spectator-follow.py'
```

## 6. 文件用途

`tools/carla_bridge/carla_bridge.py`：
连接 CARLA Python API，发布仿真数据，接收控制命令。

`tools/carla_bridge/adapter/`：
GAASD 工程中加载的 `libcarla_gaasd_adapter.so` 源码，用于 C 组件和 Bridge 之间通信。

`tools/carla_bridge/start-gaasd-carla-manual.sh`：
一键启动 CARLA、Bridge、测试车辆和视角。

`tools/carla_bridge/fix-gaasd-generated-code.py`：
GAASD 重新生成代码后，用于修复生成头文件中的 protobuf-c 依赖和枚举缺失问题。

`project/carla`：
当前最小 Demo 的 GAASD 示例工程。

`component_package/gaasd_carla_p0_acc_min_components.tar.gz`：
与 `project/carla` 匹配的 GAASD 最小 ACC 组件包。

## 7. 后续集成建议

本 Demo 只用于最小闭环验证。若要进入正式 GAASD/智控节点集成，建议将 Bridge 保留为 CARLA 仿真适配层，将 ACC 算法封装为智控节点，由节点订阅标准输入消息并发布控制输出。