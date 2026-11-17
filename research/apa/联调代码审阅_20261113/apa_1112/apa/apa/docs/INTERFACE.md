# APA 接口契约

本说明用于联调和测试判定，字段定义以 `proto/apa_config.proto`、`module_lib/apa_input_contract.cpp`、`module_lib/apa_zmq_bridge.cpp` 为准。模块职责见 [README](../README.md)，验收步骤见 [测试交接说明](TESTING.md)。

## 通道和版本

外部协议为 `gaasd_carla_bridge` / `0.13.0`。ZMQ 消息为两帧：UTF-8 topic、JSON envelope；JSON 含 `header` 和 `payload`。输入 `header.message_type` 必须匹配 topic。

| 方向（相对 Pangu） | Topic | 默认端点 |
| --- | --- | --- |
| 接收车辆与场景反馈 | `gaasd.carla.ego_state.v1` | `tcp://127.0.0.1:5701`，连接 Bridge 发布端 |
| 接收固定任务几何 | `gaasd.carla.apa_plan_request.v1` | 同上 |
| 发布规划结果 | `gaasd.carla.apa_plan_result.v1` | `tcp://127.0.0.1:5702`，Pangu 绑定 |
| 发布执行命令 | `gaasd.carla.control_cmd.v1` | 同上 |

端点分别由 `GAASD_CARLA_INPUT_ENDPOINT`、`GAASD_CARLA_OUTPUT_ENDPOINT` 覆盖；输出来源由 `GAASD_CARLA_SOURCE` 指定。worker IPC 为私有二进制接口，检查版本和结构大小；必须与核心库同批构建，不能按外部 JSON 版本推断二进制兼容性。

## 坐标、身份和时间

| 项目 | 约定 |
| --- | --- |
| 世界坐标 | `coordinate_frame=carla_world`；x/y 为 m，航向为 rad |
| 车辆及目标参考点 | `rear_axle_center`；后轴偏移只在 Bridge 应用 |
| 车辆速度 | `vehicle_speed` 为车头轴有符号速度；`vehicle_motion_speed` 为三维速度模长，须不小于前者绝对值（允许数值误差） |
| 转向反馈 | `vehicle_steering_command_rad` 是已施加的单轨等效命令，不能解释为实测轮角 |
| 重力 | `road_gravity_x/y` 为估计局部平面上的世界水平分量，单位 m/s² |
| 任务身份 | `session_id`、正整数 `task_id`；换动作或固定几何必须创建新任务 |
| 帧和场景 | `header.frame_id`、`payload.frame_id/carla_frame_id` 一致；`scene.version=carla_frame_id` |
| 时间 | `header/payload.sim_time_sec` 一致；`delta_time_sec` 与反馈 `cycle_time` 一致，且能整除 0.02 s |

APA 启用时，同一任务反馈必须逐帧连续，仿真时间增量须与帧差和周期一致。同帧重传只允许安全/恢复确认更新，其他反馈内容不能变化。旧任务、退休会话和旧帧不覆盖当前输入。

`vehicle_profile_id` 必须匹配共享车型 JSON 原始文件的 SHA-256；仅格式化该文件也会改变指纹。不得混用旧 Bridge 配置、新核心库或未配套的地图绑定。

## 请求和容量

`parking_action`：1 泊入、2 泊出。`target_slot_kind` 始终描述原车位：1 VT、2 HT、3 DTH、4 VH；泊出输入也使用原车位类型。泊出候选输出内部类型为 PARKOUT（5），泊入专用 `StepCount/St2Idx/Stp1Pt*` 字段清零。

请求包含车辆位姿、目标位姿、转弯半径、入口点、四个顺序角点、入口边编号及约束数组。四边形须非退化且凸；`slot_entry` 为 1–4。原生适配将入口旋转到前两个角点，调用方不应再次重排。泊出目标由 Bridge 依次取显式配置、此前泊入实际起点、车位默认起点。

| 内容 | 上限或约束 |
| --- | --- |
| 物理边界、虚拟线、轮挡线 | 各 64 条；各坐标数组长度必须等于对应计数 |
| 边界离散点、轮挡点 | 各 128 点；同上 |
| 完整场景 | 最多 4096 个障碍凸片，与 C 线数组容量独立 |
| 单片轮廓 | 3–8 个顺序顶点，有限、非退化、严格凸；障碍 ID 非空且不超过 96 字节 |
| JSON 输入和 worker 场景快照 | 最多 2 MiB；超限拒绝，不能截断 |
| 原生路径 | 最多 100 点；有效点数、方向段数及数组内容还须通过结构验收 |

NaN/Inf、越界值、缺字段、别名冲突、重复 JSON 键和非法容量会被拒绝。同一任务固定请求指纹不能变化；无效输入锁存任务故障，后续正常消息不能在原任务内解除。snake_case 与保留的旧别名同时出现时必须相等，完整字段集合见解析源码。

可选 `goal_tolerance` 仅能收紧运行配置中的到位容差。它作用于目的地，不能扰动泊出车辆起点。`planning_only=true` 仅适用于已启用的泊入任务：可发布路径，控制保持停车；转为执行须使用新任务。

邻位借用由 Bridge 在任务开始时以同帧占用确认后冻结。APA 使用收到的约束，不根据车位编号删除实体、未知障碍或目标自身三边。

## 输出和判定

输出携带会话、任务、CARLA 帧、仿真时间、车型指纹，以及 `replan`、`online_safety`、`input_validation`、`safety_state`。排查时必须将这些字段与原始输入对应。

| 字段或状态 | 测试含义 |
| --- | --- |
| `success=true` | 发布的路径有至少两个点；不能单独作为泊车完成依据 |
| `control_status=plan_ready` | 仅规划任务已有可用路径，车辆继续驻车 |
| `control_status=tracking/complete` | 分别为跟踪和控制器完成；最终验收还需核对实际到位、持续停稳及碰撞记录 |
| `replan.state` | `idle/stopping/planning/waiting_scene/ready/failed`；结合恢复编号、尝试数和路径版本判定 |
| `online_safety` | 当前场景版本、原因、障碍来源及已检查距离；不能把旧故障事件当成本轮结果 |
| `input_validation.error` | 输入契约拒绝，优先检查字段、版本、车型和帧身份 |
| `replan.planner_error_type=36` | 搜索预算耗尽，尚未找到可执行路径；不表示已证明几何无解 |
| `planning_timeout/planner_process_failed` | 分别检查配置的 worker 看门狗、进程退出和配套产物 |

其他原生故障码见 `apaEnums.h`。输入错误、规划错误和跟踪错误应分别记录，不能只用“泊车失败”合并。

执行命令中的 `actuator.owner=C:apaMocoActuate:v2` 标识 C 踏板所有权；Bridge 不再叠加速度/加速度闭环。`launch_hold` 之外不允许油刹重叠。路径里程为相邻点间的有符号区间量，符号表示行驶方向；反向运动不将车身航向额外旋转 π。
