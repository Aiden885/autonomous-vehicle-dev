# APA 泊车仿真

链路：CARLA ↔ `carla_bridge_apa.py` ↔ Pangu APA 节点。网页操作见[场景面板说明](../../tools/gaasd_scenario_panel/README.md)。

## 启动与停止

CARLA 在宿主机运行，默认 RPC 为 `127.0.0.1:2000`。Bridge 与 Pangu 可在 `pangu_x86` 容器内运行；容器须使用 host 网络并挂载工程目录。

1. 宿主机启动 CARLA，已运行时跳过：

   ```bash
   export CARLA_ROOT=/opt/carla
   "$CARLA_ROOT/CarlaUE4.sh" -windowed -ResX=1280 -ResY=720 -quality-level=Low
   ```

2. 容器终端一进入 Pangu 工程的 `install/` 目录，启动节点：

   ```bash
   bash run.sh apa
   ```

3. 容器终端二在 `carla/` 目录启动 Bridge 和面板。`PYTHON_BIN` 须与 CARLA Python API 兼容：

   ```bash
   CARLA_ROOT=/opt/carla PYTHON_BIN=python3.8 bash scenarios/apa/run.sh Town04 29
   ```

   Town15 示例为 `bash scenarios/apa/run.sh Town15 6`。打开 `http://127.0.0.1:8088`，启动后保持驻车，由页面明确发起规划或泊车。

停止 Bridge 和面板：在 `carla/` 执行 `bash scenarios/apa/stop.sh`。停止节点时，在 Pangu 的 `install/` 目录执行：

```bash
source setup.bash
bash stop.sh apa -m=9
```

独立启动的 CARLA 单独停止。Bridge 日志默认为 `/tmp/apa-pangu-carla/carla/bridge_apa.log`。

## 配置与接口

- `bridge_config.json` 是共享配置，修改后重启 Bridge；运行路径按接收环境设置。网页设置只在当前会话生效。
- 协议为 `0.13.0`。Bridge 与 Pangu 须匹配协议、车辆参数指纹和执行限值；命令按 `session_id/task_id/event_id` 确认，控制输出须匹配反馈帧。
- 同帧反馈用车辆姿态近似局部支撑面，发布水平重力分量及已施加转向命令；后者不是物理轮角。
- `C:apaMocoActuate:v2` 显式标记 `launch_hold`，仅该建压交接允许油刹重叠；停止请求仍要求零油门和完全制动。
- 位置、路径和目标使用 CARLA 世界坐标系的后轴中心，单位 m/rad。网页航向输入使用度。后轴偏移仅由 Bridge 应用。
- `apa.vehicle_profile_path` 指向 Pangu 的 `modules/apa/apa/conf/apa_vehicle_profile.json`。尺寸、转向几何、规划余量及到位容差以该文件为准，避免在两端重复维护。
- 自动泊车使用 `control.actuation=c_pedals_v1`：C 端生成最终踏板、手刹和挡位；Bridge 校验后执行，并将单轨等效转角换算为 CARLA 内轮命令。通信、无效指令和碰撞保护独立生效。
- `apa.drivetrain_clutch_strength` 是车型物理参数；使用对应车型的标定值，或设为 `null` 保留原值。
- 控制周期默认 0.02 s；Bridge 确认同帧控制后推进仿真。输入与控制节点的内部唤醒间隔为 1 ms，无新输入时不重复执行控制。场景最多 4096 个凸分片，JSON 上限 2 MiB，两类边界各最多 64 条，超限拒绝而不截断。

| 端口 | 方向 | Topic |
| --- | --- | --- |
| 5701 | Bridge PUB → Pangu | `gaasd.carla.ego_state.v1`、`gaasd.carla.apa_plan_request.v1` |
| 5702 | Pangu PUB → Bridge | `gaasd.carla.control_cmd.v1`、`gaasd.carla.apa_plan_result.v1` |
| 5703 | 面板 PUB → Bridge | `gaasd.carla.apa_command.v1` |

## 地图、车位与边界

- Town04 使用 `parking_slots.json`，默认 29 号；Town15 使用 `parking_slots15.json`，默认 6 号。共享顶点/边及来源位于 `slot_geometry`，加载时校验坐标、引用和 OpenDRIVE 摘要。
- Town04 的 64 个实体车位均配置尾入、头入，共 128 个方向目标。原 1–64 号保留原泊入方式，101–164 号分别为对应车位的另一方向；17、20、24、49、52、56 号原为头入，其新增目标为尾入。Town15 仍为 25 个实体车位，另有 108、115 号头入目标，分别对应 8、15 号。
- `physical_slot_id` 关联实体车位，两个方向共用边界、入口和起点，`apa_slot_kind=vertical_head_in` 路由 VH。前端下拉按实体车位编号合并，通过下方“尾入 / 头入”选择方向，提交时使用对应目标 ID。未配置或已占用的方向不可选，平行位不选择该方式。边界、显示和占用车辆生成按实体车位去重。
- 每张地图使用一份 `parking_maps/<地图>.json.gz` 静态底图。查询窗口按任务起点冻结，Town04 为 32×32 m、Town15 默认 48×48 m。按与自车相连的采集格高度带查询静态几何，再按物体所在及预测扫掠区域的高度带叠加同帧实体；反馈循环不射线采样。
- 车位目录与底图内的筛选、禁行层通过文件摘要绑定，须成套更新，不能仅格式化车位 JSON。底图还校验 CARLA、OpenDRIVE、静态几何和内容摘要，失配时拒绝泊车。
- 底图保留原始采样证据；离线简化最大外扩 2 cm，内部筛选和禁行层须有对应高度、轮廓及车位目录证据。Town15 禁行面采用全道路层允许面并集及补充道路碰撞证据。运行端不依赖 Shapely。
- 采样盲区按 `apa_boundary_policy.py` 保留为诊断；缺失高度覆盖、底图覆盖外及任务窗口外保持硬未知约束。`prepare_map` 仅重新加载底图，缺失覆盖须离线补齐。
- 实时占用按名义停车位置的实际车身判断，返回带来源帧的 `free/occupied/unknown`；泊入只接受已确认空闲。边角受限不等于停车位置被占用，空闲也不保证路径可行。启动保留地图静态车及其他客户端车辆。
- 车位占用与目标结果按地图、高度选择和相关实体的完整三维几何、速度复用。高度选择或实体输入变化时重新评估，地图重载时清空；当前帧的完整规划场景与在线碰撞检查独立更新。
- 静止实体的车体切片使用独立、有界的缓存，完整几何、速度、查询窗口或高度带变化时重新计算；移动实体保留实时扫掠。
- 目标非入口三边保持约束。泊入、泊出开始时均刷新实时占用，仅本次同帧确认空闲的邻位可借用其分隔线空间，名单按任务冻结；占用或未知邻位仍保留框线，共享边去重后保留全部来源。
- 泊出目标依次取显式配置、此前泊入的实际起点、车位默认起点。规划从实际库内位姿向库外目标搜索，复用泊入的候选排序、安全验收和执行链路；完整路径通过验收后才允许执行。
- 显式地图线按 `kind` 分类：`physical_boundary` 为物理边界，`task_forbidden` 为虚拟约束，`parking_marking` 仅作诊断。未分类线保留原通道；来源为 `legacy_calibrated_map_lines_unverified` 的遗留线仅作诊断。类型不能按数组名称猜测。
- `parking_maps/<地图>.parking-lines.json.gz` 按 OpenDRIVE 摘要匹配，仅供网页显示。Town04 四排共 64 个实体车位，方向选择受实时占用状态限制。

## 模块与更新

| 职责 | Bridge 模块 |
| --- | --- |
| 任务与执行 | `carla_bridge_apa.py`、`apa_direct_actuation.py`、`apa_steering_*.py` |
| 地图与容量 | `apa_offline_map.py`、`apa_map_interior.py`、`apa_forbidden_regions.py`、`apa_scene_compaction.py`、`apa_obstacle_tiles.py`、`apa_scene_limits.py` |
| 车位与边界 | `apa_parking_map.py`、`apa_slot_geometry.py`、`apa_parkin_goal.py`、`apa_boundary_policy.py` |
| 同帧诊断 | `apa_scene_diagnostics.py` |

模块位于 `tools/carla_bridge/`。场景资产、共享车型、执行协议及 Pangu 容量定义须配套更新。

Pangu 接口或车型变更须重建安装 `apa_conf_pb`、`apa_core`、三个 APA 节点及 `apa_planner_worker`。Bridge Python 变更需重启 Bridge；面板变更需重载面板并刷新网页。

## 诊断与验证边界

`apa_status.scene_diagnostic` 提供失败阶段、原因和障碍来源。每个发布端点在 `carla/.cache/apa_diagnostics/` 保存 `task-start.json`、`last-event.json`，包含配置、匹配输入和控制回执；后续任务会覆盖，需要时另行留存。没有匹配反馈时不会用新帧替代异常帧，也不从 `no_solution` 推断原生求解阶段。

已知限制：相连采集体积不代表已确认道路拓扑，Town04 尚缺完整停车铺装面依据。完整泊入、泊出及故障恢复需在接收环境验收，离线检查仅覆盖对应输入与几何。

泊入目标使用共享八边形。自动平行泊入在本车位内扣除静态实体占用后选择纵向可行区间中点，任务内固定目标；显式后轴目标保持原值。平行位内侧实体边使用物理余量，其他纵向库底使用库底余量。

静止车辆与底图按相同采集格和高度带选择轮廓，不把相邻格的高度合成大区间。离线文件可携带 `body_sources`；只有对象、资产名、三维包围盒和语义身份匹配、具备双轴车体命中证据的原始采样轮廓才替换静态车包围盒，未覆盖高度或未匹配对象仍保留包围盒。静态采样片按几何并集去重、合并，未知片保持独立语义。移动物体保留完整预测扫掠。
