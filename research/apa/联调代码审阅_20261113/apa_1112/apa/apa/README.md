# APA 运行模块

本目录提供 Pangu APA 节点、独立规划 worker、ZMQ 输入输出模块及测试源码。测试接收与执行见 [测试交接说明](docs/TESTING.md)，消息和状态判定见 [接口契约](docs/INTERFACE.md)。构建成功表示代码可编译，泊车能力以对应版本的离线回归和 CARLA 闭环结果为准。

## 代码职责

| 路径 | 职责 |
| --- | --- |
| `src/ApaZmqInputModule.*`、`module_lib/apa_input_contract.*` | JSON 接收、输入契约和任务快照合成 |
| `module_lib/apa_zmq_bridge.*`、`src/ApaZmqOutputModule.*` | ZMQ 收发、Protobuf 与外部 JSON 映射 |
| `src/ApaModule.*` | 任务生命周期、停稳握手、规划调度和控制状态 |
| `worker/apa_planner_worker.cpp`、`module_lib/apa_planning_job.*` | 可终止的规划进程、私有 IPC 和候选执行预演 |
| `module_lib/apa_algorithm/XXX/` | FuncModule 组合、数值端口和 C 接口适配 |
| `module_lib/apa_algorithm/apa/` | C 规划、几何、碰撞检查、跟踪控制和踏板执行器 |
| `module_lib/apa_parking_search.cpp`、`apa_parking_motion.h` | 共用结构化搜索及 RS 连接 |
| `module_lib/apa_online_safety.*`、`apa_motion_prediction.*`、`apa_tracking_execution.*` | 场景验收、运动预测和在线/候选共用控制入口 |
| `proto/apa_config.proto`、`conf/` | 消息定义、运行配置和共享车型参数 |

构建仅读取本目录运行源码；`functions/algmodule/APA/`、`export_apa_code/APA_v5AA/` 是独立副本，不会自动同步。Protobuf、车型常量和安装二进制由构建生成。

车位、目标、障碍、轮挡和虚拟线由 Bridge 提供，经 Protobuf、worker 快照及 FuncModule 端口进入 C 算法。完整多边形场景通过请求级资源编号绑定查询回调；Pangu 不维护地图表。进程内指针和场景资源不跨 worker IPC。

原生规划接口接收 `Input/Param/Output`，离散接口接收 `Input/Output`，不保存跨调用规划状态；FuncModule 保留规定的空 `State`。跟踪、执行器和恢复状态由调用方持有。调用方必须按当前头文件重新编译。

## 规划与执行

两种动作共用候选池、质量排序、结构化搜索、RS 连接、离散化、安全门和执行预演。

| 动作 | 搜索起点 | 搜索终点 | 发布前处理 |
| --- | --- | --- | --- |
| 泊入 | 库内停车目标及允许的目标区域锚点 | 车辆实际位姿 | 反转运动，恢复车辆到目标的顺序 |
| 泊出 | 车辆实际位姿，固定起点 | Bridge 指定的库外目标 | 直接采用搜索方向，转换为泊出元数据 |

泊入保留 VT、HT、DTH、VH 几何模板。非平行位在车体中心进入库内且航向适用时，可按库口法向生成直出种子；距离按完整车身越过库口计算，每步仍检查硬扫掠，曲线搜索和原始根保留。平行位继续使用曲线搜索。直出前缀属于完整候选，不能在整条路径验收前单独执行。

质量键先比较实际换向次数，再比较几何、近障和短同向段代价；短段不是直接删除路径点或拒绝可行路径的条件。泊入跳段后处理只有在质量改善且重新通过曲率、完整扫掠和控制执行验收后才能替换候选。搜索分辨率和请求级预算见 `apaParkingSearch.h`、`apa_parking_search.cpp`；预算耗尽不等于几何无解。

首次规划和重规划均先制动，收到匹配恢复编号的连续停稳反馈后，才向 worker 提交实际起点和冻结场景。worker 候选经过原生安全门和名义执行预演；接收节点再核对起点偏差，并用当前场景验收完整路径，通过后更新路径版本。

物理规划余量、虚拟约束余量、库底余量及倒角取自共享车型，轮挡保留轮轴检查。GENERAL、STATIC、DYNAMIC 按各自模式解析余量。泊入参考误差门内的已知静态来源采用跟踪净距；动态及未知来源保留完整检查。泊入在线跟踪、停车及重规划允许跨虚拟线，完整候选和最终目标仍检查虚拟约束；实体接触持续拒绝。

净距不足且车身未接触时，可使用有界、单方向恢复前缀。许可相对冻结起点计算，不随采样累积，也不适用于新几何；连接点和后续路径须恢复正常包络。策略见 `apaTrackingPolicy.h`。

`StepApaTracking` 是在线执行与候选预演的共同入口，在状态副本上试算，再提交一次控制状态更新。一次反馈内共享不可变安全场景，不跨帧复用几何。控制按带符号里程划分方向段，融合连续投影、曲率前馈、横偏和航向反馈；段末停稳、换挡确认及最终持续停稳分别检查。

`apaMocoBraking.h` 共用反应、制动建立和匀减速剖面。指导限速截到当前段，实际制动扫掠保留过冲预算；名义转向响应、实测侧滑及横摆残差参与预测。

所有停车和故障覆盖完成后，`apaMocoActuate` 生成最终踏板、手刹和挡位。Bridge 执行 C 踏板，将单轨等效转角换算为 CARLA 内轮命令。仅 `C:apaMocoActuate:v2` 的 `launch_hold` 允许受控的短时油刹重叠。候选预演使用名义模型，不能替代 CARLA 物理闭环。

## 配置与构建

| 内容 | 维护位置 |
| --- | --- |
| 尺寸、倒角、转向几何、规划余量、默认到位容差 | `conf/apa_vehicle_profile.json`；按文件内容生成车型指纹和 C 常量 |
| 速度、周期、执行限值及规划等待策略 | `conf/apa.pt`；`conf/default.pt` 为默认配置 |
| 跟踪净距与恢复策略 | `module_lib/apa_algorithm/apa/apaTrackingPolicy.h` |
| 场景容量及输入校验 | `module_lib/apa_scene_limits.h`、`apa_input_contract.*` |

应用拓扑 `configs/app_module/apa.pt` 加载 `conf/apa.pt`。调度唤醒间隔与 20 ms 控制周期不同：同帧重传复用输出，晚到安全反馈先处理，控制按仿真时间推进。

`attempt_timeout_sec=0` 关闭可选 worker 墙钟看门狗。`recovery_timeout_sec` 仅计停稳、观察和重试等待，不计求解及接收端候选验收；它不是总响应时限。规划尝试数按一次恢复计，连续恢复还要求位姿进展。

C 算法使用 C99，核心库和组合层需要支持 C++20 的编译器。生产构建需配套的 Pangu SDK、Protobuf、ZeroMQ 和工程依赖；构建命令、独立检查入口及产物清单见 [测试交接说明](docs/TESTING.md)。安装时成套更新核心库、节点、worker、ZMQ 模块、协议及配置，源码或构建目录更新不会替换已加载库。

源码注释保留字段、单位、坐标、有效性、所有权、FuncModule Traits 和 `run()` 元数据；MATLAB 来源只记录相应 `.m` 文件名。修复过程、临时日志和本机验证产物不进入运行模块。
