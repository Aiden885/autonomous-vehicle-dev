# APA 算法架构分析与拆解调研报告

> **版本：** v1.1（二次校验修订版）
> **日期：** 2026-02-07
> **关联文档：** [DEV_SPEC.md](../DEV_SPEC.md)、[DEV_RULES.md](../DEV_RULES.md)

---

## 一、系统总览

APA（Automatic Parking Assist，自动泊车辅助）系统是一套基于 **Simulink MBD** 开发的自动泊车算法，运行于 AITO M5/F515 平台，周期为 **20ms**。原始 `APA/` 源树包含 **190 个 MATLAB `.m` 文件**（根目录2 + DataDictionary 14 + PlanningSubFcn 98 + GeoPlanningSubFcn 58 + SPTSubFcn 6 + USSubFcn 12），另有顶层 Simulink 模型工件 `F515_APA_Seres_V3p24_offline.slx/.mdl`。

**原始 `APA/` 源树当前全部为 MATLAB/Simulink 资产，无原生 C/H 文件**。本次拆解的目标是将其按照 [DEV_RULES.md](../DEV_RULES.md) 和 [DEV_SPEC.md](../DEV_SPEC.md) 规范重构为 C99 代码。

---

## 二、功能架构（一级分类 → 二级分类 → 三级模块）

```
APA_System（系统顶层，20ms 周期任务）
│
├── 一级：感知融合层（@tag_level1 = 感知）
│   │
│   ├── 二级：超声波处理（@tag_level2 = 超声波）
│   │   ├── processFSPoints          — FS点云处理主流程（~3100行，复合）
│   │   ├── ultrasonicCorrection     — USS+视觉融合校正（~4600行，复合）
│   │   ├── Is_Pt_In_Poly            — 点在多边形内判断（原子）
│   │   ├── distance_point_to_segment — 点到线段距离（原子）
│   │   ├── shiftLineSegment         — 线段平移（原子）
│   │   ├── checkOverlapManual       — AABB重叠率计算（原子）
│   │   ├── rotateSlot               — 车位旋转变换（原子）
│   │   ├── SamplePointsOnLine       — 线段均匀采样（原子）
│   │   ├── expand_rectangle         — 矩形外扩（原子）
│   │   ├── sortRectClockwise        — 顺时针排序（原子）
│   │   ├── checkIntersection        — 线段-矩形相交检测（原子）
│   │   └── sortSPSLotPts            — 车位点排序（原子）
│   │
│   ├── 二级：车位感知（@tag_level2 = 车位感知）
│   │   ├── iou_area                 — IoU车位匹配（复合）
│   │   ├── boundary_filter_process  — 边界滤波预处理（复合）
│   │   ├── transform_ipm_pt_to_global — IPM→全局坐标变换（原子）
│   │   ├── transform_ego_pt_to_global — 车体→全局坐标变换（原子）
│   │   ├── transform_global_pt_to_ego — 全局→车体坐标变换（原子）
│   │   └── navi_update_flg_cal       — 导航更新标志（原子）
│   │
│   └── 二级：导航定位（@tag_level2 = 导航）
│       └── 航迹推算模块（轮速脉冲+航向角积分，在Simulink中实现）
│
├── 一级：规划决策层（@tag_level1 = 规划）
│   │
│   ├── 二级：几何路径规划（@tag_level2 = 几何规划）
│   │   │
│   │   ├── 三级：顶层调度
│   │   │   └── geo_slotPlanSelect       — 车位规划选择器（复合）
│   │   │
│   │   ├── 三级：主规划器（复合）
│   │   │   ├── geo_VT_PathPlanning      — 垂直泊车（倒车入库）
│   │   │   ├── geo_HT_PathPlanning      — 水平泊车（侧方停车）
│   │   │   ├── geo_VH_PathPlanning      — 垂直车头入库
│   │   │   ├── geo_PkOutFt_PathPlanning — 前向泊出规划
│   │   │   ├── geo_PkOut_dangerPt       — 泊出危险点分析
│   │   │   ├── geo_PkOut_dangerPt_AngSrch — 泊出危险点角度搜索
│   │   │   ├── geo_PkOut_dngPt_areaSrch — 泊出危险点区域搜索
│   │   │   └── geo_PkOut_SltLength      — 泊出车位长度计算
│   │   │
│   │   ├── 三级：几何原语（原子）
│   │   │   ├── geo_2Circle_Link         — 双圆弧S曲线连接
│   │   │   ├── geo_2Circle_LinkHT       — 水平泊车双圆弧
│   │   │   ├── geo_rev2Circle_Link      — 反向双圆弧（换向）
│   │   │   ├── geo_circle_1pt1line      — 圆弧切点-线连接
│   │   │   ├── geo_circle_2line_R       — 指定半径双线切圆
│   │   │   └── geo_axis_trans           — 坐标系变换
│   │   │
│   │   ├── 三级：入口点与危险点（原子/复合）
│   │   │   ├── geo_vt_entrPt_dngPt      — VT入口+危险点
│   │   │   ├── geo_vh_entrPt_dngPt      — VH入口+危险点
│   │   │   ├── geo_HT_dangerPt          — HT危险点分析
│   │   │   └── geo_VT_dangerPt          — VT危险点分析
│   │   │
│   │   ├── 三级：碰撞区间检测（原子）
│   │   │   ├── geo_circle_Intrv_check   — 圆弧段碰撞检测
│   │   │   ├── geo_line_Intrv_check     — 直线段碰撞检测
│   │   │   └── geo_StPtSpc_vt           — 起点左右空间测量
│   │   │
│   │   ├── 三级：路径优化（原子/复合）
│   │   │   ├── geo_vt_1R_fix            — VT单弯优化
│   │   │   ├── geo_ht_1R_fix            — HT单弯优化
│   │   │   └── geo_PthShortCut          — 路径捷径优化
│   │   │
│   │   ├── 三级：路径离散化（原子）
│   │   │   ├── geo_PthDiscret_Vt        — VT路径离散化
│   │   │   ├── geo_PthDiscret_Ht        — HT路径离散化
│   │   │   ├── geo_PthDiscret_Vh        — VH路径离散化
│   │   │   ├── geo_TrnDirectDisctet     — 弧线离散化
│   │   │   └── geo_LnDirectDisctet      — 直线离散化
│   │   │
│   │   └── 三级：分类与工具（原子）
│   │       ├── geo_PosTypClassPkIn      — 位置类型分类器（47类）
│   │       ├── geo_divide               — 安全除法
│   │       └── geo_line_cross_pt        — 线线交点
│   │
│   ├── 二级：搜索式规划（@tag_level2 = 搜索规划）
│   │   │
│   │   ├── 三级：Hybrid A* 搜索
│   │   │   ├── ClassHybridAStar         — 搜索引擎配置（类→结构体）
│   │   │   ├── HybridNode / hybrid_node — 搜索节点（类/结构体双版本）
│   │   │   ├── HashNodesList            — 哈希表节点列表（开/闭集）
│   │   │   ├── PriorityQueueNodesList   — 最小堆优先队列（开集）
│   │   │   ├── getHeuristicCost         — 启发函数 h(n)（原子）
│   │   │   ├── getTotalCost             — 代价函数 f(n)=g(n)+wh(n)（原子）
│   │   │   └── VehicleDynamic           — 自行车运动学模型（原子）
│   │   │
│   │   ├── 三级：Reeds-Shepp 路径
│   │   │   └── RSPath                   — RS路径库（18种模式）
│   │   │
│   │   ├── 三级：碰撞检测
│   │   │   ├── VehicleCollisionCheck    — 三层碰撞主检测器（~1200行，复合）
│   │   │   ├── collision_Check_SAT      — SAT矩形-线段（原子）
│   │   │   ├── collision_Check_SAT_Octagon — SAT八边形-线段（原子）
│   │   │   ├── Rect_Collision_Check_For_Boundary — 矩形-边界检测（复合）
│   │   │   ├── isLineIntersectWithRect  — 线段-矩形交叉（原子）
│   │   │   ├── is_Line_Intersect        — 线段-线段交叉（原子）
│   │   │   ├── is_Pt_In_Rect            — 点在矩形内（原子）
│   │   │   └── is_Pt_In_Ploy            — 点在多边形内（原子）
│   │   │
│   │   └── 三级：泊出与重规划
│   │       ├── ClassParallelParkOut     — 侧方泊出规划器（复合）
│   │       └── ClassParallelReplanInSlot — 侧方槽内重规划器（复合）
│   │
│   ├── 二级：车位管理（@tag_level2 = 车位管理）
│   │   ├── ClassSlot                    — 单车位数据结构
│   │   ├── ClassMultiSlot               — 多车位管理容器
│   │   ├── ClassSlotListArr             — 车位列表数组
│   │   ├── DetectSlotSpace              — 车位空间探测（复合）
│   │   └── getParkingSysMainState       — 系统主状态获取（原子）
│   │
│   └── 二级：路径管理（@tag_level2 = 路径管理）
│       ├── ClassPath                    — 路径数据结构（100点定长）
│       ├── ClassComplexPath             — 多段路径容器
│       └── ClassMultiComplexPath        — 多车位路径容器（20条）
│
├── 一级：控制执行层（@tag_level1 = 控制）
│   │
│   ├── 二级：运动控制（@tag_level2 = 运动控制）
│   │   ├── 速度控制器
│   │   ├── 转向控制器
│   │   └── 加速度斜坡生成（梯形/S型）
│   │
│   └── 二级：安全保护（@tag_level2 = 安全）
│       ├── MEB（最小紧急制动）
│       ├── AEB（自动紧急制动）
│       └── LAEB（低速AEB）
│
├── 一级：数据管理层（@tag_level1 = 数据）
│   │
│   ├── 二级：标定参数（@tag_level2 = 标定）
│   │   ├── APA_Cal                      — 车辆几何+转向标定
│   │   ├── Moco_Parameter               — 运动控制参数
│   │   ├── SPT_Parameter                — 车位跟踪卡尔曼参数
│   │   └── 其他14个标定文件
│   │
│   └── 二级：全局配置（@tag_level2 = 配置）
│       └── get_planner_global           — 全局配置单例（persistent）
│
└── 一级：通用工具层（@tag_level1 = 工具）
    │
    ├── 二级：数学工具（@tag_level2 = 数学）
    │   ├── ClassMath                    — 静态数学工具类（原子）
    │   ├── uisee_float32_equal/less/greater — 浮点比较（原子）
    │   ├── uisee_float32_norm/cross_2D/cross_3D — 向量运算（原子）
    │   ├── float32_divide_zero_process  — 除零保护（原子）
    │   ├── mod2pi                       — 角度归一化（原子）
    │   └── linear_func_slove            — 线性方程组求解（原子）
    │
    ├── 二级：坐标变换（@tag_level2 = 坐标变换）
    │   ├── GroundCS_2_VehCS             — 地面→车辆坐标（原子）
    │   ├── VehCS_2_GroundCS             — 车辆→地面坐标（原子）
    │   └── uisee_angle2dcm              — 欧拉角→旋转矩阵（原子）
    │
    ├── 二级：车辆几何（@tag_level2 = 车辆几何）
    │   ├── get_ego_corners              — 自车四角点（原子）
    │   ├── get_veh_corner_point         — 车辆角点+安全余量（原子）
    │   ├── get_octagon_corners          — 八边形角点（原子）
    │   └── calcuRectangleCorners        — 通用矩形角点（原子）
    │
    └── 二级：枚举定义（@tag_level2 = 枚举）
        └── 27个 Enum*.m 文件            — 所有状态/类型枚举
```

---

## 三、核心数据流

```
┌─────────────────────────────────────────────────────────────────────┐
│                          输入信号 (~1870个CAN信号)                    │
│  USS×12 超声波  |  4路鱼眼相机  |  轮速脉冲×4  |  IMU/YawRate  |  HMI │
└────────┬──────────────┬──────────────┬──────────────┬───────────────┘
         │              │              │              │
    ┌────▼────┐   ┌─────▼─────┐  ┌────▼────┐   ┌────▼────┐
    │ USS处理  │   │ FS点云处理 │  │ 航迹推算 │   │ 状态机  │
    │12传感器  │   │processFSPt│  │ 轮脉+YR  │   │ APA状态 │
    │距离测量  │   │DBSCAN聚类 │  │→(X,Y,θ) │   │ 0→5     │
    └────┬────┘   │RANSAC拟合 │  └────┬────┘   └────┬────┘
         │        │凸包/边界  │       │              │
         │        └────┬──────┘       │              │
         │             │              │              │
    ┌────▼─────────────▼──────────────▼──┐           │
    │    ultrasonicCorrection            │           │
    │  (USS + 视觉融合，特征锁定/校正)      │           │
    │  输出: pole_lines, wall_lines,     │           │
    │        curb_lines, pole_center     │           │
    └──────────────┬─────────────────────┘           │
                   │                                  │
    ┌──────────────▼──────────────────────┐           │
    │      车位检测与跟踪 (SPT)            │           │
    │  IoU匹配 → Kalman滤波 → 车位确认     │           │
    │  输出: ClassSlot[] (4角点+目标位姿)   │           │
    └──────────────┬──────────────────────┘           │
                   │                                  │
         ┌─────────▼──────────┐                       │
         │  状态判断: SEARCHING │◄─────────────────────┘
         │  or PARKING ?       │
         └───┬────────────┬────┘
      搜索中 │            │ 泊车中
    ┌────────▼──┐   ┌─────▼──────────────────────────┐
    │继续感知扫描│   │        路径规划                   │
    │等待车位确认│   │                                  │
    └───────────┘   │  ┌─ geo_VT/HT/VH_PathPlanning  │
                    │  │  (几何规划: 圆弧+直线组合)      │
                    │  │                               │
                    │  ├─ ClassHybridAStar             │
                    │  │  (搜索规划: 网格A*+RS捷径)      │
                    │  │                               │
                    │  └─ 融合选择最优路径               │
                    │     ClassPath(100点定长)          │
                    └──────────────┬──────────────────┘
                                   │
                    ┌──────────────▼──────────────────┐
                    │        轨迹跟踪与控制              │
                    │  PID/转向控制 → 目标转角+速度      │
                    │  加速度斜坡限制（jerk-limited）    │
                    └──────────────┬──────────────────┘
                                   │
                    ┌──────────────▼──────────────────┐
                    │        安全层 (并行运行)           │
                    │  MEB: 最小紧急制动 (USS基)        │
                    │  AEB: 自动紧急制动               │
                    │  LAEB: 低速AEB                  │
                    └──────────────┬──────────────────┘
                                   │
┌──────────────────────────────────▼──────────────────────────────────┐
│                          输出信号 (CAN)                              │
│  目标转角 HZP_StrAng | 目标速度 HZP_TarV | 档位 HZP_GearCmd         │
│  制动请求 | 泊车状态 | 路径段数 | 碰撞标志                            │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 四、系统输入输出信号

### 4.1 主要输入信号

#### HMI / 用户指令（CAN 0x708, 0x703）

| 信号 | 描述 |
|------|------|
| `APA_SW_0x708` | APA 系统开关 |
| `APA_Mode_0x708` | APA 模式（0-3） |
| `APA_Slot_Dir_0x708` | 车位方向（左/右） |
| `APA_SlotSlctX/Y_0x708` | 选定车位坐标 |
| `APA_Resume_0x703` | 恢复泊车操作 |
| `APA_Cncl_0x703` | 取消泊车 |
| `HZP_ActSW_0x703` | HZP 激活开关 |

#### 底盘 / ESC 信号（CAN 0x212, 0x2C2, 0x2A2, 0x142, 0x122, 0x272）

| 信号 | 描述 |
|------|------|
| `esc_vehicleSpeed` | 车速（0-270 km/h，分辨率 0.05625） |
| `esc_masCylBrakePressure_0x212` | 主缸制动压力（0-250 bar） |
| `esc_epbStatus_0x2C2` | 电子驻车制动状态 |
| `esc_vehicleStandstill_0x2A2` | 车辆静止标志 |
| `esc_wheelSpeedFL/FR/RL/RR` | 四轮轮速 |
| `esc_wheelDirectionFL/FR_0x272` | 轮转方向 |
| `esc_sumOfEdgeFL/FRWSS_0x272` | 轮速脉冲计数（里程计） |

#### 超声波传感器（12 个）

- **前 4 个**（FOL, FCL, FCR, FOR）— 前向
- **后 4 个**（ROL, RCL, RCR, ROR）— 后向
- **侧 4 个**（FSL, FSR, RSL, RSR）— 长距侧向

每个传感器有安装位置 `[x, y, angle]`。

#### 视觉 / 鱼眼相机

- 车位检测结果（4 角点 P1-P4）
- 相机遮挡/脏污状态
- 环视边界（SVB）数据

### 4.2 主要输出信号

#### 运动控制指令（CAN 0x775）

| 信号 | 描述 |
|------|------|
| `HZP_StrAng_0x775` | 目标转向角 |
| `HZP_TarV_0x775` | 目标速度 |
| `HZP_GearCmd_0x775` | 档位指令（N=0, D=1, R=2, P=3） |
| `HZP_StrReq_0x775` | 转向请求标志 |
| `HZP_VCU_Req_0x775` | VCU 请求 |
| `HZP_TarLen_0x775` | 目标距离 |

#### 规划状态（CAN 0x772, 0x776, 0x771）

| 信号 | 描述 |
|------|------|
| `APA_TarTht_0x772` | 目标航向角 |
| `PthSegNum_0x771` | 路径段数 |
| `GoDir_0x771` | 行驶方向 |
| `APACtrl_Dist2Go_0x771` | 剩余距离 |
| `APA_GearReq_0x776` | 档位请求 |

---

## 五、关键车辆参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 车长 | 4.763m | `k_APA_LL` |
| 车宽 | 1.855m | `k_APA_Wd` |
| 轴距 | 2.780m | `k_APA_WhlBse` |
| 后悬 | 1.007m | `k_APA_Lr` |
| 前悬 | 0.976m | `k_APA_Lf`（计算值：LL - Lr - WhlBse） |
| 后轴到前保距离 | 3.756m | `k_APA_LLf`（计算值：LL - Lr） |
| 后轴到车辆中心 | 1.3745m | `K_Dist_RrAx2VehCt`（= LL/2 - Lr） |
| 轮距 | 1.600m | `k_TrackBase` |
| 最小转弯半径 | 5.5m | `k_APA_Rmin` |
| 最大方向盘转角 | 500° | `k_APA_MaxStrAng` |
| 超声波传感器 | 12个 | 前4+后4+侧4 |
| USS无效默认距离 | 5.5m | `k_US_defaultEmpt_Dist` |
| 转向映射（正向） | 二次多项式 | `a2=-0.06139, a1=18.46`（方向盘角→前轮转角） |
| 转向映射（逆向） | 二次多项式 | `a2=1.034e-4, a1=0.03623`（前轮转角→方向盘角） |

---

## 六、APA 状态机

**`APA_Chasis_initial.m` 中定义的 6 个基础状态：**

```
APA_OFF (0) → APA_InI (1) → APA_STAND_BY (2) → APA_PreACTIVE (3) → APA_ACTIVE (4) → APA_ENDOUT (5)
```

> **注意：** `getParkingSysMainState.m` 中引用了超出 0-5 范围的状态值（10, 11, 15, 16, 17, 18, 20），这些扩展状态可能在 Simulink 模型内部或其他文件中定义，`APA_Chasis_initial.m` 仅定义了 6 个命名常量（0-5）。

**`getParkingSysMainState.m` 中的主状态映射：**

- **SEARCHING**（状态 1, 15, 16）：行驶中扫描、检测和跟踪车位
- **PARKING**（状态 3, 4, 5, 10, 11, 17, 18, 20）：执行泊车动作
- **INVALID**（其他所有状态值）：默认无效

---

## 七、Hybrid A* 搜索规划参数

| 参数组 | 关键值 |
|--------|--------|
| **网格** | XY 分辨率=0.5m, 航向分辨率=5°, 地图边长=24m, 像素数=480 |
| **运动** | 运动分辨率=0.1m, 转向离散=20级, 运动步长=6m, 碰撞检测步长=0.5m |
| **代价权重** | SB_COST=1, BACK_COST=1, BACK_COST_IN_SLOT=1.1, STEER_CHANGE_COST=0, STEER_COST=0, H_COST=5 |
| **车辆** | WB=2.78m, W=1.855m, LL=4.763m, LB=1.007m, MIN_CIRCLE=5.5m |
| **默认安全余量** | F=0.2, R=0.2, S=0.2 |
| **安全余量** | 近车位垂直: F=0.4, R=0.3, S=0.15; 近车位水平: F=0.35, R=0.3, S=0.15 |
| | 远车位垂直: F=0.4, R=0.4, S=0.35; 远车位水平: F=0.4, R=0.3, S=0.3 |
| **限制** | 开集=1000, 闭集=1000, 路径最大=100点, 路径有效=70点, 每帧计算量=180 |
| | grid开集=200, grid闭集=800, 每车位最大计算量=150, 每车位最小计算量=50 |

---

## 八、目录结构与文件统计

```
/APA/
  initCals.m                              — 启动入口：添加路径 + 运行所有标定脚本
  bak_initCals.m                          — 备份（Windows路径版）
  F515_APA_Seres_V3p24_offline.slx        — 主 Simulink 模型

  DataDictionary/                         — 14 个文件
    mfile_APA_20ms_Task_M5.m              — 24312行，1870个CAN I/O信号定义
    APA_Cal.m                             — 车辆几何 + USS位置 + 转向标定
    SPT_Parameter.m                       — 车位跟踪卡尔曼滤波参数
    Moco_Parameter.m                      — 运动控制参数
    APA_Chasis_initial.m                  — APA状态机枚举
    APA_FS_Fusion_Calibration_Para.m      — 鱼眼+USS融合参数
    US_Coliision_Detect.m                 — MEB碰撞阈值 + USS安装几何
    Navi_Par.m                            — 导航航迹推算参数
    Navi_LRAS.m                           — 长距导航参数
    SlotPos_Cal.m                         — 视觉车位标定
    AEB_Macros_initial.m                  — AEB状态机 + 控制阈值
    initial_Ramp.m                        — 加速度斜坡 / 加加速度限制
    LAEB_cal.m                            — 低速AEB参数
    LAEB_macros.m                         — LAEB档位/功率枚举

  PlanningSubFcn/                         — 98 个文件：核心规划算法
    ClassPath.m                           — 路径数据结构（x,y,theta 航点）
    ClassComplexPath.m                    — 路径 + A*搜索状态
    ClassMultiComplexPath.m               — 多车位路径容器（20条）
    ClassMultiSlot.m                      — 多车位管理
    ClassSlot.m                           — 单车位数据结构
    ClassSlotListArr.m                    — 车位列表数组
    ClassHybridAStar.m                    — Hybrid A* 规划器
    ClassMath.m                           — 数学工具（静态方法）
    get_planner_global.m                  — PERSISTENT 全局配置（map/veh/conf）
    getParkingSysMainState.m              — APA状态 → SEARCHING/PARKING
    VehicleDynamic.m                      — 自行车运动学模型
    VehicleCollisionCheck.m               — SAT碰撞检测
    DetectSlotSpace.m                     — 车位空间探测
    RSPath.m                              — Reeds-Shepp 路径
    Enum*.m (27 个文件)                    — 所有枚举定义
    geo_*, boundary_*, collision_* 等     — 几何工具函数

  GeoPlanningSubFcn/                      — 58 个文件：几何路径规划
    geo_VT_PathPlanning.m                 — 垂直车位路径规划
    geo_HT_PathPlanning.m                 — 水平车位路径规划
    geo_VH_PathPlanning.m                 — 垂直车头入库
    geo_PkOut*.m                          — 泊出规划
    geo_*dangerPt*.m                      — 危险点分析
    geo_*Discret*.m                       — 路径离散化
    geo_circle_*.m                        — 圆弧几何工具
    geo_plotVeh*.m                        — 可视化（调试用）

  SPTSubFcn/                              — 6 个文件：空间跟踪
    boundary_filter_process.m             — 边界滤波
    transform_*_pt_to_*.m                 — 坐标变换
    navi_update_flg_cal.m                 — 导航更新标志
    iou_area.m                            — IoU 面积计算

  USSubFcn/                               — 12 个文件：超声波处理
    ultrasonicCorrection.m                — USS融合校正（~4600行）
    processFSPoints.m                     — FS点云处理（~3100行）
    sortSPSLotPts.m                       — 车位点排序
    checkOverlapManual.m                  — 重叠检测
    expand_rectangle.m                    — 矩形外扩
    rotateSlot.m                          — 车位旋转
    等
```

---

## 九、拆解注意事项

### 9.1 OOP → C 结构体 + 函数族

| MATLAB 概念 | C99 映射策略 |
|-------------|-------------|
| `classdef` + `handle` | `typedef struct { ... } Xxx_t;` + 指针传递 |
| 类方法 | `void xxx_methodName(Xxx_t* self, ...)` |
| 静态方法 | 普通 C 函数（无 `self` 参数） |
| 构造函数 | `void xxx_init(Xxx_t* self, ...)` |
| `varargin`/`nargin` | 配置结构体 + 默认值，或多个显式包装函数 |
| 继承 | 组合（嵌入父结构体） |

**受影响的类（13个）：** ClassPath, ClassSlot, ClassHybridAStar, HybridNode, HashNodesList, PriorityQueueNodesList, ClassComplexPath, ClassMultiComplexPath, ClassParallelParkOut, ClassParallelReplanInSlot, ClassMath, ClassMultiSlot, RSPath

### 9.2 persistent 变量 → State_t

| 文件 | persistent 变量数 | 策略 |
|------|------------------|------|
| `processFSPoints` | **23** | 统一归入 `FsPointsState_t` |
| `ultrasonicCorrection` | **22** | 统一归入 `UltraCorrState_t` |
| `get_planner_global` | 1（大结构体） | 转为全局 `PlannerGlobal_t g_plannerGlobal` |
| Simulink Unit Delay | 大量 | 各模块 `XxxState_t` |

### 9.3 动态数组 → 定长数组

| 场景 | 最大尺寸 | 宏定义建议 |
|------|---------|-----------|
| 路径点 | 100 | `MAX_PATH_POINTS = 100` |
| A* 节点 | 1000 | `MAX_ASTAR_NODES = 1000` |
| 边界线 | 500×4 | `MAX_BOUNDARY_LINES = 500` |
| 散射点 | 1000×2 | `MAX_SCATTER_POINTS = 1000` |
| 凸包顶点 | 40 | `MAX_HULL_VERTICES = 40` |
| 车位数 | 10 | `MAX_SLOTS = 10` |
| 规划路径段 | 30 | `MAX_PATH_SEGMENTS = 30` |
| 采样点 | 100 | `MAX_SAMPLE_POINTS = 100` |
| 哈希表 | 25013 | `HASH_TABLE_SIZE = 25013` |
| 锁定特征 | 各20 | `MAX_LOCKED_POLES/WALLS/CURBS = 20` |

### 9.4 复杂算法拆解清单

| 算法 | 所在文件 | 难度 | 注意点 |
|------|---------|------|--------|
| DBSCAN 聚类 | processFSPoints (内部) | 高 | O(N²) 距离矩阵，需固定内存预算 |
| RANSAC 直线拟合 | processFSPoints (内部) | 中 | 随机迭代，需确定最大迭代次数 |
| Andrew 凸包 | processFSPoints (内部) | 中 | 需排序算法，固定大小输出 |
| Hybrid A* | PlanningSubFcn 多文件 | 高 | 哈希表+最小堆+运动学 |
| Reeds-Shepp | RSPath | 中 | 18 种路径模式查表 |
| SAT 碰撞检测 | collision_Check_SAT/Octagon | 中 | 分离轴投影 |
| 三层碰撞模型 | VehicleCollisionCheck | 高 | 自适应安全余量，~1200行 |
| 双圆弧 S 曲线 | geo_2Circle_Link | 中 | 解析几何公式 |
| IoU 多边形交 | iou_area | 中 | Sutherland-Hodgman 裁剪变体 |
| 三次多项式拟合 | ClassParallelReplanInSlot | 中 | 数值拟合，最小二乘 |

### 9.5 巨型文件拆分策略

两个"巨型"文件需特别分解：

1. **`processFSPoints.m`（~3100行，40+ 子函数）**
   - 建议拆为:
     - `fsPoints.c/h`（主调度）
     - `dbscan.c/h`（聚类）
     - `ransacLineFit.c/h`（拟合）
     - `convexHull.c/h`（凸包）
     - `fsPointsState.c/h`（状态管理）

2. **`ultrasonicCorrection.m`（~4600行，45+ 子函数）**
   - 建议拆为:
     - `ultraCorr.c/h`（主调度）
     - `visualLocking.c/h`（视觉锁定）
     - `ussCorrection.c/h`（USS校正）
     - `slotBinding.c/h`（车位绑定）
     - `ultraCorrState.c/h`（状态管理）

### 9.6 枚举类型映射

27 个 `Simulink.IntEnumType` → C99 `typedef enum`：

```c
typedef enum {
    SlotClass_Vertical  = 0,
    SlotClass_Parallel  = 1,
    SlotClass_Unknown   = 2
} SlotClass_e;
```

### 9.7 矩阵运算展开

所有矩阵都是小型的（2×2, 3×3），无需引入线性代数库，直接展开为标量运算：

```c
// 2D 旋转：MATLAB → C
// MATLAB: pt_rot = R * pt
Real xRot = cosYaw * ptX - sinYaw * ptY;
Real yRot = sinYaw * ptX + cosYaw * ptY;
```

### 9.8 浮点精度策略

- 默认使用 `Real`（float/single），对应 MATLAB 中显式的 `single()` 转换
- 精度敏感处（如坐标累积）使用 `Real64`（double）
- 统一 epsilon：`1e-6f` 用于几何比较，`1e-10f` 用于除零保护

### 9.9 跨模块依赖

> **注意：** 以下为经代码验证的实际跨目录调用关系。

```
SPTSubFcn/boundary_filter_process       → PlanningSubFcn/collinear_detection     ✅已验证
SPTSubFcn/boundary_filter_process       → PlanningSubFcn/uisee_float32_equal     ✅已验证
GeoPlanningSubFcn/geo_VT_PathPlanning   → PlanningSubFcn/VehicleCollisionCheck   ✅已验证
GeoPlanningSubFcn/geo_VT_PathPlanning   → PlanningSubFcn/get_planner_global      ✅已验证
GeoPlanningSubFcn/geo_HT_PathPlanning   → PlanningSubFcn/VehicleCollisionCheck   ✅已验证
GeoPlanningSubFcn/geo_HT_PathPlanning   → PlanningSubFcn/ClassMath.isPtInRect    ✅已验证
GeoPlanningSubFcn/geo_VH_PathPlanning   → PlanningSubFcn/VehicleCollisionCheck   ✅已验证
GeoPlanningSubFcn/geo_SurroundingCheck  → PlanningSubFcn/uisee_float32_equal     ✅已验证
GeoPlanningSubFcn/geo_circle_Intrv_check → PlanningSubFcn/VehicleCollisionCheck  （间接）
GeoPlanningSubFcn/geo_line_Intrv_check  → PlanningSubFcn/VehicleCollisionCheck   （间接）
```

> **已排除的错误依赖：** `geo_HT_PathPlanning → get_planner_global` 该调用在源码中已被注释掉（line 382），不构成实际依赖。`USSubFcn/checkIntersection → Is_Pt_In_Poly` 为同目录内调用，非跨模块依赖。

建议将 `uisee_float32_*`、`mod2pi`、`ClassMath` 等工具函数抽取到独立的 **通用数学工具库** (`mathUtils.c/h`)，避免循环依赖。

### 9.10 多返回值处理

MATLAB 函数可自由返回多个值（如 `VehicleCollisionCheck` 返回 7 个值）。C99 策略：

```c
// 方案：使用输出结构体
typedef struct {
    Boolean isCollision;
    UInteger8 collisionType;
    UInteger16 collisionIdx;
    Real collisionArea;
    Real collisionDepth;
    Real crossLine[4];
    Real safeMargin;
} CollisionResult_t;

void vehicleCollisionCheck(const CollisionCheckParams_t* params,
                           CollisionResult_t* result);
```

---

## 十、建议的拆解优先级

| 优先级 | 模块 | 原因 |
|--------|------|------|
| **P0** | 通用工具层（数学、坐标变换、枚举） | 零依赖，其他所有模块的基础 |
| **P1** | 数据管理层（标定参数、全局配置） | 定义所有参数常量和配置结构体 |
| **P2** | 碰撞检测子系统（SAT + 三层模型） | 规划层的核心依赖 |
| **P3** | 几何规划原语（双圆弧、圆弧切线等） | 原子函数，独立性强 |
| **P4** | 主规划器（VT/HT/VH + Hybrid A*） | 核心算法，依赖 P2+P3 |
| **P5** | 感知融合层（processFSPoints + ultrasonicCorrection） | 最复杂，建议最后处理 |

### P0 详细清单（零依赖基础层）

```
mathUtils.c/h
  ├── uisee_float32_equal / less / greater
  ├── uisee_float32_norm / cross_2D / cross_3D
  ├── float32_divide_zero_process
  ├── mod2pi
  ├── linear_func_slove
  └── geo_divide

coordTransform.c/h
  ├── GroundCS_2_VehCS
  ├── VehCS_2_GroundCS
  ├── uisee_angle2dcm
  ├── geo_axis_trans
  ├── transform_ipm_pt_to_global
  ├── transform_ego_pt_to_global
  └── transform_global_pt_to_ego

vehicleGeometry.c/h
  ├── get_ego_corners
  ├── get_veh_corner_point
  ├── get_octagon_corners
  └── calcuRectangleCorners

apaEnums.h
  └── 27个枚举类型定义

baseTypes.h
  └── Real / Real64 / Integer / Boolean 等类型别名
```

---

## 十一、统计概览

| 指标 | 数量 |
|------|------|
| MATLAB 文件总数 | **190** |
| 类定义 (classdef) | 13 |
| 枚举定义 | 27 |
| 独立函数 | ~150 |
| 原子函数（预估） | ~105 |
| 复合函数（预估） | ~45 |
| persistent 变量 | **46+**（processFSPoints 23 + ultrasonicCorrection 22 + get_planner_global 1） |
| CAN 信号数 | ~1870 |
| 总代码行数（估算） | ~25,000+ |
| DataDictionary 标定文件 | 14 |
| 一级功能分类 | 5 |
| 二级功能分类 | 14 |

---

## 十二、几何规划核心数据流

```
geo_slotPlanSelect
    │
    │ (选择车位ID)
    ▼
┌── geo_VT_PathPlanning  (垂直/垂直，倒车入库)
│   geo_HT_PathPlanning  (水平/侧方停车)
│   geo_VH_PathPlanning  (垂直车头入库)
    │
    │ 步骤1：碰撞预检
    │   VehicleCollisionCheck(目标位、入口位、过渡位)
    │
    │ 步骤2：空间分析 + 可选横向偏移
    │   geo_StPtSpc_vt → 左/右空间
    │   geo_2Circle_LinkHT → S曲线偏移
    │
    │ 步骤3：入口点与危险点计算
    │   geo_vt_entrPt_dngPt / geo_vh_entrPt_dngPt / geo_HT_dangerPt
    │
    │ 步骤4：迭代路径段构建（主循环，最多9段）
    │   geo_PosTypClassPkIn → 分类位置类型（1-47）
    │   │
    │   ├→ geo_circle_1pt1line  (计算自然半径)
    │   │   │
    │   │   ├→ R ≈ Rmin: 单弧（+直线）
    │   │   ├→ R > Rmin: geo_2Circle_Link（S曲线）
    │   │   └→ R < Rmin: geo_rev2Circle_Link（换向双弧）
    │   │
    │   ├→ geo_circle_2line_R（直线+弧+直线模式）
    │   │
    │   └→ 每段: geo_circle_Intrv_check / geo_line_Intrv_check
    │       → 碰撞? 截断并迭代
    │       → 无碰撞? 记录段，检查完成
    │
    │ 步骤5：1R 优化
    │   geo_vt_1R_fix / geo_ht_1R_fix
    │
    │ 步骤6：重排序 + 计算总量
    │   Path_Data（30元素数组的结构体）
    ▼
Path_Data → 离散化 → 执行
```

---

## 十三、搜索规划核心架构

```
                    +---------------------------+
                    |    外部调用者               |
                    |  (Planner_Core / S-Func)   |
                    +---------------------------+
                               │
                    +──────────▼───────────+
                    │   ClassHybridAStar    │  (配置持有者)
                    +──────────┬───────────+
                               │
            +──────────────────▼─────────────────────+
            │          Hybrid A* 搜索主循环           │
            │  节点: hybrid_node (结构体)              │
            │  开集: PriorityQueueNodesList (最小堆)   │
            │  闭集: HashNodesList (哈希表)            │
            +──┬──────────┬──────────┬───────────────+
               │          │          │
     +─────────▼──+  +────▼─────+ +─▼───────────+
     │VehicleDynam│  │Heuristic │  │目标测试      │
     │ic (自行车) │  │Cost      │  │+ RS 路径     │
     +──────┬─────+  +────┬─────+  +──┬──────────+
            │              │           │
     +──────▼──────────────▼───────────▼─────+
     │       VehicleCollisionCheck            │
     │ SAT / 点在矩形内 / 八边形模型           │
     │ 三层边界检测                            │
     +───────────────┬──────────┬─────────────+
                     │          │
              +──────▼───+ +───▼─────────+
              │ClassSlot  │ │边界处理      │
              │几何       │ │             │
              +───────────+ +─────────────+
```

---

## 十四、感知融合核心管线

```
原始传感器数据
    │
    ▼
[鱼眼FS点云]           [超声波传感器]       [车位检测]
    │                       │                   │
    ▼                       │                   │
processFSPoints()           │                   │
  - 分类(pole/curb/wall)    │                   │
  - 梯形FOV滤波             │                   │
  - DBSCAN聚类              │                   │
  - RANSAC拟合              │                   │
  - 凸包(柱)                │                   │
  - 多帧缓存                │                   │
  - 竖直线提取              │                   │
    │                       │                   │
    │ pole_contours         │ ultra_lines       │ parking_spaces
    │ curb_lines            │                   │
    │ wall_lines            │                   │
    ▼                       ▼                   ▼
ultrasonicCorrection() ◄────┴───────────────────┘
  - 视觉锁定（评分+命中计数）
  - 超声波校正（USS投影到视觉特征）
  - 竖直线与柱融合
  - 车位绑定（齐次坐标变换）
  - 线段裁剪
  - 坐标重置处理
    │
    ▼
[输出: pole_lines, curb_lines, wall_lines, pole_center]
    │
    └──→ 规划模块
```

---

## 附录 A：CAN 信号命名规则

经核实 `mfile_APA_20ms_Task_M5.m`（24312行）中的信号命名：

- **CAN 总线信号**：`<模块>_<信号名>_0x<十六进制CAN ID>`，例如 `HZP_StrAng_0x775`
- **内部/派生信号**：`<模块>_<信号名>`（无十六进制后缀），例如 `esc_vehicleSpeed`
