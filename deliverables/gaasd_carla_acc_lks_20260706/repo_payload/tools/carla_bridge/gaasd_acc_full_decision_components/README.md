# ACC 完整决策组件包说明

> 注意：该组件包已降级为备用验证件，不作为当前 GAASD ACC 主线方案。
>
> 当前主线要求 ACC 内部尽可能使用 GAASD 基础模块搭建；自定义组件只保留在 CARLA Bridge 输入/输出边界。正式画布请参考 `docs/GAASD_ACC_完整决策控制画布方案.md`。

本目录是根据 `tools/carla_bridge/gaasd_acc_full_decision_components_src/accFullDecisionComponents.c` 扫描生成的 GAASD 组件包，用于在 GAASD 画布中搭建 ACC 决策到简化纵向控制链路。

## 组件来源

- 依据文档：`ACC算法设计原理20250318.docx`
- 决策范围：S0/S1/S2/S3 状态判定，R1-R8 决策输出，速度调整、时距调整、控制使能、历史目标维护、扭矩仲裁标志
- 控制范围：暂不实现文档中的完整 SPPVT 扭矩控制，沿用当前 CARLA 联调中验证过的目标速度控制方式
- 前提简化：默认前车持续存在，不实现无前车定速巡航分支

## 组件列表

| 组件 | 作用 |
| --- | --- |
| `ACCSpeedSubState` | 根据自车速度和最低适控车速判断适速/低速 |
| `ACCSystemSubState` | 根据控制使能和历史标志判断无史/有史、待命/在控子态 |
| `ACCSystemState` | 合成 ACC 总状态 S0/S1/S2/S3 |
| `ACCNextState` | 按状态和驾驶指令查表输出下一状态 |
| `ACCDecisionOutput` | 按状态和驾驶指令查表输出 R1-R8 决策 |
| `ACCControlEnable` | 输出 ACC 控制使能 |
| `ACCHasHistoryNext` | 更新历史目标标志 |
| `ACCLastDecisionNext` | 记录上一有效控制决策 |
| `ACCTimeGapNext` | 根据 R3/R4 更新目标时距 |
| `ACCMaxSpeedNext` | 根据 R1/R2/R5 更新速度上限 |
| `ACCTorqueArbitrationFlag` | 标记 R7 扭矩仲裁 |
| `ACCDesiredDistance` | 根据目标时距计算期望跟车距离 |
| `ACCActualTimeGap` | 计算当前实际时距 |
| `ACCTimeGapTargetSpeed` | 简化时距控制，输出目标速度 |
| `ACCTorqueArbitratedTargetSpeed` | R7 时按驾驶员期望速度做简化仲裁 |

## 推荐配套组件

本包只包含 ACC 决策和控制计算。CARLA 输入输出仍复用最小联调组件：

- `CARLAACCEgoSpeed`
- `CARLAACCLeadSpeed`
- `CARLAACCLeadDistance`
- `CARLAACCLongitudinalCmd`

对应组件包：`tools/carla_bridge/gaasd_p0_acc_min_components/`

## 重要限制

1. 该组件包需要状态记忆量，包括 `controlEnabled`、`hasHistory`、`lastActiveDecision`、`timeGap`、`maxSpeed`。如果当前 GAASD 版本的状态变量、示波器或仿真脚本仍有问题，建议先用常量占位完成画布搭建，等 GAASD 修复后再接闭环状态。
2. 文档中的完整扭矩控制暂未实现，本包只保留 `ACCTorqueArbitrationFlag` 和简化目标速度仲裁。
3. 文档和 Python 参考实现对“待命状态下是否允许增/降时距”存在差异。本包按 Word 决策表实现：待命状态下增/降时距不生效，只有在控状态的 R3/R4 调整目标时距。
4. `ACCNextState` 用于核对 Word 文档中的状态转移表。第一版画布建议仍用 `controlEnabled`、`hasHistory` 和低速判断合成当前状态，避免直接回写 `nextState` 时丢失低速 S3 覆盖。
