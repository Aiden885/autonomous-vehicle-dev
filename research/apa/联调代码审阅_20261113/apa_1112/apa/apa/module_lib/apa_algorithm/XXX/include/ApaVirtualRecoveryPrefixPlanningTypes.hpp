#pragma once

#include "FuncModule.hpp"

#include <array>

namespace apa {

/** @brief 恢复使用的完整碰撞环境，与 C 接口逐字段转换，固定数组容量保持一致。 */
struct ApaRecoveryEnvironment {
  Integer CheckType = 0; ///< 碰撞检测类型，单位 1。
  Integer CollisionModel = 0; ///< 碰撞模型，单位 1。
  Integer CollisionDemand = 0; ///< 碰撞需求，单位 1。
  Boolean ParkingOutFlag = false; ///< 泊出标志，单位 1。
  Boolean VerticalLastStep = false; ///< 垂直车位最后一步标志，单位 1。
  Integer SlotClass = 0; ///< 车位类型，单位 1。
  Boolean TarPosValid = false; ///< 目标位姿有效，单位 1。
  Real TarPosX = 0.0; ///< 目标位姿X，单位 m。
  Real TarPosY = 0.0; ///< 目标位姿Y，单位 m。
  Real TarPosTheta = 0.0; ///< 目标位姿航向，单位 rad。
  Real VehPosX = 0.0; ///< 车辆后轴X，单位 m。
  Real VehPosY = 0.0; ///< 车辆后轴Y，单位 m。
  Real VehPosTheta = 0.0; ///< 车辆航向角，单位 rad。
  Real VehLF = 0.0; ///< 前悬+轴距，单位 m。
  Real VehLB = 0.0; ///< 后悬，单位 m。
  Real VehLL = 0.0; ///< 车长，单位 m。
  Real VehW = 0.0; ///< 车宽，单位 m。
  Real VehWB = 0.0; ///< 轴距，单位 m。
  Boolean UseAutoMargin = false; ///< 是否按 MATLAB 语义自动解析碰撞安全裕量。
  Boolean VehNearSlotFlag = false; ///< 车辆是否位于车位附近，用于自动裕量选择。
  Boolean VehInSlotFlag = false; ///< 车辆是否已进入车位，用于自动裕量选择。
  Real FrontSafeMargin = 0.0; ///< 前部安全裕量，单位 m。
  Real RearSafeMargin = 0.0; ///< 后部安全裕量，单位 m。
  Real SideSafeMargin = 0.0; ///< 侧向安全裕量，单位 m。
  Real WhlBseLongMargin = 0.0; ///< 轴基纵向裕量，单位 m。
  Real WhlBseLatMargin = 0.0; ///< 轴基横向裕量，单位 m。
  Real FrontSafeMarginNearSlotVt = 0.0; ///< 垂直车位附近的前部安全裕量，单位 m。
  Real RearSafeMarginNearSlotVt = 0.0; ///< 垂直车位附近的后部安全裕量，单位 m。
  Real SideSafeMarginNearSlotVt = 0.0; ///< 垂直车位附近的侧向安全裕量，单位 m。
  Real FrontSafeMarginAwaySlotVt = 0.0; ///< 远离垂直车位的前部安全裕量，单位 m。
  Real RearSafeMarginAwaySlotVt = 0.0; ///< 远离垂直车位的后部安全裕量，单位 m。
  Real SideSafeMarginAwaySlotVt = 0.0; ///< 远离垂直车位的侧向安全裕量，单位 m。
  Real FrontSafeMarginNearSlotHt = 0.0; ///< 平行车位附近的前部安全裕量，单位 m。
  Real RearSafeMarginNearSlotHt = 0.0; ///< 平行车位附近的后部安全裕量，单位 m。
  Real SideSafeMarginNearSlotHt = 0.0; ///< 平行车位附近的侧向安全裕量，单位 m。
  Real FrontSafeMarginAwaySlotHt = 0.0; ///< 远离平行车位的前部安全裕量，单位 m。
  Real RearSafeMarginAwaySlotHt = 0.0; ///< 远离平行车位的后部安全裕量，单位 m。
  Real SideSafeMarginAwaySlotHt = 0.0; ///< 远离平行车位的侧向安全裕量，单位 m。
  Real FrontTriLat = 0.0; ///< 前部三角扩展的横向长度，单位 m。
  Real FrontTriLon = 0.0; ///< 前部三角扩展的纵向长度，单位 m。
  Real BackTriLat = 0.0; ///< 后部三角扩展的横向长度，单位 m。
  Real BackTriLon = 0.0; ///< 后部三角扩展的纵向长度，单位 m。
  Integer BoundaryLineNum = 0; ///< 第一边界线段数，单位 1。
  std::array<Real, 64> BoundaryLineX1{}; ///< 边界线段起点X，单位 m。
  std::array<Real, 64> BoundaryLineY1{}; ///< 边界线段起点Y，单位 m。
  std::array<Real, 64> BoundaryLineX2{}; ///< 边界线段终点X，单位 m。
  std::array<Real, 64> BoundaryLineY2{}; ///< 边界线段终点Y，单位 m。
  Integer BoundarySepPtsNum = 0; ///< 边界离散点数，单位 1。
  std::array<Real, 128> BoundarySepPtsX{}; ///< 边界离散点X，单位 m。
  std::array<Real, 128> BoundarySepPtsY{}; ///< 边界离散点Y，单位 m。
  Integer SlotTrackStopperNum = 0; ///< slot stopper数，单位 1。
  std::array<Real, 64> SlotTrackStopperX1{}; ///< stopper端点1 X，单位 m。
  std::array<Real, 64> SlotTrackStopperY1{}; ///< stopper端点1 Y，单位 m。
  std::array<Real, 64> SlotTrackStopperX2{}; ///< stopper端点2 X，单位 m。
  std::array<Real, 64> SlotTrackStopperY2{}; ///< stopper端点2 Y，单位 m。
  Integer SeparateStopperNum = 0; ///< 离散stopper数，单位 1。
  std::array<Real, 128> SeparateStopperX{}; ///< 离散stopper X，单位 m。
  std::array<Real, 128> SeparateStopperY{}; ///< 离散stopper Y，单位 m。
  Integer VirtualBoundaryLineNum = 0; ///< 虚拟边界线段数，单位 1。
  std::array<Real, 64> VirtualBoundaryLineX1{}; ///< 虚拟边界起点X，单位 m。
  std::array<Real, 64> VirtualBoundaryLineY1{}; ///< 虚拟边界起点Y，单位 m。
  std::array<Real, 64> VirtualBoundaryLineX2{}; ///< 虚拟边界终点X，单位 m。
  std::array<Real, 64> VirtualBoundaryLineY2{}; ///< 虚拟边界终点Y，单位 m。
  Boolean SlotPtsValid = false; ///< 车位角点有效，单位 1。
  std::array<Real, 8> SlotPts{}; ///< 车位四角点[P1x,P1y,...,P4x,P4y]，单位 m。
};

/** @brief 同任务安全历史约束的恢复证书，与 C 接口逐字段转换，固定数组容量保持一致。 */
struct ApaVirtualRecoveryContext {
  Boolean Enabled = false; ///< 本任务已确认小幅虚拟约束侵入，可尝试有限恢复。
  Real OriginX = 0.0; ///< 冻结起点世界 X，m。
  Real OriginY = 0.0; ///< 冻结起点世界 Y，m。
  Real OriginYaw = 0.0; ///< 冻结起点航向，rad。
  Real AnchorX = 0.0; ///< 此前正常准入的实测后轴世界 X，m。
  Real AnchorY = 0.0; ///< 此前正常准入的实测后轴世界 Y，m。
  Real AnchorYaw = 0.0; ///< 历史安全位姿航向，rad。
  Real MaxLength = 0.0; ///< 最大允许前缀路程及起点邻域半径，m。
  Real PrefixLength = 0.0; ///< 完整候选中恢复前缀的终止路程，m。
  Real MaxInitialDepth = 0.0; ///< 初始虚拟约束最大侵入深度，m。
  Real RestoredClearance = 0.0; ///< 前缀终点恢复一侧的最小几何净距，m。
  Real MaxSpeed = 0.0; ///< 恢复前缀绝对目标速度上限，m/s。
  Real MaxDuration = 0.0; ///< 前缀执行最大仿真时长，s。
  Integer LineCount = 0; ///< 冻结任务虚拟线段数量，1。
  std::array<Boolean, 64> Allowed{}; ///< 仅初始侵入的虚拟线可有限恢复。
  std::array<Real, 64> Side{}; ///< 历史安全一侧的法向符号，+1/-1。
  std::array<Real, 64> InitialClearance{}; ///< 初始有符号有限线段净距，m。
  std::array<Real, 64> LineX1{}; ///< 冻结虚拟线起点 X，m。
  std::array<Real, 64> LineY1{}; ///< 冻结虚拟线起点 Y，m。
  std::array<Real, 64> LineX2{}; ///< 冻结虚拟线终点 X，m。
  std::array<Real, 64> LineY2{}; ///< 冻结虚拟线终点 Y，m。
};

/** @brief 常曲率单方向恢复前缀，与 C 接口逐字段转换，固定数组容量保持一致。 */
struct ApaVirtualRecoveryPrefix {
  Boolean Valid = false; ///< 前缀终点及全部采样已通过正常/恢复准入。
  Integer Count = 0; ///< 有效点数，1。
  Integer Direction = 0; ///< +1 前进，-1 倒车。
  Real Length = 0.0; ///< 前缀绝对路程，m。
  Real Curvature = 0.0; ///< 后轴常曲率，1/m。
  std::array<Real, 32> X{}; ///< 世界后轴 X，m。
  std::array<Real, 32> Y{}; ///< 世界后轴 Y，m。
  std::array<Real, 32> Yaw{}; ///< 航向，rad。
  std::array<Real, 32> Odo{}; ///< 当前点到下一点的有符号路程，m。
};

}
