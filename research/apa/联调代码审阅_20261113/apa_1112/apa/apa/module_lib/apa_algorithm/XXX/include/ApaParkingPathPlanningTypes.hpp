#pragma once

#include "FuncModule.hpp"

#include <array>

namespace apa {

/** @brief 统一泊车规划输入。 */
struct ApaParkingPathPlanningInputData {
  UInteger sceneQueryId = 0;  ///< 线程内场景资源编号；0 仅用显式原生约束，不可跨进程。
  Integer slotState = 1;  ///< 车位状态；6 为 MATLAB 库内重规划。
  Integer parkingAction = 0;  ///< 泊车动作：1 泊入，2 泊出。
  Integer targetSlotKind = 0;  ///< 目标车位类型枚举。
  Real vehicleX = 0.0;  ///< 车辆参考点 X (m)。
  Real vehicleY = 0.0;  ///< 车辆参考点 Y (m)。
  Real vehicleYaw = 0.0;  ///< 车辆航向角 (rad)。
  Real targetX = 0.0;  ///< 目标位置 X (m)。
  Real targetY = 0.0;  ///< 目标位置 Y (m)。
  Real targetTheta = 0.0;  ///< 目标航向角 (rad)。
  Real goalPositionTolerance = 0.0;  ///< 终点位置容差 (m)，与控制共用；0 使用共享默认值。
  Real goalYawTolerance = 0.0;  ///< 终点航向容差 (rad)，与控制共用；0 使用共享默认值。
  Real minimumTurningRadius = 0.0;  ///< 最小转弯半径 (m)。
  Real entrancePointX = 0.0;  ///< 车位入口点 X (m)。
  Real entrancePointY = 0.0;  ///< 车位入口点 Y (m)。
  Real slotPoint1X = 0.0;  ///< 车位角点1 X (m)。
  Real slotPoint1Y = 0.0;  ///< 车位角点1 Y (m)。
  Real slotPoint2X = 0.0;  ///< 车位角点2 X (m)。
  Real slotPoint2Y = 0.0;  ///< 车位角点2 Y (m)。
  Real slotPoint3X = 0.0;  ///< 车位角点3 X (m)。
  Real slotPoint3Y = 0.0;  ///< 车位角点3 Y (m)。
  Real slotPoint4X = 0.0;  ///< 车位角点4 X (m)。
  Real slotPoint4Y = 0.0;  ///< 车位角点4 Y (m)。
  Integer slotEntrance = 0;  ///< 车位入口边编号。
  Integer parkingType = 0;  ///< 泊车方向业务枚举。
  Integer surroundingType = 0;  ///< 车位周边类型业务枚举。
  Real availableSpaceOffset = 0.0;  ///< 可用空间修正量 (m)。
  Integer boundaryLineCount = 0;  ///< 真实边界线有效数量。
  std::array<Real, 64> boundaryLineX1{};  ///< 真实边界线起点 X (m)。
  std::array<Real, 64> boundaryLineY1{};  ///< 真实边界线起点 Y (m)。
  std::array<Real, 64> boundaryLineX2{};  ///< 真实边界线终点 X (m)。
  std::array<Real, 64> boundaryLineY2{};  ///< 真实边界线终点 Y (m)。
  Integer virtualBoundaryLineCount = 0;  ///< 虚拟边界线有效数量。
  std::array<Real, 64> virtualBoundaryPoint1X{};  ///< 虚拟边界线端点1 X (m)。
  std::array<Real, 64> virtualBoundaryPoint1Y{};  ///< 虚拟边界线端点1 Y (m)。
  std::array<Real, 64> virtualBoundaryPoint2X{};  ///< 虚拟边界线端点2 X (m)。
  std::array<Real, 64> virtualBoundaryPoint2Y{};  ///< 虚拟边界线端点2 Y (m)。
  Integer slotTrackStopperCount = 0;  ///< 车位跟踪挡停线数量。
  std::array<Real, 64> slotTrackStopperX1{};  ///< 车位挡停线端点1 X (m)。
  std::array<Real, 64> slotTrackStopperY1{};  ///< 车位挡停线端点1 Y (m)。
  std::array<Real, 64> slotTrackStopperX2{};  ///< 车位挡停线端点2 X (m)。
  std::array<Real, 64> slotTrackStopperY2{};  ///< 车位挡停线端点2 Y (m)。
  Integer boundarySepPointCount = 0;  ///< 真实离散障碍点数量。
  std::array<Real, 128> boundarySepPointX{};  ///< 真实离散障碍点 X (m)。
  std::array<Real, 128> boundarySepPointY{};  ///< 真实离散障碍点 Y (m)。
  Integer stopperPointCount = 0;  ///< 独立挡停点数量。
  std::array<Real, 128> stopperPointX{};  ///< 独立挡停点 X (m)。
  std::array<Real, 128> stopperPointY{};  ///< 独立挡停点 Y (m)。
  Integer shortcutSwitch = 0;  ///< 轨迹 shortcut 开关。
  Real shortcutThreshold = 0.0;  ///< shortcut 横向阈值 (m)。
  Integer needHybridAlgorithmFusion = 1;  ///< 本周期 Hybrid A* 融合开关。
};

/** @brief 统一泊车规划结果。 */
struct ApaParkingPathPlanningOutputData {
  Integer selectedSlotKind = 0;  ///< 实际选择的车位规划类型，用于诊断规划分支。
  Integer planFlag = 0;  ///< V5 整数规划成功标志。
  Boolean planOk = false;  ///< 规划成功标志，等于 planFlag != 0。
  Integer errorType = 0;  ///< V5 ApaPathPlanFault 故障码；与 planFlag 共同生成 plan_code。
  Integer plannerCase = 0;  ///< 规划分支诊断编号。
  Integer segmentCount = 0;  ///< V5 路径段数 SegNum。
  Integer directionChangeCount = 0;  ///< 路径换向次数。
  Real pathLength = 0.0;  ///< 规划路径总长度 (m)。
  Integer pathPointCount = 0;  ///< 有效离散路径点数量。
  Integer maneuverStepCount = 0;  ///< VT/HT/DthEdVT/VH 的 MATLAB 机动步数；泊出为0。
  Integer secondStageEndPointOrdinal = 0;  ///< 首段结束路径点的 MATLAB 1-based 序号；不适用时为0。
  Real firstForwardPointVehicleX = 0.0;  ///< 首个前进点在车体坐标系下的 X (m)。
  Real firstForwardPointVehicleY = 0.0;  ///< 首个前进点在车体坐标系下的 Y (m)。
  Real firstForwardPointVehicleTheta = 0.0;  ///< 首个前进点归一化车体相对航向 (rad)。
  std::array<Real, 100> pathPointX{};  ///< 路径点 X (m)。
  std::array<Real, 100> pathPointY{};  ///< 路径点 Y (m)。
  std::array<Real, 100> pathPointTheta{};  ///< 路径点航向 (rad)。
  std::array<Real, 100> pathPointOdometer{};  ///< 当前点到下一点的有符号步长 (m)。
  ApaParkingPathPlanningInputData resolvedScene{};  ///< C 入口解析后的完整请求。
};

}
