#pragma once

#include "FuncModule.hpp"

#include <array>

namespace control {

// 障碍物列表最大容量。容器映射约定（TYPE-015）：接口以定长 std::array 表达，
// 实际有效数量由 objectCount 给出。
inline constexpr int kMaxObjects = 64;

/**
 * @brief 单个障碍物目标的信息（业务结构体，供接口字段引用）。
 */
struct ObjectInfo {
  int id = 0;             ///< 目标 ID
  int type = 0;           ///< 目标类型，1 表示车辆，2 表示行人
  Real x = 0.0;           ///< 目标 ENU 坐标 X (m)
  Real y = 0.0;           ///< 目标 ENU 坐标 Y (m)
  Real yawRad = 0.0;      ///< 目标航向角 (rad)
  Real v = 0.0;           ///< 目标速度 (m/s)
  Real vx = 0.0;          ///< 目标 X 向速度 (m/s)
  Real vy = 0.0;          ///< 目标 Y 向速度 (m/s)
  Real length = 0.0;      ///< 目标长度 (m)
  Real width = 0.0;       ///< 目标宽度 (m)
  Real height = 0.0;      ///< 目标高度 (m)
};

// 障碍物列表定长容器类型别名（TYPE-015 容器映射）。
using ObjectArray = std::array<ObjectInfo, kMaxObjects>;

} // namespace control
