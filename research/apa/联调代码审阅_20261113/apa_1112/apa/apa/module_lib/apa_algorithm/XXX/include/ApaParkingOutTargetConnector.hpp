#pragma once

#include "ApaParkingPathPlanningTypes.hpp"
#include "ApaNativePlanning.hpp"

namespace apa {

/** @brief ApaParkingOutTargetConnectorTraits 是模块接口结构集合。 */
struct ApaParkingOutTargetConnectorTraits {
  /** @brief 输入端口集合。 */
  struct Input {
    ApaParkingPathPlanningInputData plannerInput{};  ///< 已由 Adapter 转换的统一规划请求；内部字段保留各 CARLA 映射。
    ApaParkingPathPlanningOutputData localPath{};  ///< 泊出规划器生成、尚未连接道路目标的局部路径。
  };
  /** @brief 输出端口集合。 */
  struct Output {
    Boolean connected = false;  ///< 局部泊出路径是否已经成功连接到目标位姿。
    ApaParkingPathPlanningOutputData connectedPath{};  ///< 连接后的完整路径；路径字段映射到 CARLA plan_ok、path_len 和 path_pts。
  };
  /** @brief 模块局部参数集合。 */
  struct Param {};
  /** @brief 模块局部跨周期状态集合。 */
  struct State {};
  /** @brief 子模块集合。 */
  struct Sub {};
};

/** @brief 保持 V5 业务行为的泊出目标连接 C++ FuncModule 包装。 */
class ApaParkingOutTargetConnector
    : public FuncModule<ApaParkingOutTargetConnectorTraits> {
 public:
  using FuncModule::FuncModule;

  /** @brief 显式映射 C++ 端口并调用现有 V5 目标连接 C 入口。 */
  void run(const Input& input, Output& output) override;
};

}
