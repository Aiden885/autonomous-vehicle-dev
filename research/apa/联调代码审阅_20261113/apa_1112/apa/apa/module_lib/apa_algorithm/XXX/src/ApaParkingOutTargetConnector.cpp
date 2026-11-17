#include "ApaParkingOutTargetConnector.hpp"

namespace apa {

/**
 * @brief 沿用 C 入口将局部泊出路径连接到道路目标。
 * @cn_name 泊出目标连接
 * @type atomic
 * @tag_level0 自动泊车
 * @tag_level1 规划
 * @tag_level2 泊出目标连接
 * @version 1.1
 * @date 2026-11-03
 * @author wang
 */
void ApaParkingOutTargetConnector::run(const Input& input, Output& output) {
  native::ConnectOut(input.plannerInput, global::params.planning, input.localPath,
      output.connectedPath, output.connected);
}

}
