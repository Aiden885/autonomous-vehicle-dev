#include "CARLALKSLateralOffset.hpp"

namespace control {

/**
 * @brief 通过 CARLA 适配器读取自车相对车道中心的横向偏差并写出。
 * @cn_name CARLA横向偏差
 * @type block
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 LKS最小闭环
 * @version 2.0
 * @date 2026-06-03
 * @author liuruyu
 */
void CARLALKSLateralOffset::run(const Input& input, Output& output) {
  (void)input;

  // 边界适配局部量：接收适配器读出的车道跟踪信息
  Real lateralOffset0 = 0.0;
  Real headingError0 = 0.0;
  int laneId0 = 0;
  int roadId0 = 0;
  int valid0 = 0;

  // 边界适配调用：读取 CARLA 车道跟踪数据（C ABI，需传出参数指针）
  int rc0 = carla_adapter_read_lane_tracking(
      &lateralOffset0, &headingError0, &laneId0, &roadId0, &valid0);

  // 默认横向偏差为 0，仅当读取成功且数据有效时取用真实偏差
  Real lateralOffsetOut0 = 0.0;
  if ((rc0 == 0) && (valid0 != 0)) {
    lateralOffsetOut0 = lateralOffset0;
  } else {
    lateralOffsetOut0 = 0.0;
  }

  // 顶层输出：写出横向偏差
  output.lateralOffset = lateralOffsetOut0;
}

} // namespace control
