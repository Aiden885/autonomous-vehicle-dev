#include "CARLALateralCmd.hpp"

namespace control {

/**
 * @brief 通过 CARLA 适配器发布横向转向控制命令并写出发布状态。
 * @cn_name CARLA横向控制命令
 * @type block
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 仿真输出
 * @version 2.0
 * @date 2026-06-03
 * @author liuruyu
 */
void CARLALateralCmd::run(const Input& input, Output& output) {
  // 锁存目标转向角与控制使能
  const Real steerRad0 = input.steerRad;
  const int enable0 = input.enable;

  // 边界适配调用：向 CARLA 发布横向转向命令（C ABI，按值传参）
  int status0 = carla_adapter_publish_lateral_cmd(steerRad0, enable0);

  // 顶层输出：写出发布状态
  output.status = status0;
}

} // namespace control
