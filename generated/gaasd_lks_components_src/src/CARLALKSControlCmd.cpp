#include "CARLALKSControlCmd.hpp"

namespace control {

/**
 * @brief 通过 CARLA 适配器发布车道保持所需的横纵向联合控制命令。
 * @cn_name CARLALKS控制命令
 * @type block
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 LKS最小闭环
 * @version 2.0
 * @date 2026-06-03
 * @author liuruyu
 */
void CARLALKSControlCmd::run(const Input& input, Output& output) {
  // 锁存目标速度、目标转向角与控制使能
  const Real targetSpeed0 = input.targetSpeed;
  const Real steerRad0 = input.steerRad;
  const int enable0 = input.enable;

  // 锁存目标加速度参数（LKS 测试固定为 0）
  const Real targetAccel0 = param_.targetAccel;

  // 边界适配调用：向 CARLA 发布横纵向联合控制命令（C ABI，按值传参）
  int status0 = carla_adapter_publish_control_cmd(
      targetSpeed0, targetAccel0, steerRad0, enable0);

  // 顶层输出：写出发布状态
  output.status = status0;
}

} // namespace control
