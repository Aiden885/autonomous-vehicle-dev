#include "CARLAACCLongitudinalCmd.hpp"

namespace control {

/**
 * @brief 输出结构化纵向速度控制命令。
 * @cn_name CARLA纵向控制命令
 * @type block
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 2.1
 * @date 2026-06-04
 * @author liuruyu
 */
void CARLAACCLongitudinalCmd::run(const Input& input, Output& output) {
  // 锁存目标速度与控制使能，供运行时边界层发布到 CARLA
  const Real speed0 = input.speed;
  const int enable0 = input.enable;

  // 顶层输出：写出本周期结构化控制命令
  output.speed = speed0;
  output.enable = enable0;
  output.valid = 1;
}

} // namespace control
