#include "CARLAACCLongitudinalCmd.hpp"

namespace control {

/**
 * @brief 输出供运行时边界层发布的纵向速度控制命令。
 * @cn_name CARLA纵向控制命令
 * @type block
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 2.1
 * @date 2026-06-12
 * @author liuruyu
 */
void CARLAACCLongitudinalCmd::run(const Input& input, Output& output) {
  /**
   * @brief 锁存目标速度与控制使能。
   */
  const Real speed0 = input.speed;
  const int enable0 = input.enable;

  /**
   * @brief 生成纵向控制命令有效标志。
   */
  const int valid0 = 1;

  /**
   * @brief 输出目标纵向速度。
   */
  output.speed = speed0;

  /**
   * @brief 输出纵向控制使能。
   */
  output.enable = enable0;

  /**
   * @brief 输出纵向控制命令有效标志。
   */
  output.valid = valid0;
}

} // namespace control
