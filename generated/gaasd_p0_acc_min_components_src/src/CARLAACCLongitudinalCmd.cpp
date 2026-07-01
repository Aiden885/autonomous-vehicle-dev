#include "CARLAACCLongitudinalCmd.hpp"

extern "C" int carla_adapter_publish_longitudinal_cmd(double targetSpeed,
                                                       int enable);

namespace control {

/**
 * @brief 通过 CARLA 适配器发布纵向速度控制命令。
 * @cn_name CARLA纵向控制命令
 * @type atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 2.2
 * @date 2026-06-16
 * @author liuruyu
 */
void CARLAACCLongitudinalCmd::run(const Input& input, Output& output) {
  /**
   * @brief 锁存目标速度与控制使能。
   */
  const Real speed0 = input.speed;
  const int enable0 = input.enable;

  /**
   * @brief 调用 CARLA 适配器发布纵向控制命令。
   */
  const int adapterRc0 = carla_adapter_publish_longitudinal_cmd(speed0, enable0);

  /**
   * @brief 判断纵向控制命令是否发布成功。
   */
  const bool publishOk0 = adapterRc0 == 0;

  /**
   * @brief 生成纵向控制命令有效标志。
   */
  int valid0;
  if (publishOk0) {
    valid0 = 1;
  } else {
    valid0 = 0;
  }

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
