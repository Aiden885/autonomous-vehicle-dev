#include "CARLAACCLeadDistance.hpp"

namespace control {

/**
 * @brief 校验并输出运行时边界层提供的前车净距离。
 * @cn_name CARLA前车距离
 * @type block
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 2.1
 * @date 2026-06-12
 * @author liuruyu
 */
void CARLAACCLeadDistance::run(const Input& input, Output& output) {
  /**
   * @brief 锁存运行时边界层提供的前车净距离和有效标志。
   */
  const Real distance0 = input.distance;
  const int valid0 = input.valid;

  /**
   * @brief 判断前车距离数据是否有效。
   */
  const bool dataValid0 = valid0 != 0;

  /**
   * @brief 根据有效标志选择真实距离或无前车默认距离。
   */
  Real distanceOut0;
  int validOut0;
  if (dataValid0) {
    distanceOut0 = distance0;
    validOut0 = 1;
  } else {
    distanceOut0 = Real{1000000.0};
    validOut0 = 0;
  }

  /**
   * @brief 输出校验后的前车净距离。
   */
  output.distance = distanceOut0;

  /**
   * @brief 输出前车距离有效标志。
   */
  output.valid = validOut0;
}

} // namespace control
