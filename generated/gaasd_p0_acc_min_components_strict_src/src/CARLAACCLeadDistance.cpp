#include "CARLAACCLeadDistance.hpp"

namespace control {

/**
 * @brief 透传运行时边界层提供的前车净距离。
 * @cn_name CARLA前车距离
 * @type block
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 2.1
 * @date 2026-06-04
 * @author liuruyu
 */
void CARLAACCLeadDistance::run(const Input& input, Output& output) {
  // 读取结构化输入中的前车距离和有效标志
  const Real distance0 = input.distance;
  const int valid0 = input.valid;
  const bool dataValid0 = (valid0 != 0);

  // 无有效数据时输出大距离，表示当前没有可用前车
  Real distanceOut0;
  int validOut0;
  if (dataValid0) {
    distanceOut0 = distance0;
    validOut0 = 1;
  } else {
    distanceOut0 = 1000000.0;
    validOut0 = 0;
  }

  // 顶层输出：写出前车净距离和有效标志
  output.distance = distanceOut0;
  output.valid = validOut0;
}

} // namespace control
