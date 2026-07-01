#include "CARLAACCLeadDistance.hpp"

extern "C" int carla_adapter_read_lead_vehicle(double *leadV,
                                                double *distance,
                                                double *relativeSpeed,
                                                double *ttc,
                                                int *valid);

namespace control {

/**
 * @brief 通过 CARLA 适配器读取并输出前车净距离。
 * @cn_name CARLA前车距离
 * @type atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 2.2
 * @date 2026-06-16
 * @author liuruyu
 */
void CARLAACCLeadDistance::run(const Input& input, Output& output) {
  /**
   * @brief 忽略空输入端口，保持统一 run 签名。
   */
  (void)input;

  /**
   * @brief 调用 CARLA 适配器读取前车状态。
   */
  Real leadV0 = Real{0.0};
  Real distance0 = param_.defaultDistance;
  Real relativeSpeed0 = Real{0.0};
  Real ttc0 = Real{1000000.0};
  int valid0 = 0;
  const int adapterRc0 = carla_adapter_read_lead_vehicle(&leadV0,
                                                         &distance0,
                                                         &relativeSpeed0,
                                                         &ttc0,
                                                         &valid0);

  /**
   * @brief 判断适配器调用是否成功。
   */
  const bool adapterOk0 = adapterRc0 == 0;

  /**
   * @brief 判断前车状态数据是否有效。
   */
  const bool dataValid0 = valid0 != 0;

  /**
   * @brief 判断前车距离是否可用。
   */
  const bool leadDistanceUsable0 = adapterOk0 && dataValid0;

  /**
   * @brief 根据适配状态选择真实距离或无前车默认距离。
   */
  Real distanceOut0;
  int validOut0;
  if (leadDistanceUsable0) {
    distanceOut0 = distance0;
    validOut0 = 1;
  } else {
    distanceOut0 = param_.defaultDistance;
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
