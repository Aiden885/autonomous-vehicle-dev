#include "CARLAACCLeadSpeed.hpp"

extern "C" int carla_adapter_read_lead_vehicle(double *leadV,
                                                double *distance,
                                                double *relativeSpeed,
                                                double *ttc,
                                                int *valid);

namespace control {

/**
 * @brief 通过 CARLA 适配器读取并输出前车纵向速度。
 * @cn_name CARLA前车速度
 * @type atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 2.2
 * @date 2026-06-16
 * @author liuruyu
 */
void CARLAACCLeadSpeed::run(const Input& input, Output& output) {
  /**
   * @brief 忽略空输入端口，保持统一 run 签名。
   */
  (void)input;

  /**
   * @brief 调用 CARLA 适配器读取前车状态。
   */
  Real leadV0 = Real{0.0};
  Real distance0 = Real{1000000.0};
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
   * @brief 判断前车速度是否可用。
   */
  const bool leadSpeedUsable0 = adapterOk0 && dataValid0;

  /**
   * @brief 根据适配状态选择真实速度或安全默认值。
   */
  Real leadVOut0;
  int validOut0;
  if (leadSpeedUsable0) {
    leadVOut0 = leadV0;
    validOut0 = 1;
  } else {
    leadVOut0 = Real{0.0};
    validOut0 = 0;
  }

  /**
   * @brief 输出校验后的前车纵向速度。
   */
  output.leadV = leadVOut0;

  /**
   * @brief 输出前车速度有效标志。
   */
  output.valid = validOut0;
}

} // namespace control
