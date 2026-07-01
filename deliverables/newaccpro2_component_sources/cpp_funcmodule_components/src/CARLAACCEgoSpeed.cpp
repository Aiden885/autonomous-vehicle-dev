#include "CARLAACCEgoSpeed.hpp"

extern "C" int carla_adapter_read_ego_state(double *egoV,
                                             double *egoX,
                                             double *egoY,
                                             double *egoYawRad,
                                             double *egoAcc,
                                             int *valid);

namespace control {

/**
 * @brief 通过 CARLA 适配器读取并输出自车纵向速度。
 * @cn_name CARLA自车速度
 * @type atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 2.2
 * @date 2026-06-16
 * @author liuruyu
 */
void CARLAACCEgoSpeed::run(const Input& input, Output& output) {
  /**
   * @brief 忽略空输入端口，保持统一 run 签名。
   */
  (void)input;

  /**
   * @brief 调用 CARLA 适配器读取自车状态。
   */
  Real egoV0 = Real{0.0};
  Real egoX0 = Real{0.0};
  Real egoY0 = Real{0.0};
  Real egoYawRad0 = Real{0.0};
  Real egoAcc0 = Real{0.0};
  int valid0 = 0;
  const int adapterRc0 = carla_adapter_read_ego_state(&egoV0,
                                                      &egoX0,
                                                      &egoY0,
                                                      &egoYawRad0,
                                                      &egoAcc0,
                                                      &valid0);

  /**
   * @brief 判断适配器调用是否成功。
   */
  const bool adapterOk0 = adapterRc0 == 0;

  /**
   * @brief 判断自车状态数据是否有效。
   */
  const bool dataValid0 = valid0 != 0;

  /**
   * @brief 判断自车速度是否可用。
   */
  const bool egoSpeedUsable0 = adapterOk0 && dataValid0;

  /**
   * @brief 根据适配状态选择真实速度或安全默认值。
   */
  Real egoVOut0;
  int validOut0;
  if (egoSpeedUsable0) {
    egoVOut0 = egoV0;
    validOut0 = 1;
  } else {
    egoVOut0 = Real{0.0};
    validOut0 = 0;
  }

  /**
   * @brief 输出校验后的自车纵向速度。
   */
  output.egoV = egoVOut0;

  /**
   * @brief 输出自车速度有效标志。
   */
  output.valid = validOut0;
}

} // namespace control
