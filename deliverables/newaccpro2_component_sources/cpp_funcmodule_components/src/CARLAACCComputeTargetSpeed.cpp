#include "CARLAACCComputeTargetSpeed.hpp"

namespace control {

/**
 * @brief 根据车间距与速度差计算限幅后的 ACC 目标跟车速度。
 * @cn_name CARLA计算目标速度
 * @type atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 2.2
 * @date 2026-06-16
 * @author liuruyu
 */
void CARLAACCComputeTargetSpeed::run(const Input& input, Output& output) {
  /**
   * @brief 锁存输入端口和配置参数。
   */
  const Real egoV0 = input.egoV;
  const Real leadV0 = input.leadV;
  const Real distance0 = input.distance;
  const Real desiredDistance0 = param_.desiredDistance;
  const Real maxSpeed0 = param_.maxSpeed;
  const Real kDist0 = param_.kDist;
  const Real kSpeed0 = param_.kSpeed;

  /**
   * @brief 判断前车净距离是否小于零。
   */
  const bool distanceNegative0 = distance0 < Real{0.0};

  /**
   * @brief 对前车净距离做非负保护。
   */
  Real distanceSafe0;
  if (distanceNegative0) {
    distanceSafe0 = Real{0.0};
  } else {
    distanceSafe0 = distance0;
  }

  /**
   * @brief 计算车间距相对期望距离的偏差。
   */
  const Real distanceError0 = distanceSafe0 - desiredDistance0;

  /**
   * @brief 计算前车与自车的速度差。
   */
  const Real relativeSpeed0 = leadV0 - egoV0;

  /**
   * @brief 距离项：距离偏差乘以距离增益。
   */
  const Real distanceTerm0 = kDist0 * distanceError0;

  /**
   * @brief 速度项：速度差乘以速度增益。
   */
  const Real relativeSpeedTerm0 = kSpeed0 * relativeSpeed0;

  /**
   * @brief 控制律前两项求和。
   */
  const Real feedbackSpeed0 = distanceTerm0 + relativeSpeedTerm0;

  /**
   * @brief 叠加前车速度作为前馈，得到目标速度原始值。
   */
  const Real targetSpeedRaw0 = feedbackSpeed0 + leadV0;

  /**
   * @brief 将目标速度限制在 [0, maxSpeed] 范围内。
   */
  const Real targetSpeed0 =
      std::clamp(targetSpeedRaw0, Real{0.0}, maxSpeed0);

  /**
   * @brief 输出限幅后的目标速度。
   */
  output.targetSpeed = targetSpeed0;
}

} // namespace control
