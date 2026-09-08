#include "lks_runtime_parameters.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "GlobalContext.hpp"

namespace {

double readEnvDouble(const char* name, double fallback, double minimum,
                     double maximum) {
  const char* raw = std::getenv(name);
  if (raw == nullptr || *raw == '\0') {
    return fallback;
  }

  errno = 0;
  char* end = nullptr;
  const double value = std::strtod(raw, &end);
  if (errno != 0 || end == raw || *end != '\0' || value < minimum ||
      value > maximum) {
    std::cerr << "[LKS runtime] ignored invalid " << name << '=' << raw << '\n';
    return fallback;
  }
  return value;
}

}  // namespace

namespace control {
namespace runtime {

void applyLksRuntimeParameters() {
  auto& params = global::params;

  params.lks_l0 = readEnvDouble("LKS_PARAM_L0", params.lks_l0, 1.0, 20.0);
  params.lks_rt = readEnvDouble("LKS_PARAM_RT", params.lks_rt, 0.0, 2.0);
  params.lks_rAlpha =
      readEnvDouble("LKS_PARAM_R_ALPHA", params.lks_rAlpha, 0.2, 1.0);
  params.lks_curvatureThreshold = readEnvDouble(
      "LKS_PARAM_CURVATURE_THRESHOLD", params.lks_curvatureThreshold, 0.0001,
      0.02);
  params.lks_nearPreviewDistance = readEnvDouble(
      "LKS_PARAM_NEAR_PREVIEW_DISTANCE", params.lks_nearPreviewDistance, 0.1,
      5.0);

  const double weight1 =
      readEnvDouble("LKS_PARAM_W1", params.lks_w1, 0.0, 1.0);
  const double weight2 =
      readEnvDouble("LKS_PARAM_W2", params.lks_w2, 0.0, 1.0);
  const double weight3 =
      readEnvDouble("LKS_PARAM_W3", params.lks_w3, 0.0, 1.0);
  if (std::abs(weight1 + weight2 + weight3 - 1.0) <= 1.0e-6) {
    params.lks_w1 = weight1;
    params.lks_w2 = weight2;
    params.lks_w3 = weight3;
  } else {
    std::cerr << "[LKS runtime] ignored preview weights because their sum is not 1"
              << '\n';
  }

  params.lks_Kp = readEnvDouble("LKS_PARAM_KP", params.lks_Kp, 0.0, 1.0);
  params.lks_steerScale = readEnvDouble(
      "LKS_PARAM_STEER_SCALE", params.lks_steerScale, 0.1, 1.0);
  params.lks_ayMax =
      readEnvDouble("LKS_PARAM_AY_MAX", params.lks_ayMax, 0.5, 10.0);
  params.lks_vMin =
      readEnvDouble("LKS_PARAM_V_MIN", params.lks_vMin, 0.0, 10.0);
  params.lks_driverSteerThreshold = readEnvDouble(
      "LKS_PARAM_DRIVER_STEER_THRESHOLD", params.lks_driverSteerThreshold,
      0.01, 1.0);
  params.lks_wheelBase = readEnvDouble(
      "LKS_PARAM_WHEEL_BASE", params.lks_wheelBase, 1.0, 6.0);
  params.lks_frontWheelMaxRad = readEnvDouble(
      "LKS_PARAM_FRONT_WHEEL_MAX_RAD", params.lks_frontWheelMaxRad, 0.1, 1.0);

  std::cout << "[LKS runtime] effective parameters"
            << " l0=" << params.lks_l0 << " rt=" << params.lks_rt
            << " rAlpha=" << params.lks_rAlpha
            << " curvatureThreshold=" << params.lks_curvatureThreshold
            << " nearPreviewDistance=" << params.lks_nearPreviewDistance
            << " weights=" << params.lks_w1 << ',' << params.lks_w2 << ','
            << params.lks_w3 << " Kp=" << params.lks_Kp
            << " steerScale=" << params.lks_steerScale
            << " ayMax=" << params.lks_ayMax << " vMin=" << params.lks_vMin
            << " driverSteerThreshold=" << params.lks_driverSteerThreshold
            << " wheelBase=" << params.lks_wheelBase
            << " frontWheelMaxRad=" << params.lks_frontWheelMaxRad << '\n';
}

}  // namespace runtime
}  // namespace control
