#include "LKSModule.h"

#include <cerrno>
#include <cstdlib>
#include <iostream>

#include "GlobalContext.hpp"

namespace {

double ReadEnvDouble(const char* name, double fallback, double min_value,
                     double max_value) {
  const char* raw = std::getenv(name);
  if (raw == nullptr || *raw == '\0') {
    return fallback;
  }
  errno = 0;
  char* end = nullptr;
  const double value = std::strtod(raw, &end);
  if (errno != 0 || end == raw || *end != '\0' || value < min_value ||
      value > max_value) {
    std::cerr << "[LKSModule] ignored invalid " << name << '=' << raw << '\n';
    return fallback;
  }
  return value;
}

void ApplyRuntimeParameters() {
  auto& params = control::global::params;
  params.lks_l0 = ReadEnvDouble("LKS_PARAM_L0", params.lks_l0, 1.0, 20.0);
  params.lks_rt = ReadEnvDouble("LKS_PARAM_RT", params.lks_rt, 0.0, 2.0);
  params.lks_rAlpha =
      ReadEnvDouble("LKS_PARAM_R_ALPHA", params.lks_rAlpha, 0.2, 1.0);
  params.lks_curvatureThreshold = ReadEnvDouble(
      "LKS_PARAM_CURVATURE_THRESHOLD", params.lks_curvatureThreshold, 0.0001,
      0.02);
  params.lks_nearPreviewDistance = ReadEnvDouble(
      "LKS_PARAM_NEAR_PREVIEW_DISTANCE", params.lks_nearPreviewDistance, 0.1,
      5.0);
  params.lks_w1 = ReadEnvDouble("LKS_PARAM_W1", params.lks_w1, 0.0, 1.0);
  params.lks_w2 = ReadEnvDouble("LKS_PARAM_W2", params.lks_w2, 0.0, 1.0);
  params.lks_w3 = ReadEnvDouble("LKS_PARAM_W3", params.lks_w3, 0.0, 1.0);
  params.lks_Kp = ReadEnvDouble("LKS_PARAM_KP", params.lks_Kp, 0.0, 1.0);
  params.lks_steerScale = ReadEnvDouble(
      "LKS_PARAM_STEER_SCALE", params.lks_steerScale, 0.1, 1.0);
  params.lks_ayMax =
      ReadEnvDouble("LKS_PARAM_AY_MAX", params.lks_ayMax, 0.5, 10.0);
  params.lks_vMin =
      ReadEnvDouble("LKS_PARAM_V_MIN", params.lks_vMin, 0.0, 10.0);
  params.lks_driverSteerThreshold = ReadEnvDouble(
      "LKS_PARAM_DRIVER_STEER_THRESHOLD", params.lks_driverSteerThreshold,
      0.01, 1.0);
  std::cout << "[LKSModule] runtime parameters"
            << " l0=" << params.lks_l0 << " rt=" << params.lks_rt
            << " rAlpha=" << params.lks_rAlpha << " weights=" << params.lks_w1
            << ',' << params.lks_w2 << ',' << params.lks_w3
            << " Kp=" << params.lks_Kp
            << " steerScale=" << params.lks_steerScale
            << " ayMax=" << params.lks_ayMax << " vMin=" << params.lks_vMin
            << " driverThreshold=" << params.lks_driverSteerThreshold << '\n';
}

}  // namespace

namespace pangu {
namespace modules {

REGISTER_NODE_DEFINE(LKSModule);
REGISTER_CALLBACK(LKSModule, lks_input, pangu::modules::LksInput);

void LKSModule::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SHOWVERSION();
}

bool LKSModule::Init(const std::string& ptstr) {
  SetVersion();
  ApplyRuntimeParameters();
  return LoadConfig(ptstr) && InitIO();
}

bool LKSModule::InitIO() {
  CREATE_OUTPUT_RETURN_IF_ERROR(lks_output, pangu::modules::LksOutput);
  CREATE_INPUT_RETURN_IF_ERROR(LKSModule, lks_input, pangu::modules::LksInput);
  return true;
}

bool LKSModule::LoadConfig(const std::string&) { return true; }

bool LKSModule::Proc() { return true; }

void LKSModule::lks_inputCallback(const pangu::modules::LksInput& input) {
  control::run_6d9ae1e1_3798_4640_91c6_8217a06e02aaTraits::Input algorithm_input{};
  algorithm_input.egoV = input.ego_speed_mps();
  algorithm_input.c0 = input.c0_m();
  algorithm_input.c1 = input.c1();
  algorithm_input.c2 = input.c2_per_m();
  algorithm_input.c3 = input.c3_per_m2();
  algorithm_input.curvature = input.curvature_per_m();
  algorithm_input.brakePressed = input.brake_pressed() ? 1.0 : 0.0;
  algorithm_input.driverSteerNorm = input.driver_steer_norm();

  control::run_6d9ae1e1_3798_4640_91c6_8217a06e02aaTraits::Output algorithm_output{};
  controller_.run(algorithm_input, algorithm_output);

  auto output = std::make_shared<pangu::modules::LksOutput>();
  output->set_frame_id(input.frame_id());
  output->set_steer_rad(algorithm_output.lksSteerRad);
  output->set_control_enabled(algorithm_output.controlEnabled != 0.0);
  output->set_valid(true);
  output->set_preview_distance_m(algorithm_output.previewDistance);
  output->set_weighted_error_m(algorithm_output.weightedError);
  PUB_MSG(lks_output, output);
}

DEFINE_CALLBACK_HEAD(LKSModule, lks_input, pangu::modules::LksInput) {
  std::string inputer;
  auto msg = GET_TRIGGER_MSG_DATA_PROTO(inputer, pangu::modules::LksInput);
  if (msg != nullptr) {
    lks_inputCallback(*msg);
  }
}

}  // namespace modules
}  // namespace pangu
