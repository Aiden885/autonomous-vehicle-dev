#include "LKSModule.h"

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
  control::LksMainFlowTraits::Input algorithm_input;
  algorithm_input.egoSpeed = input.ego_speed_mps();
  algorithm_input.c0 = input.c0();
  algorithm_input.c1 = input.c1();
  algorithm_input.c2 = input.c2();
  algorithm_input.c3 = input.c3();
  algorithm_input.curvature = input.curvature();
  algorithm_input.brakePressed = input.brake_pressed() ? 1.0 : 0.0;
  algorithm_input.driverSteerNorm = input.driver_steer_norm();

  control::LksMainFlowTraits::Output algorithm_output;
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
