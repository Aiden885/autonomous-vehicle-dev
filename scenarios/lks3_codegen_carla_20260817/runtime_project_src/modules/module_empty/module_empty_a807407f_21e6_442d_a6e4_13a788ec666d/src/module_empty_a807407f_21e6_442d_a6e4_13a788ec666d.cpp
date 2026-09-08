
#include "module_empty_a807407f_21e6_442d_a6e4_13a788ec666d.h"
#include <google/protobuf/text_format.h>
#include "lks_runtime_parameters.h"


namespace pangu {
namespace modules {/*节点注册*/
REGISTER_NODE_DEFINE(module_empty_a807407f_21e6_442d_a6e4_13a788ec666d);
REGISTER_CALLBACK(module_empty_a807407f_21e6_442d_a6e4_13a788ec666d, lks_input, pangu::modules::LksInput);

void module_empty_a807407f_21e6_442d_a6e4_13a788ec666d::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SETVERSION("class1", "1.0.1");
}

bool module_empty_a807407f_21e6_442d_a6e4_13a788ec666d::Init(const std::string &ptstr) {

  SetVersion();
  control::runtime::applyLksRuntimeParameters();
  if (!LoadConfig(ptstr)) {
    PLOG_ERROR("can't load proto str for module:{}", module_name_);
    return false;
  }
  if (!InitIO()) {
    PLOG_ERROR("can't create io for module:{}", module_name_);
    return false;
  }

  return true;
}
bool module_empty_a807407f_21e6_442d_a6e4_13a788ec666d::InitIO() {
  PLOG_INFO("in  InitIO");
  CREATE_OUTPUT_RETURN_IF_ERROR(lks_output, pangu::modules::LksOutput);
  
  CREATE_INPUT_RETURN_IF_ERROR(module_empty_a807407f_21e6_442d_a6e4_13a788ec666d, lks_input, pangu::modules::LksInput);
  

  return true;
}

bool module_empty_a807407f_21e6_442d_a6e4_13a788ec666d::LoadConfig(const std::string &ptstr) {
  return true;
}

bool module_empty_a807407f_21e6_442d_a6e4_13a788ec666d::Proc() {
  return true;
}
void module_empty_a807407f_21e6_442d_a6e4_13a788ec666d::lks_inputCallback(const pangu::modules::LksInput& input) {
  control::run_9554bb10_0872_4665_b691_ff372271aaf9Traits::Input executeLks_in;
  control::run_9554bb10_0872_4665_b691_ff372271aaf9Traits::Output executeLks_out;
  executeLks_in.egoSpeedMps = input.ego_speed_mps();
  executeLks_in.isBrakePressed = input.brake_pressed();
  executeLks_in.driverSteerNorm = input.driver_steer_norm();
  executeLks_in.roadCurvaturePerM = input.curvature_per_m();
  executeLks_in.c3 = input.c3_per_m2();
  executeLks_in.c2 = input.c2_per_m();
  executeLks_in.c1 = input.c1();
  executeLks_in.c0 = input.c0_m();
  execute_lks_.run(executeLks_in, executeLks_out);
  auto output = std::make_shared<pangu::modules::LksOutput>();
  output->set_steer_rad(executeLks_out.steerRad);
  output->set_control_enabled(executeLks_out.isControlEnabled);
  output->set_frame_id(input.frame_id());
  PUB_MSG(lks_output, output);
  PLOG_INFO("LKS output steer_rad:{}, control_enabled:{}, frame_id:{}", output->steer_rad(), output->control_enabled(), output->frame_id());
}

DEFINE_CALLBACK_HEAD(module_empty_a807407f_21e6_442d_a6e4_13a788ec666d, lks_input, pangu::modules::LksInput) {
  PLOG_INFO("get lks_input");
  std::string inputer;
  auto msg = GET_TRIGGER_MSG_DATA_PROTO(inputer, pangu::modules::LksInput);
  if (msg == nullptr) {
    PLOG_WARN("lks_input callback got null message from {}", inputer);
    return;
  }

  PLOG_DEBUG("input:{} lks_input: {}", inputer, msg->ShortDebugString());
  lks_inputCallback(*msg);
}



} // namespace pangu
} // namespace modules
