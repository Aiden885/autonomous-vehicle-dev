#include "ACCModule.h"

#include <google/protobuf/text_format.h>

namespace pangu {
namespace modules {

REGISTER_NODE_DEFINE(ACCModule);
REGISTER_CALLBACK(ACCModule, acc_input, pangu::modules::AccInput);

void ACCModule::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SHOWVERSION();
}

bool ACCModule::Init(const std::string& ptstr) {
  SetVersion();

  if (!LoadConfig(ptstr)) {
    PLOG_ERROR("Can't load conf file of [{}]", module_name_);
    return false;
  }

  if (!InitIO()) {
    PLOG_ERROR("creat io for {} failed, please check!", module_name_);
    return false;
  }

  return true;
}

bool ACCModule::InitIO() {
  CREATE_OUTPUT_RETURN_IF_ERROR(acc_output, pangu::modules::AccOutput);
  CREATE_INPUT_RETURN_IF_ERROR(ACCModule, acc_input, pangu::modules::AccInput);

  PLOG_INFO("in InitIO end");
  return true;
}

bool ACCModule::LoadConfig(const std::string& ptstr) {
  if (!google::protobuf::TextFormat::ParseFromString(ptstr, &ACCModule_params_)) {
    PLOG_ERROR("Can't get conf file of [{}]", module_name_);
    return false;
  }

  return true;
}

bool ACCModule::Proc() {
  return true;
}

void ACCModule::acc_inputCallback(const pangu::modules::AccInput& input) {
  control::AccMainFlowTraits::Input acc_input;
  acc_input.egoSpeed = input.ego_speed_mps();
  acc_input.leadSpeed = input.lead_speed_mps();
  acc_input.leadDistance = input.lead_distance_m();
  acc_input.commandType = input.command_type();

  control::AccMainFlowTraits::Param acc_param;
  acc_param.minDistance = ACCModule_params_.desired_distance_m();
  acc_param.distanceGain = ACCModule_params_.distance_gain();
  acc_param.speedGain = ACCModule_params_.speed_gain();
  acc_param.initialMaxSpeed = ACCModule_params_.max_speed_mps();
  acc_param.maxSpeedCap = ACCModule_params_.max_speed_mps();
  acc_main_flow_.setParam(acc_param);

  control::AccMainFlowTraits::Output acc_output;
  acc_main_flow_.run(acc_input, acc_output);

  auto output = std::make_shared<pangu::modules::AccOutput>();
  output->set_frame_id(input.frame_id());
  output->set_target_speed_mps(acc_output.targetSpeed);
  output->set_enable(acc_output.enable);
  output->set_valid(acc_output.valid);
  output->set_time_gap_s(acc_output.timeGap);
  output->set_max_speed_mps(acc_output.maxSpeed);
  output->set_decision(acc_output.decision);
  output->set_system_state(acc_output.systemState);

  PUB_MSG(acc_output, output);
  PLOG_INFO("ACC output frame_id:{}, command_type:{}, target_speed_mps:{}, "
            "enable:{}, valid:{}, decision:{}, system_state:{}",
            output->frame_id(), input.command_type(),
            output->target_speed_mps(), output->enable(), output->valid(),
            output->decision(), output->system_state());
}

DEFINE_CALLBACK_HEAD(ACCModule, acc_input, pangu::modules::AccInput) {
  PLOG_INFO("get acc_input");
  std::string inputer;
  auto msg = GET_TRIGGER_MSG_DATA_PROTO(inputer, pangu::modules::AccInput);
  if (msg == nullptr) {
    PLOG_WARN("acc_input callback got null message from {}", inputer);
    return;
  }

  PLOG_DEBUG("input:{} acc_input: {}", inputer, msg->ShortDebugString());
  acc_inputCallback(*msg);
}

}  // namespace modules
}  // namespace pangu
