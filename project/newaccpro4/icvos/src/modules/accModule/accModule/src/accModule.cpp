
#include "accModule.h"
#include <google/protobuf/text_format.h>


namespace pangu {
namespace modules {/*节点注册*/
REGISTER_NODE_DEFINE(accModule);
REGISTER_CALLBACK(accModule, acc_input, pangu::modules::AccInput);

void accModule::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SETVERSION("class1", "1.0.1");
}

bool accModule::Init(const std::string &ptstr) {

  SetVersion();
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
bool accModule::InitIO() {
  PLOG_INFO("in  InitIO");
  CREATE_OUTPUT_RETURN_IF_ERROR(acc_output, pangu::modules::AccOutput);
  
  CREATE_INPUT_RETURN_IF_ERROR(accModule, acc_input, pangu::modules::AccInput);
  

  return true;
}

bool accModule::LoadConfig(const std::string &ptstr) {
  return true;
}

bool accModule::Proc() {
  return true;
}
void accModule::acc_inputCallback(const pangu::modules::AccInput& input) {
  control::run_23d2a150_83e4_470e_a9b7_189357a9dfd1Traits::Input compositeBlockInstance_1_in;
  control::run_23d2a150_83e4_470e_a9b7_189357a9dfd1Traits::Output compositeBlockInstance_1_out;
  compositeBlockInstance_1_in.egoSpeedMps = input.ego_speed_mps();
  compositeBlockInstance_1_in.leadSpeedMps = input.lead_speed_mps();
  compositeBlockInstance_1_in.leadDistanceM = input.lead_distance_m();
  compositeBlockInstance_1_in.commandType = input.command_type();
  compositeBlockInstance_1_in.leadDistanceM = input.lead_speed_mps();
  compositeBlockInstance_1_in.leadSpeedMps = input.lead_distance_m();
  composite_block_instance_1_.run(compositeBlockInstance_1_in, compositeBlockInstance_1_out);
  auto output = std::make_shared<pangu::modules::AccOutput>();
  output->set_enable(compositeBlockInstance_1_out.isControlEnabled);
  output->set_target_speed_mps(compositeBlockInstance_1_out.targetSpeedMps);
  output->set_enable(compositeBlockInstance_1_out.isControlEnabled);
  output->set_target_speed_mps(compositeBlockInstance_1_out.targetSpeedMps);
  output->set_frame_id(input.frame_id());
  output->set_frame_id(input.frame_id());
  PUB_MSG(acc_output, output);
  PLOG_INFO("ACC output enable:{}, target_speed_mps:{}, frame_id:{}", output->enable(), output->target_speed_mps(), output->frame_id());
}

DEFINE_CALLBACK_HEAD(accModule, acc_input, pangu::modules::AccInput) {
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



} // namespace pangu
} // namespace modules