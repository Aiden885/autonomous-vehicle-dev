
#include "module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43.h"
#include <google/protobuf/text_format.h>


namespace pangu {
namespace modules {/*节点注册*/
REGISTER_NODE_DEFINE(module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43);
REGISTER_CALLBACK(module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43, acc_input, pangu::modules::AccInput);

void module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SETVERSION("class1", "1.0.1");
}

bool module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43::Init(const std::string &ptstr) {

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
bool module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43::InitIO() {
  PLOG_INFO("in  InitIO");
  CREATE_OUTPUT_RETURN_IF_ERROR(acc_output, pangu::modules::AccOutput);
  
  CREATE_INPUT_RETURN_IF_ERROR(module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43, acc_input, pangu::modules::AccInput);
  

  return true;
}

bool module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43::LoadConfig(const std::string &ptstr) {
  return true;
}

bool module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43::Proc() {
  return true;
}
void module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43::acc_inputCallback(const pangu::modules::AccInput& input) {
  control::run_f1807d24_61ef_407b_932e_4df61bda48e7Traits::Input compositeBlockInstance_in;
  control::run_f1807d24_61ef_407b_932e_4df61bda48e7Traits::Output compositeBlockInstance_out;
  compositeBlockInstance_in.egoSpeedMps = input.ego_speed_mps();
  compositeBlockInstance_in.leadSpeedMps = input.lead_speed_mps();
  compositeBlockInstance_in.leadDistanceM = input.lead_distance_m();
  compositeBlockInstance_in.commandType = input.command_type();
  composite_block_instance_.run(compositeBlockInstance_in, compositeBlockInstance_out);
  auto output = std::make_shared<pangu::modules::AccOutput>();
  output->set_enable(compositeBlockInstance_out.isControlEnabled);
  output->set_target_speed_mps(compositeBlockInstance_out.targetSpeedMps);
  output->set_frame_id(input.frame_id());
  PUB_MSG(acc_output, output);
  PLOG_INFO("ACC output enable:{}, target_speed_mps:{}, frame_id:{}", output->enable(), output->target_speed_mps(), output->frame_id());
}

DEFINE_CALLBACK_HEAD(module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43, acc_input, pangu::modules::AccInput) {
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