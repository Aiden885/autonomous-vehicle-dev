
#include "ACCModule.h"
#include <google/protobuf/text_format.h>


namespace pangu {
namespace modules {/*节点注册*/
REGISTER_NODE_DEFINE(ACCModule);
REGISTER_CALLBACK(ACCModule, acc_input, pangu::modules::AccInput);

void ACCModule::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SETVERSION("class1", "1.0.1");
}

bool ACCModule::Init(const std::string &ptstr) {

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
bool ACCModule::InitIO() {
  PLOG_INFO("in  InitIO");
  CREATE_OUTPUT_RETURN_IF_ERROR(acc_output, pangu::modules::AccOutput);
  
  CREATE_INPUT_RETURN_IF_ERROR(ACCModule, acc_input, pangu::modules::AccInput);
  

  return true;
}

bool ACCModule::LoadConfig(const std::string &ptstr) {
  return true;
}

bool ACCModule::Proc() {
  return true;
}
void ACCModule::acc_inputCallback(const pangu::modules::AccInput& input) {
  auto output = std::make_shared<pangu::modules::AccOutput>();
  output->set_frame_id(input.frame_id());
  PUB_MSG(acc_output, output);
  PLOG_INFO("ACC output frame_id:{}", output->frame_id());
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



} // namespace pangu
} // namespace modules