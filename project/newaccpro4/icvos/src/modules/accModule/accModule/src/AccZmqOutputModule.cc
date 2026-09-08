
#include "AccZmqOutputModule.h"

#include <google/protobuf/text_format.h>


namespace pangu {
namespace modules {REGISTER_NODE_DEFINE(AccZmqOutputModule);
REGISTER_CALLBACK(AccZmqOutputModule, acc_output, pangu::modules::AccOutput);

void AccZmqOutputModule::SetVersion() {
  SETVERSION(module_name_, "1.0.0");
  SHOWVERSION();
}

bool AccZmqOutputModule::Init(const std::string& ptstr) {
  SetVersion();

  if (!LoadConfig(ptstr)) {
    PLOG_ERROR("Can't load conf file of [{}]", module_name_);
    return false;
  }

  if (!InitIO()) {
    PLOG_ERROR("creat io for {} failed, please check!", module_name_);
    return false;
  }

  zmq_output_bridge_ = std::make_unique<AccZmqOutputBridge>();
  if (!zmq_output_bridge_->Init()) {
    PLOG_ERROR("AccZmqOutputModule failed to initialize CARLA ZMQ output bridge.");
    return false;
  }
  PLOG_INFO("AccZmqOutputModule init: zmq_control_enable={}",
            acc_zmq_output_params_.zmq_control_enable());
  return true;
}

bool AccZmqOutputModule::InitIO() {
    CREATE_INPUT_RETURN_IF_ERROR(AccZmqOutputModule, acc_output,
                               pangu::modules::AccOutput);
    

  PLOG_INFO("AccZmqOutputModule InitIO end");
  return true;
}

bool AccZmqOutputModule::LoadConfig(const std::string& ptstr) {
  if (!google::protobuf::TextFormat::ParseFromString(ptstr,
                                                     &acc_zmq_output_params_)) {
    PLOG_ERROR("Can't get conf file of [{}]", module_name_);
    return false;
  }

  return true;
}

bool AccZmqOutputModule::Proc() {
  return true;
}
DEFINE_CALLBACK_HEAD(AccZmqOutputModule, acc_output, pangu::modules::AccOutput) {
  std::string inputer;
  auto msg = GET_TRIGGER_MSG_DATA_PROTO(inputer, pangu::modules::AccOutput);
  if (msg == nullptr) {
    PLOG_WARN("acc_output callback got null message from {}", inputer);
    return;
  }

  PLOG_DEBUG("input:{} acc_output: {}", inputer, msg->ShortDebugString());
  if (!acc_zmq_output_params_.zmq_control_enable()) {
    PLOG_DEBUG("AccZmqOutputModule control output disabled.");
    return;
  }
  if (!zmq_output_bridge_) {
    PLOG_ERROR("AccZmqOutputModule acc_output callback before ZMQ init.");
    return;
  }
  if (!zmq_output_bridge_->PublishControl(
      msg->target_speed_mps(), msg->enable() ? 1 : 0
                                   )) {
    PLOG_WARN("AccZmqOutputModule failed to publish ZMQ control command.");
  }
}



} // namespace pangu
} // namespace modules